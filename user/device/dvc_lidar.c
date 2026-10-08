#include "dvc_lidar.h"

#include <string.h>
#include <math.h>
#include "cmsis_os2.h"
#include "drv_uart.h"
#include "usart.h"

#define LIDAR_CALI_SAMPLE_TARGET  30  // 静态均值滤波采样帧数（约 1~1.5 秒）

lidar_command_struct lidar_cmd = {0};

/* 标定内部使用的静态累加与零位偏移缓存 */
static struct {
    uint16_t sample_cnt;
    float sum_x;
    float sum_y;
    float sum_yaw;
    float offset_x;
    float offset_y;
    float offset_yaw;
} s_cali = {0};

/**
 * @brief 将航向角规约到 [-PI, PI] 闭区间，防止 0 到 2PI 跨界误差突变
 */
static inline float wrap_angle_rad(float angle)
{
    while (angle > (float)M_PI)  angle -= 2.0f * (float)M_PI;
    while (angle < -(float)M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}

/**
 * @brief 解析大端有符号 32 位整型 (Big-Endian int32)
 */
static inline int32_t parse_be_int32(const uint8_t *buf)
{
    return (int32_t)( ((uint32_t)buf[0] << 24) |
                      ((uint32_t)buf[1] << 16) |
                      ((uint32_t)buf[2] << 8)  |
                      ((uint32_t)buf[3]) );
}

/**
 * @brief 查询雷达是否已经标定完成且数据可靠
 * @return 1: 已就绪; 0: 未就绪（雷达离线或处于开机标定中）
 */
uint8_t lidar_is_ready(void)
{
    return (lidar_cmd.status == LIDAR_STATUS_READY);
}

/**
 * @brief 重新触发零位标定（清零采样计数并切入采样状态）
 */
void lidar_recalibrate(void)
{
    s_cali.sample_cnt = 0;
    s_cali.sum_x = 0.0f;
    s_cali.sum_y = 0.0f;
    s_cali.sum_yaw = 0.0f;
    lidar_cmd.status = LIDAR_STATUS_CALIBRATING;
}

/**
 * @brief 解析电脑发送的定长 14 字节位姿绝对坐标帧
 * @param buffer 接收缓存指针
 * @param length 接收到的字节长度
 * @param x 输出还原后的原始 X 坐标 (m)
 * @param y 输出还原后的原始 Y 坐标 (m)
 * @param yaw 输出还原后的原始右手系 Yaw 角度 (rad)
 * @return 1: 解析成功; 0: 数据校验失败或未找到完整帧
 */
uint8_t lidar_parse_pose_frame(const uint8_t *buffer, uint16_t length, float *x, float *y, float *yaw)
{
    if (buffer == NULL || length < LIDAR_FRAME_LEN)
    {
        return 0;
    }

    // 滑窗查找帧头 0x7E，保证处理串口粘包或错位情况
    for (uint16_t i = 0; i <= length - LIDAR_FRAME_LEN; i++)
    {
        // 1. 确认帧头 0x7E 与对应帧尾 0x21
        if (buffer[i] == LIDAR_FRAME_HEADER && buffer[i + 13] == LIDAR_FRAME_TAIL)
        {
            // 2. 按大端解析 3 个原始有符号 int32 整数 (14字节协议帧)
            int32_t raw_x   = parse_be_int32(&buffer[i + 1]);
            int32_t raw_y   = parse_be_int32(&buffer[i + 5]);
            int32_t raw_yaw = parse_be_int32(&buffer[i + 9]);

            // 3. 忠实还原雷达自身测得的物理量 (m) 原封不动
            *x = (float)raw_x / 10000.0f;
            *y = (float)raw_y / 10000.0f;

            // 4. 仅做右手系翻转（因为雷达物理硬件原生自转方向为顺时针正，翻转为逆时针为正）
            *yaw = -((float)raw_yaw / 10000.0f);
            return 1;
        }
    }

    return 0; // 校验失败或未包含完整正确帧
}

/**
 * @brief UART 接收完成回调函数（融入零位自标定状态机）
 */
void lidar_uart_callback(uint8_t *buffer, uint16_t length)
{
    float raw_x = 0.0f, raw_y = 0.0f, raw_w = 0.0f;

    // 指示灯翻转（可保留用于查看串口收包频率）
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);

    // 解析当前物理帧
    if (lidar_parse_pose_frame(buffer, length, &raw_x, &raw_y, &raw_w))
    {
        lidar_cmd.last_update_time = osKernelGetTickCount(); // 刷新心跳时间戳

        // 标定与健康状态机流转
        switch (lidar_cmd.status)
        {
        case LIDAR_STATUS_OFFLINE:
        case LIDAR_STATUS_WARMING_UP:
            // 第一次收包成功，自动切入采样标定
            lidar_recalibrate();
            break;

        case LIDAR_STATUS_CALIBRATING:
            // 累加前 LIDAR_CALI_SAMPLE_TARGET 帧做算术平均，消除开机初期的白噪声与初相偏差
            s_cali.sum_x   += raw_x;
            s_cali.sum_y   += raw_y;
            s_cali.sum_yaw += raw_w;
            s_cali.sample_cnt++;

            if (s_cali.sample_cnt >= LIDAR_CALI_SAMPLE_TARGET)
            {
                s_cali.offset_x   = s_cali.sum_x / (float)LIDAR_CALI_SAMPLE_TARGET;
                s_cali.offset_y   = s_cali.sum_y / (float)LIDAR_CALI_SAMPLE_TARGET;
                s_cali.offset_yaw = s_cali.sum_yaw / (float)LIDAR_CALI_SAMPLE_TARGET;
                lidar_cmd.status  = LIDAR_STATUS_READY; // 零位标定锁定，正式就绪
            }

            // 标定未完全收敛前，输出保持绝对零位，防止闭环外环抖动
            lidar_cmd.x = 0.0f;
            lidar_cmd.y = 0.0f;
            lidar_cmd.w = 0.0f;
            break;

        case LIDAR_STATUS_READY:
            // 1. 去除开机位置原点平移偏移，得到相对位移 (此位移仍基于雷达上电时的绝对世界坐标系)
            float dx_raw = raw_x - s_cali.offset_x;
            float dy_raw = raw_y - s_cali.offset_y;

            // 【修复核心】2. 动态偏航角逆旋转：将相对位移投影到标定时刻的车体朝向上
            // 这样无论在什么角度重新标定，当前的朝向都会被重新定义为正前方(X轴)
            float cos_yaw_offset = cosf(s_cali.offset_yaw);
            float sin_yaw_offset = sinf(s_cali.offset_yaw);
            
            float dx_cali =  dx_raw * cos_yaw_offset + dy_raw * sin_yaw_offset;
            float dy_cali = -dx_raw * sin_yaw_offset + dy_raw * cos_yaw_offset;

            // 3. Mid-360 相对三轮底盘车头的 120 度安装外参刚体旋转
            //    此时的 dx_cali, dy_cali 已经对齐了当前朝向，接着原封不动套用你原本的硬件外参变换即可：
            //    cos(-120°) = -0.5f,  sin(-120°) = -0.8660254f
            //    X 轴保持不变 (直走 +1m 产生 X = +1.0)
            lidar_cmd.x =   dx_cali * (-0.5000000f) - dy_cali * (-0.8660254f);

            //    Y 轴取负翻转回标准右手系 (向左平移 1m 产生 Y = +1.0)
            lidar_cmd.y = -(dx_cali * (-0.8660254f) + dy_cali * (-0.5000000f));

            // 4. 航向角只需相对于标定开机姿态归零即可
            lidar_cmd.w = wrap_angle_rad(raw_w - s_cali.offset_yaw);
            break;
        }
    }
}

/**
 * @brief 雷达通信初始化（显式传参绑定硬件与管理对象）
 * @param uart_manage 串口管理对象指针
 * @param huart       HAL 库串口句柄指针
 * @param rx_buf_size 接收缓冲区长度
 */
void lidar_init(uart_manage_object_struct *uart_manage, UART_HandleTypeDef *huart, uint16_t rx_buf_size)
{
    if (uart_manage == NULL || huart == NULL || rx_buf_size == 0)
    {
        return;
    }

    // 1. 复位全局数据结构体与校准静态变量
    memset(&lidar_cmd, 0, sizeof(lidar_cmd));
    memset(&s_cali, 0, sizeof(s_cali));

    // 2. 初始置为预热等待状态
    lidar_cmd.status = LIDAR_STATUS_WARMING_UP;

    // 3. 显式绑定底层 UART 驱动与雷达解析回调
    uart_manage_init(uart_manage, huart, lidar_uart_callback, rx_buf_size);
}