#include "chassis_control.h"

#include <math.h>
#include "cmsis_os2.h"
#include "Control_slave.h"
#include "dvc_lidar.h"
#include "fdcan.h"
#include "usart.h"
#include "vofa_control.h"

chassis_struct chassis;

volatile chassis_control_mode_enum g_chassis_ctrl_mode = CHASSIS_MODE_VOFA_SPEED;
volatile chassis_pos_source_enum   g_chassis_pos_source = POS_SOURCE_ODOM_ONLY;

/**
 * @brief CAN/FDCAN 接收中断回调函数
 */
void can_chassis_callback(fdcan_rx_buffer_struct* rx_buffer)
{
    uint32_t id = rx_buffer->header.Identifier;
    const uint8_t* data = rx_buffer->data;

    // CAN 反馈先由底盘认领解析，若匹配电机反馈则返回 1
    if (chassis_process_can_feedback(&chassis, id, data) == 1)
    {
        return;
    }
}

/**
 * @brief 底盘 1ms 控制主任务
 */
void chassis_task(void* argument)
{
    // 1. 底层总线与驱动初始化
    fdcan_manage_init(&fdcan1_manage_object, &hfdcan1, can_chassis_callback);
    chassis_init(&chassis);

    // 2. 初始化雷达通信及标定状态机（内部开启 UART DMA/空闲中断接收）
    lidar_init(&uart2_manage_object, &huart2, 128);

    ControlCommand rx_cmd;
    static uint8_t cmd_div = 0;

    uint32_t tick_count = osKernelGetTickCount();

    for (;;)
    {
        tick_count++;
        osDelayUntil(tick_count);

        uint32_t now = osKernelGetTickCount();

        /* ----------------- 1. 雷达数据同步与健康管理 ----------------- */
        // 通信超时判定：500ms 内有更新说明雷达物理连接在线
        if ((now - lidar_cmd.last_update_time) < 500 && lidar_cmd.last_update_time != 0)
        {
            // 实时将雷达设备层输出的坐标赋予底盘世界坐标系
            chassis.world_x = lidar_cmd.x;
            chassis.world_y = lidar_cmd.y;
            chassis.world_w = lidar_cmd.w;

            // 状态机就绪检测：区分“在线但标定中”与“标定完成已就绪”
            if (lidar_is_ready())
            {
                g_chassis_pos_source = POS_SOURCE_LIDAR_GLOBAL;
            }
            else
            {
                g_chassis_pos_source = POS_SOURCE_LIDAR_CALIBRATING;
            }
        }
        else
        {
            // 雷达离线降级
            g_chassis_pos_source = POS_SOURCE_ODOM_ONLY;

            // 若当前正处于位置控制模式，雷达失联必须触发刹停保护
            if (g_chassis_ctrl_mode == CHASSIS_MODE_VOFA_POSITION ||
                g_chassis_ctrl_mode == CHASSIS_MODE_LIDAR_POSITION)
            {
                chassis_stop(&chassis);
            }
        }

        /* ----------------- 2. 指令解析与模式调度 (10ms 周期分频) ----------------- */
        if (++cmd_div >= 10)
        {
            cmd_div = 0;

            switch (g_chassis_ctrl_mode)
            {
            /* 模式 0: VOFA 速度调试控制 */
            case CHASSIS_MODE_VOFA_SPEED:
                if ((now - vofa_cmd.last_update_time) < 60000)
                {
                    chassis_set_target_speed(&chassis,
                                             vofa_cmd.speed.vx,
                                             vofa_cmd.speed.vy,
                                             vofa_cmd.speed.vw);
                }
                else
                {
                    chassis_stop(&chassis); // VOFA 超时刹停
                }
                break;

            /* 模式 1: 遥控器手控速度 */
            case CHASSIS_MODE_RC_SPEED:
                if (ControlSlave_Get(&rx_cmd))
                {
                    int32_t raw_vx = (int32_t)rx_cmd.l_x - 2048;
                    int32_t raw_vy = -((int32_t)rx_cmd.l_y - 2048);
                    int32_t raw_vw = -((int32_t)rx_cmd.r_y - 2048);

                    // 摇杆中心死区滤除
                    if (raw_vx > -150 && raw_vx < 150) raw_vx = 0;
                    if (raw_vy > -150 && raw_vy < 150) raw_vy = 0;
                    if (raw_vw > -150 && raw_vw < 150) raw_vw = 0;

                    // 从摇杆获取的是“期望坐标系”下的 X 和 Y 速度
                    float vx_cmd = (float)raw_vx / 2048.0f * 1.20f;
                    float vy_cmd = (float)raw_vy / 2048.0f * 1.20f;
                    float vw_cmd = (float)raw_vw / 2048.0f * 6.00f;

                    float final_vx_body = 0.0f;
                    float final_vy_body = 0.0f;

                    // 假设 rx_cmd 里面有个开关变量 sw_mode (比如 1=机体, 2=全局)
                    // 如果没有开关，你可以直接把这里改成 #if 1 或 #if 0 来固定测试某种模式
                    if (1) // 车头机体坐标系控制 (FPV视角)
                    {
                        final_vx_body = vx_cmd;
                        final_vy_body = vy_cmd;
                    }
                    else // 全局绝对坐标系控制 (第三人称/无头模式)
                    {
                        // 将期望的世界系速度，根据当前车体航向，反向投影到机体坐标系
                        // 相当于给速度向量乘上一个 R(-world_w) 的旋转矩阵
                        float cos_yaw = cosf(chassis.world_w);
                        float sin_yaw = sinf(chassis.world_w);

                        final_vx_body =  vx_cmd * cos_yaw + vy_cmd * sin_yaw;
                        final_vy_body = -vx_cmd * sin_yaw + vy_cmd * cos_yaw;
                    }

                    // 最终下发给逆运动学的永远是机体坐标系下的速度
                    chassis_set_target_speed(&chassis, final_vx_body, final_vy_body, vw_cmd);
                }
                else
                {
                    chassis_stop(&chassis); // 遥控失联刹停
                }
                break;

            /* 模式 2: 雷达上位机速度控制 */
            case CHASSIS_MODE_LIDAR_SPEED:
                chassis_stop(&chassis);
                break;

            /* 模式 3: VOFA 世界坐标目标点位控 */
            case CHASSIS_MODE_VOFA_POSITION:
                // 安全门禁：若雷达还未完成 30 帧零位标定（前 10 秒等待期），强制保持刹停
                if (!lidar_is_ready())
                {
                    chassis_stop(&chassis);
                    break;
                }

                if ((now - vofa_cmd.last_update_time) < 60000)
                {
                    chassis_set_target_pose(&chassis,
                                            vofa_cmd.target_pose.x,
                                            vofa_cmd.target_pose.y,
                                            vofa_cmd.target_pose.yaw);
                }
                else
                {
                    chassis_stop(&chassis); // VOFA 目标点失联超时
                }
                break;

            /* 模式 4: 全自主路径巡线位控 */
            case CHASSIS_MODE_LIDAR_POSITION:
                if (!lidar_is_ready())
                {
                    chassis_stop(&chassis);
                    break;
                }
                chassis_stop(&chassis);
                break;

            default:
                chassis_stop(&chassis);
                break;
            }
        }

        /* ----------------- 3. 闭环解算与驱动下发 (1ms 控制周期) ----------------- */
        // 仅在位控模式且雷达标定就绪时，才执行位置外环计算
        if ((g_chassis_ctrl_mode == CHASSIS_MODE_VOFA_POSITION ||
             g_chassis_ctrl_mode == CHASSIS_MODE_LIDAR_POSITION) && lidar_is_ready())
        {
            chassis_position_pid_update(&chassis); // 计算位置环，输出期望底盘线速度与角速度
        }

        // 2. 速度控制模式：仅锁定航向 W，其余由摇杆/上位机开环控制
        else if ((g_chassis_ctrl_mode == CHASSIS_MODE_VOFA_SPEED ||
                  g_chassis_ctrl_mode == CHASSIS_MODE_RC_SPEED) && lidar_is_ready())
        {
            // 航向控制器：积分器 + 前馈闭环，生成 target_speed_x/y/w
            chassis_heading_controller_update(&chassis);
        }
        // 3. 速度控制模式 (雷达离线/未就绪：开环透传兜底，保证手控可用)
        else if (g_chassis_ctrl_mode == CHASSIS_MODE_VOFA_SPEED ||
                 g_chassis_ctrl_mode == CHASSIS_MODE_RC_SPEED)
        {
            chassis.target_speed_x = chassis.cmd_vx;
            chassis.target_speed_y = chassis.cmd_vy;
            chassis.target_speed_w = chassis.cmd_vw;
        }




        chassis_calculate_inverse_kinematics(&chassis); // 三轮全向运动学逆解，输出电机转速
        chassis_update_speed_pid(&chassis);             // 电机单环速度 PID 运算
        chassis_send_can_data(&chassis);                // CAN 驱动控制帧下发

        /* ----------------- 4. 电机离线心跳检测：独立每 50ms 判定一次 ----------------- */
        static uint16_t alive_cnt = 0;
        if (++alive_cnt >= 50)
        {
            alive_cnt = 0;
            for (int i = 0; i < 3; i++)
            {
                motor_2006_tim_alive_periodelapsedcallback(&chassis.motors[i]);
            }
        }
    }
}