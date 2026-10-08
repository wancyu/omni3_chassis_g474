#include "chassis_control.h"
#include "cmsis_os2.h"
#include "Control_slave.h"
#include "dvc_lidar.h"
#include "fdcan.h"
#include "vofa_control.h"


chassis_struct chassis;

volatile chassis_control_mode_enum g_chassis_ctrl_mode = CHASSIS_MODE_RC;

void can_chassis_callback(fdcan_rx_buffer_struct* rx_buffer)
{
    uint32_t id = rx_buffer->header.Identifier;
    const uint8_t* data = rx_buffer->data;

    // can 反馈，先由底盘认领解析，如果解析失败，切换到普通 canid 解析
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
    // 初始化流程
    // fdcan初始化
    // -> chassis 初始化(内部包含3个2006电机初始化)
    fdcan_manage_init(&fdcan1_manage_object, &hfdcan1, can_chassis_callback);
    chassis_init(&chassis);

    lidar_init(); // 开启 USART2 DMA/空闲中断接收并绑定回调



    uint32_t tick_count = osKernelGetTickCount();


    for (;;)
    {
        tick_count++;
        osDelayUntil(tick_count);


        ControlCommand rx_cmd; //

        static uint8_t cmd_div = 0;
        if (++cmd_div >= 10)
        {
            cmd_div = 0;
            uint32_t now = osKernelGetTickCount();

            switch (g_chassis_ctrl_mode)
            {
            /* 模式 0: VOFA 调试控制 */
            case CHASSIS_MODE_VOFA:
                if (now - g_vofa_last_time < 500)
                {
                    chassis_set_target_speed(&chassis, g_vofa_vx, g_vofa_vy, g_vofa_vw);
                }
                else
                {
                    chassis_set_target_speed(&chassis, 0.0f, 0.0f, 0.0f);
                }
                break;

            /* 模式 1: 遥控器手控 */
            case CHASSIS_MODE_RC:
                if (ControlSlave_Get(&rx_cmd))
                {
                    int32_t raw_vx = (int32_t)rx_cmd.l_x - 2048;
                    int32_t raw_vy = -((int32_t)rx_cmd.l_y - 2048);
                    int32_t raw_vw = -((int32_t)rx_cmd.r_y - 2048);

                    // 死区滤除
                    if (raw_vx > -150 && raw_vx < 150) raw_vx = 0;
                    if (raw_vy > -150 && raw_vy < 150) raw_vy = 0;
                    if (raw_vw > -150 && raw_vw < 150) raw_vw = 0;

                    float vx =  (float)raw_vx / 2048.0f * 1.20f;
                    float vy =  (float)raw_vy / 2048.0f * 1.20f;
                    float vw =  (float)raw_vw / 2048.0f * 6.00f;

                    chassis_set_target_speed(&chassis, vx, vy, vw);
                }
                else
                {
                    chassis_set_target_speed(&chassis, 0.0f, 0.0f, 0.0f);
                }
                break;

            /* 模式 2: 串口2 雷达 / 上位机自主导航 */
            case CHASSIS_MODE_AUTO:
                if (now - g_lidar_cmd.last_update_time < 200)
                {
                    chassis_set_target_speed(&chassis, g_lidar_cmd.vx, g_lidar_cmd.vy, g_lidar_cmd.vw);
                }
                else
                {
                    chassis_set_target_speed(&chassis, 0.0f, 0.0f, 0.0f);
                }
                break;

            default:
                chassis_set_target_speed(&chassis, 0.0f, 0.0f, 0.0f);
                break;
            }
        }

        chassis_calculate_inverse_kinematics(&chassis); // 1ms 逆解
        chassis_update_speed_pid(&chassis);             // 1ms 速度 PID
        chassis_send_can_data(&chassis);                // 1ms CAN 控制帧下发

        /* ----------------- 3. 电机离线心跳检测：独立每 50ms 判定一次 ----------------- */
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