
#include "vofa_control.h"

#include <stdio.h>
#include <string.h>

#include "chassis_3.h"
#include "usart.h"
#include "cmsis_os2.h"
#include "dvc_vofa.h"
#include "chassis_control.h"
#include "dvc_lidar.h"
vofa_struct vofa_debug;

static float vofa_buffer1;
static float vofa_buffer2;
static float vofa_buffer3;
static float vofa_buffer4;
static float vofa_buffer5;
static float vofa_buffer6;


vofa_cmd_struct vofa_cmd = {
    .speed = {0.0f, 0.0f, 0.0f},
    .target_pose = {0.0f, 0.0f, 0.0f},
    .last_update_time = 0,
    .is_new_cmd = 0
};


const char *vofa_cmd_list[] = {
    "motor_id",   // 0
    "kp",         // 1
    "ki",         // 2
    "kd",         // 3
    "target_rpm", // 4
    "target_deg", // 5
    "chassis_speed",  //6
    "ctrl_mode",   //7
    "chassis_pose", //8
    "reset_pose"   //9
};

static uint8_t cur_motor_id = 1; //



void uart_callback_function(uint8_t *Buffer, uint16_t Length)
{
    if (Buffer == NULL || Length == 0) return;
    Buffer[Length] = '\0';

    /* ================= 1. 解析 chassis_speed=vx,vy,vw# ================= */
    const char *prefix_speed = "chassis_speed=";
    const size_t prefix_speed_len = 14; // strlen("chassis_speed=")

    if (Length > prefix_speed_len && strncmp((char *)Buffer, prefix_speed, prefix_speed_len) == 0)
    {
        float vx = 0.0f;
        float vy = 0.0f;
        float vw = 0.0f;
        char end_char = 0;

        // 解析 3 个浮点数（vx, vy, vw），末尾匹配 '#'
        if (sscanf((char *)Buffer + prefix_speed_len, "%f,%f,%f%c", &vx, &vy, &vw, &end_char) == 4 && end_char == '#')
        {
            vofa_cmd.speed.vx = vx;
            vofa_cmd.speed.vy = vy;
            vofa_cmd.speed.vw = vw;
            vofa_cmd.last_update_time = osKernelGetTickCount(); // 刷新时间戳
            vofa_cmd.is_new_cmd = 1;

            return;
        }
    }

    /* ================= 2. 解析 chassis_pose=x,y,yaw# ================= */
    const char *prefix_pose = "chassis_pose=";
    const size_t prefix_pose_len = 13; // strlen("chassis_pose=")

    if (Length > prefix_pose_len && strncmp((char *)Buffer, prefix_pose, prefix_pose_len) == 0)
    {
        float x   = 0.0f;
        float y   = 0.0f;
        float yaw = 0.0f;
        char end_char = 0;

        // 解析 3 个浮点数（x, y, yaw），末尾匹配 '#'
        if (sscanf((char *)Buffer + prefix_pose_len, "%f,%f,%f%c", &x, &y, &yaw, &end_char) == 4 && end_char == '#')
        {
            vofa_cmd.target_pose.x   = x;
            vofa_cmd.target_pose.y   = y;
            vofa_cmd.target_pose.yaw = yaw;
            vofa_cmd.last_update_time = osKernelGetTickCount(); // 刷新时间戳
            vofa_cmd.is_new_cmd = 1;

            return;
        }
    }


    vofa_uart_rxcpltcallback(&vofa_debug, Buffer, Length);

    const int32_t cmd_index = vofa_get_variable_index(&vofa_debug);
    const float   cmd_value = vofa_get_variable_value(&vofa_debug);

    if (cmd_index == 0) { // 切换电机 ID
        cur_motor_id = (uint8_t)cmd_value;
        return;
    }


    if (cmd_index == 1) // 收到 kp 切换指令
    {
        if      (cur_motor_id == 0) pid_set_kp(&chassis.pid_x, cmd_value);
        else if (cur_motor_id == 1) pid_set_kp(&chassis.pid_y, cmd_value);
        else if (cur_motor_id == 2) pid_set_kp(&chassis.pid_w, cmd_value);
        else if (cur_motor_id == 3)
        {
            pid_set_kp(&chassis.motors[0].pid_omega, cmd_value);
            pid_set_kp(&chassis.motors[1].pid_omega, cmd_value);
            pid_set_kp(&chassis.motors[2].pid_omega, cmd_value);
        }

        return;
    }

    if (cmd_index == 7) // 收到 ctrl_mode 切换指令
    {
        uint8_t mode = (uint8_t)cmd_value;
        if (mode <= 4)
        {

            g_chassis_ctrl_mode = (chassis_control_mode_enum)mode;
            chassis_stop(&chassis);
        }
        return;
    }
    if (cmd_index == 9)
    {
        uint8_t mode = (uint8_t)cmd_value;
        if (mode == 1)
        {
            lidar_recalibrate(); // 触发重新零位标定
            chassis_stop(&chassis);      // 安全停车

        }

    }

}

void vofa_task(void *argument)
{


  uart_manage_init(&uart1_manage_object, &huart1, uart_callback_function, 100);
  vofa_init(&vofa_debug, &uart1_manage_object, (sizeof(vofa_cmd_list) / sizeof(char *)), vofa_cmd_list, 0x7F800000);


  vofa_set_data(&vofa_debug, 6, &vofa_buffer1 ,&vofa_buffer2, &vofa_buffer3,
                                       &vofa_buffer4 ,&vofa_buffer5, &vofa_buffer6
                                       );


  uint32_t tick_count = osKernelGetTickCount();
  for (;;)
  {
      tick_count += 100;
      osDelayUntil(tick_count);

      // vofa_buffer1 = chassis.motors[0].pid_omega.ff_out;
      // vofa_buffer2 = chassis.motors[0].pid_omega.i_out;
      // vofa_buffer3 = chassis.motors[0].target_omega;
      // vofa_buffer4    = chassis.motors[0].now_omega;
      // vofa_buffer5 = chassis.motors[1].target_omega;
      // vofa_buffer6    = chassis.motors[1].now_omega;

      // vofa_buffer1 = chassis.motors[0].target_omega;
      // vofa_buffer2    = chassis.motors[0].now_omega;
      // vofa_buffer3 = chassis.motors[1].target_omega;
      // vofa_buffer4    = chassis.motors[1].now_omega;
      // vofa_buffer5 = chassis.motors[2].target_omega;
      // vofa_buffer6    = chassis.motors[2].now_omega;

      // vofa_buffer1 = chassis.target_speed_x;
      // vofa_buffer2 = chassis.target_speed_y;
      // vofa_buffer3 = chassis.target_speed_w;
      // vofa_buffer4 = chassis.world_x;
      // vofa_buffer5 = chassis.world_y;
      // vofa_buffer6 = chassis.world_w;

      vofa_buffer1 = chassis.target_speed_x;
      vofa_buffer2 = chassis.target_speed_y;
      vofa_buffer3 = chassis.target_speed_w;
      vofa_buffer4 = chassis.world_x;
      vofa_buffer5 = chassis.world_y;
      vofa_buffer6 = chassis.world_w;


      // 调用发送函数
      vofa_update_and_send(&vofa_debug);
  }
}



