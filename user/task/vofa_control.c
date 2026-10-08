
#include "vofa_control.h"

#include <stdio.h>
#include <string.h>

#include "chassis_3.h"
#include "usart.h"
#include "cmsis_os2.h"
#include "dvc_vofa.h"
#include "chassis_control.h"
#include "Control_slave.h"




vofa_struct vofa_debug;

// 1. 实体定义全局变量
volatile float    g_vofa_vx = 0.0f;
volatile float    g_vofa_vy = 0.0f;
volatile float    g_vofa_vw = 0.0f;
volatile uint32_t g_vofa_last_time = 0;

static float vofa_motor0_target_omega;
static float vofa_motor0_now_omega;
static float vofa_motor1_target_omega;
static float vofa_motor1_now_omega;
static float vofa_motor2_target_omega;
static float vofa_motor2_now_omega;

const char *vofa_cmd_list[] = {
    "motor_id",   // 0
    "kp",         // 1
    "ki",         // 2
    "kd",         // 3
    "target_rpm", // 4
    "target_deg", // 5
    "chassis_speed",  //6
    "ctrl_mode",   //7
};
void uart_callback_function(uint8_t *Buffer, uint16_t Length)
{

    // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);

    if (Buffer == NULL || Length == 0) return;
    Buffer[Length] = '\0';
    const char *prefix = "chassis_speed=";
    const size_t prefix_len = 14; // strlen("chassis_speed=")

    // 检查开头是否为 "chassis_rpm="
    if (Length > prefix_len && strncmp((char *)Buffer, prefix, prefix_len) == 0)
    {
        float vx = 0.0f;
        float vy = 0.0f;
        char end_char = 0;

        // 2. 从等号后面硬解析两个数字（逗号分隔，兼容负号与小数）
        if (sscanf((char *)Buffer + prefix_len, "%f,%f%c", &vx, &vy, &end_char) == 3 && end_char == '#')
        {
            // === 匹配成功！执行你的底盘控制逻辑 ===
            // 将 RPM 转换为底盘速度 m/ ,直接存入全局变量
            g_vofa_vx = vx;
            g_vofa_vy = vy;
            g_vofa_vw = 0.0f;
            g_vofa_last_time = osKernelGetTickCount(); // 刷新时间戳

            return;
        }
    }


    vofa_uart_rxcpltcallback(&vofa_debug, Buffer, Length);

    const int32_t cmd_index = vofa_get_variable_index(&vofa_debug);
    const float   cmd_value = vofa_get_variable_value(&vofa_debug);

    if (cmd_index == 7) // 收到 ctrl_mode 切换指令
    {
        uint8_t mode = (uint8_t)cmd_value;
        if (mode <= 2)
        {

            g_chassis_ctrl_mode = (chassis_control_mode_enum)mode;
            // 模式切换时底盘先安全刹停，防止上个模式的速度残留
            chassis_set_target_speed(&chassis, 0.0f, 0.0f, 0.0f);
        }
        return;
    }

}

void vofa_task(void *argument)
{


  uart_manage_init(&uart1_manage_object, &huart1, uart_callback_function, 100);
  vofa_init(&vofa_debug, &uart1_manage_object, (sizeof(vofa_cmd_list) / sizeof(char *)), vofa_cmd_list, 0x7F800000);


  vofa_set_data(&vofa_debug, 6, &vofa_motor0_target_omega, &vofa_motor0_now_omega,
                                       &vofa_motor1_target_omega ,&vofa_motor1_now_omega,
                                       &vofa_motor2_target_omega ,&vofa_motor2_now_omega
                                       );


  uint32_t tick_count = osKernelGetTickCount();
  for (;;)
  {
      tick_count += 100;
      osDelayUntil(tick_count);


      vofa_motor0_target_omega = RADPS_TO_RPM * chassis.motors[0].target_omega;
      vofa_motor0_now_omega    = RADPS_TO_RPM * chassis.motors[0].now_omega;
      vofa_motor1_target_omega = RADPS_TO_RPM * chassis.motors[1].target_omega;
      vofa_motor1_now_omega    = RADPS_TO_RPM * chassis.motors[1].now_omega;
      vofa_motor2_target_omega = RADPS_TO_RPM * chassis.motors[2].target_omega;
      vofa_motor2_now_omega    = RADPS_TO_RPM * chassis.motors[2].now_omega;
      // 调用发送函数
      vofa_update_and_send(&vofa_debug);
  }
}



