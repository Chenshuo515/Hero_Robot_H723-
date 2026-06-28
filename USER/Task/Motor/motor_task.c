//
// Created by Gleam on 25-8-22.
//

#include "rm_algorithm.h"
#include "rm_module.h"

static float motor_dt;
uint32_t tec,rec;

UBaseType_t MotorHighWaterMark;
/* USER CODE END Header_TransmissionTask_Entry */
void MotorTask_Entry(void const * argument)
{
    /* USER CODE BEGIN TransmissionTask_Entry */
    /* Infinite loop */
    static float motor_start;

    motor_start = dwt_get_time_ms();

    for (;;)
    {
        // 读取发送错误计数器（TEC）和接收错误计数器（REC）
         tec = (hfdcan3.Instance->ECR & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos;
         rec = (hfdcan3.Instance->ECR & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos;
        motor_start = dwt_get_time_ms();

#ifdef BSP_USING_DJI_MOTOR
        dji_motor_control();
#endif /* BSP_USING_DJI_MOTOR */
#ifdef BSP_USING_LK_MOTOR
        lk_motor_control();
#endif /* BSP_USING_LK_MOTOR */
#ifdef BSP_USING_HT_MOTOR
        ht_motor_control();
#endif /* BSP_USING_HT_MOTOR */

        MotorHighWaterMark = uxTaskGetStackHighWaterMark( NULL );


        /* 用于调试监测线程调度使用 */
        motor_dt = dwt_get_time_ms() - motor_start;
        if (motor_dt > 3){
            printf("Motor Task is being DELAY! dt = [%f]\n", &motor_dt);
        }
        vTaskDelay(1);
    }
    /* USER CODE END TransmissionTask_Entry */
}
