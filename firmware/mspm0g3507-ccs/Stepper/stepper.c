#include "stepper.h"

#include "delay.h"
#include "ti_msp_dl_config.h"

//决定快速位置的速度和加速度
void Stepper_Init(void)
{
    /* 步进电机常用参数，放在初始化附近方便现场修改 */
    uint8_t motor_addr = 1U;
    uint16_t qpos_speed_rpm = 4000U;//速度：0~5000
    uint8_t qpos_acc = 200U;//加速度：0~255
    uint8_t qpos_abs_mode = 1U;

    fifo_initQueue();
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    Emm_V5_En_Control(motor_addr, true, false);
    delay_ms(200);
    Emm_V5_Set_QPos_Params(motor_addr, qpos_speed_rpm, qpos_acc, qpos_abs_mode, false);
}

void Stepper_Enable(uint8_t enable)
{
    uint8_t motor_addr = 1U;

    Emm_V5_En_Control(motor_addr, (enable != 0U) ? true : false, false);
}

void Stepper_SetZero(void)
{
    uint8_t motor_addr = 1U;

    Emm_V5_Reset_CurPos_To_Zero(motor_addr);
}

void Stepper_SetPulse(int32_t pulse)
{
    uint8_t motor_addr = 1U;

    Emm_V5_QPos_Control(motor_addr, pulse);
}

void Stepper_SetAngle(float angle_deg)
{
    /* 默认3200脉冲为一圈，角度转脉冲后走快速位置模式 */
    int32_t pulse = (int32_t)(angle_deg * 3200.0f / 360.0f);

    Stepper_SetPulse(pulse);
}

void Stepper_Stop(void)
{
    uint8_t motor_addr = 1U;

    Emm_V5_Stop_Now(motor_addr, false);
}
