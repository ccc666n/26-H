#ifndef STEPPER_H_
#define STEPPER_H_

#include <stdint.h>

#include "Emm_V5.h"

/* 初始化步进电机：使能电机并设置快速位置模式参数 */
void Stepper_Init(void);

/* 使能或关闭步进电机，enable=1使能，enable=0关闭 */
void Stepper_Enable(uint8_t enable);

/* 将当前位置设置为0点 */
void Stepper_SetZero(void);

/* 按脉冲位置控制步进电机 */
void Stepper_SetPulse(int32_t pulse);

/* 按角度控制摆杆位置，单位：度 */
void Stepper_SetAngle(float angle_deg);

/* 立即停止步进电机 */
void Stepper_Stop(void);

#endif /* STEPPER_H_ */
