#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

/*
 * 编码器计数输出变量。
 *
 * now_va：左电机 A 最近 50ms 的编码器计数。
 * now_vb：右电机 B 最近 50ms 的编码器计数。
 *
 * 正数表示当前轮子正向转动，负数表示当前轮子反向转动。
 * 这里的正反方向与 motorA(va)、motorB(vb) 的正负号保持同一种理解。
 *
 * 注意：
 * 这两个变量不是 RPM，不是 m/s，也不是 motorA()/motorB() 的 0~1000。
 * 它们就是最近 50ms 的编码器脉冲计数，和 24 年 H 题源码里的
 * gEncoderVal_left / gEncoderVal_right 是同一种含义。
 */
extern volatile int now_va;
extern volatile int now_vb;

/*
 * 初始化编码器模块。
 * 1. 清零左右编码器累计脉冲数。
 */
void Encoder_Init(void);

/*
 * 清零编码器数据。
 *
 * 功能：
 * 清空左右轮当前累计脉冲数、最近一次 50ms 采样脉冲数。
 * 如果你想在某个动作开始前重新统计速度，可以调用这个函数。
 */
void Encoder_Reset(void);

/*
 * 获取左电机 A 最近 50ms 的编码器计数。
 *
 * 返回值：
 * now_va，单位为最近 50ms 的带方向编码器计数。
 */
int Encoder_GetNowVA(void);

/*
 * 获取右电机 B 最近 50ms 的编码器计数。
 *
 * 返回值：
 * now_vb，单位为最近 50ms 的带方向编码器计数。
 */
int Encoder_GetNowVB(void);

/*
 * 获取左电机 A 最近 50ms 内的原始编码器计数。
 *
 * 这个值和 now_va 相同，保留这个接口是为了调试时名字更直观。
 */
int32_t Encoder_GetRawCountA(void);

/*
 * 获取右电机 B 最近 50ms 内的原始编码器计数。
 *
 * 这个值和 now_vb 相同，保留这个接口是为了调试时名字更直观。
 */
int32_t Encoder_GetRawCountB(void);

/*
 * 获取累计行驶距离计数。
 *
 * 返回值：
 * 从上次清零开始，左右轮编码器绝对计数平均后的累计值。
 * 单位是编码器计数，不是 cm，也不是 m。
 */
uint32_t Encoder_GetDistanceCount(void);

/*
 * 只清零累计距离计数，不影响 now_va / now_vb。
 */
void Encoder_ResetDistanceCount(void);

/*
 * 判断最近50ms编码器速度是否已经更新。
 * 返回1表示 now_va/now_vb 有新一轮50ms计数，可以运行一次速度PID。
 */
uint8_t Encoder_HasNewSpeed(void);

/*
 * 清除50ms速度更新标志。
 * 速度PID读完 now_va/now_vb 后调用，避免同一组编码器数据被重复积分。
 */
void Encoder_ClearNewSpeedFlag(void);

/*
 * GPIO 中断处理函数。
 *
 * 功能：
 * 根据编码器 A/B 相的上升沿和另一个相位的电平，判断轮子转动方向，
 * 并对左右轮脉冲计数进行加减。
 *
 * 注意：
 * 这个函数由 GROUP1_IRQHandler() 调用，用户一般不需要在 main() 里调用。
 */
void Encoder_GPIO_IRQHandler(void);

/*
 * 速度采样更新函数。
 *
 * 功能：
 * 每 50ms 把累计脉冲数取出来，直接保存到 now_va、now_vb，
 * 然后清零累计脉冲数。
 *
 * 注意：
 * 这个函数由 TIMER_ENCODER_INST_IRQHandler() 调用。
 */
void Encoder_UpdateSpeed(void);

#endif /* ENCODER_H_ */
