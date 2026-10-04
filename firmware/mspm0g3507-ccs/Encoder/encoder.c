#include "encoder.h"

#include "ti_msp_dl_config.h"

/*
 * 编码器计数说明。
 *
 * 本模块现在和 24 年 H 题源码保持同一种思路：
 * 1. GPIO 中断里累计编码器脉冲数。
 * 2. TIMER_ENCODER 每 50ms 读取一次累计值。
 * 3. 读取后立即清零，开始统计下一个 50ms。
 *
 * now_va：左电机最近 50ms 的编码器计数。
 * now_vb：右电机最近 50ms 的编码器计数。
 *
 * 这两个值不再换算成 motorA()/motorB() 的 0~1000，也不是 RPM 或 m/s。
 * 它们就是“最近 50ms 编码器计数”，数值大小表示速度快慢，正负号表示方向。
 */

/*
 * 左右编码器方向修正。
 *
 * 如果你发现轮子正转时 now_va 或 now_vb 是负数，
 * 就把对应的方向修正从 1 改成 -1。
 */
#define ENCODER_A_DIR (1)
#define ENCODER_B_DIR (-1)

/* 左电机 A 最近 50ms 的编码器计数。 */
volatile int now_va = 0;

/* 右电机 B 最近 50ms 的编码器计数。 */
volatile int now_vb = 0;

/* 左轮编码器实时累计计数，由 GPIO 中断持续加减。 */
static volatile int32_t g_encoder_count_a = 0;

/* 右轮编码器实时累计计数，由 GPIO 中断持续加减。 */
static volatile int32_t g_encoder_count_b = 0;

/* 左轮最近一个 50ms 周期内的原始计数。 */
static volatile int32_t g_encoder_raw_a = 0;

/* 右轮最近一个 50ms 周期内的原始计数。 */
static volatile int32_t g_encoder_raw_b = 0;

/* 小车累计行驶距离计数：单位是编码器计数，不是 cm */
static volatile uint32_t g_encoder_distance_count = 0U;

/* 50ms速度采样更新标志：TIMER_ENCODER更新一次速度后置1 */
static volatile uint8_t g_encoder_speed_updated = 0U;

void Encoder_Init(void)
{
    Encoder_Reset();

    /*
     * GPIO_ENCODER_INT_IRQN 对应 GPIOB 中断。
     * SysConfig 负责配置 E1A/E1B/E2A/E2B 为上升沿中断。
     * 这里负责清掉可能残留的中断标志，然后使能 NVIC。
     */
    DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT,
        GPIO_ENCODER_PIN_E1A_PIN | GPIO_ENCODER_PIN_E1B_PIN |
        GPIO_ENCODER_PIN_E2A_PIN | GPIO_ENCODER_PIN_E2B_PIN);
    NVIC_ClearPendingIRQ(GPIO_ENCODER_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_INT_IRQN);

    /*
     * TIMER_ENCODER 是独立的编码器速度采样定时器。
     * 它使用 TIMG7，不使用陀螺仪的 TIMER_0/TIMG6，因此不会和姿态更新中断冲突。
     */
    NVIC_ClearPendingIRQ(TIMER_ENCODER_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_ENCODER_INST_INT_IRQN);
    DL_Timer_startCounter(TIMER_ENCODER_INST);
}

void Encoder_Reset(void)
{
    g_encoder_count_a = 0;
    g_encoder_count_b = 0;
    g_encoder_raw_a = 0;
    g_encoder_raw_b = 0;
    g_encoder_distance_count = 0U;
    g_encoder_speed_updated = 0U;
    now_va = 0;
    now_vb = 0;
}

int Encoder_GetNowVA(void)
{
    return now_va;
}

int Encoder_GetNowVB(void)
{
    return now_vb;
}

int32_t Encoder_GetRawCountA(void)
{
    return g_encoder_raw_a;
}

int32_t Encoder_GetRawCountB(void)
{
    return g_encoder_raw_b;
}

uint32_t Encoder_GetDistanceCount(void)
{
    return g_encoder_distance_count;
}

void Encoder_ResetDistanceCount(void)
{
    g_encoder_distance_count = 0U;
}

uint8_t Encoder_HasNewSpeed(void)
{
    return g_encoder_speed_updated;
}

void Encoder_ClearNewSpeedFlag(void)
{
    g_encoder_speed_updated = 0U;
}

void Encoder_GPIO_IRQHandler(void)
{
    uint32_t status;

    status = DL_GPIO_getEnabledInterruptStatus(GPIO_ENCODER_PORT,
        GPIO_ENCODER_PIN_E1A_PIN | GPIO_ENCODER_PIN_E1B_PIN |
        GPIO_ENCODER_PIN_E2A_PIN | GPIO_ENCODER_PIN_E2B_PIN);

    /*
     * 左电机 A：E1A/E1B 两相编码器。
     *
     * 参考 24 年 H 题源码：
     * 任意一相出现上升沿时，读取另一相当前电平，用它判断方向。
     */
    if ((status & GPIO_ENCODER_PIN_E1A_PIN) != 0U) {
        if (DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_E1B_PIN) == 0U) {
            g_encoder_count_a -= ENCODER_A_DIR;
        } else {
            g_encoder_count_a += ENCODER_A_DIR;
        }
    } else if ((status & GPIO_ENCODER_PIN_E1B_PIN) != 0U) {
        if (DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_E1A_PIN) == 0U) {
            g_encoder_count_a += ENCODER_A_DIR;
        } else {
            g_encoder_count_a -= ENCODER_A_DIR;
        }
    }

    /*
     * 右电机 B：E2A/E2B 两相编码器。
     *
     * 逻辑与左轮一致。如果实际方向反了，优先改 ENCODER_B_DIR。
     */
    if ((status & GPIO_ENCODER_PIN_E2A_PIN) != 0U) {
        if (DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_E2B_PIN) == 0U) {
            g_encoder_count_b -= ENCODER_B_DIR;
        } else {
            g_encoder_count_b += ENCODER_B_DIR;
        }
    } else if ((status & GPIO_ENCODER_PIN_E2B_PIN) != 0U) {
        if (DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_E2A_PIN) == 0U) {
            g_encoder_count_b += ENCODER_B_DIR;
        } else {
            g_encoder_count_b -= ENCODER_B_DIR;
        }
    }

    DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT,
        GPIO_ENCODER_PIN_E1A_PIN | GPIO_ENCODER_PIN_E1B_PIN |
        GPIO_ENCODER_PIN_E2A_PIN | GPIO_ENCODER_PIN_E2B_PIN);
}

void Encoder_UpdateSpeed(void)
{
    int32_t count_a;
    int32_t count_b;
    uint32_t abs_count_a;
    uint32_t abs_count_b;

    /*
     * 取出最近 50ms 的累计脉冲数，并立即清零。
     * 后续 GPIO 中断会继续为下一个 50ms 周期累计。
     */
    count_a = g_encoder_count_a;
    count_b = g_encoder_count_b;
    g_encoder_count_a = 0;
    g_encoder_count_b = 0;

    g_encoder_raw_a = count_a;
    g_encoder_raw_b = count_b;

    /*
     * 累计距离：左右轮50ms计数取绝对值后求平均。
     * 只表示走过多少编码器计数，不表示方向。
     */
    abs_count_a = (count_a >= 0) ? (uint32_t) count_a : (uint32_t) (-count_a);
    abs_count_b = (count_b >= 0) ? (uint32_t) count_b : (uint32_t) (-count_b);
    g_encoder_distance_count += (abs_count_a + abs_count_b) / 2U;

    /*
     * 直接输出最近 50ms 的编码器计数。
     * 这里不再换算成 motorA()/motorB() 的 0~1000。
     */
    now_va = (int) count_a;
    now_vb = (int) count_b;
    g_encoder_speed_updated = 1U;
}

void GROUP1_IRQHandler(void)
{
    Encoder_GPIO_IRQHandler();
}

void TIMER_ENCODER_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_ENCODER_INST)) {
        case DL_TIMER_IIDX_ZERO:
            Encoder_UpdateSpeed();
            break;

        default:
            break;
    }
}
