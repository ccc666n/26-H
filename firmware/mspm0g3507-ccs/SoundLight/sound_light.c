#include "sound_light.h"

#include "ti_msp_dl_config.h"

#define SOUND_LIGHT_500MS_TICKS (25U)

/* 声光提示运行标志：1 表示蜂鸣器和红灯正在工作 */
static volatile uint8_t g_sound_light_active = 0U;

/* 20ms 计数器：25 次就是 500ms */
static volatile uint8_t g_sound_light_count = 0U;

/* 行驶计时，单位为20ms */
static volatile uint32_t g_time_20ms_count = 0U;

/* 计时运行标志：1运行，0停止 */
static volatile uint8_t g_time_running = 0U;

/* 独立系统时间，单位ms，不受行驶计时启停影响 */
static volatile uint32_t g_sys_time_ms = 0U;

void SoundLight_Init(void)
{
    g_sound_light_active = 0U;
    g_sound_light_count = 0U;
    g_time_20ms_count = 0U;
    g_time_running = 0U;
    g_sys_time_ms = 0U;

    DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_BEEP_PIN);
    DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_RGB_R_PIN);

    SysTick_Config(CPUCLK_FREQ / 1000U);

    NVIC_ClearPendingIRQ(TIMER_SOUND_LIGHT_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_SOUND_LIGHT_INST_INT_IRQN);
    DL_Timer_startCounter(TIMER_SOUND_LIGHT_INST);
}

void SoundLight_Start(void)
{
    g_sound_light_count = 0U;
    g_sound_light_active = 1U;

    // DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_BEEP_PIN);//蜂鸣器开关
    DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_RGB_R_PIN);
}

void SoundLight_Update20ms(void)
{
    if (g_time_running != 0U) {
        g_time_20ms_count++;
    }

    if (g_sound_light_active == 0U) {
        return;
    }

    g_sound_light_count++;

    if (g_sound_light_count >= SOUND_LIGHT_500MS_TICKS) {
        DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_BEEP_PIN);
        DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_RGB_R_PIN);

        g_sound_light_count = 0U;
        g_sound_light_active = 0U;
    }
}

uint8_t SoundLight_IsActive(void)
{
    return g_sound_light_active;
}

void SoundLight_TimeStart(void)
{
    g_time_20ms_count = 0U;
    g_time_running = 1U;
}

void SoundLight_TimeStop(void)
{
    g_time_running = 0U;
}

void SoundLight_TimeReset(void)
{
    g_time_20ms_count = 0U;
}

uint32_t SoundLight_GetTimeMs(void)
{
    return g_time_20ms_count * 20U;
}

uint32_t SoundLight_GetSysMs(void)
{
    return g_sys_time_ms;
}

void SysTick_Handler(void)
{
    g_sys_time_ms++;
}

void TIMER_SOUND_LIGHT_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_SOUND_LIGHT_INST)) {
        case DL_TIMER_IIDX_ZERO:
            SoundLight_Update20ms();
            break;

        default:
            break;
    }
}
