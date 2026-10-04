#include "get_yaw.h"
#include "IMU.h"
#include "ti_msp_dl_config.h"

static volatile float g_ypr[3] = {0.0f, 0.0f, 0.0f};
static volatile float g_yaw_zero = 0.0f;
static volatile uint8_t g_yaw_ready = 0;

/* 把角度限制在 -180 到 180 度之间 */
static float NormalizeAngle180(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }

    while (angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}

/* 初始化 IMU，并打开 TIMER_0 中断 */
void GetYaw_Init(void)
{
    IMU_init();

    g_ypr[0] = 0.0f;
    g_ypr[1] = 0.0f;
    g_ypr[2] = 0.0f;
    g_yaw_zero = 0.0f;
    g_yaw_ready = 0;

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

/* 更新 yaw、pitch、roll，建议 20ms 调用一次 */
void GetYaw_Update(void)
{
    float temp_ypr[3];

    IMU_getYawPitchRoll(temp_ypr);

    g_ypr[0] = temp_ypr[0];
    g_ypr[1] = temp_ypr[1];
    g_ypr[2] = temp_ypr[2];

    g_yaw_ready = 1;
}

/* 获取相对 yaw，单位：度 */
float GetYaw_Value(void)
{
    float yaw;

    yaw = g_ypr[0] - g_yaw_zero;
    yaw = NormalizeAngle180(yaw);

    return yaw;
}

/* 把当前方向设为 0 度 */
void GetYaw_Reset(void)
{
    g_yaw_zero = g_ypr[0];
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            GetYaw_Update();
            break;

        default:
            break;
    }
}
