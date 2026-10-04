#include "analog_gray.h"

#include "ADC.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "ti_msp_dl_config.h"

/*
 * 黑白校准值先沿用示例工程。
 * 后续如果实测通道黑白值变化，优先改这里。
 */
// static unsigned short g_gray_white[8] = {1036, 1262, 1242, 1732, 1450, 1323, 1583, 1133};
// static unsigned short g_gray_black[8] = {87, 90, 94, 93, 93, 94, 95, 96};
static unsigned short g_gray_white[8] = {3056, 3099, 3096, 3067, 3056, 3031, 3030, 3088};
static unsigned short g_gray_black[8] = {1300, 700, 800, 800, 500, 180, 250, 600};

static No_MCU_Sensor g_gray_sensor;
static uint8_t g_gray_digital = 0U;

void AnalogGray_Init(void)
{
    No_MCU_Ganv_Sensor_Init(&g_gray_sensor, g_gray_white, g_gray_black);

    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,
        (uint32_t) &ADC12_0_INST->ULLMEM.MEMRES[0]);
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &ADC_VALUE[0]);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_ADC12_startConversion(ADC12_0_INST);
}

void AnalogGray_Update(void)
{
    No_Mcu_Ganv_Sensor_Task_Without_tick(&g_gray_sensor);
    g_gray_digital = Get_Digtal_For_User(&g_gray_sensor);
}

uint8_t AnalogGray_GetDigital(void)
{
    return g_gray_digital;
}

uint8_t AnalogGray_GetAnalog(uint16_t *result)
{
    if (result == 0) {
        return 0U;
    }

    return Get_Anolog_Value(&g_gray_sensor, result);
}

uint8_t AnalogGray_GetNormalize(uint16_t *result)
{
    if (result == 0) {
        return 0U;
    }

    return Get_Normalize_For_User(&g_gray_sensor, result);
}
