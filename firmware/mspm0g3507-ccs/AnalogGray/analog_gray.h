#ifndef ANALOG_GRAY_H_
#define ANALOG_GRAY_H_

#include <stdint.h>

/* 初始化模拟量灰度模块：传感器参数 + DMA + ADC */
void AnalogGray_Init(void);

/* 更新一次8路模拟灰度数据，建议主循环周期性调用 */
void AnalogGray_Update(void);

/* 获取8位数字量：bit0~bit7对应灰度通道1~8，1为白场，0为黑线 */
uint8_t AnalogGray_GetDigital(void);

/* 获取8路原始ADC值，返回1表示数据有效 */
uint8_t AnalogGray_GetAnalog(uint16_t *result);

/* 获取8路归一化值，返回1表示数据有效 */
uint8_t AnalogGray_GetNormalize(uint16_t *result);

#endif /* ANALOG_GRAY_H_ */
