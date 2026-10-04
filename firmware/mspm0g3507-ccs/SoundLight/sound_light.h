#ifndef SOUND_LIGHT_H_
#define SOUND_LIGHT_H_

#include <stdint.h>

/* 初始化声光提示模块，并启动 20ms 专用定时器 */
void SoundLight_Init(void);

/* 开始 0.5s 蜂鸣器 + 红灯提示，非阻塞 */
void SoundLight_Start(void);

/* 每 20ms 调用一次，用于计时并自动关闭声光 */
void SoundLight_Update20ms(void);

/* 返回声光提示是否正在运行：1 表示正在提示，0 表示已经关闭 */
uint8_t SoundLight_IsActive(void);

/* 开始行驶计时，并清零 */
void SoundLight_TimeStart(void);

/* 停止行驶计时 */
void SoundLight_TimeStop(void);

/* 清零行驶计时 */
void SoundLight_TimeReset(void);

/* 获取行驶时间，单位ms */
uint32_t SoundLight_GetTimeMs(void);

/* 获取独立系统时间，单位ms，不受行驶计时启停影响 */
uint32_t SoundLight_GetSysMs(void);

#endif /* SOUND_LIGHT_H_ */
