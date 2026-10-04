#ifndef OLED_H_
#define OLED_H_

#include <stdint.h>

/*
 * SSD1306 I2C OLED, software I2C.
 * Wiring:
 *   SCL -> PA8
 *   SDA -> PA9
 */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t row, uint8_t column, char ch);
void OLED_ShowString(uint8_t row, uint8_t column, const char *string);
void OLED_ShowNum(uint8_t row, uint8_t column, uint32_t number, uint8_t length);
void OLED_ShowSignedFloat1(uint8_t row, uint8_t column, float value);

#endif /* OLED_H_ */
