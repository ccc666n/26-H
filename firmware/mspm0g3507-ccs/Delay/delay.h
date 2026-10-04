#ifndef DELAY_H_
#define DELAY_H_

#include <stdint.h>

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void delay_s(uint32_t s);

void delay_1us(uint32_t us);
void delay_1ms(uint32_t ms);

#endif /* DELAY_H_ */
