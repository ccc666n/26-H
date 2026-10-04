#include "motor.h"

#include <stdint.h>

#include "delay.h"
#include "ti_msp_dl_config.h"

#define MOTOR_PWM_PERIOD_COUNTS (2500U)
#define MOTOR_MAX_SPEED         (1000)

static int clamp_speed_abs(int speed)
{
    if (speed < 0) {
        speed = -speed;
    }

    if (speed > MOTOR_MAX_SPEED) {
        speed = MOTOR_MAX_SPEED;
    }

    return speed;
}

static void set_pwm_duty(uint8_t channel, int speed)
{
    uint32_t compareValue;
    uint32_t speedAbs;

    speedAbs = (uint32_t) clamp_speed_abs(speed);
    compareValue =
        MOTOR_PWM_PERIOD_COUNTS -
        ((MOTOR_PWM_PERIOD_COUNTS * speedAbs) / MOTOR_MAX_SPEED);

    if (channel == 0U) {
        DL_Timer_setCaptureCompareValue(
            PWM_0_INST, compareValue, DL_TIMER_CC_0_INDEX);
    } else {
        DL_Timer_setCaptureCompareValue(
            PWM_0_INST, compareValue, DL_TIMER_CC_1_INDEX);
    }
}

void motor_init(void)
{
    DL_Timer_startCounter(PWM_0_INST);
    motors_sleep();
}

void motorA(int va)
{
    DL_GPIO_setPins(GPIO_STBY_PORT, GPIO_STBY_PIN_STBY_PIN);

    if (va > 0) {
        DL_GPIO_setPins(GPIO_IN_PORT, GPIO_IN_PIN_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_IN_PORT, GPIO_IN_PIN_AIN2_PIN);
    } else if (va < 0) {
        DL_GPIO_clearPins(GPIO_IN_PORT, GPIO_IN_PIN_AIN1_PIN);
        DL_GPIO_setPins(GPIO_IN_PORT, GPIO_IN_PIN_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(
            GPIO_IN_PORT, GPIO_IN_PIN_AIN1_PIN | GPIO_IN_PIN_AIN2_PIN);
    }

    set_pwm_duty(1U, va);
}

void motorB(int vb)
{
    DL_GPIO_setPins(GPIO_STBY_PORT, GPIO_STBY_PIN_STBY_PIN);

    if (vb > 0) {
        DL_GPIO_setPins(GPIO_IN_PORT, GPIO_IN_PIN_BIN1_PIN);
        DL_GPIO_clearPins(GPIO_IN_PORT, GPIO_IN_PIN_BIN2_PIN);
    } else if (vb < 0) {
        DL_GPIO_clearPins(GPIO_IN_PORT, GPIO_IN_PIN_BIN1_PIN);
        DL_GPIO_setPins(GPIO_IN_PORT, GPIO_IN_PIN_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(
            GPIO_IN_PORT, GPIO_IN_PIN_BIN1_PIN | GPIO_IN_PIN_BIN2_PIN);
    }

    set_pwm_duty(0U, vb);
}

void motors_stop(void)
{
    motorA(0);
    motorB(0);
}

void motors_brake(void)
{
    DL_GPIO_setPins(GPIO_STBY_PORT, GPIO_STBY_PIN_STBY_PIN);

    /*
     * TB6612 类驱动短刹车：方向脚 1/1，并给 PWM 输出。
     * 这里只在状态切换或最终停车时短时间调用，不要放在循迹循环里频繁调用。
     */
    DL_GPIO_setPins(GPIO_IN_PORT,
        GPIO_IN_PIN_AIN1_PIN | GPIO_IN_PIN_AIN2_PIN |
        GPIO_IN_PIN_BIN1_PIN | GPIO_IN_PIN_BIN2_PIN);
    set_pwm_duty(1U, MOTOR_MAX_SPEED);
    set_pwm_duty(0U, MOTOR_MAX_SPEED);
    delay_ms(50);

    motors_stop();
}

void motors_brake_ms(int ms)
{
    DL_GPIO_setPins(GPIO_STBY_PORT, GPIO_STBY_PIN_STBY_PIN);

    /*
     * TB6612 类驱动短刹车：方向脚 1/1，并给 PWM 输出。
     * 这里只在状态切换或最终停车时短时间调用，不要放在循迹循环里频繁调用。
     */
    DL_GPIO_setPins(GPIO_IN_PORT,
        GPIO_IN_PIN_AIN1_PIN | GPIO_IN_PIN_AIN2_PIN |
        GPIO_IN_PIN_BIN1_PIN | GPIO_IN_PIN_BIN2_PIN);
    set_pwm_duty(1U, MOTOR_MAX_SPEED);
    set_pwm_duty(0U, MOTOR_MAX_SPEED);
    delay_ms(ms);

    motors_stop();
}

int motor_soft_speed(int target, int start, int step, int reset)
{
    static int speed = 0;

    if (reset != 0U) {
        speed = start;
    }

    if (step <= 0) {
        step = 1;
    }

    if (speed < target) {
        speed += step;
        if (speed > target) {
            speed = target;
        }
    } else if (speed > target) {
        speed -= step;
        if (speed < target) {
            speed = target;
        }
    }

    return speed;
}

void motors_sleep(void)
{
    motors_stop();
    DL_GPIO_clearPins(GPIO_STBY_PORT, GPIO_STBY_PIN_STBY_PIN);
}
