#include "ti_msp_dl_config.h"

#include <stdio.h>

#include "OLED.h"
#include "stepper.h"
#include "Camera/camera.h"
#include "delay.h"
#include "get_yaw.h"
#include "IMU.h"
#include "encoder.h"
#include "motor.h"
#include "sound_light.h"
#include "track.h"

#define INTEG_LIMIT 1200//积分限幅
#define PWM_MAX     1000

// printf重定向到UART_1，PA14用于串口调试输出
int fputc(int ch, FILE *f)
{
    DL_UART_Main_transmitDataBlocking(UART_1_INST, (uint8_t)ch);
    return ch;
}

/* 角度环 PID 参数，track1() 会通过 extern 调用这个函数 */
float kp = 5.0f;
float ki = 0.0f;
float kd = 0.0f;
float err = 0.0f;
float err_last = 0.0f;
float err_integral = 0.0f;
//角度环PID
float PID(float target_yaw, float now_yaw)
{
    err = target_yaw - now_yaw;
    if (err > 180.0f) {
        err -= 360.0f;
    }
    if (err < -180.0f) {
        err += 360.0f;
    }//关掉了我的雷霆大转弯功能

    err_integral += err;
    if (err_integral > INTEG_LIMIT) {
        err_integral = INTEG_LIMIT;
    }
    if (err_integral < -INTEG_LIMIT) {
        err_integral = -INTEG_LIMIT;
    }

    float err_diff = err - err_last;

    float p_out = kp * err;
    float i_out = ki * err_integral;
    float d_out = kd * err_diff;
    float pid_out = p_out + i_out + d_out;

    if (pid_out > PWM_MAX) {
        pid_out = PWM_MAX;
    }
    if (pid_out < -PWM_MAX) {
        pid_out = -PWM_MAX;
    }

    err_last = err;

    return pid_out;
}

/* 转弯角度环 PID 参数，track1_test1() 会通过 extern 调用这个函数 */
float kp2 = 3.0f;
float ki2 = 0.0f;
float kd2 = 0.01f;
float err2 = 0.0f;
float err_last2 = 0.0f;
float err_integral2 = 0.0f;
//转弯角度环PID
float PID2(float target_yaw, float now_yaw)
{
    err2 = target_yaw - now_yaw;
    if (err2 > 180.0f) {
        err2 -= 360.0f;
    }
    if (err2 < -180.0f) {
        err2 += 360.0f;
    }//关掉了我的雷霆大转弯功能

    err_integral2 += err2;
    if (err_integral2 > INTEG_LIMIT) {
        err_integral2 = INTEG_LIMIT;
    }
    if (err_integral2 < -INTEG_LIMIT) {
        err_integral2 = -INTEG_LIMIT;
    }

    float err_diff = err2 - err_last2;

    float p_out = kp2 * err2;
    float i_out = ki2 * err_integral2;
    float d_out = kd2 * err_diff;
    float pid_out = p_out + i_out + d_out;

    if (pid_out > PWM_MAX) {
        pid_out = PWM_MAX;
    }
    if (pid_out < -PWM_MAX) {
        pid_out = -PWM_MAX;
    }

    err_last2 = err2;

    return pid_out;
}

int main(void)
{
    SYSCFG_DL_init();

    motor_init();
    track_init();
    GetYaw_Init();
    Encoder_Init();
    SoundLight_Init();
    Camera_Init();
    Stepper_Init();
    delay_ms(200);
    Stepper_SetZero();
    OLED_Init();
    delay_ms(20);
    GetYaw_Reset();
    // float imu_data[7];

    int a = 0;
    uint8_t mode = 0;//初始状态
    uint8_t running = 0;

    float target_yaw = 0.0;//使小车走直线的目标角
    float now_yaw;
    float pid_out;

    while (1) {
        Camera_Task();
        now_yaw = GetYaw_Value();

        //编码器计数总值--距离
        uint32_t distance_count = Encoder_GetDistanceCount();

        // 编码器50ms计数--电机速度
        now_va = Encoder_GetNowVA();
        now_vb = Encoder_GetNowVB();

        // K1：菜单选择，只在未运行时切换 mode
        if ((running == 0U) && (!DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_K1_PIN))) {
            delay_ms(20);
            if (!DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_K1_PIN)) {
                mode = (uint8_t)((mode + 1U) % 5U);
                while (!DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_K1_PIN)) {
                    delay_ms(10);
                }
            }
        }

        // K2：确认启动，mode=0 时不执行任务
        if ((running == 0U) && (mode != 0U) &&
            (!DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_K2_PIN))) {
            delay_ms(20);
            if (!DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_K2_PIN)) {
                motors_stop();
                delay_ms(1000);
                SoundLight_TimeStart();
                GetYaw_Reset();
                track_reset(mode);
                running = 1U;

                while (!DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_K2_PIN)) {
                    delay_ms(10);
                }
            }
        }
       
        // K3：调试启动，直接进入调试状态6
        if ((running == 0U) && (!DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_K3_PIN))) {
            delay_ms(20);
            if (!DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_K3_PIN)) {
                motors_stop();
                delay_ms(500);
                SoundLight_TimeStart();
                GetYaw_Reset();
                mode = 6U;
                track_reset(mode);
                running = 1U;
                while (!DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_K3_PIN)) {
                    delay_ms(10);
                }
            }
        }

        if (running == 0U) {
            motors_stop();
        } 
        else {//running==1
            switch (mode) {
                case 1:
                    // track1();
                    t2();
                    break;

                case 2:
                    t3();
                    break;

                case 3:
                    t4_weizhi();
                    break;

                case 4:
                    t6();
                    break;

                case 6:
                    t3_yuce();   // 调试函数，这里换成你要调试的函数
                    break;    

                default:
                    motors_stop();
                    running = 0U;
                    break;
            }
        }
        // OLED_ShowNum(2, 1, a, 3);
        // OLED_ShowSignedFloat1(1, 1, now_yaw);

        // OLED_ShowSignedFloat1(3, 1, (float)now_va);
        // OLED_ShowSignedFloat1(4, 1, (float)now_vb);
        // OLED_ShowNum(3, 1, now_va, 8);
        // OLED_ShowNum(4, 1, now_vb, 8);

        OLED_ShowString(2, 6, "M:");
        OLED_ShowNum(2, 8, mode, 1);
        OLED_ShowString(2, 11, "R:");
        OLED_ShowNum(2, 13, running, 1);
        
        // OLED_ShowNum(5,1 , distance_count, 12);
        // //灰度8路的值
        // OLED_ShowNum(6, 1, D1, 1);
        // OLED_ShowNum(6, 2, D2, 1);
        // OLED_ShowNum(6, 3, D3, 1);
        // OLED_ShowNum(6, 4, D4, 1);
        // OLED_ShowNum(6, 5, D5, 1);
        // OLED_ShowNum(6, 6, D6, 1);
        // OLED_ShowNum(6, 7, D7, 1);
        // OLED_ShowNum(6, 8, D8, 1);
        
        uint32_t t = SoundLight_GetTimeMs();
        OLED_ShowNum(7, 1, t / 1000, 3);   // 显示秒

        OLED_ShowSignedFloat1(1, 1, Camera_GetBallError());
        // OLED_ShowSignedFloat1(1, 9, Camera_GetBallSpeed());

        
        // IMU_TT_getgyro(imu_data);
        // OLED_ShowSignedFloat1(2, 1, -imu_data[1]);

        // delay_ms(20);
        a++;
    }
}
