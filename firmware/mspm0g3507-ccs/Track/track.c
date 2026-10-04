#include "track.h"
#include <stdio.h>
#include "camera.h"
#include "analog_gray.h"
#include "encoder.h"
#include "get_yaw.h"
#include "IMU.h"
#include "motor.h"
#include "sound_light.h"
#include "stepper.h"
// #include "PID/PID.h"
#include "ti_msp_dl_config.h"

#define TRACK_BLACK_LEVEL (0U)//检测到黑为0
#define TRACK_WHITE_LEVEL (1U)//检测到白为1
#define TRACK_BASE_SPEED  (200)
#define TRACK_MAX_SPEED   (1000)
#define TRACK_ENCODER_TARGET_DIVISOR (6)
#define TRACK2_YAW_NEAR_DEG (20.0f)//角度误差判断状态是否切换
#define TRACK_PID_KP        (0.7f)//灰度位置误差比例系数，反应慢就适当加大
#define TRACK_PID_KD        (0.2f)//灰度位置误差微分系数，左右晃就适当加大或减小kp
#define TRACK_PID_KI        (0.0f)//灰度寻迹暂时不用积分，先保持0
#define TRACK_PID_BIAS_MAX  (500)//灰度PID最大差速，弯道甩得厉害就减小
#define TRACK_PID_LOST_ERROR (180.0f)//短暂全白但角度没到时，用上次偏差方向轻微找线
#define TRACK3_AC_TARGET_YAW (38.66f)//第3问 A->C 直线目标角，标准38.66
#define TRACK3_BD_TARGET_YAW (141.34f)//第3问 B->D 直线目标角，标准141.34
#define TRACK3_B_EXIT_YAW    (141.34f)//C->B 圆弧到 B 后的出线角，后续可实测修改
#define TRACK3_A_EXIT_YAW    (0.0f)//D->A 圆弧回到 A 的出线角，后续可实测修改
#define TRACK3_YAW_NEAR_DEG  (20.0f)//第3问圆弧出点 yaw 允许误差
// #define TRACK_LOST_BIAS   (200)   //涓㈢嚎鍚?鍥炵嚎鐨勯€熷害

extern float PID(float target_yaw, float now_yaw);
extern float PID2(float target_yaw, float now_yaw);

// 串口调试发送单个字符，使用UART_1的PA14输出
static void debug_uart_print_char(char ch)
{
    DL_UART_Main_transmitDataBlocking(UART_1_INST, (uint8_t)ch);
}

// 串口调试发送字符串，避免依赖printf格式化
static void debug_uart_print_string(const char *str)
{
    while ((str != 0) && (*str != '\0')) {
        debug_uart_print_char(*str);
        str++;
    }
}

// 串口调试发送整数，避免minimal printf不显示数值
static void debug_uart_print_int(int32_t value)
{
    char num_buf[11];
    uint8_t index = 0U;
    uint32_t magnitude;

    if (value < 0) {
        debug_uart_print_char('-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }

    do {
        num_buf[index] = (char)('0' + (magnitude % 10U));
        index++;
        magnitude /= 10U;
    } while ((magnitude > 0U) && (index < (uint8_t)sizeof(num_buf)));

    while (index > 0U) {
        index--;
        debug_uart_print_char(num_buf[index]);
    }
}

static int g_last_bias = 0;//丢线处理要使用的参数

/* AB 娈电涓€娆¤繘鍏?track1() 鏃讹紝鐢ㄥ綋鍓?yaw 浣滀负鐩磋鐩爣瑙?*/
static uint8_t g_track1_first_check = 1U;

/* 涓婁竴杞?8 璺伆搴︽槸鍚﹀叏鐧斤紝鐢ㄦ潵妫€娴嬧€滃叏鐧?-> 鏈夐粦绾库€濈殑杈规部 */
static uint8_t g_track1_last_all_white = 1U;

/* AB 娈靛畬鎴愭爣蹇楋細鍒拌揪 B 鐐瑰悗缃?1锛屼箣鍚?track1() 鍙繚鎸佸仠杞?*/
static uint8_t g_track1_ab_done = 0U;

/* AB 娈电洿琛岀洰鏍?yaw 瑙?*/
static float g_track1_target_yaw = 0.0f;

typedef enum {
    TRACK2_AB_STRAIGHT = 0,
    TRACK2_BC_ARC,
    TRACK2_CD_STRAIGHT,
    TRACK2_DA_ARC,
    TRACK2_DONE
} track2_state_t;

/* track2 鐙珛鐘舵€佸彉閲忥紝涓嶅鐢?track1 鐨勫彉閲?*/
static track2_state_t g_track2_state = TRACK2_AB_STRAIGHT;
static uint8_t g_track2_first_check = 1U;
static uint8_t g_track2_last_all_white = 1U;

/* track2_weizhiPID 测试用状态变量：状态逻辑照 track2()，圆弧段改为编码器速度环 */
static track2_state_t g_track2_weizhipid_state = TRACK2_AB_STRAIGHT;
static uint8_t g_track2_weizhipid_first_check = 1U;
static uint8_t g_track2_weizhipid_last_all_white = 1U;
static float g_speed_pid_left_integral = 0.0f;
static float g_speed_pid_right_integral = 0.0f;
static float g_speed_pid_left_last_error = 0.0f;
static float g_speed_pid_right_last_error = 0.0f;
static int g_speed_pid_left_output = 0;
static int g_speed_pid_right_output = 0;

typedef enum {
    TRACK2_PID1_AB_STRAIGHT = 0,
    TRACK2_PID1_BC_ARC,
    TRACK2_PID1_CD_STRAIGHT,
    TRACK2_PID1_DA_ARC,
    TRACK2_PID1_DONE
} track2_pid1_state_t;

/* track2_PID1 是测试用灰度PID状态机，不影响原来的 track2() */
static track2_pid1_state_t g_track2_pid1_state = TRACK2_PID1_AB_STRAIGHT;
static uint8_t g_track2_pid1_first_check = 1U;
static uint8_t g_track2_pid1_last_all_white = 1U;
static float g_track2_pid1_last_error = 0.0f;//上一次灰度位置误差，用于D项和丢线保持

typedef enum {
    TRACK3_AC_STRAIGHT = 0,
    TRACK3_CB_ARC,
    TRACK3_BD_STRAIGHT,
    TRACK3_DA_ARC,
    TRACK3_DONE
} track3_state_t;

/* track3 独立状态变量，用于第3问 A->C->B->D->A */
static track3_state_t g_track3_state = TRACK3_AC_STRAIGHT;
static uint8_t g_track3_first_check = 1U;
static uint8_t g_track3_last_all_white = 1U;

typedef enum {
    TRACK4_AC_STRAIGHT = 0,
    TRACK4_CB_ARC,
    TRACK4_BD_STRAIGHT,
    TRACK4_DA_ARC,
    TRACK4_DONE
} track4_state_t;

/* track4 独立状态变量，用于第4问按第3问路线跑4圈 */
static track4_state_t g_track4_state = TRACK4_AC_STRAIGHT;
static uint8_t g_track4_first_check = 1U;
static uint8_t g_track4_last_all_white = 1U;
static uint8_t g_track4_lap_count = 0U;//只在回到A点时加1圈

//判断某一路是否为黑
static uint8_t is_black(uint8_t value)
{
    return value == TRACK_BLACK_LEVEL;
}
// 判断8路灰度是否全白：0是黑线，1是白场
static uint8_t is_all_white(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4,
    uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8)
{
    return ((d1 == TRACK_WHITE_LEVEL) && (d2 == TRACK_WHITE_LEVEL) &&
        (d3 == TRACK_WHITE_LEVEL) && (d4 == TRACK_WHITE_LEVEL) &&
        (d5 == TRACK_WHITE_LEVEL) && (d6 == TRACK_WHITE_LEVEL) &&
        (d7 == TRACK_WHITE_LEVEL) && (d8 == TRACK_WHITE_LEVEL)) ? 1U : 0U;
}

static uint8_t is_all_white2(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4,
    uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8)
{
    return ((d1 == TRACK_WHITE_LEVEL) && (d2 == TRACK_WHITE_LEVEL) &&
        (d3 == TRACK_WHITE_LEVEL)  &&
        (d5 == TRACK_WHITE_LEVEL) && (d6 == TRACK_WHITE_LEVEL) &&
        (d7 == TRACK_WHITE_LEVEL) && (d8 == TRACK_WHITE_LEVEL)) ? 1U : 0U;
}

void track_init(void)
{
    g_last_bias = 0;
    g_track1_first_check = 1U;
    g_track1_last_all_white = 1U;
    g_track1_ab_done = 0U;
    g_track1_target_yaw = 0.0f;
    g_track2_state = TRACK2_AB_STRAIGHT;
    g_track2_first_check = 1U;
    g_track2_last_all_white = 1U;
    g_track2_weizhipid_state = TRACK2_AB_STRAIGHT;
    g_track2_weizhipid_first_check = 1U;
    g_track2_weizhipid_last_all_white = 1U;
    g_speed_pid_left_integral = 0.0f;
    g_speed_pid_right_integral = 0.0f;
    g_speed_pid_left_last_error = 0.0f;
    g_speed_pid_right_last_error = 0.0f;
    g_speed_pid_left_output = 0;
    g_speed_pid_right_output = 0;
    g_track2_pid1_state = TRACK2_PID1_AB_STRAIGHT;
    g_track2_pid1_first_check = 1U;
    g_track2_pid1_last_all_white = 1U;
    g_track2_pid1_last_error = 0.0f;
    g_track3_state = TRACK3_AC_STRAIGHT;
    g_track3_first_check = 1U;
    g_track3_last_all_white = 1U;
    g_track4_state = TRACK4_AC_STRAIGHT;
    g_track4_first_check = 1U;
    g_track4_last_all_white = 1U;
    g_track4_lap_count = 0U;
    Encoder_Init();
    motorA(0);
    motorB(0);
}

void track_reset(uint8_t mode)
{
    if (mode == 1U) {
        g_track1_first_check = 1U;
        g_track1_last_all_white = 1U;
        g_track1_ab_done = 0U;
        g_track1_target_yaw = 0.0f;
    } else if (mode == 2U) {
        g_track2_state = TRACK2_AB_STRAIGHT;
        g_track2_first_check = 1U;
        g_track2_last_all_white = 1U;
        g_track2_weizhipid_state = TRACK2_AB_STRAIGHT;
        g_track2_weizhipid_first_check = 1U;
        g_track2_weizhipid_last_all_white = 1U;
        g_speed_pid_left_integral = 0.0f;
        g_speed_pid_right_integral = 0.0f;
        g_speed_pid_left_last_error = 0.0f;
        g_speed_pid_right_last_error = 0.0f;
        g_speed_pid_left_output = 0;
        g_speed_pid_right_output = 0;
        g_track2_pid1_state = TRACK2_PID1_AB_STRAIGHT;
        g_track2_pid1_first_check = 1U;
        g_track2_pid1_last_all_white = 1U;
        g_track2_pid1_last_error = 0.0f;
    } else if (mode == 3U) {
        g_track3_state = TRACK3_AC_STRAIGHT;
        g_track3_first_check = 1U;
        g_track3_last_all_white = 1U;
    } else if (mode == 4U) {
        g_track4_state = TRACK4_AC_STRAIGHT;
        g_track4_first_check = 1U;
        g_track4_last_all_white = 1U;
        g_track4_lap_count = 0U;
    }

    motorA(0);
    motorB(0);
}

uint8_t track_digital(uint8_t channel)//获取灰度传感器数据
{
    uint32_t pin = 0U;

    switch (channel) {
        case 1:
            pin = GPIO_TRACK_PIN_D1_PIN;
            break;
        case 2:
            pin = GPIO_TRACK_PIN_D2_PIN;
            break;
        case 3:
            pin = GPIO_TRACK_PIN_D3_PIN;
            break;
        case 4:
            pin = GPIO_TRACK_PIN_D4_PIN;
            break;
        case 5:
            pin = GPIO_TRACK_PIN_D5_PIN;
            break;
        case 6:
            pin = GPIO_TRACK_PIN_D6_PIN;
            break;
        case 7:
            pin = GPIO_TRACK_PIN_D7_PIN;
            break;
        case 8:
            pin = GPIO_TRACK_PIN_D8_PIN;
            break;
        default:
            return 1U;
    }

    return (DL_GPIO_readPins(GPIO_TRACK_PORT, pin) != 0U) ? 1U : 0U;
}

static int limit_speed(int speed)
{
    if (speed > TRACK_MAX_SPEED) {
        return TRACK_MAX_SPEED;
    }

    if (speed < -TRACK_MAX_SPEED) {
        return -TRACK_MAX_SPEED;
    }

    return speed;
}

static int motor_speed_to_encoder_target(int speed)
{
    if (speed >= 0) {
        return (speed + TRACK_ENCODER_TARGET_DIVISOR / 2) /
            TRACK_ENCODER_TARGET_DIVISOR;
    }

    return -((-speed + TRACK_ENCODER_TARGET_DIVISOR / 2) /
        TRACK_ENCODER_TARGET_DIVISOR);
}

//灰度寻迹差速-track
static int track_bias(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4,
    uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8)
{
    int d5_bias = 20;
    int d6_bias = 40;
    int d7_bias = 60;
    int d8_bias = 80;

    if (is_black(d4) && is_black(d5)) {
        return 0;
    } else if (is_black(d4)) {
        return -d5_bias;
    } else if (is_black(d5)) {
        return d5_bias;
    } else if (is_black(d3)) {
        return -d6_bias;
    } else if (is_black(d6)) {
        return d6_bias;
    } else if (is_black(d2)) {
        return -d7_bias;
    } else if (is_black(d7)) {
        return d7_bias;
    } else if (is_black(d1)) {
        return -d8_bias;
    } else if (is_black(d8)) {
        return d8_bias;
    }

    //涓㈢嚎澶勭悊
    // if (g_last_bias < 0) {          //涓㈢嚎鍚庣嚎鍦ㄥ乏杈?    //     return -TRACK_LOST_BIAS;
    // } 
    // else if (g_last_bias > 0) {     //涓㈢嚎鍚庣嚎鍦ㄥ彸杈?    //     return TRACK_LOST_BIAS;
    // }


    return 0;//都是白色，差速为0走直线
}

// track2_weizhiPID专用离散差速：基础速度400时使用，数值比原track_bias更大
static int track2_weizhipid_bias(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4,
    uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8)
{
    if (is_black(d4) && is_black(d5)) {
        return 0;
    } else if (is_black(d4)) {
        return -40;
    } else if (is_black(d5)) {
        return 40;
    } else if (is_black(d3)) {
        return -80;
    } else if (is_black(d6)) {
        return 80;
    } else if (is_black(d2)) {
        return -120;
    } else if (is_black(d7)) {
        return 120;
    } else if (is_black(d1)) {
        return -160;
    } else if (is_black(d8)) {
        return 160;
    }

    return 0;
}

static int SpeedPID_Left(int target_encoder, int real_encoder, int base_motor_speed)
{
    /* 左轮速度环调试参数：目标编码器计数约等于电机速度指令 * 6 */
    float speed_kp = 0.08f;
    float speed_ki = 0.0f;
    float speed_kd = 0.02f;
    float integral_limit = 8000.0f;
    float error = (float)(target_encoder - real_encoder);
    float Pout;
    float Iout;
    float Dout;
    float output;

    g_speed_pid_left_integral += error;
    if (g_speed_pid_left_integral > integral_limit) {
        g_speed_pid_left_integral = integral_limit;
    }
    if (g_speed_pid_left_integral < -integral_limit) {
        g_speed_pid_left_integral = -integral_limit;
    }

    Pout = speed_kp * error;
    Iout = speed_ki * g_speed_pid_left_integral;
    Dout = speed_kd * (error - g_speed_pid_left_last_error);
    output = (float)base_motor_speed + Pout + Iout + Dout;
    g_speed_pid_left_last_error = error;

    return limit_speed((int)output);
}

static int SpeedPID_Right(int target_encoder, int real_encoder, int base_motor_speed)
{
    /* 右轮速度环调试参数：左右轮变量独立，后续可以单独调不同参数 */
    float speed_kp = 0.08f;
    float speed_ki = 0.0f;
    float speed_kd = 0.02f;
    float integral_limit = 8000.0f;
    float error = (float)(target_encoder - real_encoder);
    float Pout;
    float Iout;
    float Dout;
    float output;

    g_speed_pid_right_integral += error;
    if (g_speed_pid_right_integral > integral_limit) {
        g_speed_pid_right_integral = integral_limit;
    }
    if (g_speed_pid_right_integral < -integral_limit) {
        g_speed_pid_right_integral = -integral_limit;
    }

    Pout = speed_kp * error;
    Iout = speed_ki * g_speed_pid_right_integral;
    Dout = speed_kd * (error - g_speed_pid_right_last_error);
    output = (float)base_motor_speed + Pout + Iout + Dout;
    g_speed_pid_right_last_error = error;

    return limit_speed((int)output);
}

void track1_test(void)
{
    motorA(1000);
    motorB(1000);
}



//直线加直角转弯
void track1_test1(void)
{
    // static int saved_side = 0;  // 保存目标数字所在方向：0未保存，1左，2右
    // int num = Camera_GetMatchedDigit();
    // int side = Camera_GetMatchedSide();
    // if ((saved_side == 0) && (num != 0) && ((side == 1) || (side == 2))) {
    //     saved_side = side;//保存第一次就识别的数字
    // }
    
    //摄像头连续识别3次成功才保存识别的数字
    static int saved_side = 0;//0表示识别的数字不是目标数字
    static int last_side = 0;
    static int side_count = 0;
    int num = Camera_GetMatchedDigit();
    int side = Camera_GetMatchedSide();
    if ((saved_side == 0) && (num != 0) && ((side == 1) || (side == 2))) {//1表示数字在左边，2表示在右边
        if (side == last_side) {
            side_count++;
        } 
        else {
            last_side = side;
            side_count = 1;
        }

        if (side_count >= 3) {
            saved_side = side;
        }
    }
       
    /* track1_test1测试参数，放函数内部方便现场调 */
    
    /* 测试状态变量：0寻迹，1检测到全黑后开始计数并刹车，2转90度，3继续前行，4刹车停下 */
    static uint8_t state = 0U;//状态机状态
    static int soft_reset = 1;//软启动状态
    static uint8_t last_all_white = 1U;//上一拍是否全白，用来做边沿触发

    static float error_l = 0.0f;
    static float last_error_l = 0.0f;
    static float last_last_error_l = 0.0f;
    static float error_r = 0.0f;
    static float last_error_r = 0.0f;
    static float last_last_error_r = 0.0f;
    static int left_output_speed = 0;
    static int right_output_speed = 0;

    static float turn_target_yaw = 0.0f; // 左转90度目标角

    uint8_t d1 = D1;
    uint8_t d2 = D2;
    uint8_t d3 = D3;
    uint8_t d4 = D4;
    uint8_t d5 = D5;
    uint8_t d6 = D6;
    uint8_t d7 = D7;
    uint8_t d8 = D8;
    uint8_t black_count = 0U;
    if (is_black(d1)) black_count++;
    if (is_black(d2)) black_count++;
    if (is_black(d3)) black_count++;
    if (is_black(d4)) black_count++;
    if (is_black(d5)) black_count++;
    if (is_black(d6)) black_count++;
    if (is_black(d7)) black_count++;
    if (is_black(d8)) black_count++;
    //全黑
    uint8_t all_black = ((d1 == 0U) && (d2 == 0U) &&
        (d3 == 0U) && (d4 == 0U) &&
        (d5 == 0U) && (d6 == 0U) &&
        (d7 == 0U) && (d8 == 0U)) ? 1U : 0U;


    //状态机，先写停止/切换条件
    if (state == 4U) {
        motors_stop();
        // last_all_white = now_all_white;
        return;
    }
    //状态0的寻迹过程中遇十字路口且识别到目标数字，则切换为状态1
    if ((state == 0U) && (black_count >=4) && (saved_side != 0)) {
        Encoder_ResetDistanceCount();//编码器开始计数
        state = 1U;
    }
    //状态1，编码器到达计数后切换状态2
    uint32_t stop_count = 22500U;//车长
    float now_yaw;
    float yaw_error;
    float pid_out;
    int turn_speed;
    if ((state == 1U) && (Encoder_GetDistanceCount() >= stop_count)) {
        motors_brake();
        now_yaw = GetYaw_Value();
        //根据摄像头识别数字转弯
        if(saved_side == 1){
            turn_target_yaw = now_yaw - 90.0f;
        }
        else if(saved_side == 2){
            turn_target_yaw = now_yaw + 90.0f;
        }
        //转弯角度绕回处理
        if (turn_target_yaw > 180.0f) {
            turn_target_yaw -= 360.0f;
        }
        if (turn_target_yaw < -180.0f) {
            turn_target_yaw += 360.0f;
        }
        left_output_speed = 0;
        right_output_speed = 0;
        soft_reset = 1;
        state = 2U;
        // last_all_white = now_all_white;
        return;
    }

    if(state==2){
        now_yaw = GetYaw_Value();
        yaw_error = turn_target_yaw - now_yaw;

        if (yaw_error > 180.0f) {
            yaw_error -= 360.0f;
        }
        if (yaw_error < -180.0f) {
            yaw_error += 360.0f;
        }
        //转弯完成后
        if ((yaw_error <= 5.0f) && (yaw_error >= -5.0f)) {//转弯完成判断
            motors_brake();
            //继续直行            
            left_output_speed = 0;
            right_output_speed = 0;
            soft_reset = 1;
            state = 3U;//共用寻迹直行
            return;
        }

        pid_out = PID2(turn_target_yaw, now_yaw);
        turn_speed = limit_speed((int)pid_out);
        //最大转弯力度
        // if (turn_speed > 250) {
        //     turn_speed = 250;
        // }
        // if (turn_speed < -250) {
        //     turn_speed = -250;
        // }
        //最小转弯力度为180
        if ((turn_speed > 0) && (turn_speed < 180)) {
            turn_speed = 180;
        }
        if ((turn_speed < 0) && (turn_speed > -180)) {
            turn_speed = -180;
        }

            /* 左转：左轮反转，右轮正转 */
            motorA(turn_speed);
            motorB(-turn_speed);
            return;
    }

    if((state == 3) && (black_count>=3)){
        motors_brake();
        state = 4;
        return;
    }



    //软启动-->缓慢加速到设定速度防止翘头
    int target_base_speed = 200;//要加度到的最终速度
    int start_base_speed = 100;//软启动初始速度
    int soft_step = 20;//每次加速20  
    int base_speed = motor_soft_speed(
        target_base_speed,
        start_base_speed,
        soft_step,
        soft_reset
    );
    soft_reset = 0;
    //由灰度得到的左右轮位置环差速
    int bias;
    int d5_bias = (int)(base_speed * 0.02f);//直线寻迹时，中间几路差速调小一些
    int d6_bias = (int)(base_speed * 0.05f);
    int d7_bias = (int)(base_speed * 0.3f);
    int d8_bias = (int)(base_speed * 0.4f);
    if (is_black(d4) && is_black(d5)) {
        bias = 0;
    } else if (is_black(d4)) {
        bias = -d5_bias;
    } else if (is_black(d5)) {
        bias = d5_bias;
    } else if (is_black(d3)) {
        bias = -d6_bias;
    } else if (is_black(d6)) {
        bias = d6_bias;
    } else if (is_black(d2)) {
        bias = -d7_bias;
    } else if (is_black(d7)) {
        bias = d7_bias;
    } else if (is_black(d1)) {
        bias = -d8_bias;
    } else if (is_black(d8)) {
        bias = d8_bias;
    } else {
        bias = 0;
    }
    if (bias != 0) {
        g_last_bias = bias;//丢线处理会用到，现在暂时没用
    }
    int left_target_speed = limit_speed(base_speed + bias);
    int right_target_speed = limit_speed(base_speed - bias);

    //直线寻迹增量PID
    int encoder_ratio = 6;//编码器计数与电机速度的比值
    float kp_l = 0.18f;
    float ki_l = 0.05f;
    float kd_l = 0.08f;
    float kp_r = 0.18f;
    float ki_r = 0.05f;
    float kd_r = 0.08f;
    int output_max = 1000;
    int output_min = -1000;
    float Pout, Iout, Dout, delta_output;
    if (Encoder_HasNewSpeed() != 0U) {
        int left_target_encoder = left_target_speed * encoder_ratio;
        int right_target_encoder = right_target_speed * encoder_ratio;
        int left_real_encoder = Encoder_GetNowVA();
        int right_real_encoder = Encoder_GetNowVB();
        error_l = (float)(left_target_encoder - left_real_encoder);
        //增量式PID的三项
        Pout = kp_l * (error_l - last_error_l);
        Iout = ki_l * error_l;
        Dout = kd_l * (error_l - 2.0f * last_error_l + last_last_error_l);
        delta_output = Pout + Iout + Dout;
        left_output_speed += (int)delta_output;
        if (left_output_speed > output_max) {
            left_output_speed = output_max;
        }
        if (left_output_speed < output_min) {
            left_output_speed = output_min;
        }
        last_last_error_l = last_error_l;
        last_error_l = error_l;
        error_r = (float)(right_target_encoder - right_real_encoder);
        Pout = kp_r * (error_r - last_error_r);
        Iout = ki_r * error_r;
        Dout = kd_r * (error_r - 2.0f * last_error_r + last_last_error_r);
        delta_output = Pout + Iout + Dout;
        right_output_speed += (int)delta_output;
        if (right_output_speed > output_max) {
            right_output_speed = output_max;
        }
        if (right_output_speed < output_min) {
            right_output_speed = output_min;
        }
        last_last_error_r = last_error_r;
        last_error_r = error_r;
        Encoder_ClearNewSpeedFlag();
    }

    motorA(left_output_speed);//PID的输出结果
    motorB(right_output_speed);
}

void t2(void)
{
    static uint8_t state = 0U;//状态机状态
    static int soft_reset = 1;//软启动状态
    static uint8_t last_all_white = 1U;//上一拍是否全白，用来做边沿触发

    static float error_l = 0.0f;
    static float last_error_l = 0.0f;
    static float last_last_error_l = 0.0f;
    static float error_r = 0.0f;
    static float last_error_r = 0.0f;
    static float last_last_error_r = 0.0f;
    static int left_output_speed = 0;
    static int right_output_speed = 0;

    uint8_t d1 = D1;
    uint8_t d2 = D2;
    uint8_t d3 = D3;
    uint8_t d4 = D4;
    uint8_t d5 = D5;
    uint8_t d6 = D6;
    uint8_t d7 = D7;
    uint8_t d8 = D8;
    uint8_t black_count = 0U;
    if (is_black(d1)) black_count++;
    if (is_black(d2)) black_count++;
    if (is_black(d3)) black_count++;
    if (is_black(d4)) black_count++;
    if (is_black(d5)) black_count++;
    if (is_black(d6)) black_count++;
    if (is_black(d7)) black_count++;
    if (is_black(d8)) black_count++;
    //全黑
    uint8_t all_black = ((d1 == 0U) && (d2 == 0U) &&
        (d3 == 0U) && (d4 == 0U) &&
        (d5 == 0U) && (d6 == 0U) &&
        (d7 == 0U) && (d8 == 0U)) ? 1U : 0U;

    Encoder_GetDistanceCount();//编码器开始计数
    //停止条件
    if(state == 2){
        motors_stop();
        return;
    }
    uint32_t decel_start_count = 500000U;//到半弯道开始减速
    if((state == 0) && (Encoder_GetDistanceCount() >= decel_start_count) ){
        state = 1;
    }
    if((state == 1) && ((black_count>=4)|| (Encoder_GetDistanceCount()>=688000))){
        SoundLight_TimeStop();
        motors_brake_ms(80);
        state = 2;
        return;
    }
    int target_base_speed = 400;//要加度到的最终速度
    if(state == 1){
        target_base_speed = 150;
    }

    //软启动-->缓慢加速到设定速度防止翘头
    
    int start_base_speed = 300;//软启动初始速度
    int soft_step = 10;//每次加速20  
    int base_speed = motor_soft_speed(
        target_base_speed,
        start_base_speed,
        soft_step,
        soft_reset
    );
    soft_reset = 0;

    //由灰度得到的左右轮位置环差速
    int bias;
    int d5_bias = (int)(base_speed * 0.1f);//直线寻迹时，中间几路差速调小一些
    int d6_bias = (int)(base_speed * 0.2f);
    int d7_bias = (int)(base_speed * 0.3f);
    int d8_bias = (int)(base_speed * 0.4f);
    if (is_black(d4) && is_black(d5)) {
        bias = 0;
    } else if (is_black(d4)) {
        bias = -d5_bias;
    } else if (is_black(d5)) {
        bias = d5_bias;
    } else if (is_black(d3)) {
        bias = -d6_bias;
    } else if (is_black(d6)) {
        bias = d6_bias;
    } else if (is_black(d2)) {
        bias = -d7_bias;
    } else if (is_black(d7)) {
        bias = d7_bias;
    } else if (is_black(d1)) {
        bias = -d8_bias;
    } else if (is_black(d8)) {
        bias = d8_bias;
    } else {
        bias = 0;
    }
    if (bias != 0) {
        g_last_bias = bias;//丢线处理会用到，现在暂时没用
    }
    int left_target_speed = limit_speed(base_speed + bias);
    int right_target_speed = limit_speed(base_speed - bias);

    //直线寻迹增量PID
    int encoder_ratio = 6;//编码器计数与电机速度的比值
    float kp_l = 0.2f;
    float ki_l = 0.01f;
    float kd_l = 0.0f;
    float kp_r = 0.2f;
    float ki_r = 0.01f;
    float kd_r = 0.0f;
    int output_max = 1000;
    int output_min = -1000;
    float Pout, Iout, Dout, delta_output;
    if (Encoder_HasNewSpeed() != 0U) {
        int left_target_encoder = left_target_speed * encoder_ratio;
        int right_target_encoder = right_target_speed * encoder_ratio;
        int left_real_encoder = Encoder_GetNowVA();
        int right_real_encoder = Encoder_GetNowVB();
        error_l = (float)(left_target_encoder - left_real_encoder);
        //增量式PID的三项
        Pout = kp_l * (error_l - last_error_l);
        Iout = ki_l * error_l;
        Dout = kd_l * (error_l - 2.0f * last_error_l + last_last_error_l);
        delta_output = Pout + Iout + Dout;
        left_output_speed += (int)delta_output;
        if (left_output_speed > output_max) {
            left_output_speed = output_max;
        }
        if (left_output_speed < output_min) {
            left_output_speed = output_min;
        }
        last_last_error_l = last_error_l;
        last_error_l = error_l;
        error_r = (float)(right_target_encoder - right_real_encoder);
        Pout = kp_r * (error_r - last_error_r);
        Iout = ki_r * error_r;
        Dout = kd_r * (error_r - 2.0f * last_error_r + last_last_error_r);
        delta_output = Pout + Iout + Dout;
        right_output_speed += (int)delta_output;
        if (right_output_speed > output_max) {
            right_output_speed = output_max;
        }
        if (right_output_speed < output_min) {
            right_output_speed = output_min;
        }
        last_last_error_r = last_error_r;
        last_error_r = error_r;
        Encoder_ClearNewSpeedFlag();
    }

    motorA(left_output_speed);//PID的输出结果
    motorB(right_output_speed);
}

void t3(void)
{
    static uint8_t state = 0U;                 // 第3题状态：0去+5cm，1折返到-5cm，2稳定在-5cm
    static uint8_t first_check = 1U;           // 第一次进入函数标志
    static uint32_t start_time = 0U;           // 第3题开始计时
    static uint32_t last_stepper_time = 0U;    // 上一次发送步进电机指令的时间
    static uint32_t last_debug_time = 0U;      // 上一次串口打印时间
    static float pos_integral = 0.0f;          // 位置误差积分
    static float last_predict_error = 0.0f;    // 上一次预测误差

    uint32_t now_time = SoundLight_GetSysMs();
    uint32_t stepper_period_ms = 20U;          // 步进电机指令发送周期
    if (first_check != 0U) {
        start_time = now_time;
        first_check = 0U;
    }
    uint32_t elapsed_ms = now_time - start_time;

    if ((now_time - last_stepper_time) < stepper_period_ms) {
        return;
    }
    last_stepper_time = now_time;

    if (Camera_HasBall() == 0U) {
        pos_integral = 0.0f;
        last_predict_error = 0.0f;
        Stepper_SetAngle(0.0f);
        return;
    }

    /* 摄像头数据：位置范围约-12到+12，速度范围约-300到+300 */
    float ball_pos = Camera_GetBallError();
    float ball_speed = Camera_GetBallSpeed();
    float target_right = 5.0f;                 // 右侧目标位置，单位cm
    float target_left = -5.0f;                 // 左侧目标位置，单位cm
    float arrive_error = 0.5f;                 // 到点误差阈值，单位cm
    float target_pos = (state == 0U) ? target_right : target_left;//当前位置目标，单位cm
    float ball_error = ball_pos - target_pos;  // 真实位置误差
    //切换状态
    if ((state == 0U) && (ball_error <= arrive_error) && (ball_error >= -arrive_error)) {
        state = 1U;
        target_pos = target_left;
        ball_error = ball_pos - target_pos;
        pos_integral = 0.0f;
        last_predict_error = 0.0f;
    } 
    else if ((state == 1U) && (ball_error <= arrive_error) && (ball_error >= -arrive_error)) {
        state = 2U;
        target_pos = target_left;
        ball_error = ball_pos - target_pos;
        pos_integral = 0.0f;
        last_predict_error = 0.0f;
    }

    float kv_pos = 0.5f;                       // 速度提前系数，现场调
    float kp_pos = 5.0f;                        // 位置环P
    float ki_pos = 0.0f;                        // 位置环I
    float kd_pos = 0.05f;                       // 位置环D
    float pos_integral_limit = 800.0f;          // 位置积分限幅
    float rod_angle_limit = 50.0f;              // 摆杆角度限幅
    /* 根据当前状态选择PID参数 */
    if (state == 0U) {//去5cm
        kp_pos = 5.0f;
        ki_pos = 0.025f;
        kd_pos = 0.025f;
    } 
    else if (state == 1U) {//回摆-5cm
        kv_pos = 0.78f;
        kp_pos = 5.0f;
        ki_pos = 0.0f;
        kd_pos = 0.5f;//0.12
    } 
    else {//state==2时要稳定下来
        kp_pos = 5.0f;
        ki_pos = 0.0005f;
        kd_pos = 0.0f;//0.06、0.25
    }
    float predict_error = ball_error + kv_pos * ball_speed;// 预测误差

    pos_integral += ball_error;
    if (pos_integral > pos_integral_limit) {
        pos_integral = pos_integral_limit;
    }
    if (pos_integral < -pos_integral_limit) {
        pos_integral = -pos_integral_limit;
    }

    float Pout = kp_pos * predict_error;
    float Iout = ki_pos * pos_integral;
    float Dout = kd_pos * (predict_error - last_predict_error);
    float pid_out = Pout + Iout + Dout;
    float rod_angle = -pid_out;                 // 正位置误差时，摆杆给负角度压回来

    if (rod_angle > rod_angle_limit) {
        rod_angle = rod_angle_limit;
    }
    if (rod_angle < -rod_angle_limit) {
        rod_angle = -rod_angle_limit;
    }

    last_predict_error = predict_error;
    Stepper_SetAngle(rod_angle);
}

void t3_yuce(void)
{
    static uint32_t last_stepper_ms = 0U;       // 上一次发送步进电机角度的时间
    static float pos_integral = 0.0f;           // 位置误差积分
    static float last_predict_error = 0.0f;     // 上一次预测误差

    uint32_t now_ms = SoundLight_GetSysMs();
    uint32_t stepper_period_ms = 20U;           // 步进电机发送周期
    if ((now_ms - last_stepper_ms) < stepper_period_ms) {
        return;
    }
    last_stepper_ms = now_ms;

    if (Camera_HasBall() == 0U) {
        pos_integral = 0.0f;
        last_predict_error = 0.0f;
        Stepper_SetAngle(0.0f);
        return;
    }

    float target_pos = -9.0f;                    // 目标位置，单位cm，后续可改成5或-5
    float kv_pos = 0.5f;                       // 速度提前系数，现场调
    float kp_pos = 5.0f;                        // 预测误差P项系数
    float ki_pos = 0.0f;                        // 位置积分I项系数
    float kd_pos = 0.0f;                        // 预测误差D项系数
    float pos_integral_limit = 800.0f;          // 位置积分限幅
    float rod_angle_limit = 50.0f;              // 摆杆目标角度限幅

    float ball_pos = Camera_GetBallError();     // 小球当前位置误差
    float ball_speed = Camera_GetBallSpeed();   // 小球当前速度
    float ball_error = ball_pos - target_pos;   // 真实位置误差
    float predict_error = ball_error + kv_pos * ball_speed;// 预测误差

    pos_integral += ball_error;
    if (pos_integral > pos_integral_limit) {
        pos_integral = pos_integral_limit;
    }
    if (pos_integral < -pos_integral_limit) {
        pos_integral = -pos_integral_limit;
    }

    float p_out = kp_pos * predict_error;
    float i_out = ki_pos * pos_integral;
    float d_out = kd_pos * (predict_error - last_predict_error);
    float pid_out = p_out + i_out + d_out;
    float rod_angle = -pid_out;                 // 正位置误差时，摆杆给负角度压回来

    if (rod_angle > rod_angle_limit) {
        rod_angle = rod_angle_limit;
    }
    if (rod_angle < -rod_angle_limit) {
        rod_angle = -rod_angle_limit;
    }

    last_predict_error = predict_error;
    Stepper_SetAngle(rod_angle);
}

void t3_2(void)
{
    /* 摆杆双环PID状态量 */
    static float speed_error_last = 0.0f;      // 上一次速度误差
    static float speed_integral = 0.0f;        // 速度环积分
    static uint32_t last_stepper_time = 0U;    // 上一次发送步进电机指令的时间
    static uint32_t last_debug_time = 0U;      // 上一次串口打印时间

    uint32_t now_time = SoundLight_GetTimeMs();
    uint32_t stepper_period_ms = 40U;          // 步进电机指令发送间隔，建议50~100ms

    if (Camera_HasBall() == 0U) {
        speed_error_last = 0.0f;
        speed_integral = 0.0f;

        if ((now_time - last_stepper_time) >= stepper_period_ms) {
            Stepper_SetAngle(0.0f);
            last_stepper_time = now_time;
        }
        return;
    }

    /* 摄像头数据：位置范围约-12到+12，速度范围约-300到+300 */
    float ball_error = Camera_GetBallError()-5;
    float ball_speed = Camera_GetBallSpeed();

    /* 位置外环：位置误差 -> 目标速度
       注意：电机正角度会让小球往正方向运动，所以小球在正方向时，目标速度应为负 */
    float kp_pos = 8.0f;
    float target_speed_limit = 1200.0f;

    float target_speed = -kp_pos * ball_error;

    if (target_speed > target_speed_limit) {
        target_speed = target_speed_limit;
    }
    if (target_speed < -target_speed_limit) {
        target_speed = -target_speed_limit;
    }

    /* 速度内环：目标速度 - 实际速度 -> 摆杆角度 */
    float kp_speed = 0.2f;
    float ki_speed = 0.005f;
    float kd_speed = 0.0f;
    float speed_integral_limit = 800.0f;    
    float rod_angle_limit = 30.0f;              // 摆杆最大转角

    float speed_error = target_speed - ball_speed;

    speed_integral += speed_error;
    if (speed_integral > speed_integral_limit) {
        speed_integral = speed_integral_limit;
    }
    if (speed_integral < -speed_integral_limit) {
        speed_integral = -speed_integral_limit;
    }

    float speed_diff = speed_error - speed_error_last;

    float Pout = kp_speed * speed_error;
    float Iout = ki_speed * speed_integral;
    float Dout = kd_speed * speed_diff;
    float rod_angle = Pout + Iout + Dout;

    /* 中线附近死区，避免接近目标后电机一直抖 */
    if ((ball_error > -0.3f) && (ball_error < 0.3f) ) {
        rod_angle = 0.0f;
    }

    if (rod_angle > rod_angle_limit) {
        rod_angle = rod_angle_limit;
    }
    if (rod_angle < -rod_angle_limit) {
        rod_angle = -rod_angle_limit;
    }

    speed_error_last = speed_error;

    if ((now_time - last_debug_time) >= 100U) {
        // minimal printf可能不显示%d数值，这里改用手动整数打印
        // printf("e=%d v=%d ts=%d se=%d p=%d i=%d d=%d a=%d\r\n",
        //     (int)(ball_error * 10.0f),
        //     (int)(ball_speed * 10.0f),
        //     (int)(target_speed * 10.0f),
        //     (int)(speed_error * 10.0f),
        //     (int)(Pout * 10.0f),
        //     (int)(Iout * 10.0f),
        //     (int)(Dout * 10.0f),
        //     (int)(rod_angle * 10.0f));

        debug_uart_print_string("e=");
        debug_uart_print_int((int32_t)(ball_error * 10.0f));
        debug_uart_print_string(" v=");
        debug_uart_print_int((int32_t)(ball_speed * 10.0f));
        debug_uart_print_string(" ts=");
        debug_uart_print_int((int32_t)(target_speed * 10.0f));
        debug_uart_print_string(" se=");
        debug_uart_print_int((int32_t)(speed_error * 10.0f));
        debug_uart_print_string(" p=");
        debug_uart_print_int((int32_t)(Pout * 10.0f));
        debug_uart_print_string(" i=");
        debug_uart_print_int((int32_t)(Iout * 10.0f));
        debug_uart_print_string(" d=");
        debug_uart_print_int((int32_t)(Dout * 10.0f));
        debug_uart_print_string(" a=");
        debug_uart_print_int((int32_t)(rod_angle * 10.0f));
        debug_uart_print_string("\r\n");

        last_debug_time = now_time;
    }

    if ((now_time - last_stepper_time) >= stepper_period_ms) {
        Stepper_SetAngle(rod_angle);
        last_stepper_time = now_time;
    }
}

void t3_test(void){
    Stepper_SetAngle(2.0);
}

void t3_formal(void)
{
    static uint8_t state = 0U;                 // 第3题状态：0去+5cm，1折返到-5cm，2稳定在-5cm
    static uint8_t first_check = 1U;           // 第一次进入函数标志
    static uint32_t start_time = 0U;           // 第3题开始计时
    static uint32_t last_stepper_time = 0U;    // 上一次发送步进电机指令的时间
    static uint32_t last_debug_time = 0U;      // 上一次串口打印时间
    static float pos_integral = 0.0f;          // 位置误差积分
    static float last_predict_error = 0.0f;    // 上一次预测误差

    uint32_t now_time = SoundLight_GetSysMs();
    uint32_t stepper_period_ms = 20U;          // 步进电机指令发送周期
    if (first_check != 0U) {
        start_time = now_time;
        first_check = 0U;
    }
    uint32_t elapsed_ms = now_time - start_time;

    if ((now_time - last_stepper_time) < stepper_period_ms) {
        return;
    }
    last_stepper_time = now_time;

    if (Camera_HasBall() == 0U) {
        pos_integral = 0.0f;
        last_predict_error = 0.0f;
        Stepper_SetAngle(0.0f);
        return;
    }

    /* 摄像头数据：位置范围约-12到+12，速度范围约-300到+300 */
    float ball_pos = Camera_GetBallError();
    float ball_speed = Camera_GetBallSpeed();
    float target_right = 5.0f;                 // 右侧目标位置，单位cm
    float target_left = -5.0f;                 // 左侧目标位置，单位cm
    float arrive_error = 0.5f;                 // 到点误差阈值，单位cm
    float target_pos = (state == 0U) ? target_right : target_left;//当前位置目标，单位cm
    float ball_error = ball_pos - target_pos;  // 真实位置误差

    if ((state == 0U) && (ball_error <= arrive_error) && (ball_error >= -arrive_error)) {
        state = 1U;
        target_pos = target_left;
        ball_error = ball_pos - target_pos;
        pos_integral = 0.0f;
        last_predict_error = 0.0f;
    } else if ((state == 1U) && (ball_error <= arrive_error) && (ball_error >= -arrive_error)) {
        state = 2U;
        target_pos = target_left;
        ball_error = ball_pos - target_pos;
        pos_integral = 0.0f;
        last_predict_error = 0.0f;
    }

    float kv_pos = 0.5f;                       // 速度提前系数，现场调
    float kp_pos = 5.0f;                        // 位置环P
    float ki_pos = 0.0f;                        // 位置环I
    float kd_pos = 0.0f;                        // 位置环D
    float pos_integral_limit = 800.0f;          // 位置积分限幅
    float rod_angle_limit = 50.0f;              // 摆杆角度限幅

    float predict_error = ball_error + kv_pos * ball_speed;// 预测误差

    pos_integral += ball_error;
    if (pos_integral > pos_integral_limit) {
        pos_integral = pos_integral_limit;
    }
    if (pos_integral < -pos_integral_limit) {
        pos_integral = -pos_integral_limit;
    }

    float Pout = kp_pos * predict_error;
    float Iout = ki_pos * pos_integral;
    float Dout = kd_pos * (predict_error - last_predict_error);
    float pid_out = Pout + Iout + Dout;
    float rod_angle = -pid_out;                 // 正位置误差时，摆杆给负角度压回来

    if (rod_angle > rod_angle_limit) {
        rod_angle = rod_angle_limit;
    }
    if (rod_angle < -rod_angle_limit) {
        rod_angle = -rod_angle_limit;
    }

    last_predict_error = predict_error;
    Stepper_SetAngle(rod_angle);
}

void t4_yuce(void)
{
    static uint8_t state = 0U;//状态机状态
    static int soft_reset = 1;//软启动状态
    static uint8_t last_all_white = 1U;//上一拍是否全白，用来做边沿触发

    static float error_l = 0.0f;
    static float last_error_l = 0.0f;
    static float last_last_error_l = 0.0f;
    static float error_r = 0.0f;
    static float last_error_r = 0.0f;
    static float last_last_error_r = 0.0f;
    static int left_output_speed = 0;
    static int right_output_speed = 0;
    static float ball_speed_error_last = 0.0f;  // 上一次小球速度误差
    static float ball_speed_integral = 0.0f;    // 小球速度环积分
    static uint32_t last_stepper_time = 0U;     // 上一次发送步进电机角度的时间
    static uint32_t last_ball_debug_time = 0U;  // 上一次摆杆调试打印时间
    static float accel_filter = 0.0f;           // 小车加速度滤波值，后续可接入IMU

    uint8_t d1 = D1;
    uint8_t d2 = D2;
    uint8_t d3 = D3;
    uint8_t d4 = D4;
    uint8_t d5 = D5;
    uint8_t d6 = D6;
    uint8_t d7 = D7;
    uint8_t d8 = D8;
    uint8_t black_count = 0U;
    if (is_black(d1)) black_count++;
    if (is_black(d2)) black_count++;
    if (is_black(d3)) black_count++;
    if (is_black(d4)) black_count++;
    if (is_black(d5)) black_count++;
    if (is_black(d6)) black_count++;
    if (is_black(d7)) black_count++;
    if (is_black(d8)) black_count++;
    //全黑
    uint8_t all_black = ((d1 == 0U) && (d2 == 0U) &&
        (d3 == 0U) && (d4 == 0U) &&
        (d5 == 0U) && (d6 == 0U) &&
        (d7 == 0U) && (d8 == 0U)) ? 1U : 0U;

    uint32_t now_time = SoundLight_GetSysMs();//摆杆周期使用独立系统时间，停车后仍继续增加

    // 第4题摆杆控制：行驶时让小球保持在中心O附近
    uint32_t stepper_period_ms = 40U;       // 步进电机发送周期
    float kp_pos = 8.0f;                    // 位置外环比例系数
    float target_speed_limit = 1200.0f;     // 目标速度限幅
    float kp_speed = 0.1f;                  // 速度内环P
    float ki_speed = 0.005f;                // 速度内环I
    float kd_speed = 0.0f;                  // 速度内环D
    float speed_integral_limit = 800.0f;    // 速度环积分限幅
    float rod_angle_limit = 20.0f;          // 摆杆最终角度限幅
    if (Camera_HasBall() == 0U) {
        ball_speed_error_last = 0.0f;
        ball_speed_integral = 0.0f;

        if ((now_time - last_stepper_time) >= stepper_period_ms) {
            Stepper_SetAngle(0.0f);
            last_stepper_time = now_time;
        }
    } else {
        float target_pos = 0.0f;                    // 第4题目标位置，单位cm，保持在中心O
        float ball_pos = Camera_GetBallError();     // 小球当前位置，单位cm，相对中心O
        float ball_speed = Camera_GetBallSpeed();   // 小球速度，来自摄像头
        float ball_error = ball_pos - target_pos;   // 小球位置误差，当前位置-目标位置
        float target_speed = -kp_pos * ball_error;
        if (target_speed > target_speed_limit) {
            target_speed = target_speed_limit;
        }
        if (target_speed < -target_speed_limit) {
            target_speed = -target_speed_limit;
        }

        float speed_error = target_speed - ball_speed;
        ball_speed_integral += speed_error;
        if (ball_speed_integral > speed_integral_limit) {
            ball_speed_integral = speed_integral_limit;
        }
        if (ball_speed_integral < -speed_integral_limit) {
            ball_speed_integral = -speed_integral_limit;
        }

        float speed_diff = speed_error - ball_speed_error_last;
        float Pout_ball = kp_speed * speed_error;
        float Iout_ball = ki_speed * ball_speed_integral;
        float Dout_ball = kd_speed * speed_diff;
        float rod_angle = Pout_ball + Iout_ball + Dout_ball;

        // float car_accel = 0.0f;     // 小车前后方向加速度，后续替换成实际IMU加速度
        float imu_data[7];              // IMU缓存数据
        IMU_TT_getgyro(imu_data);
        float car_accel = imu_data[1] + 30.0;

        float k_acc = 0.01f;        // 加速度前馈系数，方向不对就改正负号
        float ff_limit = 8.0f;      // 前馈角度限幅，防止加速度噪声导致摆杆乱抖
        accel_filter = 0.8f * accel_filter + 0.2f * car_accel;
        float accel_feedforward = k_acc * accel_filter;
        if (accel_feedforward > ff_limit) {
            accel_feedforward = ff_limit;
        }
        if (accel_feedforward < -ff_limit) {
            accel_feedforward = -ff_limit;
        }

        float final_angle = rod_angle + accel_feedforward;
        if (final_angle > rod_angle_limit) {
            final_angle = rod_angle_limit;
        }
        if (final_angle < -rod_angle_limit) {
            final_angle = -rod_angle_limit;
        }

        ball_speed_error_last = speed_error;

        if ((now_time - last_ball_debug_time) >= 100U) {
            debug_uart_print_string("bp=");
            debug_uart_print_int((int32_t)(ball_pos * 10.0f));
            debug_uart_print_string(" be=");
            debug_uart_print_int((int32_t)(ball_error * 10.0f));
            debug_uart_print_string(" bv=");
            debug_uart_print_int((int32_t)(ball_speed * 10.0f));
            debug_uart_print_string(" ts=");
            debug_uart_print_int((int32_t)(target_speed * 10.0f));
            debug_uart_print_string(" se=");
            debug_uart_print_int((int32_t)(speed_error * 10.0f));
            debug_uart_print_string(" p=");
            debug_uart_print_int((int32_t)(Pout_ball * 10.0f));
            debug_uart_print_string(" i=");
            debug_uart_print_int((int32_t)(Iout_ball * 10.0f));
            debug_uart_print_string(" d=");
            debug_uart_print_int((int32_t)(Dout_ball * 10.0f));
            debug_uart_print_string(" ff=");
            debug_uart_print_int((int32_t)(accel_feedforward * 10.0f));
            debug_uart_print_string(" a=");
            debug_uart_print_int((int32_t)(final_angle * 10.0f));
            debug_uart_print_string("\r\n");

            last_ball_debug_time = now_time;
        }

        if ((now_time - last_stepper_time) >= stepper_period_ms) {
            Stepper_SetAngle(final_angle);
            last_stepper_time = now_time;
        }
    }

    Encoder_GetDistanceCount();//编码器开始计数
    //停止条件
    if(state == 2){
        motors_stop();
        return;
    }
    uint32_t decel_start_count = 150000U;//到半弯道开始减速
    if((state == 0) && (Encoder_GetDistanceCount() >= decel_start_count) ){
        state = 1;
    }
    if((state == 1) && ((black_count>=4)|| (Encoder_GetDistanceCount()>=181000))){
        SoundLight_TimeStop();
        motors_stop();
        state = 2;
        return;
    }
    int target_base_speed = 300;//要加度到的最终速度
    if(state == 1){
        target_base_speed = 150;
    }

    //软启动-->缓慢加速到设定速度防止翘头
    
    int start_base_speed = 150;//软启动初始速度
    int soft_step = 20;//每次加速20  
    int base_speed = motor_soft_speed(
        target_base_speed,
        start_base_speed,
        soft_step,
        soft_reset
    );
    soft_reset = 0;


    //由灰度得到的左右轮位置环差速
    int bias;
    int d5_bias = (int)(base_speed * 0.1f);//直线寻迹时，中间几路差速调小一些
    int d6_bias = (int)(base_speed * 0.2f);
    int d7_bias = (int)(base_speed * 0.3f);
    int d8_bias = (int)(base_speed * 0.4f);
    if (is_black(d4) && is_black(d5)) {
        bias = 0;
    } else if (is_black(d4)) {
        bias = -d5_bias;
    } else if (is_black(d5)) {
        bias = d5_bias;
    } else if (is_black(d3)) {
        bias = -d6_bias;
    } else if (is_black(d6)) {
        bias = d6_bias;
    } else if (is_black(d2)) {
        bias = -d7_bias;
    } else if (is_black(d7)) {
        bias = d7_bias;
    } else if (is_black(d1)) {
        bias = -d8_bias;
    } else if (is_black(d8)) {
        bias = d8_bias;
    } else {
        bias = 0;
    }
    if (bias != 0) {
        g_last_bias = bias;//丢线处理会用到，现在暂时没用
    }
    int left_target_speed = limit_speed(base_speed + bias);
    int right_target_speed = limit_speed(base_speed - bias);

    //直线寻迹增量PID
    int encoder_ratio = 6;//编码器计数与电机速度的比值
    float kp_l = 0.2f;
    float ki_l = 0.01f;
    float kd_l = 0.0f;
    float kp_r = 0.2f;
    float ki_r = 0.01f;
    float kd_r = 0.0f;
    int output_max = 1000;
    int output_min = -1000;
    float Pout, Iout, Dout, delta_output;
    if (Encoder_HasNewSpeed() != 0U) {
        int left_target_encoder = left_target_speed * encoder_ratio;
        int right_target_encoder = right_target_speed * encoder_ratio;
        int left_real_encoder = Encoder_GetNowVA();
        int right_real_encoder = Encoder_GetNowVB();
        error_l = (float)(left_target_encoder - left_real_encoder);
        //增量式PID的三项
        Pout = kp_l * (error_l - last_error_l);
        Iout = ki_l * error_l;
        Dout = kd_l * (error_l - 2.0f * last_error_l + last_last_error_l);
        delta_output = Pout + Iout + Dout;
        left_output_speed += (int)delta_output;
        if (left_output_speed > output_max) {
            left_output_speed = output_max;
        }
        if (left_output_speed < output_min) {
            left_output_speed = output_min;
        }
        last_last_error_l = last_error_l;
        last_error_l = error_l;
        error_r = (float)(right_target_encoder - right_real_encoder);
        Pout = kp_r * (error_r - last_error_r);
        Iout = ki_r * error_r;
        Dout = kd_r * (error_r - 2.0f * last_error_r + last_last_error_r);
        delta_output = Pout + Iout + Dout;
        right_output_speed += (int)delta_output;
        if (right_output_speed > output_max) {
            right_output_speed = output_max;
        }
        if (right_output_speed < output_min) {
            right_output_speed = output_min;
        }
        last_last_error_r = last_error_r;
        last_error_r = error_r;
        Encoder_ClearNewSpeedFlag();
    }

    motorA(left_output_speed);//PID的输出结果
    motorB(right_output_speed);
}

void t4(void)
{
    static uint8_t state = 0U;//状态机状态
    static uint32_t last_stepper_ms = 0U;       // 上一次发送步进电机角度的时间
    static float pos_integral = 0.0f;           // 位置误差积分
    static float last_predict_error = 0.0f;     // 上一次预测误差

    uint32_t now_ms = SoundLight_GetSysMs();
    uint32_t stepper_period_ms = 20U;           // 步进电机发送周期
    uint8_t stepper_update = 0U;                // 是否更新步进电机
    if ((now_ms - last_stepper_ms) >= stepper_period_ms) {
        stepper_update = 1U;
        last_stepper_ms = now_ms;
    }

    if (Camera_HasBall() == 0U) {
        pos_integral = 0.0f;
        last_predict_error = 0.0f;
        if (stepper_update != 0U) {
            Stepper_SetAngle(0.0f);
        }
    } 
    else {
        float target_pos = 0.0f;                    // 目标位置，单位cm，后续可改成5或-5
        float kv_pos = 0.5f;                       // 速度提前系数，现场调
        float kp_pos = 5.0f;                        // 预测误差P项系数
        float ki_pos = 0.0f;                        // 位置积分I项系数
        float kd_pos = 0.0f;                        // 预测误差D项系数
        float pos_integral_limit = 800.0f;          // 位置积分限幅
        float rod_angle_limit = 50.0f;              // 摆杆目标角度限幅

        float ball_pos = Camera_GetBallError();     // 小球当前位置误差
        float ball_speed = Camera_GetBallSpeed();   // 小球当前速度
        float ball_error = ball_pos - target_pos;   // 真实位置误差
        float predict_error = ball_error + kv_pos * ball_speed;// 预测误差

        pos_integral += ball_error;
        if (pos_integral > pos_integral_limit) {
            pos_integral = pos_integral_limit;
        }
        if (pos_integral < -pos_integral_limit) {
            pos_integral = -pos_integral_limit;
        }

        float p_out = kp_pos * predict_error;
        float i_out = ki_pos * pos_integral;
        float d_out = kd_pos * (predict_error - last_predict_error);
        float pid_out = p_out + i_out + d_out;
        float rod_angle = -pid_out;                 // 正位置误差时，摆杆给负角度压回来

        if (rod_angle > rod_angle_limit) {
            rod_angle = rod_angle_limit;
        }
        if (rod_angle < -rod_angle_limit) {
            rod_angle = -rod_angle_limit;
        }

        last_predict_error = predict_error;
        if (stepper_update != 0U) {
            Stepper_SetAngle(rod_angle);
        }
    }

/*****************************分*******************割*************************线***************************/

    static int soft_reset = 1;//软启动状态
    static uint8_t last_all_white = 1U;//上一拍是否全白，用来做边沿触发

    static float error_l = 0.0f;
    static float last_error_l = 0.0f;
    static float last_last_error_l = 0.0f;
    static float error_r = 0.0f;
    static float last_error_r = 0.0f;
    static float last_last_error_r = 0.0f;
    static int left_output_speed = 0;
    static int right_output_speed = 0;

    uint8_t d1 = D1;
    uint8_t d2 = D2;
    uint8_t d3 = D3;
    uint8_t d4 = D4;
    uint8_t d5 = D5;
    uint8_t d6 = D6;
    uint8_t d7 = D7;
    uint8_t d8 = D8;
    uint8_t black_count = 0U;
    if (is_black(d1)) black_count++;
    if (is_black(d2)) black_count++;
    if (is_black(d3)) black_count++;
    if (is_black(d4)) black_count++;
    if (is_black(d5)) black_count++;
    if (is_black(d6)) black_count++;
    if (is_black(d7)) black_count++;
    if (is_black(d8)) black_count++;
    //全黑
    uint8_t all_black = ((d1 == 0U) && (d2 == 0U) &&
        (d3 == 0U) && (d4 == 0U) &&
        (d5 == 0U) && (d6 == 0U) &&
        (d7 == 0U) && (d8 == 0U)) ? 1U : 0U;

    Encoder_GetDistanceCount();//编码器开始计数
    //停止条件
    if(state == 2){
        motors_stop();
        return;
    }
    uint32_t decel_start_count = 150000U;//到半弯道开始减速
    if((state == 0) && (Encoder_GetDistanceCount() >= decel_start_count) ){
        state = 1;
    }
    if((state == 1) && (Encoder_GetDistanceCount()>=1810000)){
        SoundLight_TimeStop();
        motors_stop();
        state = 2;
        return;
    }
    int target_base_speed = 300;//要加速到的最终速度
    if(state == 1){
        target_base_speed = 0;//减速到的速度
    }

 /***********软启动-->缓慢加速到设定速度防止翘头************/   
    
    static uint32_t last_soft_ms = 0U;   // 上一次软启动更新时间
    static int base_speed = 0;           // 当前基础速度

    int start_base_speed = 50;           // 软启动初始速度
    int soft_step = 5;                   // 每次加减速度
    uint32_t soft_period_ms = 30U;       // 每20ms更新一次速度

    if (soft_reset != 0) {
        base_speed = start_base_speed;
        last_soft_ms = now_ms;
        soft_reset = 0;
    }

    if ((now_ms - last_soft_ms) >= soft_period_ms) {
        if (base_speed < target_base_speed) {
            base_speed += soft_step;
            if (base_speed > target_base_speed) {
                base_speed = target_base_speed;
            }
        } else if (base_speed > target_base_speed) {
            base_speed -= soft_step;
            if (base_speed < target_base_speed) {
                base_speed = target_base_speed;
            }
        }

        last_soft_ms = now_ms;
    }
/**********************************************************/

    //由灰度得到的左右轮位置环差速
    int bias;
    int d5_bias = (int)(base_speed * 0.1f);//直线寻迹时，中间几路差速调小一些
    int d6_bias = (int)(base_speed * 0.2f);
    int d7_bias = (int)(base_speed * 0.3f);
    int d8_bias = (int)(base_speed * 0.4f);
    if (is_black(d4) && is_black(d5)) {
        bias = 0;
    } else if (is_black(d4)) {
        bias = -d5_bias;
    } else if (is_black(d5)) {
        bias = d5_bias;
    } else if (is_black(d3)) {
        bias = -d6_bias;
    } else if (is_black(d6)) {
        bias = d6_bias;
    } else if (is_black(d2)) {
        bias = -d7_bias;
    } else if (is_black(d7)) {
        bias = d7_bias;
    } else if (is_black(d1)) {
        bias = -d8_bias;
    } else if (is_black(d8)) {
        bias = d8_bias;
    } else {
        bias = 0;
    }
    if (bias != 0) {
        g_last_bias = bias;//丢线处理会用到，现在暂时没用
    }
    int left_target_speed = limit_speed(base_speed + bias);
    int right_target_speed = limit_speed(base_speed - bias);

    //直线寻迹增量PID
    int encoder_ratio = 6;//编码器计数与电机速度的比值
    float kp_l = 0.2f;
    float ki_l = 0.01f;
    float kd_l = 0.0f;
    float kp_r = 0.2f;
    float ki_r = 0.01f;
    float kd_r = 0.0f;
    int output_max = 1000;
    int output_min = -1000;
    float Pout, Iout, Dout, delta_output;
    if (Encoder_HasNewSpeed() != 0U) {
        int left_target_encoder = left_target_speed * encoder_ratio;
        int right_target_encoder = right_target_speed * encoder_ratio;
        int left_real_encoder = Encoder_GetNowVA();
        int right_real_encoder = Encoder_GetNowVB();
        error_l = (float)(left_target_encoder - left_real_encoder);
        //增量式PID的三项
        Pout = kp_l * (error_l - last_error_l);
        Iout = ki_l * error_l;
        Dout = kd_l * (error_l - 2.0f * last_error_l + last_last_error_l);
        delta_output = Pout + Iout + Dout;
        left_output_speed += (int)delta_output;
        if (left_output_speed > output_max) {
            left_output_speed = output_max;
        }
        if (left_output_speed < output_min) {
            left_output_speed = output_min;
        }
        last_last_error_l = last_error_l;
        last_error_l = error_l;
        error_r = (float)(right_target_encoder - right_real_encoder);
        Pout = kp_r * (error_r - last_error_r);
        Iout = ki_r * error_r;
        Dout = kd_r * (error_r - 2.0f * last_error_r + last_last_error_r);
        delta_output = Pout + Iout + Dout;
        right_output_speed += (int)delta_output;
        if (right_output_speed > output_max) {
            right_output_speed = output_max;
        }
        if (right_output_speed < output_min) {
            right_output_speed = output_min;
        }
        last_last_error_r = last_error_r;
        last_error_r = error_r;
        Encoder_ClearNewSpeedFlag();
    }

    motorA(left_output_speed);//PID的输出结果
    motorB(right_output_speed);

}


void t4_weizhi(void)
{
    static uint8_t state = 0U;                 // 小车状态：0加速，1匀速，2减速
    static int soft_reset = 1;                 // 软加速初始化标志
    static uint8_t first_check = 1U;           // 第一次运行标志
    static uint32_t last_run_ms = 0U;          // 上一次行驶计时
    static uint32_t last_stepper_ms = 0U;      // 上一次发送步进电机角度的时间
    static uint32_t last_soft_ms = 0U;         // 上一次软加减速更新时间
    static float pos_integral = 0.0f;          // 小球位置误差积分
    static float last_predict_error = 0.0f;    // 上一次预测误差
    static int base_speed = 0;                 // 当前基础速度

    uint32_t now_ms = SoundLight_GetSysMs();
    uint32_t run_ms = SoundLight_GetTimeMs();

    if ((first_check != 0U) || (run_ms < last_run_ms)) {
        state = 0U;
        soft_reset = 1;
        last_stepper_ms = 0U;
        last_soft_ms = 0U;
        pos_integral = 0.0f;
        last_predict_error = 0.0f;
        base_speed = 0;

        g_speed_pid_left_integral = 0.0f;
        g_speed_pid_right_integral = 0.0f;
        g_speed_pid_left_last_error = 0.0f;
        g_speed_pid_right_last_error = 0.0f;
        g_speed_pid_left_output = 0;
        g_speed_pid_right_output = 0;

        first_check = 0U;
    }
    last_run_ms = run_ms;

    uint32_t stepper_period_ms = 20U;          // 步进电机发送周期
    uint8_t stepper_update = 0U;               // 是否更新步进电机
    if ((now_ms - last_stepper_ms) >= stepper_period_ms) {
        stepper_update = 1U;
        last_stepper_ms = now_ms;
    }

    if (Camera_HasBall() == 0U) {
        pos_integral = 0.0f;
        last_predict_error = 0.0f;
        if (stepper_update != 0U) {
            Stepper_SetAngle(0.0f);
        }
    } else {
        float target_pos = 0.0f;               // 小球目标位置，单位cm
        float kv_pos = 0.53f;                  // 速度提前系数
        float kp_pos = 5.0f;                  // 摆杆P参数
        float ki_pos = 0.0f;                  // 摆杆I参数
        float kd_pos = 0.0f;                  // 摆杆D参数

        if (state == 0U) {
            kp_pos = 6.5f;                    // 加速阶段摆杆P
            ki_pos = 0.0f;                    // 加速阶段摆杆I
            kd_pos = 0.0f;                    // 加速阶段摆杆D
        } else if (state == 1U) {
            kp_pos = 5.0f;                    // 匀速阶段摆杆P
            ki_pos = 0.0f;                    // 匀速阶段摆杆I
            kd_pos = 0.0f;                    // 匀速阶段摆杆D
        } else if (state == 2U) {
            kv_pos = 0.55f; 
            kp_pos = 6.3f;                    // 减速阶段摆杆P
            ki_pos = 0.0f;                    // 减速阶段摆杆I
            kd_pos = 0.0f;                   // 减速阶段摆杆D
        }

        float pos_integral_limit = 800.0f;     // 位置积分限幅
        float rod_angle_limit = 50.0f;         // 摆杆目标角度限幅
        float ball_pos = Camera_GetBallError();// 小球当前位置误差
        float ball_speed = Camera_GetBallSpeed();// 小球当前速度
        float ball_error = ball_pos - target_pos;
        float predict_error = ball_error + kv_pos * ball_speed;

        pos_integral += ball_error;
        if (pos_integral > pos_integral_limit) {
            pos_integral = pos_integral_limit;
        }
        if (pos_integral < -pos_integral_limit) {
            pos_integral = -pos_integral_limit;
        }

        float p_out = kp_pos * predict_error;
        float i_out = ki_pos * pos_integral;
        float d_out = kd_pos * (predict_error - last_predict_error);
        float pid_out = p_out + i_out + d_out;
        float rod_angle = -pid_out;

        if (rod_angle > rod_angle_limit) {
            rod_angle = rod_angle_limit;
        }
        if (rod_angle < -rod_angle_limit) {
            rod_angle = -rod_angle_limit;
        }

        last_predict_error = predict_error;
        if (stepper_update != 0U) {
            Stepper_SetAngle(rod_angle);
        }
    }

/*****************************分*******************割*************************线***************************/

    uint8_t d1 = D1;
    uint8_t d2 = D2;
    uint8_t d3 = D3;
    uint8_t d4 = D4;
    uint8_t d5 = D5;
    uint8_t d6 = D6;
    uint8_t d7 = D7;
    uint8_t d8 = D8;

    Encoder_GetDistanceCount();

    uint32_t decel_start_count = 150000U;      // 开始软减速的编码器计数
    if ((state == 1U) && (Encoder_GetDistanceCount() >= decel_start_count)) {
        state = 2U;
    }

    int run_base_speed = 250;                  // 匀速阶段目标基础速度
    int target_base_speed = run_base_speed;    // 当前目标基础速度

    if (state == 2U) {
        target_base_speed = 0;
    }

    int start_base_speed = 5;                 // 软启动初始速度
    int soft_step = 5;                         // 每次加减速度
    uint32_t soft_period_ms = 60U;             // 每60ms更新一次速度

    if (soft_reset != 0) {
        base_speed = start_base_speed;
        last_soft_ms = now_ms;
        soft_reset = 0;
    }

    if ((now_ms - last_soft_ms) >= soft_period_ms) {
        if (base_speed < target_base_speed) {
            base_speed += soft_step;
            if (base_speed > target_base_speed) {
                base_speed = target_base_speed;
            }
        } else if (base_speed > target_base_speed) {
            base_speed -= soft_step;
            if (base_speed < target_base_speed) {
                base_speed = target_base_speed;
            }
        }

        last_soft_ms = now_ms;
    }

    if ((state == 0U) && (base_speed >= run_base_speed)) {
        state = 1U;
    }

    if ((state == 2U) && (base_speed <= 0)) {
        motors_stop();
        return;
    }

    int d5_bias = (int)(base_speed * 0.1f);
    int d6_bias = (int)(base_speed * 0.2f);
    int d7_bias = (int)(base_speed * 0.3f);
    int d8_bias = (int)(base_speed * 0.4f);
    int bias = 0;

    if (is_black(d4) && is_black(d5)) {
        bias = 0;
    } else if (is_black(d4)) {
        bias = -d5_bias;
    } else if (is_black(d5)) {
        bias = d5_bias;
    } else if (is_black(d3)) {
        bias = -d6_bias;
    } else if (is_black(d6)) {
        bias = d6_bias;
    } else if (is_black(d2)) {
        bias = -d7_bias;
    } else if (is_black(d7)) {
        bias = d7_bias;
    } else if (is_black(d1)) {
        bias = -d8_bias;
    } else if (is_black(d8)) {
        bias = d8_bias;
    }

    if (bias != 0) {
        g_last_bias = bias;
    }

    int left_target_speed = limit_speed(base_speed + bias);
    int right_target_speed = limit_speed(base_speed - bias);

    int encoder_ratio = 6;                     // 编码器计数与电机速度的比值

    if (Encoder_HasNewSpeed() != 0U) {
        int left_target_encoder = left_target_speed * encoder_ratio;
        int right_target_encoder = right_target_speed * encoder_ratio;
        int left_real_encoder = Encoder_GetNowVA();
        int right_real_encoder = Encoder_GetNowVB();

        g_speed_pid_left_output = SpeedPID_Left(left_target_encoder, left_real_encoder, left_target_speed);
        g_speed_pid_right_output = SpeedPID_Right(right_target_encoder, right_real_encoder, right_target_speed);

        Encoder_ClearNewSpeedFlag();
    }

    motorA(g_speed_pid_left_output);
    motorB(g_speed_pid_right_output);
}

void t5_weizhi(void)
{
    static uint8_t state = 0U;                 // 小车状态：0加速，1匀速，2减速
    static int soft_reset = 1;                 // 软加速初始化标志
    static uint8_t first_check = 1U;           // 第一次运行标志
    static uint32_t last_run_ms = 0U;          // 上一次行驶计时
    static uint32_t last_stepper_ms = 0U;      // 上一次发送步进电机角度的时间
    static uint32_t last_soft_ms = 0U;         // 上一次软加减速更新时间
    static float pos_integral = 0.0f;          // 小球位置误差积分
    static float last_predict_error = 0.0f;    // 上一次预测误差
    static int base_speed = 0;                 // 当前基础速度

    uint32_t now_ms = SoundLight_GetSysMs();
    uint32_t run_ms = SoundLight_GetTimeMs();

    if ((first_check != 0U) || (run_ms < last_run_ms)) {
        state = 0U;
        soft_reset = 1;
        last_stepper_ms = 0U;
        last_soft_ms = 0U;
        pos_integral = 0.0f;
        last_predict_error = 0.0f;
        base_speed = 0;

        g_speed_pid_left_integral = 0.0f;
        g_speed_pid_right_integral = 0.0f;
        g_speed_pid_left_last_error = 0.0f;
        g_speed_pid_right_last_error = 0.0f;
        g_speed_pid_left_output = 0;
        g_speed_pid_right_output = 0;

        first_check = 0U;
    }
    last_run_ms = run_ms;

    uint32_t stepper_period_ms = 20U;          // 步进电机发送周期
    uint8_t stepper_update = 0U;               // 是否更新步进电机
    if ((now_ms - last_stepper_ms) >= stepper_period_ms) {
        stepper_update = 1U;
        last_stepper_ms = now_ms;
    }

    if (Camera_HasBall() == 0U) {
        pos_integral = 0.0f;
        last_predict_error = 0.0f;
        if (stepper_update != 0U) {
            Stepper_SetAngle(0.0f);
        }
    } else {
        float target_pos = 0.0f;               // 小球目标位置，单位cm
        float kv_pos = 0.53f;                  // 速度提前系数
        float kp_pos = 5.0f;                  // 摆杆P参数
        float ki_pos = 0.0f;                  // 摆杆I参数
        float kd_pos = 0.0f;                  // 摆杆D参数

        if (state == 0U) {
            kp_pos = 6.5f;                    // 加速阶段摆杆P
            ki_pos = 0.0f;                    // 加速阶段摆杆I
            kd_pos = 0.0f;                    // 加速阶段摆杆D
        } else if (state == 1U) {
            kp_pos = 5.5f;                    // 匀速阶段摆杆P
            ki_pos = 0.0f;                    // 匀速阶段摆杆I
            kd_pos = 0.0f;                    // 匀速阶段摆杆D
        } else if (state == 2U) {
            kv_pos = 0.58f;
            kp_pos = 6.5f;                    // 减速阶段摆杆P
            ki_pos = 0.0f;                    // 减速阶段摆杆I
            kd_pos = 0.0f;                   // 减速阶段摆杆D
        }

        float pos_integral_limit = 800.0f;     // 位置积分限幅
        float rod_angle_limit = 50.0f;         // 摆杆目标角度限幅
        float ball_pos = Camera_GetBallError();// 小球当前位置误差
        float ball_speed = Camera_GetBallSpeed();// 小球当前速度
        float ball_error = ball_pos - target_pos;
        float predict_error = ball_error + kv_pos * ball_speed;

        pos_integral += ball_error;
        if (pos_integral > pos_integral_limit) {
            pos_integral = pos_integral_limit;
        }
        if (pos_integral < -pos_integral_limit) {
            pos_integral = -pos_integral_limit;
        }

        float p_out = kp_pos * predict_error;
        float i_out = ki_pos * pos_integral;
        float d_out = kd_pos * (predict_error - last_predict_error);
        float pid_out = p_out + i_out + d_out;
        float rod_angle = -pid_out;

        if (rod_angle > rod_angle_limit) {
            rod_angle = rod_angle_limit;
        }
        if (rod_angle < -rod_angle_limit) {
            rod_angle = -rod_angle_limit;
        }

        last_predict_error = predict_error;
        if (stepper_update != 0U) {
            Stepper_SetAngle(rod_angle);
        }
    }

/*****************************分*******************割*************************线***************************/

    uint8_t d1 = D1;
    uint8_t d2 = D2;
    uint8_t d3 = D3;
    uint8_t d4 = D4;
    uint8_t d5 = D5;
    uint8_t d6 = D6;
    uint8_t d7 = D7;
    uint8_t d8 = D8;

    Encoder_GetDistanceCount();

    uint32_t decel_start_count = 688000U;      // 开始软减速的编码器计数
    if ((state == 1U) && (Encoder_GetDistanceCount() >= decel_start_count)) {
        state = 2U;
    }

    int run_base_speed = 230;                  // 匀速阶段目标基础速度
    int target_base_speed = run_base_speed;    // 当前目标基础速度

    if (state == 2U) {
        target_base_speed = 0;
    }

    int start_base_speed = 5;                 // 软启动初始速度
    int soft_step = 5;                         // 每次加减速度
    uint32_t soft_period_ms = 60U;             // 每20ms更新一次速度

    if (soft_reset != 0) {
        base_speed = start_base_speed;
        last_soft_ms = now_ms;
        soft_reset = 0;
    }

    if ((now_ms - last_soft_ms) >= soft_period_ms) {
        if (base_speed < target_base_speed) {
            base_speed += soft_step;
            if (base_speed > target_base_speed) {
                base_speed = target_base_speed;
            }
        } else if (base_speed > target_base_speed) {
            base_speed -= soft_step;
            if (base_speed < target_base_speed) {
                base_speed = target_base_speed;
            }
        }

        last_soft_ms = now_ms;
    }

    if ((state == 0U) && (base_speed >= run_base_speed)) {
        state = 1U;
    }

    if ((state == 2U) && (base_speed <= 0)) {
        motors_stop();
        return;
    }

    int d5_bias = (int)(base_speed * 0.1f);
    int d6_bias = (int)(base_speed * 0.2f);
    int d7_bias = (int)(base_speed * 0.3f);
    int d8_bias = (int)(base_speed * 0.4f);
    int bias = 0;

    if (is_black(d4) && is_black(d5)) {
        bias = 0;
    } else if (is_black(d4)) {
        bias = -d5_bias;
    } else if (is_black(d5)) {
        bias = d5_bias;
    } else if (is_black(d3)) {
        bias = -d6_bias;
    } else if (is_black(d6)) {
        bias = d6_bias;
    } else if (is_black(d2)) {
        bias = -d7_bias;
    } else if (is_black(d7)) {
        bias = d7_bias;
    } else if (is_black(d1)) {
        bias = -d8_bias;
    } else if (is_black(d8)) {
        bias = d8_bias;
    }

    if (bias != 0) {
        g_last_bias = bias;
    }

    int left_target_speed = limit_speed(base_speed + bias);
    int right_target_speed = limit_speed(base_speed - bias);

    int encoder_ratio = 6;                     // 编码器计数与电机速度的比值

    if (Encoder_HasNewSpeed() != 0U) {
        int left_target_encoder = left_target_speed * encoder_ratio;
        int right_target_encoder = right_target_speed * encoder_ratio;
        int left_real_encoder = Encoder_GetNowVA();
        int right_real_encoder = Encoder_GetNowVB();

        g_speed_pid_left_output = SpeedPID_Left(left_target_encoder, left_real_encoder, left_target_speed);
        g_speed_pid_right_output = SpeedPID_Right(right_target_encoder, right_real_encoder, right_target_speed);

        Encoder_ClearNewSpeedFlag();
    }

    motorA(g_speed_pid_left_output);
    motorB(g_speed_pid_right_output);
}

void t5(void)
{
    static uint8_t state = 0U;//状态机状态
    static int soft_reset = 1;//软启动状态
    static uint8_t last_all_white = 1U;//上一拍是否全白，用来做边沿触发
    static uint8_t first_check = 1U;           // t5第一次运行标志
    static uint32_t last_run_ms = 0U;          // 上一次行驶计时，用来判断重新启动
    static uint32_t last_stepper_ms = 0U;       // 上一次发送步进电机角度的时间
    static float pos_integral = 0.0f;           // 位置误差积分
    static float last_predict_error = 0.0f;     // 上一次预测误差
    static float error_l = 0.0f;
    static float last_error_l = 0.0f;
    static float last_last_error_l = 0.0f;
    static float error_r = 0.0f;
    static float last_error_r = 0.0f;
    static float last_last_error_r = 0.0f;
    static int left_output_speed = 0;
    static int right_output_speed = 0;
    static uint32_t last_soft_ms = 0U;          // 上一次软启动更新时间
    static int base_speed = 0;                  // 当前基础速度

    uint32_t now_ms = SoundLight_GetSysMs();
    uint32_t run_ms = SoundLight_GetTimeMs();
    if ((first_check != 0U) || (run_ms < last_run_ms)) {
        state = 0U;
        soft_reset = 1;
        last_all_white = 1U;
        last_stepper_ms = 0U;
        pos_integral = 0.0f;
        last_predict_error = 0.0f;
        error_l = 0.0f;
        last_error_l = 0.0f;
        last_last_error_l = 0.0f;
        error_r = 0.0f;
        last_error_r = 0.0f;
        last_last_error_r = 0.0f;
        left_output_speed = 0;
        right_output_speed = 0;
        last_soft_ms = 0U;
        base_speed = 0;
        first_check = 0U;
    }
    last_run_ms = run_ms;

    uint32_t stepper_period_ms = 20U;           // 步进电机发送周期
    uint8_t stepper_update = 0U;                // 是否更新步进电机
    if ((now_ms - last_stepper_ms) >= stepper_period_ms) {
        stepper_update = 1U;
        last_stepper_ms = now_ms;
    }

    if (Camera_HasBall() == 0U) {
        pos_integral = 0.0f;
        last_predict_error = 0.0f;
        if (stepper_update != 0U) {
            Stepper_SetAngle(0.0f);
        }
    } else {
        float target_pos = 0.0f;                    // 目标位置，单位cm，后续可改成5或-5
        float kv_pos = 0.5f;                       // 速度提前系数，现场调
        float kp_pos = 5.0f;                        // 预测误差P项系数
        float ki_pos = 0.0f;                        // 位置积分I项系数
        float kd_pos = 0.0f;                        // 预测误差D项系数
        float pos_integral_limit = 800.0f;          // 位置积分限幅
        float rod_angle_limit = 50.0f;              // 摆杆目标角度限幅

        float ball_pos = Camera_GetBallError();     // 小球当前位置误差
        float ball_speed = Camera_GetBallSpeed();   // 小球当前速度
        float ball_error = ball_pos - target_pos;   // 真实位置误差
        float predict_error = ball_error + kv_pos * ball_speed;// 预测误差

        pos_integral += ball_error;
        if (pos_integral > pos_integral_limit) {
            pos_integral = pos_integral_limit;
        }
        if (pos_integral < -pos_integral_limit) {
            pos_integral = -pos_integral_limit;
        }

        float p_out = kp_pos * predict_error;
        float i_out = ki_pos * pos_integral;
        float d_out = kd_pos * (predict_error - last_predict_error);
        float pid_out = p_out + i_out + d_out;
        float rod_angle = -pid_out;                 // 正位置误差时，摆杆给负角度压回来

        if (rod_angle > rod_angle_limit) {
            rod_angle = rod_angle_limit;
        }
        if (rod_angle < -rod_angle_limit) {
            rod_angle = -rod_angle_limit;
        }

        last_predict_error = predict_error;
        if (stepper_update != 0U) {
            Stepper_SetAngle(rod_angle);
        }
    }

/*****************************分*******************割*************************线***************************/

    uint8_t d1 = D1;
    uint8_t d2 = D2;
    uint8_t d3 = D3;
    uint8_t d4 = D4;
    uint8_t d5 = D5;
    uint8_t d6 = D6;
    uint8_t d7 = D7;
    uint8_t d8 = D8;
    uint8_t black_count = 0U;
    if (is_black(d1)) black_count++;
    if (is_black(d2)) black_count++;
    if (is_black(d3)) black_count++;
    if (is_black(d4)) black_count++;
    if (is_black(d5)) black_count++;
    if (is_black(d6)) black_count++;
    if (is_black(d7)) black_count++;
    if (is_black(d8)) black_count++;
    //全黑
    uint8_t all_black = ((d1 == 0U) && (d2 == 0U) &&
        (d3 == 0U) && (d4 == 0U) &&
        (d5 == 0U) && (d6 == 0U) &&
        (d7 == 0U) && (d8 == 0U)) ? 1U : 0U;

    Encoder_GetDistanceCount();//编码器开始计数
    //停止条件
    if(state == 2){
        motors_stop();
        return;
    }
    uint32_t decel_start_count = 688000U;//到半弯道开始减速
    if((state == 0) && (Encoder_GetDistanceCount() >= decel_start_count) ){
        state = 1;
    }
    if((state == 1) && ( Encoder_GetDistanceCount()>=6880000)){
        SoundLight_TimeStop();
        motors_stop();
        state = 2;
        return;
    }
    int target_base_speed = 200;//要加度到的最终速度
    if(state == 1){
        target_base_speed = 0;
    }

    //软启动-->按固定时间周期缓慢加减速，防止主循环快慢影响速度变化
    int start_base_speed = 10;//软启动初始速度
    int soft_step = 5;//每次加减速度
    uint32_t soft_period_ms = 20U;//每20ms更新一次速度

    if (soft_reset != 0) {
        base_speed = start_base_speed;
        last_soft_ms = now_ms;
        soft_reset = 0;
    }

    if ((now_ms - last_soft_ms) >= soft_period_ms) {
        if (base_speed < target_base_speed) {
            base_speed += soft_step;
            if (base_speed > target_base_speed) {
                base_speed = target_base_speed;
            }
        } else if (base_speed > target_base_speed) {
            base_speed -= soft_step;
            if (base_speed < target_base_speed) {
                base_speed = target_base_speed;
            }
        }

        last_soft_ms = now_ms;
    }

    //由灰度得到的左右轮位置环差速
    int bias;
    int d5_bias = (int)(base_speed * 0.1f);//直线寻迹时，中间几路差速调小一些
    int d6_bias = (int)(base_speed * 0.2f);
    int d7_bias = (int)(base_speed * 0.3f);
    int d8_bias = (int)(base_speed * 0.4f);
    if (is_black(d4) && is_black(d5)) {
        bias = 0;
    } else if (is_black(d4)) {
        bias = -d5_bias;
    } else if (is_black(d5)) {
        bias = d5_bias;
    } else if (is_black(d3)) {
        bias = -d6_bias;
    } else if (is_black(d6)) {
        bias = d6_bias;
    } else if (is_black(d2)) {
        bias = -d7_bias;
    } else if (is_black(d7)) {
        bias = d7_bias;
    } else if (is_black(d1)) {
        bias = -d8_bias;
    } else if (is_black(d8)) {
        bias = d8_bias;
    } else {
        bias = 0;
    }
    if (bias != 0) {
        g_last_bias = bias;//丢线处理会用到，现在暂时没用
    }
    int left_target_speed = limit_speed(base_speed + bias);
    int right_target_speed = limit_speed(base_speed - bias);

    //直线寻迹增量PID
    int encoder_ratio = 6;//编码器计数与电机速度的比值
    float kp_l = 0.2f;
    float ki_l = 0.01f;
    float kd_l = 0.0f;
    float kp_r = 0.2f;
    float ki_r = 0.01f;
    float kd_r = 0.0f;
    int output_max = 1000;
    int output_min = -1000;
    float Pout, Iout, Dout, delta_output;
    if (Encoder_HasNewSpeed() != 0U) {
        int left_target_encoder = left_target_speed * encoder_ratio;
        int right_target_encoder = right_target_speed * encoder_ratio;
        int left_real_encoder = Encoder_GetNowVA();
        int right_real_encoder = Encoder_GetNowVB();
        error_l = (float)(left_target_encoder - left_real_encoder);
        //增量式PID的三项
        Pout = kp_l * (error_l - last_error_l);
        Iout = ki_l * error_l;
        Dout = kd_l * (error_l - 2.0f * last_error_l + last_last_error_l);
        delta_output = Pout + Iout + Dout;
        left_output_speed += (int)delta_output;
        if (left_output_speed > output_max) {
            left_output_speed = output_max;
        }
        if (left_output_speed < output_min) {
            left_output_speed = output_min;
        }
        last_last_error_l = last_error_l;
        last_error_l = error_l;
        error_r = (float)(right_target_encoder - right_real_encoder);
        Pout = kp_r * (error_r - last_error_r);
        Iout = ki_r * error_r;
        Dout = kd_r * (error_r - 2.0f * last_error_r + last_last_error_r);
        delta_output = Pout + Iout + Dout;
        right_output_speed += (int)delta_output;
        if (right_output_speed > output_max) {
            right_output_speed = output_max;
        }
        if (right_output_speed < output_min) {
            right_output_speed = output_min;
        }
        last_last_error_r = last_error_r;
        last_error_r = error_r;
        Encoder_ClearNewSpeedFlag();
    }

    motorA(left_output_speed);//PID的输出结果
    motorB(right_output_speed);

}

void t6(void)
{
    static uint8_t state = 0U;                 // 小车状态：0加速，1匀速，2减速
    static int soft_reset = 1;                 // 软加速初始化标志
    static uint8_t first_check = 1U;           // 第一次运行标志
    static uint32_t last_run_ms = 0U;          // 上一次行驶计时
    static uint32_t last_stepper_ms = 0U;      // 上一次发送步进电机角度的时间
    static uint32_t last_soft_ms = 0U;         // 上一次软加减速更新时间
    static float pos_integral = 0.0f;          // 小球位置误差积分
    static float last_predict_error = 0.0f;    // 上一次预测误差
    static int base_speed = 0;                 // 当前基础速度

    uint32_t now_ms = SoundLight_GetSysMs();
    uint32_t run_ms = SoundLight_GetTimeMs();

    if ((first_check != 0U) || (run_ms < last_run_ms)) {
        state = 0U;
        soft_reset = 1;
        last_stepper_ms = 0U;
        last_soft_ms = 0U;
        pos_integral = 0.0f;
        last_predict_error = 0.0f;
        base_speed = 0;

        g_speed_pid_left_integral = 0.0f;
        g_speed_pid_right_integral = 0.0f;
        g_speed_pid_left_last_error = 0.0f;
        g_speed_pid_right_last_error = 0.0f;
        g_speed_pid_left_output = 0;
        g_speed_pid_right_output = 0;

        first_check = 0U;
    }
    last_run_ms = run_ms;

    uint32_t stepper_period_ms = 20U;          // 步进电机发送周期
    uint8_t stepper_update = 0U;               // 是否更新步进电机
    if ((now_ms - last_stepper_ms) >= stepper_period_ms) {
        stepper_update = 1U;
        last_stepper_ms = now_ms;
    }

    if (Camera_HasBall() == 0U) {
        pos_integral = 0.0f;
        last_predict_error = 0.0f;
        if (stepper_update != 0U) {
            Stepper_SetAngle(0.0f);
        }
    } 
    else {
        float ball_error = Camera_GetBallError();       // 当前位置-目标位置
        float ball_abs_pos = Camera_GetBallAbsPos();    // 小球绝对坐标
        float ball_speed = Camera_GetBallSpeed();       // 小球速度
        float target_pos = ball_abs_pos - ball_error;   // 反推出目标位置

        float kv_pos = 0.50f;                  // 速度提前系数
        float kp_pos = 5.0f;                   // 摆杆P参数
        float ki_pos = 0.0f;                   // 摆杆I参数
        float kd_pos = 0.0f;                   // 摆杆D参数
        float rod_angle_limit = 50.0f;         // 摆杆角度限幅

        if ((target_pos >= -12.5f) && (target_pos < -7.5f)) {
            // 负方向边缘区：自由端附近，参数保守一些
            if (state == 0U) {
                kv_pos = 0.49f;
                kp_pos = 5.6f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 45.0f;
            } 
            else if (state == 1U) {
                kv_pos = 0.45f;
                kp_pos = 4.5f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 45.0f;
            } 
            else if (state == 2U) {
                kv_pos = 0.48f;
                kp_pos = 5.5f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 45.0f;
            }
        } 
        else if ((target_pos >= -7.5f) && (target_pos < -2.5f)) {
            // 负方向中间区：比自由端积极一点
            if (state == 0U) {
                kv_pos = 0.53f;
                kp_pos = 6.2f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 48.0f;
            } 
            else if (state == 1U) {
                kv_pos = 0.49f;
                kp_pos = 5.0f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 48.0f;
            } 
            else if (state == 2U) {
                kv_pos = 0.51f;
                kp_pos = 6.2f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 48.0f;
            }
        } 
        else if ((target_pos >= -2.5f) && (target_pos <= 2.5f)) {
            // 中心区：目标接近0cm，用你原来0点附近稳定的参数
            if (state == 0U) {
                kv_pos = 0.58f;
                kp_pos = 7.0f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 50.0f;
            } else if (state == 1U) {
                kv_pos = 0.53f;
                kp_pos = 5.5f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 50.0f;
            } else if (state == 2U) {
                kv_pos = 0.58f;
                kp_pos = 7.0f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 50.0f;
            }
        } 
        else if ((target_pos > 2.5f) && (target_pos <= 7.5f)) {
            // 正方向中间区：靠固定点方向，参数逐步加大，+5cm
            if (state == 0U) {
                kv_pos = 0.66f;
                kp_pos = 8.3f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 52.0f;
            } else if (state == 1U) {
                kv_pos = 0.58f;
                kp_pos = 6.5f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 52.0f;
            } else if (state == 2U) {
                kv_pos = 0.65f;
                kp_pos = 8.3f;
                ki_pos = 0.01f;
                kd_pos = 0.0f;
                rod_angle_limit = 52.0f;
            }
        } 
        else if ((target_pos > 7.5f) && (target_pos <= 12.5f)) {
            // 正方向边缘区：最靠近固定点，P和速度提前量最大
            if (state == 0U) {
                kv_pos = 0.70f;
                kp_pos = 9.0f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 55.0f;
            } else if (state == 1U) {
                kv_pos = 0.63f;
                kp_pos = 7.5f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 55.0f;
            } else if (state == 2U) {
                kv_pos = 0.68f;
                kp_pos = 9.0f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 55.0f;
            }
        } 
        else {
            // 超出标定范围时，先用中心区保守参数兜底
            if (state == 0U) {
                kv_pos = 0.58f;
                kp_pos = 7.0f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 50.0f;
            } else if (state == 1U) {
                kv_pos = 0.53f;
                kp_pos = 5.5f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 50.0f;
            } else if (state == 2U) {
                kv_pos = 0.55f;
                kp_pos = 7.0f;
                ki_pos = 0.0f;
                kd_pos = 0.0f;
                rod_angle_limit = 50.0f;
            }
        }

        float pos_integral_limit = 800.0f;     // 位置积分限幅
        float predict_error = ball_error + kv_pos * ball_speed;

        pos_integral += ball_error;
        if (pos_integral > pos_integral_limit) {
            pos_integral = pos_integral_limit;
        }
        if (pos_integral < -pos_integral_limit) {
            pos_integral = -pos_integral_limit;
        }

        float p_out = kp_pos * predict_error;
        float i_out = ki_pos * pos_integral;
        float d_out = kd_pos * (predict_error - last_predict_error);
        float pid_out = p_out + i_out + d_out;
        float rod_angle = -pid_out;

        if (rod_angle > rod_angle_limit) {
            rod_angle = rod_angle_limit;
        }
        if (rod_angle < -rod_angle_limit) {
            rod_angle = -rod_angle_limit;
        }

        last_predict_error = predict_error;
        if (stepper_update != 0U) {
            Stepper_SetAngle(rod_angle);
        }
    }

/*****************************分*******************割*************************线***************************/

    uint8_t d1 = D1;
    uint8_t d2 = D2;
    uint8_t d3 = D3;
    uint8_t d4 = D4;
    uint8_t d5 = D5;
    uint8_t d6 = D6;
    uint8_t d7 = D7;
    uint8_t d8 = D8;

    Encoder_GetDistanceCount();

    uint32_t decel_start_count = 688000U;      // 开始软减速的编码器计数
    if ((state == 1U) && (Encoder_GetDistanceCount() >= decel_start_count)) {
        state = 2U;
    }

    int run_base_speed = 230;                  // 匀速阶段目标基础速度
    int target_base_speed = run_base_speed;    // 当前目标基础速度

    if (state == 2U) {
        target_base_speed = 0;
    }

    int start_base_speed = 5;                  // 软启动初始速度
    int soft_step = 5;                         // 每次加减速度
    uint32_t soft_period_ms = 60U;             // 每60ms更新一次速度

    if (soft_reset != 0) {
        base_speed = start_base_speed;
        last_soft_ms = now_ms;
        soft_reset = 0;
    }

    if ((now_ms - last_soft_ms) >= soft_period_ms) {
        if (base_speed < target_base_speed) {
            base_speed += soft_step;
            if (base_speed > target_base_speed) {
                base_speed = target_base_speed;
            }
        } else if (base_speed > target_base_speed) {
            base_speed -= soft_step;
            if (base_speed < target_base_speed) {
                base_speed = target_base_speed;
            }
        }

        last_soft_ms = now_ms;
    }

    if ((state == 0U) && (base_speed >= run_base_speed)) {
        state = 1U;
    }

    if ((state == 2U) && (base_speed <= 0)) {
        motors_stop();
        return;
    }

    int d5_bias = (int)(base_speed * 0.1f);
    int d6_bias = (int)(base_speed * 0.2f);
    int d7_bias = (int)(base_speed * 0.3f);
    int d8_bias = (int)(base_speed * 0.4f);
    int bias = 0;

    if (is_black(d4) && is_black(d5)) {
        bias = 0;
    } else if (is_black(d4)) {
        bias = -d5_bias;
    } else if (is_black(d5)) {
        bias = d5_bias;
    } else if (is_black(d3)) {
        bias = -d6_bias;
    } else if (is_black(d6)) {
        bias = d6_bias;
    } else if (is_black(d2)) {
        bias = -d7_bias;
    } else if (is_black(d7)) {
        bias = d7_bias;
    } else if (is_black(d1)) {
        bias = -d8_bias;
    } else if (is_black(d8)) {
        bias = d8_bias;
    }

    if (bias != 0) {
        g_last_bias = bias;
    }

    int left_target_speed = limit_speed(base_speed + bias);
    int right_target_speed = limit_speed(base_speed - bias);

    int encoder_ratio = 6;                     // 编码器计数与电机速度的比值

    if (Encoder_HasNewSpeed() != 0U) {
        int left_target_encoder = left_target_speed * encoder_ratio;
        int right_target_encoder = right_target_speed * encoder_ratio;
        int left_real_encoder = Encoder_GetNowVA();
        int right_real_encoder = Encoder_GetNowVB();

        g_speed_pid_left_output = SpeedPID_Left(left_target_encoder, left_real_encoder, left_target_speed);
        g_speed_pid_right_output = SpeedPID_Right(right_target_encoder, right_real_encoder, right_target_speed);

        Encoder_ClearNewSpeedFlag();
    }

    motorA(g_speed_pid_left_output);
    motorB(g_speed_pid_right_output);
}

void track2_weizhiPID(void)
{
    uint8_t d1 = D1;
    uint8_t d2 = D2;
    uint8_t d3 = D3;
    uint8_t d4 = D4;
    uint8_t d5 = D5;
    uint8_t d6 = D6;
    uint8_t d7 = D7;
    uint8_t d8 = D8;
    uint8_t now_all_white = is_all_white(d1, d2, d3, d4, d5, d6, d7, d8);
    float now_yaw;
    float pid_out;
    float cd_target_yaw;
    float target_yaw;
    float yaw_error;
    int bias;
    int left_speed;
    int right_speed;
    int left_target_speed;
    int right_target_speed;
    int left_target_encoder;
    int right_target_encoder;
    int left_real_encoder;
    int right_real_encoder;

    if (g_track2_weizhipid_first_check != 0U) {
        g_track2_weizhipid_last_all_white = now_all_white;
        g_track2_weizhipid_first_check = 0U;
    }

    switch (g_track2_weizhipid_state) {
        case TRACK2_AB_STRAIGHT:
            // A->B：白场直行，遇到黑线到 B 点，状态逻辑保持和track2一致
            if ((g_track2_weizhipid_last_all_white == 1U) && (now_all_white == 0U)) {
                SoundLight_Start();
                g_track2_weizhipid_state = TRACK2_BC_ARC;
                g_speed_pid_left_integral = 0.0f;
                g_speed_pid_right_integral = 0.0f;
                g_speed_pid_left_last_error = 0.0f;
                g_speed_pid_right_last_error = 0.0f;
                g_speed_pid_left_output = 400;
                g_speed_pid_right_output = 400;
                g_track2_weizhipid_last_all_white = now_all_white;
                break;
            }

            now_yaw = GetYaw_Value();
            pid_out = PID(0.0f, now_yaw);
            left_speed = limit_speed(400 + (int)pid_out);
            right_speed = limit_speed(400 - (int)pid_out);
            motorA(left_speed);
            motorB(right_speed);
            break;

        case TRACK2_BC_ARC:
            // B->C：灰度离散差速给目标速度，编码器速度环负责让真实轮速跟上
            if (now_all_white == 1U) {
                now_yaw = GetYaw_Value();
                target_yaw = (now_yaw >= 0.0f) ? 180.0f : -180.0f;
                yaw_error = target_yaw - now_yaw;
                if (yaw_error > 180.0f) {
                    yaw_error -= 360.0f;
                }
                if (yaw_error < -180.0f) {
                    yaw_error += 360.0f;
                }

                if ((yaw_error <= TRACK2_YAW_NEAR_DEG) &&
                    (yaw_error >= -TRACK2_YAW_NEAR_DEG)) {//满足角度限制
                    SoundLight_Start();
                    g_track2_weizhipid_state = TRACK2_CD_STRAIGHT;//状态切换
                    g_track2_weizhipid_last_all_white = now_all_white;
                    break;
                }

                // // 全白但角度没到，认为是脱线，根据上一次线偏向搜索
                // if (g_last_bias > 0) {
                //     left_speed = limit_speed(TRACK_BASE_SPEED + 60);
                //     right_speed = limit_speed(TRACK_BASE_SPEED - 60);
                // } else if (g_last_bias < 0) {
                //     left_speed = limit_speed(TRACK_BASE_SPEED - 60);
                //     right_speed = limit_speed(TRACK_BASE_SPEED + 60);
                // } else {
                //     left_speed = limit_speed(TRACK_BASE_SPEED - 100);
                //     right_speed = limit_speed(TRACK_BASE_SPEED - 100);
                // }

                // motorA(left_speed);
                // motorB(right_speed);
                break;
            }

            bias = track2_weizhipid_bias(d1, d2, d3, d4, d5, d6, d7, d8);
            if (bias != 0) {
                g_last_bias = bias;
            }

            left_target_speed = limit_speed(400 + bias);
            right_target_speed = limit_speed(400 - bias);

            if (Encoder_HasNewSpeed() != 0U) {
                left_target_encoder = left_target_speed * 6;
                right_target_encoder = right_target_speed * 6;
                left_real_encoder = Encoder_GetNowVA();
                right_real_encoder = Encoder_GetNowVB();

                g_speed_pid_left_output = SpeedPID_Left(
                    left_target_encoder, left_real_encoder, left_target_speed);
                g_speed_pid_right_output = SpeedPID_Right(
                    right_target_encoder, right_real_encoder, right_target_speed);
                Encoder_ClearNewSpeedFlag();
            }

            motorA(g_speed_pid_left_output);
            motorB(g_speed_pid_right_output);
            break;

        case TRACK2_CD_STRAIGHT:
            // C->D：白场直行，目标方向约为 180 度，遇到黑线到 D 点
            if ((g_track2_weizhipid_last_all_white == 1U) && (now_all_white == 0U)) {
                SoundLight_Start();
                g_track2_weizhipid_state = TRACK2_DA_ARC;
                g_speed_pid_left_integral = 0.0f;
                g_speed_pid_right_integral = 0.0f;
                g_speed_pid_left_last_error = 0.0f;
                g_speed_pid_right_last_error = 0.0f;
                g_speed_pid_left_output = 400;
                g_speed_pid_right_output = 400;
                g_track2_weizhipid_last_all_white = now_all_white;
                break;
            }

            now_yaw = GetYaw_Value();
            cd_target_yaw = (now_yaw >= 0.0f) ? 180.0f : -180.0f;
            pid_out = PID(cd_target_yaw, now_yaw);
            left_speed = limit_speed(400 + (int)pid_out);
            right_speed = limit_speed(400 - (int)pid_out);
            motorA(left_speed);
            motorB(right_speed);
            break;

        case TRACK2_DA_ARC:
            // D->A：灰度离散差速给目标速度，编码器速度环负责让真实轮速跟上
            if (now_all_white == 1U) {
                now_yaw = GetYaw_Value();
                target_yaw = 0.0f;
                yaw_error = target_yaw - now_yaw;
                if (yaw_error > 180.0f) {
                    yaw_error -= 360.0f;
                }
                if (yaw_error < -180.0f) {
                    yaw_error += 360.0f;
                }

                if ((yaw_error <= TRACK2_YAW_NEAR_DEG) &&
                    (yaw_error >= -TRACK2_YAW_NEAR_DEG)) {
                    SoundLight_Start();
                    g_track2_weizhipid_state = TRACK2_DONE;
                    g_track2_weizhipid_last_all_white = now_all_white;
                    g_speed_pid_left_output = 0;
                    g_speed_pid_right_output = 0;
                    motorA(0);
                    motorB(0);
                    break;
                }

                // 全白但角度没到，认为是脱线，根据上一次线偏向搜索
                // if (g_last_bias > 0) {
                //     left_speed = limit_speed(TRACK_BASE_SPEED + 60);
                //     right_speed = limit_speed(TRACK_BASE_SPEED - 60);
                // } else if (g_last_bias < 0) {
                //     left_speed = limit_speed(TRACK_BASE_SPEED - 60);
                //     right_speed = limit_speed(TRACK_BASE_SPEED + 60);
                // } else {
                //     left_speed = limit_speed(TRACK_BASE_SPEED - 100);
                //     right_speed = limit_speed(TRACK_BASE_SPEED - 100);
                // }

                // motorA(left_speed);
                // motorB(right_speed);
                break;
            }

            bias = track2_weizhipid_bias(d1, d2, d3, d4, d5, d6, d7, d8);
            if (bias != 0) {
                g_last_bias = bias;
            }

            left_target_speed = limit_speed(400 + bias);
            right_target_speed = limit_speed(400 - bias);

            if (Encoder_HasNewSpeed() != 0U) {
                left_target_encoder = left_target_speed * 6;
                right_target_encoder = right_target_speed * 6;
                left_real_encoder = Encoder_GetNowVA();
                right_real_encoder = Encoder_GetNowVB();

                g_speed_pid_left_output = SpeedPID_Left(
                    left_target_encoder, left_real_encoder, left_target_speed);
                g_speed_pid_right_output = SpeedPID_Right(
                    right_target_encoder, right_real_encoder, right_target_speed);
                Encoder_ClearNewSpeedFlag();
            }

            motorA(g_speed_pid_left_output);
            motorB(g_speed_pid_right_output);
            break;

        case TRACK2_DONE:
        default:
            // 一圈完成后保持停车
            g_speed_pid_left_output = 0;
            g_speed_pid_right_output = 0;
            motorA(0);
            motorB(0);
            break;
    }

    g_track2_weizhipid_last_all_white = now_all_white;
}

void track2_zengliangPID(void)
{
    /* 这些参数放在函数内部，方便现场直接调 */
    // int base_speed = 400;缓慢加速到基础速度
    static uint8_t soft_reset = 1U;
    int base_speed = motor_soft_speed(500, 200, 50, soft_reset);//软加速
    soft_reset = 0U;
   
    int encoder_ratio = 6;
    float kp_l = 0.18f;
    float ki_l = 0.0f;
    float kd_l = 0.04f;
    float kp_r = 0.18f;
    float ki_r = 0.0f;
    float kd_r = 0.04f;
    int output_max = 1000;
    int output_min = -1000;

    /* track2_zengliangPID自己的状态变量，不影响原track2() */
    static track2_state_t state = TRACK2_AB_STRAIGHT;
    static uint8_t first_check = 1U;
    static uint8_t last_all_white = 1U;
    static float error_l = 0.0f;
    static float last_error_l = 0.0f;
    static float last_last_error_l = 0.0f;
    static float error_r = 0.0f;
    static float last_error_r = 0.0f;
    static float last_last_error_r = 0.0f;
    static int left_output_speed = 0;
    static int right_output_speed = 0;

    uint8_t d1 = D1;
    uint8_t d2 = D2;
    uint8_t d3 = D3;
    uint8_t d4 = D4;
    uint8_t d5 = D5;
    uint8_t d6 = D6;
    uint8_t d7 = D7;
    uint8_t d8 = D8;
    uint8_t now_all_white = is_all_white(d1, d2, d3, d4, d5, d6, d7, d8);
    float now_yaw;
    float pid_out;
    float cd_target_yaw;
    float target_yaw;
    float yaw_error;
    float Pout;
    float Iout;
    float Dout;
    float delta_output;
    int bias;
    int left_speed;
    int right_speed;
    int left_target_speed;
    int right_target_speed;
    int left_target_encoder;
    int right_target_encoder;
    int left_real_encoder;
    int right_real_encoder;

    if (first_check != 0U) {
        last_all_white = now_all_white;
        first_check = 0U;
    }

    switch (state) {
        case TRACK2_AB_STRAIGHT:
            // A->B：白场直行，遇到黑线到 B 点，状态切换照原track2()
            if ((last_all_white == 1U) && (now_all_white == 0U)) {
                SoundLight_Start();
                state = TRACK2_BC_ARC;
                error_l = 0.0f;
                last_error_l = 0.0f;
                last_last_error_l = 0.0f;
                error_r = 0.0f;
                last_error_r = 0.0f;
                last_last_error_r = 0.0f;
                left_output_speed = base_speed;
                right_output_speed = base_speed;
                last_all_white = now_all_white;
                break;
            }

            now_yaw = GetYaw_Value();
            pid_out = PID(0.0f, now_yaw);
            left_speed = limit_speed(base_speed + (int)pid_out);
            right_speed = limit_speed(base_speed - (int)pid_out);
            motorA(left_speed);
            motorB(right_speed);
            break;

        case TRACK2_BC_ARC:
            // B->C：全白时不直接切状态，先判断yaw是否接近180度
            if (now_all_white == 1U) {
                now_yaw = GetYaw_Value();
                target_yaw = (now_yaw >= 0.0f) ? 180.0f : -180.0f;
                yaw_error = target_yaw - now_yaw;
                if (yaw_error > 180.0f) {
                    yaw_error -= 360.0f;
                }
                if (yaw_error < -180.0f) {
                    yaw_error += 360.0f;
                }

                if ((yaw_error <= TRACK2_YAW_NEAR_DEG) &&
                    (yaw_error >= -TRACK2_YAW_NEAR_DEG)) {//满足角度限制
                    SoundLight_Start();
                    state = TRACK2_CD_STRAIGHT;//状态切换
                    last_all_white = now_all_white;
                    break;
                }

                // // 全白但角度没到，认为是脱线，根据上一次线偏向搜索
                // if (g_last_bias > 0) {
                //     left_speed = limit_speed(TRACK_BASE_SPEED + 60);
                //     right_speed = limit_speed(TRACK_BASE_SPEED - 60);
                // } else if (g_last_bias < 0) {
                //     left_speed = limit_speed(TRACK_BASE_SPEED - 60);
                //     right_speed = limit_speed(TRACK_BASE_SPEED + 60);
                // } else {
                //     left_speed = limit_speed(TRACK_BASE_SPEED - 100);
                //     right_speed = limit_speed(TRACK_BASE_SPEED - 100);
                // }

                // motorA(left_speed);
                // motorB(right_speed);
                break;
            }

            // 灰度离散差速：这里只生成左右轮目标速度，不直接输出给电机
            int d5_bias = base_speed * 0.1;
            int d6_bias = base_speed * 0.2;
            int d7_bias = base_speed * 0.3;
            int d8_bias = base_speed * 0.4;           
            if (is_black(d4) && is_black(d5)) {
                bias = 0;
            } else if (is_black(d4)) {
                bias = -d5_bias;
            } else if (is_black(d5)) {
                bias = d5_bias;
            } else if (is_black(d3)) {
                bias = -d6_bias;
            } else if (is_black(d6)) {
                bias = d6_bias;
            } else if (is_black(d2)) {
                bias = -d7_bias;
            } else if (is_black(d7)) {
                bias = d7_bias;
            } else if (is_black(d1)) {
                bias = -d8_bias;
            } else if (is_black(d8)) {
                bias = d8_bias;
            } else {
                bias = 0;
            }
            if (bias != 0) {
                g_last_bias = bias;
            }

            left_target_speed = limit_speed(base_speed + bias);
            right_target_speed = limit_speed(base_speed - bias);

            // 编码器50ms更新一次，所以增量式速度PID也只在有新速度时算一次
            if (Encoder_HasNewSpeed() != 0U) {
                left_target_encoder = left_target_speed * encoder_ratio;
                right_target_encoder = right_target_speed * encoder_ratio;
                left_real_encoder = Encoder_GetNowVA();
                right_real_encoder = Encoder_GetNowVB();

                error_l = (float)(left_target_encoder - left_real_encoder);
                Pout = kp_l * (error_l - last_error_l);
                Iout = ki_l * error_l;
                Dout = kd_l * (error_l - 2.0f * last_error_l + last_last_error_l);
                delta_output = Pout + Iout + Dout;
                left_output_speed += (int)delta_output;
                if (left_output_speed > output_max) {
                    left_output_speed = output_max;
                }
                if (left_output_speed < output_min) {
                    left_output_speed = output_min;
                }
                last_last_error_l = last_error_l;
                last_error_l = error_l;

                error_r = (float)(right_target_encoder - right_real_encoder);
                Pout = kp_r * (error_r - last_error_r);
                Iout = ki_r * error_r;
                Dout = kd_r * (error_r - 2.0f * last_error_r + last_last_error_r);
                delta_output = Pout + Iout + Dout;
                right_output_speed += (int)delta_output;
                if (right_output_speed > output_max) {
                    right_output_speed = output_max;
                }
                if (right_output_speed < output_min) {
                    right_output_speed = output_min;
                }
                last_last_error_r = last_error_r;
                last_error_r = error_r;

                Encoder_ClearNewSpeedFlag();
            }

            motorA(left_output_speed);
            motorB(right_output_speed);
            break;

        case TRACK2_CD_STRAIGHT:
            // C->D：白场直行，目标方向约为 180 度，遇到黑线到 D 点
            if ((last_all_white == 1U) && (now_all_white == 0U)) {
                SoundLight_Start();
                state = TRACK2_DA_ARC;
                error_l = 0.0f;
                last_error_l = 0.0f;
                last_last_error_l = 0.0f;
                error_r = 0.0f;
                last_error_r = 0.0f;
                last_last_error_r = 0.0f;
                left_output_speed = base_speed;
                right_output_speed = base_speed;
                last_all_white = now_all_white;
                break;
            }

            now_yaw = GetYaw_Value();
            cd_target_yaw = (now_yaw >= 0.0f) ? 180.0f : -180.0f;
            pid_out = PID(cd_target_yaw, now_yaw);
            left_speed = limit_speed(base_speed + (int)pid_out);
            right_speed = limit_speed(base_speed - (int)pid_out);
            motorA(left_speed);
            motorB(right_speed);
            break;

        case TRACK2_DA_ARC:
            // D->A：全白时不直接结束，先判断yaw是否接近0度
            if (now_all_white == 1U) {
                now_yaw = GetYaw_Value();
                target_yaw = 0.0f;
                yaw_error = target_yaw - now_yaw;
                if (yaw_error > 180.0f) {
                    yaw_error -= 360.0f;
                }
                if (yaw_error < -180.0f) {
                    yaw_error += 360.0f;
                }

                if ((yaw_error <= TRACK2_YAW_NEAR_DEG) &&
                    (yaw_error >= -TRACK2_YAW_NEAR_DEG)) {
                    SoundLight_Start();
                    motors_brake();
                    state = TRACK2_DONE;
                    last_all_white = now_all_white;
                    left_output_speed = 0;
                    right_output_speed = 0;
                    motorA(0);
                    motorB(0);
                    break;
                }

                // 全白但角度没到，认为是脱线，根据上一次线偏向搜索
                // if (g_last_bias > 0) {
                //     left_speed = limit_speed(TRACK_BASE_SPEED + 60);
                //     right_speed = limit_speed(TRACK_BASE_SPEED - 60);
                // } else if (g_last_bias < 0) {
                //     left_speed = limit_speed(TRACK_BASE_SPEED - 60);
                //     right_speed = limit_speed(TRACK_BASE_SPEED + 60);
                // } else {
                //     left_speed = limit_speed(TRACK_BASE_SPEED - 100);
                //     right_speed = limit_speed(TRACK_BASE_SPEED - 100);
                // }

                // motorA(left_speed);
                // motorB(right_speed);
                break;
            }

            // 灰度离散差速：这里只生成左右轮目标速度，不直接输出给电机
            d5_bias = base_speed * 0.1;
            d6_bias = base_speed * 0.2;
            d7_bias = base_speed * 0.3;
            d8_bias = base_speed * 0.4;           
            if (is_black(d4) && is_black(d5)) {
                bias = 0;
            } else if (is_black(d4)) {
                bias = -d5_bias;
            } else if (is_black(d5)) {
                bias = d5_bias;
            } else if (is_black(d3)) {
                bias = -d6_bias;
            } else if (is_black(d6)) {
                bias = d6_bias;
            } else if (is_black(d2)) {
                bias = -d7_bias;
            } else if (is_black(d7)) {
                bias = d7_bias;
            } else if (is_black(d1)) {
                bias = -d8_bias;
            } else if (is_black(d8)) {
                bias = d8_bias;
            } else {
                bias = 0;
            }
            if (bias != 0) {
                g_last_bias = bias;
            }

            left_target_speed = limit_speed(base_speed + bias);
            right_target_speed = limit_speed(base_speed - bias);

            // 编码器50ms更新一次，所以增量式速度PID也只在有新速度时算一次
            if (Encoder_HasNewSpeed() != 0U) {
                left_target_encoder = left_target_speed * encoder_ratio;
                right_target_encoder = right_target_speed * encoder_ratio;
                left_real_encoder = Encoder_GetNowVA();
                right_real_encoder = Encoder_GetNowVB();

                error_l = (float)(left_target_encoder - left_real_encoder);
                Pout = kp_l * (error_l - last_error_l);
                Iout = ki_l * error_l;
                Dout = kd_l * (error_l - 2.0f * last_error_l + last_last_error_l);
                delta_output = Pout + Iout + Dout;
                left_output_speed += (int)delta_output;
                if (left_output_speed > output_max) {
                    left_output_speed = output_max;
                }
                if (left_output_speed < output_min) {
                    left_output_speed = output_min;
                }
                last_last_error_l = last_error_l;
                last_error_l = error_l;

                error_r = (float)(right_target_encoder - right_real_encoder);
                Pout = kp_r * (error_r - last_error_r);
                Iout = ki_r * error_r;
                Dout = kd_r * (error_r - 2.0f * last_error_r + last_last_error_r);
                delta_output = Pout + Iout + Dout;
                right_output_speed += (int)delta_output;
                if (right_output_speed > output_max) {
                    right_output_speed = output_max;
                }
                if (right_output_speed < output_min) {
                    right_output_speed = output_min;
                }
                last_last_error_r = last_error_r;
                last_error_r = error_r;

                Encoder_ClearNewSpeedFlag();
            }

            motorA(left_output_speed);
            motorB(right_output_speed);
            break;

        case TRACK2_DONE:
        default:
            // 一圈完成后保持停车
            left_output_speed = 0;
            right_output_speed = 0;
            motorA(0);
            motorB(0);
            break;
    }

    last_all_white = now_all_white;
}

void track3(void)
{
    uint8_t d1 = D1;
    uint8_t d2 = D2;
    uint8_t d3 = D3;
    uint8_t d4 = D4;
    uint8_t d5 = D5;
    uint8_t d6 = D6;
    uint8_t d7 = D7;
    uint8_t d8 = D8;
    uint8_t now_all_white = is_all_white(d1, d2, d3, d4, d5, d6, d7, d8);
    float now_yaw;
    float pid_out;
    float target_yaw;
    float yaw_error;
    int bias;
    int left_speed;
    int right_speed;

    if (g_track3_first_check != 0U) {
        g_track3_last_all_white = now_all_white;
        g_track3_first_check = 0U;
    }

    switch (g_track3_state) {
        case TRACK3_AC_STRAIGHT:
            // A->C：白场斜向直行，遇到黑线认为到 C 点
            if ((g_track3_last_all_white == 1U) && (now_all_white == 0U)) {
                SoundLight_Start();
                g_track3_state = TRACK3_CB_ARC;
                g_track3_last_all_white = now_all_white;
                break;
            }

            now_yaw = GetYaw_Value();
            pid_out = PID(TRACK3_AC_TARGET_YAW, now_yaw);
            left_speed = limit_speed(TRACK_BASE_SPEED + (int)pid_out);
            right_speed = limit_speed(TRACK_BASE_SPEED - (int)pid_out);
            motorA(left_speed);
            motorB(right_speed);
            break;

        case TRACK3_CB_ARC:
            // C->B：沿右半圆黑线寻迹；全白不一定到 B，也可能是脱线
            if (now_all_white == 1U) {
                now_yaw = GetYaw_Value();
                // target_yaw = TRACK3_B_EXIT_YAW;
                target_yaw = (now_yaw >= 0.0f) ? 180.0f : -180.0f;
                yaw_error = target_yaw - now_yaw;
                if (yaw_error > 180.0f) {
                    yaw_error -= 360.0f;
                }
                if (yaw_error < -180.0f) {
                    yaw_error += 360.0f;
                }

                if ((yaw_error <= (TRACK3_YAW_NEAR_DEG-10)) &&
                    (yaw_error >= -(TRACK3_YAW_NEAR_DEG-10))) {
                    SoundLight_Start();
                    g_track3_state = TRACK3_BD_STRAIGHT;
                    g_track3_last_all_white = now_all_white;
                    break;
                }

                // 全白但角度没到，先认为脱线，按上一次偏差找线
                // if (g_last_bias > 0) {
                //     left_speed = limit_speed(TRACK_BASE_SPEED + 60);
                //     right_speed = limit_speed(TRACK_BASE_SPEED - 60);
                // } else if (g_last_bias < 0) {
                //     left_speed = limit_speed(TRACK_BASE_SPEED - 60);
                //     right_speed = limit_speed(TRACK_BASE_SPEED + 60);
                // } else {
                //     left_speed = limit_speed(TRACK_BASE_SPEED - 100);
                //     right_speed = limit_speed(TRACK_BASE_SPEED - 100);
                // }

                // motorA(left_speed);
                // motorB(right_speed);
                break;
            }

            bias = track_bias(d1, d2, d3, d4, d5, d6, d7, d8);
            if (bias != 0) {
                g_last_bias = bias;
            }
            left_speed = limit_speed(TRACK_BASE_SPEED + bias);
            right_speed = limit_speed(TRACK_BASE_SPEED - bias);
            motorA(left_speed);
            motorB(right_speed);
            break;

        case TRACK3_BD_STRAIGHT:
            // B->D：白场斜向直行，遇到黑线认为到 D 点
            if ((g_track3_last_all_white == 1U) && (now_all_white == 0U)) {
                SoundLight_Start();
                g_track3_state = TRACK3_DA_ARC;
                g_track3_last_all_white = now_all_white;
                break;
            }

            now_yaw = GetYaw_Value();
            pid_out = PID(TRACK3_BD_TARGET_YAW, now_yaw);
            left_speed = limit_speed(TRACK_BASE_SPEED + (int)pid_out);
            right_speed = limit_speed(TRACK_BASE_SPEED - (int)pid_out);
            motorA(left_speed);
            motorB(right_speed);
            break;

        case TRACK3_DA_ARC:
            // D->A：沿左半圆黑线寻迹；全白时要叠加 yaw 判断是否真的回到 A
            if (now_all_white == 1U) {
                now_yaw = GetYaw_Value();
                target_yaw = TRACK3_A_EXIT_YAW;
                // target_yaw = 0.0;
                yaw_error = target_yaw - now_yaw;
                if (yaw_error > 180.0f) {
                    yaw_error -= 360.0f;
                }
                if (yaw_error < -180.0f) {
                    yaw_error += 360.0f;
                }

                if ((yaw_error <= TRACK3_YAW_NEAR_DEG) &&
                    (yaw_error >= -TRACK3_YAW_NEAR_DEG)) {
                    SoundLight_Start();
                    g_track3_state = TRACK3_DONE;
                    g_track3_last_all_white = now_all_white;
                    motorA(0);
                    motorB(0);
                    break;
                }

                // 全白但角度没到，先认为脱线，按上一次偏差找线
                // if (g_last_bias > 0) {
                //     left_speed = limit_speed(TRACK_BASE_SPEED + 60);
                //     right_speed = limit_speed(TRACK_BASE_SPEED - 60);
                // } else if (g_last_bias < 0) {
                //     left_speed = limit_speed(TRACK_BASE_SPEED - 60);
                //     right_speed = limit_speed(TRACK_BASE_SPEED + 60);
                // } else {
                //     left_speed = limit_speed(TRACK_BASE_SPEED - 100);
                //     right_speed = limit_speed(TRACK_BASE_SPEED - 100);
                // }

                // motorA(left_speed);
                // motorB(right_speed);
                break;
            }

            bias = track_bias(d1, d2, d3, d4, d5, d6, d7, d8);
            if (bias != 0) {
                g_last_bias = bias;
            }
            left_speed = limit_speed(TRACK_BASE_SPEED + bias);
            right_speed = limit_speed(TRACK_BASE_SPEED - bias);
            motorA(left_speed);
            motorB(right_speed);
            break;

        case TRACK3_DONE:
        default:
            // 第3问完成后保持停车
            motorA(0);
            motorB(0);
            break;
    }

    g_track3_last_all_white = now_all_white;
}

void track3_zengliangPID(void)
{
    /* 调试参数放在函数内部，方便现场修改 */
    // int base_speed = 200;
    static uint8_t soft_reset = 1U;
    int base_speed = motor_soft_speed(300, 200, 50, soft_reset);//软加速
    soft_reset = 0U;//
    int encoder_ratio = 6;
    float kp_l = 0.15f;
    float ki_l = 0.0f;
    float kd_l = 0.02f;
    float kp_r = 0.15f;
    float ki_r = 0.0f;
    float kd_r = 0.02f;
    int output_max = 1000;
    int output_min = -1000;

    /* 灰度离散差速参数：base_speed=400时分别约为40/80/120/160 */
    int d5_bias = (int)(base_speed * 0.1f);
    int d6_bias = (int)(base_speed * 0.2f);
    int d7_bias = (int)(base_speed * 0.3f);
    int d8_bias = (int)(base_speed * 0.4f);

    /*
     * 状态定义：
     * 0：AC_STRAIGHT，1：CB_ARC，2：BD_STRAIGHT，3：DA_ARC，4：DONE
     */
    static int state = 0;
    static uint8_t first_check = 1U;
    static uint8_t last_all_white = 1U;
    static float error_l = 0.0f;
    static float last_error_l = 0.0f;
    static float last_last_error_l = 0.0f;
    static float error_r = 0.0f;
    static float last_error_r = 0.0f;
    static float last_last_error_r = 0.0f;
    static int left_output_speed = 0;
    static int right_output_speed = 0;
    static int x = 0;//车头是否摆正角度标志位

    uint8_t d1 = D1;
    uint8_t d2 = D2;
    uint8_t d3 = D3;
    uint8_t d4 = D4;
    uint8_t d5 = D5;
    uint8_t d6 = D6;
    uint8_t d7 = D7;
    uint8_t d8 = D8;
    uint8_t now_all_white = is_all_white(d1, d2, d3, d4, d5, d6, d7, d8);
    float now_yaw;
    float pid_out;
    float target_yaw;
    float yaw_error;
    float Pout;
    float Iout;
    float Dout;
    float delta_output;
    int bias;
    int left_speed;
    int right_speed;
    int left_target_speed;
    int right_target_speed;
    int left_target_encoder;
    int right_target_encoder;
    int left_real_encoder;
    int right_real_encoder;

    if (first_check != 0U) {
        last_all_white = now_all_white;
        first_check = 0U;
    }

    switch (state) {
        case 0:
            // A->C：白场斜线直行，遇到黑线认为到 C 点，不刹车
            //先判断状态转换条件
            if ((last_all_white == 1U) && (now_all_white == 0U)) {
                SoundLight_Start();
                motors_brake();
                x=0;//车头还没摆正
                state = 1;
                error_l = 0.0f;
                last_error_l = 0.0f;
                last_last_error_l = 0.0f;
                error_r = 0.0f;
                last_error_r = 0.0f;
                last_last_error_r = 0.0f;
                left_output_speed = base_speed;
                right_output_speed = base_speed;
                last_all_white = now_all_white;
                break;
            }
            //后执行当前状态动作
            now_yaw = GetYaw_Value();
            pid_out = PID(TRACK3_AC_TARGET_YAW, now_yaw);
            left_speed = limit_speed(base_speed + (int)pid_out);
            right_speed = limit_speed(base_speed - (int)pid_out);
            motorA(left_speed);
            motorB(right_speed);
            break;

        case 1:
            // C->B：黑线圆弧寻迹；全白时必须叠加yaw判断，满足才刹车切到BD直线
            if (now_all_white == 1U) {
                now_yaw = GetYaw_Value();
                target_yaw = (now_yaw >= 0.0f) ? 180.0f : -180.0f;
                yaw_error = target_yaw - now_yaw;
                if (yaw_error > 180.0f) {
                    yaw_error -= 360.0f;
                }
                if (yaw_error < -180.0f) {
                    yaw_error += 360.0f;
                }
                if ((yaw_error <= (TRACK3_YAW_NEAR_DEG - 10.0f)) &&
                    (yaw_error >= -(TRACK3_YAW_NEAR_DEG - 10.0f))) {
                    SoundLight_Start();
                    motors_brake();//
                    state = 2;//满足角度限制后切换状态
                    last_all_white = now_all_white;
                    break;
                }
                // 全白但yaw没到，不新增复杂脱线处理，保持上一拍电机输出
                break;
            }
            //灰度差速
            if (is_black(d4) && is_black(d5)) {
                bias = 0;
            } else if (is_black(d4)) {
                bias = -d5_bias;
            } else if (is_black(d5)) {
                bias = d5_bias;
            } else if (is_black(d3)) {
                bias = -d6_bias;
            } else if (is_black(d6)) {
                bias = d6_bias;
            } else if (is_black(d2)) {
                bias = -d7_bias;
            } else if (is_black(d7)) {
                bias = d7_bias;
            } else if (is_black(d1)) {
                bias = -d8_bias;
            } else if (is_black(d8)) {
                bias = d8_bias;
            } else {
                bias = 0;
            }

            if (bias != 0) {
                g_last_bias = bias;
            }

            left_target_speed = limit_speed(base_speed + bias);
            right_target_speed = limit_speed(base_speed - bias);

            //这里写摆正车头
            if(x==0){
                

                x=1;
                break;
            }

            //增量PID
            if (Encoder_HasNewSpeed() != 0U) {
                left_target_encoder = left_target_speed * encoder_ratio;
                right_target_encoder = right_target_speed * encoder_ratio;
                left_real_encoder = Encoder_GetNowVA();
                right_real_encoder = Encoder_GetNowVB();

                error_l = (float)(left_target_encoder - left_real_encoder);
                Pout = kp_l * (error_l - last_error_l);
                Iout = ki_l * error_l;
                Dout = kd_l * (error_l - 2.0f * last_error_l + last_last_error_l);
                delta_output = Pout + Iout + Dout;
                left_output_speed += (int)delta_output;
                if (left_output_speed > output_max) {
                    left_output_speed = output_max;
                }
                if (left_output_speed < output_min) {
                    left_output_speed = output_min;
                }
                last_last_error_l = last_error_l;
                last_error_l = error_l;

                error_r = (float)(right_target_encoder - right_real_encoder);
                Pout = kp_r * (error_r - last_error_r);
                Iout = ki_r * error_r;
                Dout = kd_r * (error_r - 2.0f * last_error_r + last_last_error_r);
                delta_output = Pout + Iout + Dout;
                right_output_speed += (int)delta_output;
                if (right_output_speed > output_max) {
                    right_output_speed = output_max;
                }
                if (right_output_speed < output_min) {
                    right_output_speed = output_min;
                }
                last_last_error_r = last_error_r;
                last_error_r = error_r;

                Encoder_ClearNewSpeedFlag();
            }

            motorA(left_output_speed);
            motorB(right_output_speed);
            break;

        case 2:
            // B->D：白场斜线直行，遇到黑线认为到 D 点，不刹车
            if ((last_all_white == 1U) && (now_all_white == 0U)) {
                SoundLight_Start();
                state = 3;
                error_l = 0.0f;
                last_error_l = 0.0f;
                last_last_error_l = 0.0f;
                error_r = 0.0f;
                last_error_r = 0.0f;
                last_last_error_r = 0.0f;
                left_output_speed = base_speed;
                right_output_speed = base_speed;
                last_all_white = now_all_white;
                break;
            }

            now_yaw = GetYaw_Value();
            pid_out = PID(TRACK3_BD_TARGET_YAW, now_yaw);
            left_speed = limit_speed(base_speed + (int)pid_out);
            right_speed = limit_speed(base_speed - (int)pid_out);
            motorA(left_speed);
            motorB(right_speed);
            break;

        case 3:
            // D->A：黑线圆弧寻迹；全白时必须叠加yaw判断，满足才刹车并结束
            if (now_all_white == 1U) {
                now_yaw = GetYaw_Value();
                target_yaw = TRACK3_A_EXIT_YAW;
                yaw_error = target_yaw - now_yaw;
                if (yaw_error > 180.0f) {
                    yaw_error -= 360.0f;
                }
                if (yaw_error < -180.0f) {
                    yaw_error += 360.0f;
                }

                if ((yaw_error <= TRACK3_YAW_NEAR_DEG) &&
                    (yaw_error >= -TRACK3_YAW_NEAR_DEG)) {
                    SoundLight_Start();
                    motors_brake();
                    left_output_speed = 0;
                    right_output_speed = 0;
                    motorA(0);
                    motorB(0);
                    state = 4;
                    last_all_white = now_all_white;
                    break;
                }

                // 全白但yaw没到，不新增复杂脱线处理，保持上一拍电机输出
                break;
            }

            if (is_black(d4) && is_black(d5)) {
                bias = 0;
            } else if (is_black(d4)) {
                bias = -d5_bias;
            } else if (is_black(d5)) {
                bias = d5_bias;
            } else if (is_black(d3)) {
                bias = -d6_bias;
            } else if (is_black(d6)) {
                bias = d6_bias;
            } else if (is_black(d2)) {
                bias = -d7_bias;
            } else if (is_black(d7)) {
                bias = d7_bias;
            } else if (is_black(d1)) {
                bias = -d8_bias;
            } else if (is_black(d8)) {
                bias = d8_bias;
            } else {
                bias = 0;
            }

            if (bias != 0) {
                g_last_bias = bias;
            }

            left_target_speed = limit_speed(base_speed + bias);
            right_target_speed = limit_speed(base_speed - bias);

            if (Encoder_HasNewSpeed() != 0U) {
                left_target_encoder = left_target_speed * encoder_ratio;
                right_target_encoder = right_target_speed * encoder_ratio;
                left_real_encoder = Encoder_GetNowVA();
                right_real_encoder = Encoder_GetNowVB();

                error_l = (float)(left_target_encoder - left_real_encoder);
                Pout = kp_l * (error_l - last_error_l);
                Iout = ki_l * error_l;
                Dout = kd_l * (error_l - 2.0f * last_error_l + last_last_error_l);
                delta_output = Pout + Iout + Dout;
                left_output_speed += (int)delta_output;
                if (left_output_speed > output_max) {
                    left_output_speed = output_max;
                }
                if (left_output_speed < output_min) {
                    left_output_speed = output_min;
                }
                last_last_error_l = last_error_l;
                last_error_l = error_l;

                error_r = (float)(right_target_encoder - right_real_encoder);
                Pout = kp_r * (error_r - last_error_r);
                Iout = ki_r * error_r;
                Dout = kd_r * (error_r - 2.0f * last_error_r + last_last_error_r);
                delta_output = Pout + Iout + Dout;
                right_output_speed += (int)delta_output;
                if (right_output_speed > output_max) {
                    right_output_speed = output_max;
                }
                if (right_output_speed < output_min) {
                    right_output_speed = output_min;
                }
                last_last_error_r = last_error_r;
                last_error_r = error_r;

                Encoder_ClearNewSpeedFlag();
            }

            motorA(left_output_speed);
            motorB(right_output_speed);
            break;

        case 4:
        default:
            // DONE状态只保持停车，不反复调用motors_brake()
            motorA(0);
            motorB(0);
            break;
    }

    last_all_white = now_all_white;
}

void track4(void)
{
    uint8_t d1 = D1;
    uint8_t d2 = D2;
    uint8_t d3 = D3;
    uint8_t d4 = D4;
    uint8_t d5 = D5;
    uint8_t d6 = D6;
    uint8_t d7 = D7;
    uint8_t d8 = D8;
    uint8_t now_all_white = is_all_white(d1, d2, d3, d4, d5, d6, d7, d8);
    float now_yaw;
    float pid_out;
    float target_yaw;
    float yaw_error;
    int bias;
    int left_speed;
    int right_speed;

    if (g_track4_first_check != 0U) {
        g_track4_last_all_white = now_all_white;
        g_track4_first_check = 0U;
    }

    switch (g_track4_state) {
        case TRACK4_AC_STRAIGHT:
            // 第4问 A->C：照第3问路线跑，白场斜向直行
            if ((g_track4_last_all_white == 1U) && (now_all_white == 0U)) {
                SoundLight_Start();
                g_track4_state = TRACK4_CB_ARC;
                g_track4_last_all_white = now_all_white;
                break;
            }

            now_yaw = GetYaw_Value();
            // pid_out = PID(TRACK3_AC_TARGET_YAW, now_yaw);
            pid_out = PID(35, now_yaw);
            left_speed = limit_speed(TRACK_BASE_SPEED + (int)pid_out);
            right_speed = limit_speed(TRACK_BASE_SPEED - (int)pid_out);
            motorA(left_speed);
            motorB(right_speed);
            break;

        case TRACK4_CB_ARC:
            // 第4问 C->B：沿右半圆黑线寻迹，全白时加 yaw 限制
            if (now_all_white == 1U) {
                now_yaw = GetYaw_Value();
                // target_yaw = TRACK3_B_EXIT_YAW;
                target_yaw = (now_yaw >= 0.0f) ? 180.0f : -180.0f;
                yaw_error = target_yaw - now_yaw;
                if (yaw_error > 180.0f) {
                    yaw_error -= 360.0f;
                }
                if (yaw_error < -180.0f) {
                    yaw_error += 360.0f;
                }

                if ((yaw_error <= (TRACK3_YAW_NEAR_DEG)) &&
                    (yaw_error >= -(TRACK3_YAW_NEAR_DEG))) {
                    SoundLight_Start();
                    g_track4_state = TRACK4_BD_STRAIGHT;
                    g_track4_last_all_white = now_all_white;
                    break;
                }

                // 全白但角度没到，先认为脱线，按上一次偏差找线
                // if (g_last_bias > 0) {
                //     left_speed = limit_speed(TRACK_BASE_SPEED + 60);
                //     right_speed = limit_speed(TRACK_BASE_SPEED - 60);
                // } else if (g_last_bias < 0) {
                //     left_speed = limit_speed(TRACK_BASE_SPEED - 60);
                //     right_speed = limit_speed(TRACK_BASE_SPEED + 60);
                // } else {
                //     left_speed = limit_speed(TRACK_BASE_SPEED - 100);
                //     right_speed = limit_speed(TRACK_BASE_SPEED - 100);
                // }

                // motorA(left_speed);
                // motorB(right_speed);
                break;
            }

            bias = track_bias(d1, d2, d3, d4, d5, d6, d7, d8);
            if (bias != 0) {
                g_last_bias = bias;
            }
            left_speed = limit_speed(TRACK_BASE_SPEED + bias);
            right_speed = limit_speed(TRACK_BASE_SPEED - bias);
            motorA(left_speed);
            motorB(right_speed);
            break;

        case TRACK4_BD_STRAIGHT:
            // 第4问 B->D：白场斜向直行
            if ((g_track4_last_all_white == 1U) && (now_all_white == 0U)) {
                SoundLight_Start();
                g_track4_state = TRACK4_DA_ARC;
                g_track4_last_all_white = now_all_white;
                break;
            }

            now_yaw = GetYaw_Value();
            // pid_out = PID(TRACK3_BD_TARGET_YAW, now_yaw);
            pid_out = PID(143, now_yaw);
            left_speed = limit_speed(TRACK_BASE_SPEED + (int)pid_out);
            right_speed = limit_speed(TRACK_BASE_SPEED - (int)pid_out);
            motorA(left_speed);
            motorB(right_speed);
            break;

        case TRACK4_DA_ARC:
            // 第4问 D->A：回到 A 点才算完整一圈
            if (now_all_white == 1U) {
                now_yaw = GetYaw_Value();
                target_yaw = TRACK3_A_EXIT_YAW;
                // target_yaw = 0.0;
                yaw_error = target_yaw - now_yaw;
                if (yaw_error > 180.0f) {
                    yaw_error -= 360.0f;
                }
                if (yaw_error < -180.0f) {
                    yaw_error += 360.0f;
                }

                if ((yaw_error <= TRACK3_YAW_NEAR_DEG) &&
                    (yaw_error >= -TRACK3_YAW_NEAR_DEG)) {
                    SoundLight_Start();
                    g_track4_lap_count++;//只有回到A点才加1圈
                    g_track4_last_all_white = now_all_white;

                    if (g_track4_lap_count < 4U) {
                        g_track4_state = TRACK4_AC_STRAIGHT;
                    } else {
                        g_track4_state = TRACK4_DONE;
                        motorA(0);
                        motorB(0);
                    }
                    break;
                }

                // 全白但角度没到，先认为脱线，按上一次偏差找线
                // if (g_last_bias > 0) {
                //     left_speed = limit_speed(TRACK_BASE_SPEED + 60);
                //     right_speed = limit_speed(TRACK_BASE_SPEED - 60);
                // } else if (g_last_bias < 0) {
                //     left_speed = limit_speed(TRACK_BASE_SPEED - 60);
                //     right_speed = limit_speed(TRACK_BASE_SPEED + 60);
                // } else {
                //     left_speed = limit_speed(TRACK_BASE_SPEED - 100);
                //     right_speed = limit_speed(TRACK_BASE_SPEED - 100);
                // }

                // motorA(left_speed);
                // motorB(right_speed);
                break;
            }

            bias = track_bias(d1, d2, d3, d4, d5, d6, d7, d8);
            if (bias != 0) {
                g_last_bias = bias;
            }
            left_speed = limit_speed(TRACK_BASE_SPEED + bias);
            right_speed = limit_speed(TRACK_BASE_SPEED - bias);
            motorA(left_speed);
            motorB(right_speed);
            break;

        case TRACK4_DONE:
        default:
            // 第4问完成4圈后保持停车
            motorA(0);
            motorB(0);
            break;
    }

    g_track4_last_all_white = now_all_white;
}
