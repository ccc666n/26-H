#include "camera.h"

#include <stdio.h>
#include <string.h>

#include "ti_msp_dl_config.h"

#define CAMERA_LINE_BUFFER_SIZE (128U)

/* 鎽勫儚澶翠覆鍙ｆ帴鏀剁姸鎬?*/

/* 鎽勫儚澶碅SCII琛屾帴鏀剁紦瀛橈紝涓柇鍙礋璐ｆ敹瀛楃 */
static volatile char g_camera_rx_line[CAMERA_LINE_BUFFER_SIZE];
static volatile uint8_t g_camera_rx_index = 0U;
static volatile uint8_t g_camera_line_ready = 0U;

/* 鏈€鏂伴拤瀛愭暟閲?*/
static volatile uint8_t g_camera_nail_count = 0U;
static volatile uint8_t g_camera_new_nail_count = 0U;

/* 鏈€鏂版暟瀛楄瘑鍒粨鏋滐細鍜岄拤瀛愭暟閲忚瘑鍒垎寮€淇濆瓨 */
static volatile uint8_t g_camera_target_digit = 0U;
static volatile uint8_t g_camera_matched_digit = 0U;
static volatile uint8_t g_camera_matched_side = 0U;
static volatile uint8_t g_camera_new_digit_data = 0U;

/* 最新钢球识别结果，外部通过简单getter读取 */
static volatile uint8_t g_camera_ball_valid = 0U;
static volatile uint8_t g_camera_new_ball_data = 0U;
static volatile float g_camera_ball_error_cm = 0.0f;
static volatile float g_camera_ball_abs_pos_cm = 0.0f;  // 钢球绝对坐标，单位cm
static volatile float g_camera_ball_speed = 0.0f;
static volatile float g_camera_ball_confidence = 0.0f;

void Camera_Init(void)
{
    g_camera_rx_index = 0U;
    g_camera_line_ready = 0U;
    g_camera_nail_count = 0U;
    g_camera_new_nail_count = 0U;
    g_camera_target_digit = 0U;
    g_camera_matched_digit = 0U;
    g_camera_matched_side = 0U;
    g_camera_new_digit_data = 0U;
    g_camera_ball_valid = 0U;
    g_camera_new_ball_data = 0U;
    g_camera_ball_error_cm = 0.0f;
    g_camera_ball_abs_pos_cm = 0.0f;
    g_camera_ball_speed = 0.0f;
    g_camera_ball_confidence = 0.0f;

    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
}

void Camera_Task(void)
{
    char line[CAMERA_LINE_BUFFER_SIZE];

    if (g_camera_line_ready == 0U) {
        return;
    }

    NVIC_DisableIRQ(UART_2_INST_INT_IRQN);
    for (uint8_t i = 0U; i < CAMERA_LINE_BUFFER_SIZE; i++) {
        line[i] = (char)g_camera_rx_line[i];
        if (line[i] == '\0') {
            break;
        }
    }
    g_camera_line_ready = 0U;
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    char *star = strchr(line, '*');
    if ((line[0] != '$') || (star == 0)) {
        return;
    }

    uint8_t calc_checksum = 0U;
    for (char *p = &line[1]; p < star; p++) {
        calc_checksum ^= (uint8_t)(*p);
    }

    unsigned int recv_checksum = 0U;
    if (sscanf(star + 1, "%2x", &recv_checksum) != 1) {
        return;
    }
    if (calc_checksum != (uint8_t)recv_checksum) {
        return;
    }

    *star = '\0';
    unsigned int valid = 0U;
    unsigned long timestamp = 0UL;
    float error_cm = 0.0f;
    float abs_pos_cm = 0.0f;
    float speed = 0.0f;
    float confidence = 0.0f;

    if (sscanf(line, "$BALL,%u,%f,%f,%f,%f,%lu",
        &valid, &error_cm, &abs_pos_cm, &speed, &confidence, &timestamp) == 6) {
        g_camera_ball_valid = (valid != 0U) ? 1U : 0U;
        g_camera_ball_error_cm = error_cm;
        g_camera_ball_abs_pos_cm = abs_pos_cm;
        g_camera_ball_speed = speed;
        g_camera_ball_confidence = confidence;
        g_camera_new_ball_data = 1U;
    }
}

uint8_t Camera_ReadNailCount(uint8_t *count)
{
    uint8_t has_new_data = 0U;

    if (count == 0) {
        return 0U;
    }

    NVIC_DisableIRQ(UART_2_INST_INT_IRQN);
    if (g_camera_new_nail_count != 0U) {
        *count = g_camera_nail_count;
        g_camera_new_nail_count = 0U;
        has_new_data = 1U;
    }
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    return has_new_data;
}

void Camera_DigitReset(void)
{
    NVIC_DisableIRQ(UART_2_INST_INT_IRQN);
    g_camera_target_digit = 0U;
    g_camera_matched_digit = 0U;
    g_camera_matched_side = 0U;
    g_camera_new_digit_data = 0U;
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
}

uint8_t Camera_ReadDigit(CameraDigit_t *data)
{
    uint8_t has_new_data = 0U;

    if (data == 0) {
        return 0U;
    }

    NVIC_DisableIRQ(UART_2_INST_INT_IRQN);
    if (g_camera_new_digit_data != 0U) {
        data->target_digit = g_camera_target_digit;
        data->matched_digit = g_camera_matched_digit;
        data->matched_side = g_camera_matched_side;
        g_camera_new_digit_data = 0U;
        has_new_data = 1U;
    }
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    return has_new_data;
}

uint8_t Camera_GetTargetDigit(void)
{
    uint8_t value;

    NVIC_DisableIRQ(UART_2_INST_INT_IRQN);
    value = g_camera_target_digit;
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    return value;
}

uint8_t Camera_GetMatchedDigit(void)
{
    uint8_t value;

    NVIC_DisableIRQ(UART_2_INST_INT_IRQN);
    value = g_camera_matched_digit;
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    return value;
}

uint8_t Camera_GetMatchedSide(void)
{
    uint8_t value;

    NVIC_DisableIRQ(UART_2_INST_INT_IRQN);
    value = g_camera_matched_side;
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    return value;
}

uint8_t Camera_HasMatchedDigit(void)
{
    uint8_t has_matched;

    NVIC_DisableIRQ(UART_2_INST_INT_IRQN);
    has_matched = (g_camera_matched_digit != 0U) ? 1U : 0U;
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    return has_matched;
}

uint8_t Camera_HasBall(void)
{
    uint8_t has_ball;

    NVIC_DisableIRQ(UART_2_INST_INT_IRQN);
    has_ball = g_camera_ball_valid;
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    return has_ball;
}

uint8_t Camera_HasNewBallData(void)
{
    uint8_t has_new_data;

    NVIC_DisableIRQ(UART_2_INST_INT_IRQN);
    has_new_data = g_camera_new_ball_data;
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    return has_new_data;
}

float Camera_GetBallError(void)
{
    float value;

    NVIC_DisableIRQ(UART_2_INST_INT_IRQN);
    value = g_camera_ball_error_cm;
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    return value;
}

float Camera_GetBallAbsPos(void)
{
    float value;

    NVIC_DisableIRQ(UART_2_INST_INT_IRQN);
    value = g_camera_ball_abs_pos_cm;
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    return value;
}

float Camera_GetBallSpeed(void)
{
    float value;

    NVIC_DisableIRQ(UART_2_INST_INT_IRQN);
    value = g_camera_ball_speed;
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    return value;
}

float Camera_GetBallConfidence(void)
{
    float value;

    NVIC_DisableIRQ(UART_2_INST_INT_IRQN);
    value = g_camera_ball_confidence;
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    return value;
}

void UART_2_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_2_INST)) {
        case DL_UART_MAIN_IIDX_RX: {
            char ch = (char)DL_UART_Main_receiveData(UART_2_INST);

            if (g_camera_line_ready != 0U) {
                break;
            }

            if (ch == '$') {
                g_camera_rx_index = 0U;
                g_camera_rx_line[g_camera_rx_index++] = ch;
            } else if (g_camera_rx_index == 0U) {
                break;
            } else if (ch == '\r') {
                break;
            } else if (ch == '\n') {
                g_camera_rx_line[g_camera_rx_index] = '\0';
                g_camera_line_ready = 1U;
                g_camera_rx_index = 0U;
            } else if (g_camera_rx_index < (CAMERA_LINE_BUFFER_SIZE - 1U)) {
                g_camera_rx_line[g_camera_rx_index++] = ch;
            } else {
                /* 缓冲区溢出后丢弃当前帧，等待下一个$重新开始 */
                g_camera_rx_index = 0U;
            }
            break;
        }

        default:
            break;
    }
}
