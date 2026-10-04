#ifndef CAMERA_H_
#define CAMERA_H_

#include <stdint.h>

typedef struct {
    uint8_t target_digit;   // 起始目标数字
    uint8_t matched_digit;  // 当前匹配到的目标数字，未识别到为0
    uint8_t matched_side;   // 目标位置：0无，1左，2右
} CameraDigit_t;

/* 初始化摄像头串口接收 */
void Camera_Init(void);

/* 摄像头任务接口，预留给后续数据处理 */
void Camera_Task(void);

/* 读取新的钉子数量，返回1表示有新数据 */
uint8_t Camera_ReadNailCount(uint8_t *count);

/* 清空数字识别结果，用于重新开始一轮测试 */
void Camera_DigitReset(void);

/* 读取最新数字识别结果，返回1表示有新数据 */
uint8_t Camera_ReadDigit(CameraDigit_t *data);

/* 获取起始目标数字，未识别到为0 */
uint8_t Camera_GetTargetDigit(void);

/* 获取当前匹配到的目标数字，未识别到为0 */
uint8_t Camera_GetMatchedDigit(void);

/* 获取目标在哪边：0无，1左，2右 */
uint8_t Camera_GetMatchedSide(void);

/* 是否识别到目标数字 */
uint8_t Camera_HasMatchedDigit(void);

/* 是否识别到钢球 */
uint8_t Camera_HasBall(void);

/* 是否收到新的钢球数据 */
uint8_t Camera_HasNewBallData(void);

/* 获取钢球误差距离，单位cm */
float Camera_GetBallError(void);

/* 获取钢球绝对坐标，单位cm */
float Camera_GetBallAbsPos(void);

/* 获取钢球速度，暂时只保存 */
float Camera_GetBallSpeed(void);

/* 获取识别置信度 */
float Camera_GetBallConfidence(void);

#endif /* CAMERA_H_ */
