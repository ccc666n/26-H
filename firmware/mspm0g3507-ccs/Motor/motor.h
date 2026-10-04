#ifndef MOTOR_H_
#define MOTOR_H_

void motor_init(void);
void motorA(int va);
void motorB(int vb);
void motors_stop(void);
void motors_brake(void);
void motors_brake_ms(int ms);
int motor_soft_speed(int target, int start, int step, int reset);//缓慢加速到设定速度
void motors_sleep(void);

#endif /* MOTOR_H_ */
