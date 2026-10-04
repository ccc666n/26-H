#ifndef TRACK_H_
#define TRACK_H_

#include <stdint.h>

void track_init(void);
void track_reset(uint8_t mode);
void track1(void);
void track1_test();
void track1_test1();
void track2(void);
void track2_weizhiPID(void);
void track2_zengliangPID(void);
void track2_monihuidu(void);
void track2_PID1(void);
void track3(void);
void track3_zengliangPID(void);
void track4(void);
uint8_t track_digital(uint8_t channel);

#define D1 track_digital(1)
#define D2 track_digital(2)
#define D3 track_digital(3)
#define D4 track_digital(4)
#define D5 track_digital(5)
#define D6 track_digital(6)
#define D7 track_digital(7)
#define D8 track_digital(8)

#endif /* TRACK_H_ */
