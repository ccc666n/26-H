#ifndef NO_MCU_GANV_GRAYSCALE_SENSOR_CONFIG_H_
#define NO_MCU_GANV_GRAYSCALE_SENSOR_CONFIG_H_
#include <string.h>
#include "ti_msp_dl_config.h"
#include "ADC.h"
#include "delay.h"
/**************************** 浼犳劅鍣ㄧ増鏈厤缃?****************************/
#define Class		    0

/**************************** ADC鍒嗚鲸鐜囬厤缃?****************************/
#define _14Bits 0     // 14浣岮DC妯″紡
#define _12Bits 1     // 12浣岮DC妯″紡
#define _10Bits 2     // 10浣岮DC妯″紡
#define _8Bits  3     // 8浣岮DC妯″紡

/**************************** 鐢ㄦ埛鍙厤缃尯鍩?***************************/
// 浼犳劅鍣ㄧ増鏈€夋嫨
#define Sensor_Edition Class


/************************* 鏍规嵁鍗曠墖鏈鸿嚜琛岄€夋嫨 **************************/
// 杈撳嚭缁撴灉鏂瑰悜锛屼笌棰勬湡鏂瑰悜涓嶅悓閫?
#define Direction 1
// ADC鍒嗚鲸鐜囬€夋嫨锛堝洓閫変竴锛?
// #define Sensor_ADCbits _14Bits
#define Sensor_ADCbits _12Bits
// #define Sensor_ADCbits _10Bits
// #define Sensor_ADCbits _8Bits

/*************************** 纭欢鎶借薄灞傞厤缃?****************************/
// GPIO鍦板潃鍒囨崲瀹忓畾涔?
#ifdef GPIO_AnalogGray_Address_PORT
#define Gray_Address_PORT       GPIO_AnalogGray_Address_PORT
#define Gray_Address_PIN_0_PIN  GPIO_AnalogGray_Address_AD1_PIN
#define Gray_Address_PIN_1_PIN  GPIO_AnalogGray_Address_AD2_PIN
#define Gray_Address_PIN_2_PIN  GPIO_AnalogGray_Address_AD3_PIN
#define Switch_Address_0(i) ((i)?(DL_GPIO_setPins(Gray_Address_PORT,Gray_Address_PIN_0_PIN)) : (DL_GPIO_clearPins(Gray_Address_PORT,Gray_Address_PIN_0_PIN)))// 鍦板潃浣?鎺у埗

#define Switch_Address_1(i) ((i)?(DL_GPIO_setPins(Gray_Address_PORT,Gray_Address_PIN_1_PIN)) : (DL_GPIO_clearPins(Gray_Address_PORT,Gray_Address_PIN_1_PIN)))// 鍦板潃浣?鎺у埗

#define Switch_Address_2(i) ((i)?(DL_GPIO_setPins(Gray_Address_PORT,Gray_Address_PIN_2_PIN)) : (DL_GPIO_clearPins(Gray_Address_PORT,Gray_Address_PIN_2_PIN)))// 鍦板潃浣?鎺у埗
#else
/* 未使用模拟灰度地址线时不占用GPIO，只保留函数编译通过 */
#define Switch_Address_0(i) ((void)(i))
#define Switch_Address_1(i) ((void)(i))
#define Switch_Address_2(i) ((void)(i))
#endif

// ADC鍊艰幏鍙栨帴鍙ｅ畯瀹氫箟 闇€瑕佽嚜宸辨牴鎹崟鐗囨満瀹屾垚瀵瑰簲浣嶆暟鐨凙DC閲囨牱鍑芥暟
#define Get_adc_of_user() adc_getValue()  // 鐢ㄦ埛鑷畾涔堿DC璇诲彇鍑芥暟
/**********************************************************************/

/*************************** 浼犳劅鍣ㄦ暟鎹粨鏋?***************************/
typedef struct {
    unsigned short Analog_value[8];    // 鍘熷妯℃嫙閲忓€?
    unsigned short Normal_value[8];   // 褰掍竴鍖栧悗鐨勫€?
    unsigned short Calibrated_white[8]; // 鐧芥牎鍑嗗熀鍑嗗€?
    unsigned short Calibrated_black[8]; // 榛戞牎鍑嗗熀鍑嗗€?
    unsigned short Gray_white[8];      // 鐧藉钩琛＄伆搴﹀€?
    unsigned short Gray_black[8];      // 榛戝钩琛＄伆搴﹀€?
    double Normal_factor[8];          // 褰掍竴鍖栫郴鏁?
    double bits;                      // ADC鍒嗚鲸鐜囧搴斾綅鏁?
    unsigned char Digtal;              // 鏁板瓧杈撳嚭鐘舵€?
    unsigned char Time_out;            // 瓒呮椂鏍囧織
    unsigned char Tick;                // 鏃跺熀璁℃暟鍣?
    unsigned char ok;                  // 浼犳劅鍣ㄥ氨缁爣蹇?
} No_MCU_Sensor;

#ifdef __cplusplus
extern "C" {
#endif

/*************************** 鍑芥暟澹版槑鍖哄煙 *****************************/
// 鍒濆鍖栧嚱鏁?
void No_MCU_Ganv_Sensor_Init_Frist(No_MCU_Sensor* sensor); // 棣栨鍒濆鍖?
void No_MCU_Ganv_Sensor_Init(No_MCU_Sensor* sensor,unsigned short* Calibrated_white, unsigned short* Calibrated_black);// 甯︽牎鍑嗗弬鏁扮殑鍒濆鍖?

// 浠诲姟澶勭悊鍑芥暟
void No_Mcu_Ganv_Sensor_Task_Without_tick(No_MCU_Sensor* sensor); // 鏃犳椂鍩虹増鏈?

// 鐢ㄦ埛鎺ュ彛鍑芥暟
unsigned char Get_Digtal_For_User(No_MCU_Sensor* sensor);          									// 鑾峰彇鏁板瓧閲?
unsigned char Get_Normalize_For_User(No_MCU_Sensor* sensor,unsigned short* result); // 鑾峰彇褰掍竴鍖栧€?
unsigned char Get_Anolog_Value(No_MCU_Sensor* sensor,unsigned short* result);       // 鑾峰彇妯℃嫙鍊?

#ifdef __cplusplus
}
#endif

#endif /* NO_MCU_GANV_GRAYSCALE_SENSOR_CONFIG_H_ */
