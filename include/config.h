#include <Arduino.h>
// #include <SPI.h>
// #include <Ethernet.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

// for esp32 core
#define CORE_0 0
#define CORE_1 1
// end for esp32 core


// for W5500
#define PORT 502
#define CS_PIN 5
// end for W5500

// 74HC595
#define DS 22
#define SHcp 26
#define STcp 27
// end 74HC595

//M1
#define M1_Enc_A 13
#define M1_Enc_B 25
#define M1_PWM 16
#define M1_FW B01000000 
#define M1_BW B00100000
#define ST B00000000
//end M1

//M2
#define M2_Enc_A 33
#define M2_Enc_B 32
#define M2_PWM 17
#define M2_FW B01000000 
#define M2_BW B00100000
// #define ST B00000000
//end M2

//M3
#define M3_Enc_A 34
#define M3_Enc_B 35
#define M3_PWM 21
#define M3_FW B01000000 
#define M3_BW B00100000
// #define ST B00000000
//end M3


#define M3_ST_M2_ST_M1_ST B00000000
#define M3_ST_M2_ST_M1_BW B00000010
#define M3_ST_M2_ST_M1_FW B00000100
#define M3_ST_M2_BW_M1_ST B00001000
#define M3_ST_M2_BW_M1_BW B00001010
#define M3_ST_M2_BW_M1_FW B00001100
#define M3_ST_M2_FW_M1_ST B00010000
#define M3_ST_M2_FW_M1_BW B00010010
#define M3_ST_M2_FW_M1_FW B00010100
#define M3_BW_M2_ST_M1_ST B00100000
#define M3_BW_M2_ST_M1_BW B00100010
#define M3_BW_M2_ST_M1_FW B00100100
#define M3_BW_M2_BW_M1_ST B00101000
#define M3_BW_M2_BW_M1_BW B00101010
#define M3_BW_M2_BW_M1_FW B00101100
#define M3_BW_M2_FW_M1_ST B00110000
#define M3_BW_M2_FW_M1_BW B00110010
#define M3_BW_M2_FW_M1_FW B00110100
#define M3_FW_M2_ST_M1_ST B01000000
#define M3_FW_M2_ST_M1_BW B01000010
#define M3_FW_M2_ST_M1_FW B01000100
#define M3_FW_M2_BW_M1_ST B01001000
#define M3_FW_M2_BW_M1_BW B01001010
#define M3_FW_M2_BW_M1_FW B01001100
#define M3_FW_M2_FW_M1_ST B01010000
#define M3_FW_M2_FW_M1_BW B01010010
#define M3_FW_M2_FW_M1_FW B01010100

 






