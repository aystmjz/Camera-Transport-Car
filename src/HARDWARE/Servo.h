#ifndef __SERVOCONTROL_H__
#define __SERVOCONTROL_H__
#include "stm32f10x.h"
#include "LobotServoController.h"
#include "Delay.h"
#include "Usart.h"

#define UP                1
#define Down              0

#define SERVO_Claw        0
#define SERVO_Platform    1
#define SERVO_Hook_Left   4
#define SERVO_Hook_Middle 5
#define SERVO_Hook_Right  6
#define Time_ms           200
#define Time_Hook         300

#if 1

#define Claw_MAX         1700
#define Claw_MIN         1300
#define Platform_MAX     1700
#define Platform_MIN     610
#define Hook_Left_UP     1500 // 1800
#define Hook_Left_Down   820
#define Hook_Middle_UP   900
#define Hook_Middle_Down 1720
#define Hook_Right_UP    1000 // 700
#define Hook_Right_Down  1720

#else
#define Claw_MAX         1700
#define Claw_MIN         1280//1290
#define Platform_MAX     2040
#define Platform_MIN     1030
#define Hook_Left_UP     1600 // 1800
#define Hook_Left_Down   910
#define Hook_Middle_UP   1000
#define Hook_Middle_Down 2150
#define Hook_Right_UP    1000 // 700
#define Hook_Right_Down  1820
#define OTHER_CAR
#endif

void Servo_Init(void);
void Init_Status(void);
void Claw(int16_t Location);
void Platform(int16_t Location);
void Hook_Left(uint8_t State);
void Hook_Middle(uint8_t State);
void Hook_Right(uint8_t State);

void Hook_All_Down(void);
void Hook_All_UP(void);
void Hook_Release_FrontLeft(void);
void Hook_Release_FrontRight(void);
void Hook_Release_BehindLeft(void);
void Hook_Release_BehindMiddle(void);
void Hook_Release_BehindRight(void);

#endif
