#include "Servo.h"

void Servo_Init(void)
{
    Serial2_Init();
}

/// @brief 舵机回到初始位置
void Init_Status(void)
{
    Claw(100);
    Platform(0);
    Hook_Left(UP);
    Hook_Middle(UP);
    Hook_Right(UP);
}

/// @brief 机械爪控制
/// @param Location 机械爪张开的大小（0~100）
void Claw(int16_t Location)
{
    if (Location > 100) Location = 100;
    Location = (Claw_MAX - Claw_MIN) * Location / 100;
    moveServo(SERVO_Claw, Claw_MIN + Location, Time_ms);
    Delay_ms(300);
}

/// @brief 机械平台控制
/// @param Location 机械平台高度（0~100）
void Platform(int16_t Location)
{
    if (Location > 100) Location = 100;
    Location = (Platform_MAX - Platform_MIN) * Location / 100;
    moveServo(SERVO_Platform, Platform_MIN + Location, Time_ms);
    Delay_ms(200);
}

/// @brief 机械钩子控制(左)
/// @param State 状态 UP:抬起 Down:放下
void Hook_Left(uint8_t State)
{
    if (State)
        moveServo(SERVO_Hook_Left, Hook_Left_UP, Time_Hook);
    else
        moveServo(SERVO_Hook_Left, Hook_Left_Down, Time_Hook);
}

/// @brief 机械钩子控制(中间)
/// @param State 状态 UP:抬起 Down:放下
void Hook_Middle(uint8_t State)
{
    if (State)
        moveServo(SERVO_Hook_Middle, Hook_Middle_UP, Time_Hook);
    else
        moveServo(SERVO_Hook_Middle, Hook_Middle_Down, Time_Hook);
}

/// @brief 机械钩子控制(右)
/// @param State 状态 UP:抬起 Down:放下
void Hook_Right(uint8_t State)
{
    if (State)
        moveServo(SERVO_Hook_Right, Hook_Right_UP, Time_Hook);
    else
        moveServo(SERVO_Hook_Right, Hook_Right_Down, Time_Hook);
}

void Hook_All_Down(void)
{
    Delay_ms(500);
    Hook_Left(Down);
    Hook_Middle(Down);
    Hook_Right(Down);
    Delay_ms(800);
}

void Hook_All_UP(void)
{
    Hook_Left(UP);
    Hook_Middle(UP);
    Hook_Right(UP);
    Delay_ms(500);
}

// void Hook_Release(uint8_t ColorData)
// {
//     switch (ColorData) {
//         case Left:

//     }
// }

void Hook_Release_FrontLeft(void)
{
    Hook_Left(UP);
    Hook_Middle(UP);
    Hook_Right(Down);
    Delay_ms(500);
}

void Hook_Release_FrontRight(void)
{
    Hook_Left(Down);
    Hook_Middle(UP);
    Hook_Right(UP);
    Delay_ms(500);
}

void Hook_Release_BehindLeft(void)
{
    Hook_Left(UP);
    Hook_Middle(Down);
    Hook_Right(Down);
    Delay_ms(500);
}

void Hook_Release_BehindMiddle(void)
{
    Hook_Left(Down);
    Hook_Middle(UP);
    Hook_Right(Down);
    Delay_ms(500);
}

void Hook_Release_BehindRight(void)
{
    Hook_Left(Down);
    Hook_Middle(Down);
    Hook_Right(UP);
    Delay_ms(500);
}
