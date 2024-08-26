#include "OLED.h"
#include "OpenCV.h"
#include "Delay.h"
#include "Servo.h"
#include "Encoder.h"
#include "Timer.h"
#include "PID.h"
#include "Buzzer.h"
#include "HWT101CT.h"

PID PID_L, PID_R;
extern int16_t MOTOR_LeftSpead, MOTOR_RightSpead;
extern float Angle_SET;

uint8_t Buzzer_Flag = 0;
uint8_t Buzzer_Debug = 0;
uint8_t Camera_Flag = 0;
uint8_t Sensor_Spin_Flag = 0;
uint8_t Sensor_Lift_Flag = 0;
#define Buzzer_Delay 50

#define Data(x) getUsartBuf((x))

#define MOTOR_SpeadDelta 0
#define E(camera) (MOTOR_SpeadDelta + Get_Excursion(camera))
#define E_(camera) (MOTOR_SpeadDelta - Get_Excursion(camera))
#define D(camera) Get_Condition(camera)
#define C Get_Color()

#define _Distance 7000

#define Spin_Distance 500

#define Spead 40
#ifdef OTHER_CAR
#define A_Spead 65 // 60
#else
#define A_Spead 60 // 55
#endif
#define P_Spead 30
#define B_Spead 30
#define F_Spead 25

#define DELAY_TIME 100

int8_t DeltaAngleToSpead(void);
void Display__()
{
    OLED_ShowString(1, 1, "X:");
    // OLED_ShowSignedNum(1, 3, getUsartBuf_float(2) * 10, 4);
    OLED_ShowString(2, 1, "Y:");
    OLED_ShowSignedNum(2, 3, HWT_getAngle(), 4);
    OLED_ShowString(3, 1, "Flag:");
    OLED_ShowSignedNum(3, 6, getUsartBuf(10), 2);
    OLED_ShowString(4, 1, "DATE:");
}
void Display()
{
    OLED_ShowSignedNum(2, 1, PID_L.e_l, 5);
    // OLED_ShowSignedNum(2, 1, PID_L.e_l, 5);
    OLED_ShowSignedNum(2, 9, PID_R.e_l, 5);
    OLED_ShowSignedNum(3, 1, PID_L.TOTAL_OUT, 5);
    OLED_ShowSignedNum(3, 9, PID_R.TOTAL_OUT, 5);
    OLED_ShowSignedNum(4, 1, MOTOR_LeftSpead, 3);
    OLED_ShowSignedNum(4, 6, PID_L.e_l - PID_R.e_l, 3);
    OLED_ShowSignedNum(4, 11, MOTOR_RightSpead, 3);
    // OLED_ShowSignedNum(1,1,Encoder_Left_Get(),5);
    OLED_ShowSignedNum(1, 9, Encoder_Right_Get(), 5);
    OLED_ShowSignedNum(1, 1, DeltaAngleToSpead(), 3);
    // OLED_ShowNum(1, 1, Data(1), 2);E
    // OLED_ShowNum(1, 3, Data(2), 2);
    OLED_ShowSignedNum(1, 5, E(BEHIND), 3);
    // OLED_ShowSignedNum(1, 7, E_, 1);
}

void Display_()
{

    OLED_ShowSignedNum(1, 1, HWT_getAngle(), 4);
    OLED_ShowSignedNum(1, 6, HWT_getAngleSpeed(), 4);
}

#define Angle_K 0.15
#define HWT_Angle (HWT_getAngle())
int8_t DeltaAngleToSpead(void)
{
    float spead;
    spead = HWT_getDeltaAngle(Angle_SET) * Angle_K;
    if (spead > 5)
        spead = 5;
    if (spead < -5)
        spead = -5;
    return (int8_t)spead;
}

#define MOTOR_SlowLong 600
void MOTOR_X(int32_t x, Camera camera, int32_t spead)
{
    MOTOR_Clear(MOTOR_Left | MOTOR_Right);
    if (x >= 0)
    {
        if (x >= MOTOR_SlowLong)
        {
            while ((MOTOR_CONTROL(spead / 2 + E(camera) - DeltaAngleToSpead(), x, MOTOR_Left) > (x - MOTOR_SlowLong)) & (MOTOR_CONTROL(spead / 2 + E_(camera) + DeltaAngleToSpead(), x, MOTOR_Right) > (x - MOTOR_SlowLong)))
            { /*Display();*/
            }
            while (MOTOR_CONTROL(spead + E(camera) - DeltaAngleToSpead(), x, MOTOR_Left) > MOTOR_SlowLong & MOTOR_CONTROL(spead + E_(camera) + DeltaAngleToSpead(), x, MOTOR_Right) > MOTOR_SlowLong)
            { /*Display();*/
            }
            while (MOTOR_CONTROL(spead / 2 + E(camera) - DeltaAngleToSpead(), x, MOTOR_Left) & MOTOR_CONTROL(spead / 2 + E_(camera) + DeltaAngleToSpead(), x, MOTOR_Right))
            {
            }
        }
        else
        {
            while (MOTOR_CONTROL(spead + E(camera) - DeltaAngleToSpead(), x, MOTOR_Left) & MOTOR_CONTROL(spead + E_(camera) + DeltaAngleToSpead(), x, MOTOR_Right))
            { /*Display();*/
            }
        }
    }
    else
    {
        if (x <= -MOTOR_SlowLong)
        {
            while ((MOTOR_CONTROL(spead / 2 + E_(camera) + DeltaAngleToSpead(), x, MOTOR_Left) > (-x - MOTOR_SlowLong)) & (MOTOR_CONTROL(spead / 2 + E(camera) - DeltaAngleToSpead(), x, MOTOR_Right) > (-x - MOTOR_SlowLong)))
            { /*Display();*/
            }
            while (MOTOR_CONTROL(spead + E_(camera) + DeltaAngleToSpead(), x, MOTOR_Left) > MOTOR_SlowLong & MOTOR_CONTROL(spead + E(camera) - DeltaAngleToSpead(), x, MOTOR_Right) > MOTOR_SlowLong)
            { /* Display();*/
            }
            while (MOTOR_CONTROL(spead / 2 + E_(camera) + DeltaAngleToSpead(), x, MOTOR_Left) & MOTOR_CONTROL(spead / 2 + E(camera) - DeltaAngleToSpead(), x, MOTOR_Right))
            { /* Display();*/
            }
        }
        else
        {
            while (MOTOR_CONTROL(spead + E_(camera) + DeltaAngleToSpead(), x, MOTOR_Left) & MOTOR_CONTROL(spead + E(camera) - DeltaAngleToSpead(), x, MOTOR_Right))
            { /* Display();*/
            }
        }
    }
    MOTOR_Clear(MOTOR_Left | MOTOR_Right);
    MOTOR_CONTROL(spead, 0, MOTOR_Left | MOTOR_Right);
    Delay_ms(DELAY_TIME);
}

#define BackCenter 1770
#ifdef OTHER_CAR
#define PutBlock 950
#else
#define PutBlock 900
#endif

#define ScanSpead 15
#define BackSpead 30
#define Adjust 1
#define NoAdjust 0
uint8_t MOTOR_Adjust_X(int x, Camera camera, uint16_t conditionLong, uint8_t spead)
{
    MOTOR_Clear(MOTOR_Left | MOTOR_Right);
    if (x >= 0)
    {
        while (MOTOR_CONTROL(spead + E(camera) - DeltaAngleToSpead(), x, MOTOR_Left) & MOTOR_CONTROL(spead + E_(camera) + DeltaAngleToSpead(), x, MOTOR_Right))
        {
            if (D(camera))
            {
                Buzzer_One();
                MOTOR_Clear(MOTOR_Left | MOTOR_Right);
                MOTOR_CONTROL(spead, 0, MOTOR_Left | MOTOR_Right);
                Delay_ms(500);
                MOTOR_Clear(MOTOR_Left | MOTOR_Right);
                while (MOTOR_CONTROL(spead + E(NULL_Camera) - DeltaAngleToSpead(), conditionLong, MOTOR_Left) & MOTOR_CONTROL(spead + E_(NULL_Camera) + DeltaAngleToSpead(), conditionLong, MOTOR_Right))
                {
                }
                MOTOR_Clear(MOTOR_Left | MOTOR_Right);
                MOTOR_CONTROL(spead, 0, MOTOR_Left | MOTOR_Right);
                Buzzer_Tow(100);
                return 0;
            }
        }
    }
    else
    {
        while (MOTOR_CONTROL(spead + E_(camera) + DeltaAngleToSpead(), x, MOTOR_Left) & MOTOR_CONTROL(spead + E(camera) - DeltaAngleToSpead(), x, MOTOR_Right))
        { /* Display();*/
            if (D(camera))
            {
                Buzzer_One();
                MOTOR_Clear(MOTOR_Left | MOTOR_Right);
                MOTOR_CONTROL(spead, 0, MOTOR_Left | MOTOR_Right);
                Delay_ms(500);
                MOTOR_Clear(MOTOR_Left | MOTOR_Right);
                while (MOTOR_CONTROL(spead + E_(NULL_Camera) + DeltaAngleToSpead(), -conditionLong, MOTOR_Left) & MOTOR_CONTROL(spead + E(NULL_Camera) - DeltaAngleToSpead(), -conditionLong, MOTOR_Right))
                {
                }
                MOTOR_CONTROL(spead, 0, MOTOR_Left | MOTOR_Right);
                Buzzer_Tow(100);
                return 0;
            }
        }
    }
    MOTOR_Clear(MOTOR_Left | MOTOR_Right);
    MOTOR_CONTROL(spead, 0, MOTOR_Left | MOTOR_Right);
    Buzzer_Tow(400);
    return 1;
}
// #define MOTOR_F(x, spead)                                                                                 \
//     MOTOR_Clear(MOTOR_Left | MOTOR_Right);                                                                \
//     while (MOTOR_CONTROL(spead, (x), MOTOR_Left) | MOTOR_CONTROL(Spead, (x), MOTOR_Right)) { Display(); } \
//     Delay_ms(DELAY_TIME)
void MOTOR_F(int32_t x, int32_t spead)
{
    MOTOR_Clear(MOTOR_Left | MOTOR_Right);
    if (x >= 0)
    {
        while (MOTOR_CONTROL(spead - DeltaAngleToSpead(), x, MOTOR_Left) & MOTOR_CONTROL(spead + DeltaAngleToSpead(), x, MOTOR_Right))
        { /*Display();*/
        }
    }
    else
    {
        while (MOTOR_CONTROL(spead + DeltaAngleToSpead(), x, MOTOR_Left) & MOTOR_CONTROL(spead - DeltaAngleToSpead(), x, MOTOR_Right))
        { /* Display();*/
        }
    }
    Delay_ms(500);
}

void Put_TwoBlock(int32_t Back_spead)
{
    uint8_t ret;
    Claw(100);
    Platform(40);
    Buzzer_One();
    MOTOR_X(1600, FRONT, P_Spead);
    Platform(0);
    Claw(0);
    Buzzer_One();
    MOTOR_X(1300, FRONT, P_Spead);
    Platform(80);
    Buzzer_One();
    MOTOR_X(1200, FRONT, P_Spead);
    Platform(40);
    Delay_ms(200);
    Claw(90);
    // Delay_ms(200);
    Platform(0);
    Claw(0);
    ret = MOTOR_Adjust_X(3300, FRONT, PutBlock, 15);
    Claw(100);
    MOTOR_F(-600, F_Spead);
    MOTOR_Spin(Left, 180, Spead, A_Spead);
    if (ret) // 未识别到黑线
        MOTOR_Adjust_X(6850, FRONT_NEAR, BackCenter, Back_spead);
    else
        MOTOR_X(3200, FRONT, Back_spead);
}

void Put_ThreeBlock(int32_t Back_spead)
{
    uint8_t ret;
    Claw(100);
    Platform(40);
    Buzzer_One();
    MOTOR_X(1600, FRONT, P_Spead);
    Platform(0);
    Claw(0);
    Buzzer_One();
    MOTOR_X(1300, FRONT, P_Spead);
    Platform(80);
    Buzzer_One();
    MOTOR_X(1200, FRONT, P_Spead);
    Platform(40);
    Delay_ms(200);
    Claw(90);
    // Delay_ms(200);
    Platform(0);
    Claw(0);
    Platform(80);
    Buzzer_One();
    ret = MOTOR_Adjust_X(3200, FRONT, 0, 15);
    Platform(40);
    Delay_ms(200);
    Claw(90);
    // Delay_ms(200);
    Platform(0);
    Claw(0);
    MOTOR_X(PutBlock, FRONT, 15);
    Claw(100);
    MOTOR_F(-600, F_Spead);
    MOTOR_Spin(Left, 180, Spead, A_Spead);
    if (ret) // 未识别到黑线
        MOTOR_Adjust_X(6850, FRONT_NEAR, BackCenter, Back_spead);
    else
        MOTOR_X(3200, FRONT, Back_spead);
}

void Place_Block(uint8_t ColorData, uint8_t adjust)
{
    MOTOR_PutSpin(ColorData, Spead, A_Spead);
    MOTOR_X(3200, FRONT, Spead);
    Platform(0);
    Claw(30);
    Platform(50);
    Claw(0);
    Platform(70);
    MOTOR_F(-600, F_Spead);
    Platform(20);
    MOTOR_Spin(Left, 180, Spead, A_Spead);
    if (adjust)
    {
        Claw(100);
        MOTOR_Adjust_X(6250, FRONT_NEAR, BackCenter, BackSpead);
    }
    else
        MOTOR_X(2500, FRONT, Spead);
}

void Get_Block(uint16_t GetLong)
{
    Buzzer_One();
    MOTOR_X(GetLong, FRONT, Spead);
    Platform(80);
    Buzzer_One();
    MOTOR_X(1200, FRONT, Spead);
    Platform(40);
    // Delay_ms(300);
    Claw(100);
    // Delay_ms(300);
    Platform(0);
    Claw(0);
    Platform(10);
}

#define Ajust_Angle 5
#define Spin_Long 750

int main(void)
{
    Delay_ms(100);
    Buzzer_Init();
    Buzzer_One();
    // OLED_Init();
    OpenCV_Init();
    Servo_Init();
    HWT101CT_Init();
    Timer_Init();
    Encoder_Init();
    PID_Init();
    PID_Inint(&PID_L);
    PID_Inint(&PID_R);
    Init_Status();
    Angle_SET = 0;

    MOTOR_X(4250, FRONT, 50);
    MOTOR_Spin(Right, 45, Spead, A_Spead - 10);
    MOTOR_X(-4100, BEHIND, Spead);
    Save_ColorData();
    Hook_All_Down();
    Delay_ms(500);
    MOTOR_X(4200, FRONT, Spead);

    MOTOR_Spin(Left, 45, Spead, A_Spead);
    MOTOR_X(2500, FRONT, Spead);
    Claw(0);
    Platform(20);
    MOTOR_Spin(Left, 180, Spead, A_Spead);
    MOTOR_X(2400, FRONT, Spead);
    MOTOR_Spin(Right, 90, Spead, A_Spead);
    Get_Block(1800);
    MOTOR_Spin(Left, 180, Spead, A_Spead);
    Get_Block(4000);
    MOTOR_Spin(Left, 180, Spead, A_Spead);
    MOTOR_Adjust_X(6350, FRONT, BackCenter, BackSpead);

    Place_Block(BehindLeft, NoAdjust);
    Place_Block(FrontLeft, NoAdjust);
    Place_Block(BehindRight, Adjust);

    Claw(100);
    MOTOR_PutBackSpin(BehindMiddle, Spead, A_Spead);
    MOTOR_X(-2000, BEHIND, B_Spead);
    Hook_Release_BehindMiddle();
    // MOTOR_SpinAjust(A_Spead, Ajust_Angle);
    //  MOTOR_F(400, F_Spead);
    MOTOR_Adjust_X(3700, FRONT_NEAR, BackCenter, BackSpead);
    Hook_All_Down();

    MOTOR_PutBackSpin(BehindLeft, Spead, A_Spead - 2);
    MOTOR_X(-2000, BEHIND, B_Spead);
    MOTOR_SingleSpin(Left, Spin_Long, Spead);
    Hook_Release_BehindLeft();
    MOTOR_SingleSpin(Right, Spin_Long, Spead);
    MOTOR_Adjust_X(3500, FRONT_NEAR, BackCenter, BackSpead);
    Hook_All_Down();

    MOTOR_PutBackSpin(BehindRight, Spead, A_Spead - 4);
    MOTOR_X(-2000, BEHIND, B_Spead);
    MOTOR_SingleSpin(Right, Spin_Long, Spead);
    Hook_Release_BehindRight();
    MOTOR_SingleSpin(Left, Spin_Long, Spead);
    MOTOR_Adjust_X(3500, FRONT_NEAR, BackCenter, BackSpead);
    Hook_All_Down();

    MOTOR_PutBackSpin(FrontLeft, Spead, A_Spead - 6);
    MOTOR_X(-2000, BEHIND, B_Spead);
    Hook_Release_FrontLeft();
    // MOTOR_SpinAjust(A_Spead, Ajust_Angle);
    // MOTOR_F(400, F_Spead);
    MOTOR_Adjust_X(3700, FRONT_NEAR, BackCenter, BackSpead);
    Hook_All_Down();

    MOTOR_PutBackSpin(FrontRight, Spead, A_Spead - 10);
    MOTOR_X(-2000, BEHIND, B_Spead);
    Hook_Release_FrontRight();
    // MOTOR_SpinAjust(A_Spead, Ajust_Angle);
    // MOTOR_F(400, F_Spead);
    MOTOR_Adjust_X(3700, FRONT_NEAR, BackCenter, BackSpead);
    Hook_All_UP();
    //*************************************

    MOTOR_PutSpin(2, Spead, A_Spead - 10);
    MOTOR_X(-4100, BEHIND, Spead);
    Hook_All_Down();
    MOTOR_X(4200, FRONT, Spead);

    MOTOR_PutBackSpin(BehindMiddle, Spead, A_Spead);
    MOTOR_F(-200, F_Spead);
    Hook_Release_BehindMiddle();
    MOTOR_F(500, F_Spead);
    Hook_All_Down();
    MOTOR_Spin(Right, 180, Spead, A_Spead);
    Put_TwoBlock(B_Spead);

    MOTOR_PutBackSpin(BehindRight, Spead, A_Spead - 2);
    MOTOR_F(-200, F_Spead);
    MOTOR_SingleSpin(Left, Spin_Long, Spead);
    Hook_Release_BehindLeft();
    MOTOR_SingleSpin(Right, Spin_Long, Spead);
    Hook_All_Down();
    MOTOR_SpinAjust(A_Spead - 2, Ajust_Angle);
    MOTOR_F(300, F_Spead);
    MOTOR_Spin(Right, 180, Spead, A_Spead - 2);
    Put_ThreeBlock(B_Spead);

    MOTOR_PutBackSpin(BehindLeft, Spead, A_Spead - 4);
    MOTOR_F(-200, F_Spead);
    MOTOR_SingleSpin(Right, Spin_Long, Spead);
    Hook_Release_BehindRight();
    MOTOR_SingleSpin(Left, Spin_Long, Spead);
    Hook_All_Down();
    MOTOR_SpinAjust(A_Spead - 4, Ajust_Angle);
    MOTOR_F(300, F_Spead);
    MOTOR_Spin(Right, 180, Spead, A_Spead - 4);
    Put_ThreeBlock(B_Spead);

    MOTOR_PutBackSpin(FrontRight, Spead, A_Spead - 6);
    MOTOR_F(-200, F_Spead);
    Hook_Release_FrontLeft();
    MOTOR_F(500, F_Spead);
    Hook_All_Down();
    MOTOR_Spin(Right, 180, Spead, A_Spead - 6);
    Put_TwoBlock(B_Spead);

    MOTOR_PutBackSpin(FrontLeft, Spead, A_Spead - 10);
    MOTOR_F(-200, F_Spead);
    Hook_Release_FrontRight();
    MOTOR_F(500, F_Spead);
    Hook_All_Down();
    MOTOR_Spin(Right, 180, Spead, A_Spead - 10);
    Put_ThreeBlock(60);

    MOTOR_PutBackSpin(3, Spead, A_Spead - 10);
    MOTOR_X(4300, FRONT, 70);
    Buzzer_Tow(100);
    Buzzer_Tow(100);
    return 0;
}

void TIM6_IRQHandler(void)
{
    static uint16_t MOTOR_Spead_Counter = 0, Buzzer_Counter = 0;
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)
    {
        if (MOTOR_Spead_Counter >= 30) // 13
        {
            MOTOR_Spead_Counter = 0;
            MOTOR_Spead_calc();
        }
        PID_L.current_l = Encoder_Left_Get();
        PID_L.current_s = MOTOR_LeftSpead;
        PID_R.current_l = Encoder_Right_Get();
        PID_R.current_s = MOTOR_RightSpead;
        MOTOR_Run_all();
        PID_calc_all();

        if (Buzzer_Flag)
        {
            Buzzer_ON();
            Buzzer_Counter++;
        }
        if (Buzzer_Counter > Buzzer_Delay && !Buzzer_Debug)
        {
            Buzzer_Counter = 0;
            Buzzer_Flag = 0;
            Buzzer_OFF();
        }
        // if (!Buzzer_Flag && (Sensor_Lift_Flag && (!LightSensor_Lift()) || (Sensor_Spin_Flag && (!LightSensor_Spin())))) {
        //     Buzzer_Counter = 1;
        //     Buzzer_Flag    = 1;
        // }
        // if (Buzzer_Counter <= 40)
        //     Buzzer_ON();
        // else if (!Buzzer_Debug)
        //     Buzzer_OFF();
        // if (Buzzer_Counter >= Buzzer_Delay) {
        //     Buzzer_Counter = Buzzer_Delay;
        //     Buzzer_Flag    = 0;
        // }
        MOTOR_Spead_Counter++;

        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    }
}
