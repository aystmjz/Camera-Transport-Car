#include "PID.h"

int16_t MOTOR_LeftSpead, MOTOR_RightSpead;
extern float Angle_SET;

void PID_Inint(PID *PID)
{
    PID->KP_L     = 0.6; // 0.5 0.25
    PID->KI_L     = 0;
    PID->KD_L     = 1.8; // 1.2
    PID->target_l = 0;

    PID->KP_S     = 3.2; // 3.1
    PID->KI_S     = 0;
    PID->KD_S     = 2.8; // 2.8
    PID->target_s = 0;
}
extern PID PID_L, PID_R;

void PID_LocationSet(PID *PID, int32_t Location)
{
    PID->target_l = Location;
}

void PID_SpeadSet(PID *PID, int32_t Spead)
{
    PID->target_s = Spead;
}

/// @brief 电机控制
/// @param Spead 速度
/// @param Location 位置偏移量
/// @param MOTOR 目标电机
/// @return 0: 达到目标偏差值以内  other: 当前偏差绝对值
int16_t MOTOR_CONTROL(int32_t Spead, int32_t Location, uint16_t MOTOR)
{
    if (Location < 0) {
        Spead = -Spead;
        if (Spead < -100) Spead = -100;
        if (Spead > -10) Spead = -10;
    } else {
        if (Spead > 100) Spead = 100;
        if (Spead < 10) Spead = 10;
    }
    switch (MOTOR) {
        case 0x01:
            // if (((Location > 0 ? Location : -Location) - (PID_L.e_l > 0 ? PID_L.e_l : -PID_L.e_l) < 200)) Spead = Spead/2;
            PID_LocationSet(&PID_L, Location);
            PID_SpeadSet(&PID_L, Spead);
            PID_L.e_l = PID_L.target_l - PID_L.current_l;
            if (PID_L.e_l < MOTOR_MAX && PID_L.e_l > -MOTOR_MAX)
                return 0;
            else
                return PID_L.e_l > 0 ? PID_L.e_l : -PID_L.e_l;

        case 0x02:
            // if (((Location > 0 ? Location : -Location) - (PID_R.e_l > 0 ? PID_R.e_l : -PID_R.e_l) < 200)) Spead = Spead/2;
            PID_LocationSet(&PID_R, Location);
            PID_SpeadSet(&PID_R, Spead);
            PID_R.e_l = PID_R.target_l - PID_R.current_l;
            if (PID_R.e_l < MOTOR_MAX && PID_R.e_l > -MOTOR_MAX)
                return 0;
            else
                return PID_R.e_l > 0 ? PID_R.e_l : -PID_R.e_l;

        case 0x03:
            PID_LocationSet(&PID_L, Location);
            PID_LocationSet(&PID_R, Location);
            PID_SpeadSet(&PID_L, Spead);
            PID_SpeadSet(&PID_R, Spead);
            PID_L.e_l = PID_L.target_l - PID_L.current_l;
            PID_R.e_l = PID_R.target_l - PID_R.current_l;
            if (PID_L.e_l < MOTOR_MAX && PID_L.e_l > -MOTOR_MAX && PID_R.e_l < MOTOR_MAX && PID_R.e_l > -MOTOR_MAX)
                return 0;
            else
                return ((PID_L.e_l > 0 ? PID_L.e_l : -PID_L.e_l) + (PID_R.e_l > 0 ? PID_R.e_l : -PID_R.e_l)) / 2;
    }
    return 0;
}

float PID_GetErro(uint16_t MOTOR)
{
    switch (MOTOR) {
        case 0x01:
            return fabs(PID_L.target_l - PID_L.current_l);
        case 0x02:
            return fabs(PID_R.target_l - PID_R.current_l);
        case 0x03:
            return (fabs(PID_L.target_l - PID_L.current_l) + fabs(PID_R.target_l - PID_R.current_l)) / 2;
    }
    return 0;
}

void PID_calc(PID *PID)
{
    PID->e_l = PID->target_l - PID->current_l;

#ifdef WSDC2412D
    PID->e_s = PID->target_s - PID->current_s;
#else
    if (PID->target_s >= 0)
        PID->e_s = PID->target_s - PID->current_s;
    else
        PID->e_s = PID->current_s - PID->target_s;
#endif

    if (PID->e_l >= 300 || PID->e_l <= -300) {
        PID->p_s = (int32_t)(PID->KP_S * PID->e_s);
        PID->i_s += (int32_t)(PID->KI_S * PID->e_s);
        PID->d_s = (int32_t)(PID->KD_S * (PID->e_s - PID->last_e_s));

        if (PID->d_s > 50) PID->d_s = 50;
        if (PID->d_s < -50) PID->d_s = -50;

#ifdef WSDC2412D
        PID->total_s = PID->target_s + PID->p_s + PID->i_s + PID->d_s;
// if(PID->total_s>PID->target_s+10)PID->total_s=PID->target_s+10;
//  if(PID->total_s<PID->target_s-10)PID->total_s=PID->target_s-10;
#else
        if (PID->target_s >= 0)
            PID->total_s = PID->target_s + PID->p_s + PID->i_s + PID->d_s;
        else
            PID->total_s = -(-PID->target_s + PID->p_s + PID->i_s + PID->d_s);
#endif
        /* if(PID->total_s>PID->target_s+10)PID->total_s=PID->target_s+10;
        if(PID->total_s<PID->target_s-10)PID->total_s=PID->target_s-10;   */

#ifndef WSDC2412D
        if (PID->target_l >= 0)
            PID->TOTAL_OUT = PID->total_s;
        else
            PID->TOTAL_OUT = -PID->total_s;
#else
        PID->TOTAL_OUT = PID->total_s;
#endif
        PID->last_e_s = PID->e_s;
    } else {
        PID->p_l = (int32_t)(PID->KP_L * PID->e_l);
        PID->i_l += (int32_t)(PID->KI_L * PID->e_l);
        PID->d_l = (int32_t)(PID->KD_L * (PID->e_l - PID->last_e_l));

        if (PID->d_l > 50) PID->d_l = 50;
        if (PID->d_l < -50) PID->d_l = -50;

        PID->total_l = PID->p_l + PID->i_l + PID->d_l;

#ifndef WSDC2412D
// if (PID->total_l > PID->target_s) PID->total_l = PID->target_s;
#else
        if (PID->target_s >= 0) {
            if (PID->total_l > PID->target_s) PID->total_l = PID->target_s;
        } else {
            if (PID->total_l < PID->target_s) PID->total_l = PID->target_s;
        }
#endif
        PID->TOTAL_OUT = PID->total_l;
        PID->last_e_l  = PID->e_l;
    }
    if (PID->TOTAL_OUT > 100) PID->TOTAL_OUT = 100;
    if (PID->TOTAL_OUT < -100) PID->TOTAL_OUT = -100;
}

void PID_calc_all()
{
    PID_calc(&PID_L);
    PID_calc(&PID_R);
}

void PWM_TIM1_Init(void)
{

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
#ifndef WSDC2412D
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
#endif

    //	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    //	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);
    //	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

#ifndef WSDC2412D
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_13;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

    TIM_InternalClockConfig(TIM1);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period        = 200 - 1; // ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler     = 20 - 1;  // PSC
    // TIM_PrescalerConfig(TIMx,Prescaler,TIM_PSCReloadMode_Immediate);
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; /*输出使能*/
#ifndef WSDC2412D
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; /*互补输出使能*/
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;    /*互补输出有效电平为高电平*/
#endif
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High; /*输出有效电平为高电平*/
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Set; /*输出空闲时为高电平*/
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OCInitStructure.TIM_Pulse        = 0; // CCR

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); /*开启预装载，在更新时间后才会重新装载数值*/
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

#ifndef WSDC2412D
    TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
    TIM_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Enable; /*运行模式下“关闭模式”选择 = 1*/
    TIM_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Enable; /*空闲模式下“关闭模式”选择 = 1*/
    TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_1;      /*锁定级别1，见参考手册*/
    TIM_BDTRInitStruct.TIM_DeadTime  = 0x80;                 /*死区时间：12.8ms*/
    // TIM_BDTRInitStruct.TIM_Break = TIM_Break_Enable;						/*开启刹车功能*/
    // TIM_BDTRInitStruct.TIM_BreakPolarity = TIM_BreakPolarity_Low;		/*刹车输入低电平有效，如果引脚检测到高电平则会停止PWM的输出，不会产生任何波形*/
    TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable; /*开启自动输出*/
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStruct);
#endif

    TIM_Cmd(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void MOTOR_Rotation_Init()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void PID_Init()
{
    PWM_TIM1_Init();
#ifdef WSDC2412D
    MOTOR_Rotation_Init();
#endif
}

void MOTOR_Rotation(Rotation Vale, uint16_t MOTOR)
{
    switch (MOTOR) {
        case 0x01:
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction)(!Vale));
            GPIO_WriteBit(GPIOC, GPIO_Pin_15, (BitAction)(Vale));
            break;

        case 0x02:
            GPIO_WriteBit(GPIOE, GPIO_Pin_2, (BitAction)(!Vale));
            GPIO_WriteBit(GPIOE, GPIO_Pin_4, (BitAction)(Vale));
            break;
#ifndef WSDC2412D
        case 0x04:
            GPIO_WriteBit(GPIOB, GPIO_Pin_14, (BitAction)(Vale));
            GPIO_WriteBit(GPIOB, GPIO_Pin_15, (BitAction)(!Vale));
            break;

        case 0x08:
            GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)(Vale));
            GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)(!Vale));
            break;
#endif
    }
}

void MOTOR_Set(uint16_t Spead, Rotation Vale, uint16_t MOTOR)
{
#ifndef WSDC2412D
    Spead += 100;
#endif
    switch (MOTOR) {
        case 0x01:
            TIM_SetCompare1(TIM1, Spead);
            break;

        case 0x02:
            TIM_SetCompare4(TIM1, Spead);
            break;
    }
#ifdef WSDC2412D
    MOTOR_Rotation(Vale, MOTOR);
#endif
}

void MOTOR_Clear(uint16_t MOTOR)
{
    switch (MOTOR) {
        case 0x01:
            Encoder_Left_Clear();
            break;

        case 0x02:
            Encoder_Right_Clear();
            break;

        case 0x03:
            Encoder_Left_Clear();
            Encoder_Right_Clear();
    }
}

void MOTOR_Stop(uint16_t MOTOR, int32_t Spead, int32_t Distance)
{
    switch (MOTOR) {
        case 0x01:
            MOTOR_Clear(MOTOR);
            while (MOTOR_CONTROL(Spead, Distance, MOTOR_Left)) {}
            MOTOR_CONTROL(Spead, 0, MOTOR_Left);
            MOTOR_Clear(MOTOR);
            break;

        case 0x02:
            MOTOR_Clear(MOTOR);
            while (MOTOR_CONTROL(Spead, Distance, MOTOR_Right)) {}
            MOTOR_CONTROL(Spead, 0, MOTOR_Right);
            MOTOR_Clear(MOTOR);
            break;

        case 0x03:
            MOTOR_Clear(MOTOR);
            while (MOTOR_CONTROL(Spead, Distance, MOTOR_Left) & MOTOR_CONTROL(Spead, Distance, MOTOR_Right)) {}
            MOTOR_CONTROL(Spead, 0, MOTOR_Left);
            MOTOR_CONTROL(Spead, 0, MOTOR_Right);
            MOTOR_Clear(MOTOR);
            break;
    }
}

void MOTOR_Spead_calc()
{
    static int16_t Last_Left = 0, Now_Left = 0, Last_Right = 0, Now_Right = 0;
    Last_Left        = Now_Left;
    Now_Left         = Encoder_Left_Get();
    Last_Right       = Now_Right;
    Now_Right        = Encoder_Right_Get();
    MOTOR_LeftSpead  = Now_Left - Last_Left;
    MOTOR_RightSpead = Now_Right - Last_Right;
}

uint16_t MOTOR_Spead_Get(uint8_t MOTOR)
{
    switch (MOTOR) {
        case 0x01:
            return MOTOR_LeftSpead;
        case 0x02:
            return MOTOR_RightSpead;
    }
    return 0;
}

/// @brief 用指定PID方法控制目标电机
/// @param PID PID控制方法
/// @param MOTOR 目标电机
void MOTOR_Run(PID *PID, uint8_t MOTOR)
{
#ifdef WSDC2412D
    if (PID->TOTAL_OUT < 0)
        MOTOR_Set(-PID->TOTAL_OUT, Backward, MOTOR);
    else
        MOTOR_Set(PID->TOTAL_OUT, Forward, MOTOR);
#else
    if (PID->TOTAL_OUT < 0)
        MOTOR_Set(PID->TOTAL_OUT, Backward, MOTOR);
    else
        MOTOR_Set(PID->TOTAL_OUT, Forward, MOTOR);
#endif
}

void MOTOR_Run_all()
{
    MOTOR_Run(&PID_L, MOTOR_Left);
    MOTOR_Run(&PID_R, MOTOR_Right);
}

#define Angle_AjustEX       2
#define Angle_AjustEX_Again 3
#ifdef OTHER_CAR
#define Angle_180 1800
#define Angle_135 1470
#define Angle_90  1150
#define Angle_45  810
#else
#define Angle_180 1715
#define Angle_135 1401
#define Angle_90  1095
#define Angle_45  780
#endif

#define HWT_Angle (HWT_getAngle())
void MOTOR_SpinAjust(int32_t spead, float AjustAngle)
{
    float Delta_Angle   = Angle_SET - HWT_Angle;
    int32_t Delta_Spead = 40, Delta_Long = spead;
    if (Delta_Long < 30) Delta_Long = 30;
    if (Delta_Angle >= 180) Delta_Angle -= 360;
    if (Delta_Angle <= -180) Delta_Angle += 360;
    while (fabs(Delta_Angle) > AjustAngle) {
        MOTOR_Clear(MOTOR_Left | MOTOR_Right);
        MOTOR_CONTROL(Delta_Spead, Delta_Angle > 0 ? -Delta_Long : Delta_Long, MOTOR_Right);
        MOTOR_CONTROL(Delta_Spead, Delta_Angle > 0 ? Delta_Long : -Delta_Long, MOTOR_Left);
        Delta_Angle = Angle_SET - HWT_Angle;
        if (Delta_Angle >= 180) Delta_Angle -= 360;
        if (Delta_Angle <= -180) Delta_Angle += 360;
        Delay_ms(20);
        Delta_Spead++;
        if (Delta_Spead > 55) Delta_Spead = 50;
        if (Delta_Spead == 50) Delta_Long += 3;
        if (Delta_Long > 170) Delta_Long = 170;
    }
    Buzzer_One();
    MOTOR_Clear(MOTOR_Left | MOTOR_Right);
    MOTOR_CONTROL(40, 0, MOTOR_Left | MOTOR_Right);
    Delay_ms(100);
    Delta_Angle = Angle_SET - HWT_Angle;
    if (Delta_Angle >= 180) Delta_Angle -= 360;
    if (Delta_Angle <= -180) Delta_Angle += 360;
    if (fabs(Delta_Angle) > AjustAngle) {
        MOTOR_SpinAjust(spead - 5, Angle_AjustEX_Again);
    }
}

void MOTOR_Spin(Direction Vale, int32_t Angle, int32_t Spead, int32_t Ajust_Spead)
{
    int32_t spead = Spead;
    if (Angle == 0 || Angle == 360) return;
    if (Vale == Right) {
        if (Angle_SET + Angle > 360.0) Angle_SET -= 360.0;
        Angle_SET += Angle;
    } else {
        if (Angle_SET - Angle < 0) Angle_SET += 360.0;
        Angle_SET -= Angle;
    }
    if (Angle_SET == 360) Angle_SET = 0;
    Delay_ms(100);
    if (Angle == 45) {
        Angle = Angle_45;
#ifdef OTHER_CAR
        Spead = Spead * 2 / 3;
#else
        Spead = Spead * 2 / 3;//Spead = Spead / 2;
#endif
    }
    if (Angle == 90) Angle = Angle_90;
    if (Angle == 135) Angle = Angle_135;
    if (Angle == 180) Angle = Angle_180;
    switch (Vale) {
        case Left:
            MOTOR_Clear(MOTOR_Left | MOTOR_Right);
            while ((MOTOR_CONTROL(spead, Angle, MOTOR_Right) | MOTOR_CONTROL(spead, -Angle, MOTOR_Left))) {
                if (PID_GetErro(MOTOR_Left | MOTOR_Right) > (Angle * 0.9))
                    spead = Spead / 2;
                else if (PID_GetErro(MOTOR_Left | MOTOR_Right) > (Angle * 0.5))
                    spead = Spead;
                else
                    spead = Spead / 2;
            }
            MOTOR_Clear(MOTOR_Left | MOTOR_Right);
            MOTOR_CONTROL(spead, 0, MOTOR_Left | MOTOR_Right);
            Buzzer_Tow(100);
            MOTOR_SpinAjust(Ajust_Spead, Angle_AjustEX);
            break;

        case Right:
            MOTOR_Clear(MOTOR_Left | MOTOR_Right);
            while ((MOTOR_CONTROL(spead, -Angle, MOTOR_Right) | MOTOR_CONTROL(spead, Angle, MOTOR_Left))) {
                if (PID_GetErro(MOTOR_Left | MOTOR_Right) > (Angle * 0.9))
                    spead = Spead / 2;
                else if (PID_GetErro(MOTOR_Left | MOTOR_Right) > (Angle * 0.5))
                    spead = Spead;
                else
                    spead = Spead / 2;
            }
            MOTOR_Clear(MOTOR_Left | MOTOR_Right);
            MOTOR_CONTROL(spead, 0, MOTOR_Left | MOTOR_Right);
            Buzzer_Tow(100);
            MOTOR_SpinAjust(Ajust_Spead, Angle_AjustEX);
            break;
    }
    Delay_ms(100);
}

void MOTOR_SingleSpin(Direction Vale, int32_t Long, int32_t Spead)
{
    switch (Vale) {
        case 0x01:
            MOTOR_Clear(MOTOR_Right);
            while (MOTOR_CONTROL(Spead, Long, MOTOR_Right)) {}
            MOTOR_Clear(MOTOR_Right);
            MOTOR_CONTROL(Spead, 0, MOTOR_Right);
            break;

        case 0x00:
            MOTOR_Clear(MOTOR_Left);
            while (MOTOR_CONTROL(Spead, Long, MOTOR_Left)) {}
            MOTOR_Clear(MOTOR_Left);
            MOTOR_CONTROL(Spead, 0, MOTOR_Left);
            break;
    }
    Delay_ms(500);
}

float Angle[6] = {0, 270, 315, 0, 45, 90};
void MOTOR_PutSpin(uint8_t ColorData, int32_t Spead, int32_t Ajust_Spead)
{
    Delay_ms(200);
    float Delta_Angle = fabs(((Angle[ColorData] + Angle_SET) / 2 - Angle_SET) * 2);
    if (Delta_Angle >= 180) Delta_Angle = 360 - Delta_Angle;
    if (fabs((Angle[ColorData] + Angle_SET) / 2 - Angle_SET) > 90)
        MOTOR_Spin(((Angle[ColorData] + Angle_SET) / 2 - Angle_SET) > 0 ? Left : Right, Delta_Angle, Spead, Ajust_Spead);
    else
        MOTOR_Spin(((Angle[ColorData] + Angle_SET) / 2 - Angle_SET) > 0 ? Right : Left, Delta_Angle, Spead, Ajust_Spead);
    Delay_ms(200);
}

float AngleBack[6] = {0, 90, 135, 180, 225, 270};
void MOTOR_PutBackSpin(uint8_t ColorData, int32_t Spead, int32_t Ajust_Spead)
{
    Delay_ms(200);
    float Delta_Angle = fabs(((AngleBack[ColorData] + Angle_SET) / 2 - Angle_SET) * 2);
    if (Delta_Angle >= 180) Delta_Angle = 360 - Delta_Angle;
    if (fabs((AngleBack[ColorData] + Angle_SET) / 2 - Angle_SET) > 90)
        MOTOR_Spin(((AngleBack[ColorData] + Angle_SET) / 2 - Angle_SET) > 0 ? Left : Right, Delta_Angle, Spead, Ajust_Spead);
    else
        MOTOR_Spin(((AngleBack[ColorData] + Angle_SET) / 2 - Angle_SET) > 0 ? Right : Left, Delta_Angle, Spead, Ajust_Spead);
    Delay_ms(200);
}
