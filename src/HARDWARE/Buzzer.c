#include "Buzzer.h"

extern uint8_t Buzzer_Debug;
extern uint8_t Buzzer_Flag;

void Buzzer_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, GPIO_Pin_9);
}

void Buzzer_ON(void)
{
#ifdef OTHER_CAR
    GPIO_SetBits(GPIOB, GPIO_Pin_9);
#else
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);
#endif
}

void Buzzer_OFF(void)
{
#ifdef OTHER_CAR
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);
#else
    GPIO_SetBits(GPIOB, GPIO_Pin_9);
#endif
}

void Buzzer_Turn(void)
{
    if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_9) == 0)
        GPIO_SetBits(GPIOB, GPIO_Pin_9);
    else
        GPIO_ResetBits(GPIOB, GPIO_Pin_9);
}

void Buzzer_Tow(uint16_t Time)
{
    Buzzer_Debug = 1;
    Buzzer_ON();
    Delay_ms(Time);
    Buzzer_OFF();
    Delay_ms(Time / 2);
    Buzzer_ON();
    Delay_ms(Time);
    Buzzer_OFF();
    Buzzer_Debug = 0;
}

void Buzzer_One(void)
{
    Buzzer_Flag = 1;
}
