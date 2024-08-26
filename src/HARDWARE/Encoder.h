#ifndef __ENCODER_H
#define __ENCODER_H

void Encoder_Init(void);
int16_t Encoder_Left_Get(void);
int16_t Encoder_Right_Get(void);
void Encoder_Init(void);
void Encoder_Right_Clear(void);
void Encoder_Left_Clear(void);
void Encoder_Right_Set(uint16_t Counter);
void Encoder_Left_Set(uint16_t Counter);

#endif
