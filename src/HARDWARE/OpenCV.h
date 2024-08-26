#ifndef __USARTOpenCV_H__
#define __USARTOpenCV_H__

#include "stm32f10x.h"
#include "Usart.h"
#include "HWT101CT.h"
#include "Buzzer.h"

#define ReceivedArrLength 6    // 接收数组的长度
#define FrameHeaderData   0x21 // 帧头
#define FrameTailData     0x41 // 帧尾

#define HEADER_CAMER_CMD  0x45 // 帧头
#define TAIL_CAMER_CMD    0x46 // 帧尾
#define MAIN_MODE         0X30 // 主模式
#define CIRCLE_MODE       0X31 // 寻圆模式
#define CAMERA_LEN        6

#define FrontMiddle       Get_FrontMiddleColor()
#define BehindMiddle      ColorData[1]
#define BehindLeft        ColorData[2]
#define BehindRight       ColorData[3]
#define FrontLeft         ColorData[4]
#define FrontRight        ColorData[5]
#define Place_A           1
#define Place_C           3
#define Place_E           5

typedef enum {
    G = 1,
    W = 2,
    R = 3,
    D = 4,
    B = 5
} Color_;

typedef enum {
    NOCOLOR = 0,
    RED     = 1,
    GREEN   = 2,
    BLUE    = 3
} Color;

typedef enum {
    FRONT       = 0,
    FRONT_NEAR  = 1,
    BEHIND      = 2,
    NULL_Camera = 3
} Camera;

extern uint8_t ColorData[6];

void OpenCV_memcpy(void *dest, void *src, int n);
void OpenCV_Init(void);
void OpenCV_Calc(void);
uint8_t Usart3SerialGetRxFlag(void);
uint8_t Usart3SerialGetRxData(void);
int8_t getSignedUsartBuf(unsigned char index);
uint8_t getUsartBuf(unsigned char index);
uint16_t getUsartBuf_16(unsigned char index);
uint32_t getUsartBuf_32(unsigned char index);
float getUsartBuf_float(unsigned char index);
void uart3WriteBuf(uint8_t *buf, uint8_t len);
void Send_CMD(uint8_t main_mode, uint8_t color);

int32_t Get_Excursion(Camera camera);
uint8_t Get_Condition(Camera camera);
uint8_t Get_FrontMiddleColor(void);
void Save_ColorData(void);

#endif
