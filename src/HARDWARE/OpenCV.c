#include "OpenCV.h"

#define A_ !A
#define B_ !B
#define C_ !C
#define D_ !D
#define E_ !E
#define F_ !F

uint8_t Serial3_RxData;
uint8_t Serial3_RxFlag;
static uint8_t databuf[ReceivedArrLength]; // 接收数组，用来存储OpenCV传递过来的数据
uint8_t ColorData[6];
/**
 * @brief 简单的memcpy函数
 *
 */
void OpenCV_memcpy(void *dest, void *src, int n)
{
    uint8_t *d = (uint8_t *)dest;
    uint8_t *s = (uint8_t *)src;
    for (int i = 0; i < n; i++) {
        d[i] = s[i];
    }
}

/**
 * @brief 初始化OpenCV所需要的串口
 * @retval 无
 */
void OpenCV_Init()
{
    Usart3SerialInit(); // 初始化串口3
}

/**
 * @brief  接收OpenCV传来的数据并存储
 * @retval 无
 */
void OpenCV_Calc()
{
    static unsigned char index = 0; // 接收数组的下标
    static uint8_t rdata       = 0; // 从串口中接收到的数据
    rdata                      = Usart3SerialGetRxData();

    static enum {
        FrameHeaderFlag,
        CalcFlag,
        FrameTailFlag
    } state = FrameHeaderFlag;

    // 串口传输中信息的处理
    if (state == FrameHeaderFlag && rdata == FrameHeaderData) { // 处理帧头
        state = CalcFlag;
    } else if (state == CalcFlag) { // 处理数据
        databuf[index++] = rdata;
        if (index == ReceivedArrLength) {
            state = FrameTailFlag;
        }
    } else if (state == FrameTailFlag && rdata == FrameTailData) { // 判断数据处理结束
        state = FrameHeaderFlag;
        index = 0;
    }
}

/**
 * @brief 从串口中取得8位数据
 * @param index 数组里的下标为index的数据
 * @retval uint8_t
 */
uint8_t getUsartBuf(unsigned char index)
{
    return databuf[index];
}

/**
 * @brief 从串口中取得16位数据
 *
 * @param index
 * @return uint16_t
 */
uint16_t getUsartBuf_16(unsigned char index)
{
    uint16_t recevdata;
    OpenCV_memcpy(&recevdata, databuf + index, sizeof(uint16_t));
    return recevdata;
}

/**
 * @brief 从串口中取得32位数据
 *
 * @param index
 * @return uint32_t
 */
uint32_t getUsartBuf_32(unsigned char index)
{
    uint32_t recevdata;
    OpenCV_memcpy(&recevdata, databuf + index, sizeof(uint32_t));
    return recevdata;
}

/**
 * @brief 从串口中取得float类型数据
 *
 * @param index
 * @return float
 */
float getUsartBuf_float(unsigned char index)
{
    float recevdata;
    OpenCV_memcpy(&recevdata, databuf + index, sizeof(float));
    return recevdata;
}

/**
 * @brief Get the Signed Usart Buf object
 *
 * @param index
 * @return int8_t
 */
int8_t
getSignedUsartBuf(unsigned char index)
{
    return (int8_t)databuf[index];
}

/**
 * @brief USART3的中断处理函数
 * @retval 无
 */
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET) {
        Serial3_RxFlag = 1;
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
        OpenCV_Calc();
    }
}

uint8_t Usart3SerialGetRxFlag(void)
{
    if (Serial3_RxFlag == 1) {
        Serial3_RxFlag = 0;
        return 1;
    }
    return 0;
}

uint8_t Usart3SerialGetRxData(void)
{
    Serial3_RxData = USART_ReceiveData(USART3);
    return Serial3_RxData;
}

void uart3WriteBuf(uint8_t *buf, uint8_t len)
{
    while (len--) {
        while ((USART3->SR & 0x40) == 0);
        USART_SendData(USART3, *buf++);
    }
}

/// @brief 向树莓派发送命令
/// @param main_mode 模式
/// @param color 颜色
void Send_CMD(uint8_t main_mode, uint8_t color)
{
    uint8_t outbuf[CAMERA_LEN] = {HEADER_CAMER_CMD, MAIN_MODE, 0x30, TAIL_CAMER_CMD};

    outbuf[1] = main_mode;
    outbuf[2] = color + 0x30;

    for (int i = 0; i < CAMERA_LEN; i++) {
        uart3WriteBuf(&outbuf[i], 1);
    }
}

// void DataTest()
// {
//     OLED_ShowNum(1, 1, getUsartBuf(0), 2);
//     OLED_ShowNum(2, 1, getUsartBuf(1), 2);
//     OLED_ShowNum(3, 1, getUsartBuf(2), 2);
// }
uint8_t Get_FrontMiddleColor(void)
{
    while (!(getUsartBuf(6))) {
        Buzzer_One();
        Delay_ms(1000);
    }
    ColorData[0] = getUsartBuf(6);
    return ColorData[0];
}

void Save_ColorData(void)
{
    while (!((getUsartBuf(4) >> 4) &&
             (getUsartBuf(4) & 0x0f) &&
             (getUsartBuf(5) >> 4) &&
             (getUsartBuf(5) & 0x0f) &&
             ((getUsartBuf(4) >> 4) != (getUsartBuf(4) & 0x0f)) &&
             ((getUsartBuf(4) >> 4) != (getUsartBuf(5) >> 4)) &&
             ((getUsartBuf(4) >> 4) != (getUsartBuf(5) & 0x0f)) &&
             ((getUsartBuf(4) & 0x0f) != (getUsartBuf(5) >> 4)) &&
             ((getUsartBuf(4) & 0x0f) != (getUsartBuf(5) & 0x0f)) &&
             ((getUsartBuf(5) >> 4) != (getUsartBuf(5) & 0x0f)))) {
        Buzzer_One();
        Delay_ms(1000);
    }
    ColorData[2] = getUsartBuf(4) & 0x0f;
    ColorData[3] = getUsartBuf(5) & 0x0f;
    ColorData[4] = getUsartBuf(4) >> 4;
    ColorData[5] = getUsartBuf(5) >> 4;
    ColorData[1] = 15 - ((ColorData[2]) + (ColorData[3]) + (ColorData[4]) + (ColorData[5]));
}


float Angle_SET = 0;
extern uint8_t Camera_Flag;
#define target      100
#define HWT_Angle   (HWT_getAngle())
#define HWT_Angle_K 4
int32_t Get_Excursion(Camera camera)
{
    //float kp = 1.8, ki = 0.02, kd = -0.01;
    float kp = 1.3;//, ki = 0, kd = 0;
    float Excursion;                  // 此次的误差
    //static float integral;
    //static float  last_data; // 积分累加项和上次的误差
    float output;                     // PID输出
    int8_t A, B, C, D, E, F;

    if(camera==NULL_Camera){
        B = getUsartBuf(FRONT) & 0x10;
        C = getUsartBuf(FRONT) & 0x08;
        D = getUsartBuf(FRONT) & 0x04;
        E = getUsartBuf(FRONT) & 0x02;
        if (!(B_ || C_ || D || E)) Excursion = -3;
        else if (!(B || C_ || D || E)) Excursion = -2;
        else if (!(B || C || D || E)) Excursion = 0;
        else if (!(B || C || D_ || E)) Excursion = 2;
        else if (!(B || C || D_ || E_)) Excursion = 3;
        return Excursion;
    }

    A = getUsartBuf(camera) & 0x20;
    B = getUsartBuf(camera) & 0x10;
    C = getUsartBuf(camera) & 0x08;
    D = getUsartBuf(camera) & 0x04;
    E = getUsartBuf(camera) & 0x02;
    F = getUsartBuf(camera) & 0x01;

    if (!(A_ || B || C || D || E || F)) Excursion = -6;
    else if (!(A_ || B_ || C || D || E || F)) Excursion = -5;
    else if (!(A || B_ || C || D || E || F)) Excursion = -4;
    else if (!(A || B_ || C_ || D || E || F)) Excursion = -3;
    else if (!(A || B || C_ || D || E || F)) Excursion = -2;
    else if (!(A || B || C || D || E || F)) {
        Excursion = 0;
        //if (last_data == 5) Excursion = 5;
        //if (last_data == -5) Excursion = -5;
    }
    else if (!(A || B || C || D_ || E || F)) Excursion = 2;
    else if (!(A || B || C || D_ || E_ || F)) Excursion = 3;
    else if (!(A || B || C || D || E_ || F)) Excursion = 4;
    else if (!(A || B || C || D || E_ || F_)) Excursion = 5;
    else if (!(A || B || C || D || E || F_)) Excursion = 6;
    else Excursion=0;

    //last_data = Excursion;
    //integral += Excursion;
    //if (integral > 5) integral = 5;
    //if (integral < -5) integral = -5;
    output = kp * Excursion;// + ki * integral + kd * (Excursion - last_data);
    //if (output > 12) output = 12;
    //if (output < -12) output = -12;
    return (int32_t)output;
}

uint8_t Get_Condition(Camera camera)
{
    uint8_t A, B, C, D, E , F, Condition = 0;
    if(camera==BEHIND){
        A = getUsartBuf(camera) & 0x20;
        B = getUsartBuf(camera) & 0x10;
        C = getUsartBuf(camera) & 0x08;
        D = getUsartBuf(camera) & 0x04;
        E = getUsartBuf(camera) & 0x02;
        F = getUsartBuf(camera) & 0x01;
        if (!(A_ || B_ || C_ || D_ || E_ || F_)) Condition = 1;
        else if (!(A_ || B_ || C_ || D_ || E_ || F)) Condition = 0;
        else if (!(A || B_ || C_ || D_ || E_ || F_)) Condition = 0;
        else Condition = 0;
    }
    else {
        A = getUsartBuf(3) & 0x01;
        B = getUsartBuf(3) & 0x02;
        if (!(A_ || B_ )) Condition = 1;
        else Condition = 0;
    }
    return Condition;
}
