#include <REGX52.H>

// 延时函数（毫秒）
void Delay(unsigned int xms)
{
    unsigned char i, j;
    while(xms--)
    {
        i = 2;
        j = 239;
        do
        {
            while (--j);
        } while (--i);
    }
}

// 电机控制引脚
sbit ENA    = P2^0;
sbit IN1    = P2^2;
sbit IN2    = P2^3;
sbit IN3    = P2^4;
sbit IN4    = P2^5;
sbit ENB    = P2^1;

// 红外传感器引脚（1=黑线，0=白线）
sbit LEFT1  = P0^1;
sbit LEFT2  = P0^2;
sbit RIGHT1 = P0^0;
sbit RIGHT2 = P0^3;

// PWM相关变量
unsigned char PWMA, PWMB, i, j;

// 抖动检测相关变量
unsigned char last_sensor_state = 0xFF;
unsigned char jitter_count = 0;
unsigned char straight_hold_time = 0;
#define JITTER_THRESHOLD 3

// 定时器1初始化（50ms中断）
void Timer1_Init(void)
{
    TMOD |= 0x10;
    TH1 = 0x4C;
    TL1 = 0x00;
    ET1 = 1;
    EA = 1;
    TR1 = 1;
}

// 定时器1中断服务程序（50ms）
void Timer1_ISR(void) interrupt 3
{
    TH1 = 0x4C;
    TL1 = 0x00;
    if (straight_hold_time > 0) straight_hold_time--;
}

// 定时器0初始化（PWM）
void Timer0Init(void)
{
    TMOD &= 0xF0;
    TMOD |= 0x01;
    TL0 = 0x18;
    TH0 = 0xFC;
    TF0 = 0;
    TR0 = 1;
    ET0 = 1;
    EA = 1;
    PT0 = 0;
}

// 定时器0中断（PWM输出）
void T0_PWM() interrupt 1
{
    TL0 = 0x18;
    TH0 = 0xFC;
    i++;
    j++;
    if (i < PWMA) ENA = 1; else ENA = 0;
    if (i >= 30) i = 0;
    if (j < PWMB) ENB = 1; else ENB = 0;
    if (j >= 30) j = 0;
}

// 运动函数
void straight(void)
{
    IN1 = 1; IN2 = 0;
    PWMA = 6;
    IN3 = 1; IN4 = 0;
    PWMB = 6;
}

void turnleft(void)
{
    IN1 = 0; IN2 = 1;
    PWMA = 8;
    IN3 = 1; IN4 = 0;
    PWMB = 15;
}

void turnright(void)
{
    IN1 = 1; IN2 = 0;
    PWMA = 10;
    IN3 = 0; IN4 = 1;
    PWMB = 16;
}

void turnleft_big(void)
{
    IN1 = 0; IN2 = 1;
    PWMA = 15;
    IN3 = 1; IN4 = 0;
    PWMB = 23;
}

void turnright_big(void)
{
    IN1 = 1; IN2 = 0;
    PWMA = 23;
    IN3 = 0; IN4 = 1;
    PWMB = 15;
}

// 寻迹控制（1=黑线，0=白线）
void Track(void)
{
    // 合成当前传感器状态（用于抖动检测）
    unsigned char cur_state = 0;
    if (LEFT1) cur_state |= 0x08;   // 左内侧
    if (LEFT2) cur_state |= 0x04;   // 左外侧
    if (RIGHT1) cur_state |= 0x02;  // 右内侧
    if (RIGHT2) cur_state |= 0x01;  // 右外侧

    // 抖动检测
    if (cur_state != last_sensor_state)
    {
        jitter_count++;
        if (jitter_count >= JITTER_THRESHOLD)
        {
            straight_hold_time = 10;   // 强制直行0.5秒
            jitter_count = 0;
        }
    }
    else
    {
        if (jitter_count > 0) jitter_count--;
    }
    last_sensor_state = cur_state;

    // 强制直行优先
    if (straight_hold_time > 0)
    {
        straight();
        return;
    }

    // ---------- 以下所有传感器判断均基于 1=黑线 ----------
    // 全黑线（四个传感器都检测到黑线）→ 直行
    if (LEFT1 == 1 && RIGHT1 == 1 && LEFT2 == 1 && RIGHT2 == 1)
    {
        straight();
    }

    // 十字路口 / 特殊路况（多数传感器为黑线，少量白线）
    if ((LEFT1 == 1 && RIGHT1 == 0 && LEFT2 == 0 && RIGHT2 == 0)   // 只有左内侧黑，其他白
     || (LEFT1 == 0 && RIGHT1 == 1 && LEFT2 == 0 && RIGHT2 == 0)   // 只有右内侧黑
     || (LEFT1 == 0 && RIGHT1 == 0 && LEFT2 == 1 && RIGHT2 == 0)   // 只有左外侧黑
     || (LEFT1 == 0 && RIGHT1 == 0 && LEFT2 == 0 && RIGHT2 == 1)   // 只有右外侧黑
     || (LEFT1 == 1 && RIGHT1 == 1 && LEFT2 == 0 && RIGHT2 == 0))  // 两内侧黑，外侧白
    {
        straight();
        Delay(30);
    }

    // 小转弯（一侧内侧传感器偏离黑线）
    if (LEFT1 == 0 && RIGHT1 == 1 && LEFT2 == 1 && RIGHT2 == 1)   // 左内侧白，其他黑 → 左转
    {
        turnleft();
    }
    if (LEFT1 == 1 && RIGHT1 == 0 && LEFT2 == 1 && RIGHT2 == 1)   // 右内侧白，其他黑 → 右转
    {
        turnright();
    }

    // 大转弯（一侧外侧传感器偏离黑线）
    if (LEFT1 == 1 && RIGHT1 == 1 && LEFT2 == 0 && RIGHT2 == 1)   // 左外侧白，其他黑 → 左大转
    {
        turnleft_big();
    }
    if (LEFT1 == 1 && RIGHT1 == 1 && LEFT2 == 1 && RIGHT2 == 0)   // 右外侧白，其他黑 → 右大转
    {
        turnright_big();
    }
}

void main(void)
{
    Timer1_Init();
    Timer0Init();
    while (1)
    {
        Track();
    }
}