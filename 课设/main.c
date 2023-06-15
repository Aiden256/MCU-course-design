#include <STC89C5xRC.H>
#include "intrins.h"
#include "stdio.h"
#include "LCD1602.h"
sbit TX=P2^1;
sbit RX=P2^0;
unsigned int Timer_500ms=0;
unsigned char Timer_100ms=0;
unsigned char Timer_20ms=0;
unsigned char PWM=1;
void Timer0_Init(void)		//100微秒@11.0592MHz
{
	TMOD &= 0xF0;			//设置定时器模式
	TMOD |= 0x01;			//设置定时器模式
	TL0 = 0x00;				//设置定时初始值
	TH0 = 0x00;				//设置定时初始值
}
void UartInit(void)		//9600bps@11.0592MHz
{
	PCON &= 0x7F;		//波特率不倍速
	SCON = 0x50;		//8位数据,可变波特率
	TMOD &= 0x0F;		//清除定时器1模式位
	TMOD |= 0x20;		//设定定时器1为8位自动重装方式
	TL1 = 0xFD;		//设定定时初值
	TH1 = 0xFD;		//设定定时器重装值
	ET1 = 0;		//禁止定时器1中断
	TR1 = 1;		//启动定时器1
}
void Timer2_Init(){
    T2MOD = 0;        //初始化模式寄存器
    T2CON = 0;        //初始化控制寄存器
    TL2 = 0x66;        //设置定时初始值
    TH2 = 0xFC;        //设置定时初始值
    RCAP2L = 0x66;        //设置定时重载值
    RCAP2H = 0xFC;        //设置定时重载值
    TR2 = 1;        //定时器2开始计时
    PT2=0;            //感觉比定时器0快一些
		EA=1;
    ET2=1;
}
void Delay10us()		//@11.0592MHz
{
	unsigned char i;
	i = 2;
	while (--i);
}
void Send_wave()
{
	TX=0;
	TX=1;
	Delay10us();
	TX=0;
}
float distance=0;
void Sonic_get()
{
	EA=0;
	Send_wave();
	while(RX == 0);
	TR0 = 1;
	while(RX == 1);
	TR0 = 0;
	distance = (TH0 * 256 + TL0)*1.085;
	distance = distance * 0.017;
	if(TF0==1)
	{
		TF0=0;
		distance=999;
	}
	TH0 = 0;
	TL0 = 0;
	EA=1;
}
void Send_byte(unsigned char byte)
{
	SBUF=byte;
	while(TI==0);
	TI=0;
}
char putchar(char c){
	Send_byte(c);
	return c;  
}
void main()
{
	Timer0_Init();
	Timer2_Init();
//	UartInit();
	LCD_Init();
	while(1)
	{
		if(Timer_100ms>=100)
		{
			//100ms
			Sonic_get();
			LCD_ShowString(1,1,"L=");
			LCD_ShowNum(1,3,distance,3);
			LCD_ShowString(1,6,"cm");
			//
			Timer_100ms=0;
		}
	}
}
void Timer2_Routine() interrupt 5
{
    TF2=0;
    Timer_500ms++;
		Timer_100ms++;
		Timer_20ms++;
		//1ms
		
		//
    if(Timer_500ms>=500)
		{
        Timer_500ms=0;
				//500ms
				printf("distance=%f\r\n",distance);
				//
    }
}