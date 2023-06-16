#include <STC89C5xRC.H>
#include "stdio.h"
//#include "LCD1602.h"
#include "motor_driver.h"
unsigned int Left_count=0;
unsigned int Right_count=0;
float n_l=0;
float n_r=0;
unsigned int Timer_100ms=0;
unsigned int Timer_500ms=0;
//unsigned char LCD_buf_1[16];
//unsigned char LCD_buf_2[16];
void Timer2_Init(void)		//10微秒@11.0592MHz
{
	T2MOD = 0;				//初始化模式寄存器
	T2CON = 0;				//初始化控制寄存器
	TL2 = 0xF7;				//设置定时初始值
	TH2 = 0xFF;				//设置定时初始值
	RCAP2L = 0xF7;			//设置定时重载值
	RCAP2H = 0xFF;			//设置定时重载值
	TR2 = 1;				//定时器2开始计时
	ET2 = 1;				//使能定时器2中断
	EA = 1;
}
//void Timer0_Init(void)		//50微秒@11.0592MHz
//{
//	TMOD &= 0xF0;			//设置定时器模式
//	TMOD |= 0x01;			//设置定时器模式
//	TL0 = 0xD2;				//设置定时初始值
//	TH0 = 0xFF;				//设置定时初始值
//	TF0 = 0;				//清除TF0标志
//	TR0 = 1;				//定时器0开始计时
//	ET0 = 1;				//使能定时器0中断
//	PT0=1;
//}
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
void INT0_Init()
{
	IT0 = 1;
	EX0 = 1;
}
void INT1_Init()
{
	IT1 = 1;
	EX1 = 1;
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
	Timer2_Init();
	INT0_Init();
	INT1_Init();
	UartInit();
	wheel(left,0);
	while(1)
	{

	}
}
void Timer2_Routine(void) interrupt 5
{
	TF2 = 0;
	Timer_100ms++;
	Timer_500ms++;
	if(Timer_100ms>1000)
	{
		n_l=Left_count;
		n_r=Right_count;
		PID_control();
		Left_count=0;
		Right_count=0;
		printf("L=%0.1f,R=%0.1f",n_l,n_r);
		Timer_100ms=0;
	}
	if(Timer_500ms>5000)
	{
		Timer_500ms=0;
	}
	pwm_output();
}
void INT0_Routine() interrupt 0
{
	Left_count++;
}
void INT1_Routine() interrupt 2
{
	Right_count++;
}