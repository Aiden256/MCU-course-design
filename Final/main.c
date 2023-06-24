#include <STC89C5xRC.H>
#include "stdio.h"
#include "LCD1602.h"
#include "motor_driver.h"
unsigned int Left_count=0;
unsigned int Right_count=0;
int n_l=0;
int n_r=0;
unsigned int Timer_100ms=0;
unsigned int Timer_450ms=0;
unsigned char LCD_buf_1[16];
unsigned char LCD_buf_2[16];
unsigned int x;
unsigned char state=0;
void Timer2_Init(void)		//10微秒@11.0592MHz
{
	T2MOD = 0;				//初始化模式寄存器
	T2CON = 0;				//初始化控制寄存器
	TL2 = 0x66;				//设置定时初始值
	TH2 = 0xFC;				//设置定时初始值
	RCAP2L = 0x66;			//设置定时重载值
	RCAP2H = 0xFC;			//设置定时重载值
	TR2 = 1;				//定时器2开始计时
	ET2 = 1;				//使能定时器2中断
	EA = 1;
}
void Timer0_Init(void)		//1毫秒@11.0592MHz
{
	TMOD &= 0xF0;			//设置定时器模式
	TMOD |= 0x01;			//设置定时器模式
	TL0 = 0x66;				//设置定时初始值
	TH0 = 0xFC;				//设置定时初始值
	TF0 = 0;				//清除TF0标志
	TR0 = 1;				//定时器0开始计时
	ET0 = 1;				//使能定时器0中断
	PT0=1;
	EA=1;
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
void trail_detect()
{
	if(state==3)
	{
		wheel(forward,5);
		Delay_ms(50);
		wheel(right,6);
		Delay_ms(50);
		if(P36==1)//中寻迹
		{
			wheel(forward,5);
		}
		else
		{
			wheel(left,5);
			while(P27==0)
			{
				if(P20==0)
				{
					wheel(backward,5);
					Delay_ms(200);
					state++;
					break;
				}
			}
			wheel(right,5);
			if(P20==0)
			{
				wheel(backward,5);
				Delay_ms(200);
				state++;
			}
		}
			if(P20==0)
			{
				wheel(backward,5);
				Delay_ms(200);
				state++;
			}
	}
		if(state==4)
		{
			state++;
			wheel(forward,0);
			Delay_ms(10);
			P22=0;
			P23=0;
			wheel(left,6);
			Delay_ms(300);
			wheel(forward,5);
			Delay_ms(350);
			wheel(right,6);
			Delay_ms(250);
			wheel(forward,5);
			Delay_ms(350);
			wheel(right,6);
			Delay_ms(250);
			wheel(forward,5);
			Delay_ms(250);
			wheel(left,6);
			Delay_ms(250);
			P22=P23=1;
			
			wheel(forward,5);
		}
	if(state==5)
		{	
			wheel(forward,5);
			if(P36==1)
			{
				state++;
				P23=0;
				wheel(forward,5);
				Delay_ms(400);
				Turn_360();
				P23=1;
				wheel(forward,0);
				Delay_ms(10);
			}
		}
		if(state==6)
		{
			wheel(forward,0);
			Delay_ms(10);
		}
}
void avoid()
{
	if(P20==0&&state!=3&&state!=0)//前避障
	{
		wheel(backward,5);
		Delay_ms(300);
		wheel(forward,5);
	}
	if(P34==0)//左避障
	{
		if(state==2)
		{
			state++;
			wheel(forward,5);
			Delay_ms(25);
			P24=0;
			P23=0;
			wheel(left,6);
			Delay_ms(200);
			P24=P23=1;
			wheel(forward,5);
			Delay_ms(400);
			P22=0;
			P23=0;
			wheel(right,6);
			Delay_ms(210);
			P22=P23=1;
			wheel(forward,5);
			Delay_ms(600);
		}
	}
	if(P35==0)//右避障
	{
		if(state==1)
		{
			state++;
			wheel(forward,5);
			Delay_ms(200);
			P22=0;
			P23=0;
			wheel(right,6);
			Delay_ms(200);
			wheel(forward,0);
			Delay_ms(10);
			P22=1;
			P23=1;
			wheel(forward,5);
			Delay_ms(400);
			P24=0;
			P23=0;
			wheel(left,6);
			Delay_ms(200);
			wheel(forward,0);
			Delay_ms(10);
			P24=P23=1;
			wheel(forward,5);
		} 
	}
}
void main()
{
	Timer0_Init();
	Timer2_Init();
	INT0_Init();
	INT1_Init();
	LCD_Init();
	P22=P23=P24=1;
	wheel(left,0);
	Delay_ms(500);
	while(1)
	{
		if(state==0)
		{
			state++;
			P22=0;
			P23=0;
			wheel(left,6);
			Delay_ms(200);
			P22=1;
			P23=1;
			wheel(forward,5);
			Delay_ms(450);
			P24=0;
			P23=0;
			wheel(right,6);
			Delay_ms(200);
			P24=1;
			P23=1;
			wheel(forward,5);
		}
		trail_detect();
		avoid();
	}
}
void Timer2_Routine(void) interrupt 5
{
	TF2 = 0;
	Timer_100ms++;
	Timer_450ms++;
	if(Timer_100ms>100)
	{
		n_l=Left_count;
		n_r=Right_count;
//		PID_control();
		Left_count=0;
		Right_count=0;
		sprintf(LCD_buf_1,"L=%2dr/s",n_l);
		sprintf(LCD_buf_2,"R=%2dr/s",n_r);
		Timer_100ms=0;
	}
	if(Timer_450ms>450)
		{
			LCD_ShowString(1,1,LCD_buf_1);
			LCD_ShowString(2,1,LCD_buf_2);
			Timer_450ms=0;
		}
}
void Timer0_Isr() interrupt 1
{
	TL0 = 0x66;				
	TH0 = 0xFC;				
	if(pwm_flag)
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