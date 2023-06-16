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
void Timer2_Init(void)		//10΢��@11.0592MHz
{
	T2MOD = 0;				//��ʼ��ģʽ�Ĵ���
	T2CON = 0;				//��ʼ�����ƼĴ���
	TL2 = 0xF7;				//���ö�ʱ��ʼֵ
	TH2 = 0xFF;				//���ö�ʱ��ʼֵ
	RCAP2L = 0xF7;			//���ö�ʱ����ֵ
	RCAP2H = 0xFF;			//���ö�ʱ����ֵ
	TR2 = 1;				//��ʱ��2��ʼ��ʱ
	ET2 = 1;				//ʹ�ܶ�ʱ��2�ж�
	EA = 1;
}
//void Timer0_Init(void)		//50΢��@11.0592MHz
//{
//	TMOD &= 0xF0;			//���ö�ʱ��ģʽ
//	TMOD |= 0x01;			//���ö�ʱ��ģʽ
//	TL0 = 0xD2;				//���ö�ʱ��ʼֵ
//	TH0 = 0xFF;				//���ö�ʱ��ʼֵ
//	TF0 = 0;				//���TF0��־
//	TR0 = 1;				//��ʱ��0��ʼ��ʱ
//	ET0 = 1;				//ʹ�ܶ�ʱ��0�ж�
//	PT0=1;
//}
void UartInit(void)		//9600bps@11.0592MHz
{
	PCON &= 0x7F;		//�����ʲ�����
	SCON = 0x50;		//8λ����,�ɱ䲨����
	TMOD &= 0x0F;		//�����ʱ��1ģʽλ
	TMOD |= 0x20;		//�趨��ʱ��1Ϊ8λ�Զ���װ��ʽ
	TL1 = 0xFD;		//�趨��ʱ��ֵ
	TH1 = 0xFD;		//�趨��ʱ����װֵ
	ET1 = 0;		//��ֹ��ʱ��1�ж�
	TR1 = 1;		//������ʱ��1
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