#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__
#include <STC89C5xRC.H>
unsigned int pwm1;
unsigned int pwm2;
unsigned int pwm3;
unsigned int pwm4;
unsigned int ms_1;
#define moter_L 1
#define moter_R 0
#define left 3
#define right 4
void wheel(unsigned char wheel,int compare);
void moter_set(unsigned char moter_select,int compare);
void pwm_output();
void wheel(unsigned char wheel,int compare)
{
	if(wheel==left)
	{
		moter_set(moter_L,-compare);
		moter_set(moter_R,compare);
	}
		if(wheel==right)
	{
		moter_set(moter_L,compare);
		moter_set(moter_R,-compare);
	}
}
void moter_set(unsigned char moter_select,int compare)
{
	if(moter_select==moter_L)
	{
		    if(compare>0)
		{
				pwm1=compare;
	      pwm2= 0;   
		}
				else if(compare==0)
		{
				pwm1= 0;
	      pwm2= 0;  
		}
				else if(compare<0)
		{
				pwm1=0;
				compare=compare*(-1);
	      pwm2=compare;
		}
	}
		if(moter_select==moter_R)
	{
		    if(compare>0)
		{
				pwm3=compare;
	      pwm4= 0; 
		}
				else if(compare==0)
		{
				pwm3= 0;
	      pwm4= 0; 
		}
				else if(compare<0)
		{
				pwm3=0;
			  compare=compare*(-1);
	      pwm4=compare;
		}
	}
}

void pwm_output()
{
	ms_1++;
	 if(ms_1<pwm1)
	{ P12=1;}
	 else if(ms_1>=pwm1)
	{ P12=0;}
	
	 if(ms_1<pwm2)
	{ P13=1;}
	 else if(ms_1>=pwm2)
	{ P13=0;}
	
	 if(ms_1<pwm3)
	{ P14=1;}
	 else if(ms_1>=pwm3)
	{ P14=0;}
	
	 if(ms_1<pwm4)
	{ P15=1;}
	 else if(ms_1>=pwm4)
	{ P15=0;}
	
	if(ms_1 == 19)
		ms_1=0;
}
#endif