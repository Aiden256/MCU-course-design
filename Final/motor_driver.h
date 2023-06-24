#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__
#include <STC89C5xRC.H>
#include "intrins.h"
unsigned int pwm1;
unsigned int pwm2;
unsigned int pwm3;
unsigned int pwm4;
unsigned int ms_1;
#define moter_L 1
#define moter_R 0
#define forward 1
#define backward 2
#define left 3
#define right 4
void wheel(unsigned char wheel,int compare);
void moter_set(unsigned char moter_select,int compare);
void pwm_output();
void Delay_ms(unsigned int t);
bit pwm_flag=1;
bit PID_state_Volocity=0;
unsigned int PWM_L;
unsigned int PWM_R;
unsigned int PID_Target_L=5;
unsigned int PID_Target_R=5;
extern int n_l;
extern int n_r;
void Delay_ms(unsigned int t)		//@11.0592MHz
{
	unsigned char data i, j;
	unsigned int k=0;
	for(k=0;k<t;k++)
	{
		_nop_();
		i = 2;
		j = 199;
		do
		{
			while (--j);
		} while (--i);
	}
}
void brake(unsigned int t)
{
	unsigned int temp_l=0,temp_r=0;
	temp_l=PID_Target_L;
	temp_r=PID_Target_R;
	PID_Target_L=0;
	PID_Target_R=0;
	Delay_ms(t);
	PID_Target_L=temp_l;
	PID_Target_R=temp_r;
}
void Turn_90(unsigned char Order)
{
	PID_state_Volocity=0;
	if(Order==left)
	{
		P22=0;
		P23=0;
		wheel(left,6);
		Delay_ms(550);
	}
	if(Order==right)
	{
		P24=0;
		P23=0;
		wheel(right,6);
		Delay_ms(550);
	}
	P22=P23=P24=1;
	PID_state_Volocity=1;
}
void Turn_360()
{
	PID_state_Volocity=0;
	P23=P22=0;
	wheel(left,8);
	Delay_ms(1200);
	P23=P22=1;
	PID_state_Volocity=1;
}
void wheel(unsigned char wheel,int compare)
{
	if(wheel==forward)
	{
		moter_set(moter_L,compare);
		moter_set(moter_R,compare);
	}
		if(wheel==backward)
	{
		moter_set(moter_L,-compare);
		moter_set(moter_R,-compare);
	}
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
	if(ms_1 >=9)
		ms_1=0;
}
int Velcity_Kp=1,  Velcity_Ki=1,  Velcity_Kd=0; //相关速度PID参数
int Velocity_FeedbackControl_L(int TargetVelocity, int CurrentVelocity)
{
		int Bias_L;  //定义相关变量
		static int ControlVelocity_L, Last_bias_L; //静态变量，函数调用结束后其值依然存在
		Bias_L=TargetVelocity-CurrentVelocity; //求速度偏差
		ControlVelocity_L=Velcity_Kp*Bias_L+Velcity_Ki*Bias_L+Velcity_Kd*(Bias_L-Last_bias_L);  //位置式PI控制器
                                                                   //Velcity_Kp*(Bias-Last_bias) 作用为限制加速度
	                                                                 //Velcity_Ki*Bias             速度控制值由Bias不断积分得到 偏差越大加速度越大
		
		Last_bias_L=Bias_L;	
		if(ControlVelocity_L>9)
		{
			ControlVelocity_L=9;
		}
		if(ControlVelocity_L<0)
		{
			ControlVelocity_L=0;
		}
		return ControlVelocity_L; //返回速度控制值
}
int Velocity_FeedbackControl_R(int TargetVelocity, int CurrentVelocity)
{
		int Bias_R;  //定义相关变量
		static int ControlVelocity_R, Last_bias_R; //静态变量，函数调用结束后其值依然存在
		Bias_R=TargetVelocity-CurrentVelocity; //求速度偏差
		ControlVelocity_R=Velcity_Kp*Bias_R+Velcity_Ki*Bias_R+Velcity_Kd*(Bias_R-Last_bias_R);  //位置式PI控制器
                                                                   //Velcity_Kp*(Bias-Last_bias) 作用为限制加速度
	                                                                 //Velcity_Ki*Bias             速度控制值由Bias不断积分得到 偏差越大加速度越大
		Last_bias_R=Bias_R;	
		if(ControlVelocity_R>9)
		{
			ControlVelocity_R=9;
		}
		if(ControlVelocity_R<0)
		{
			ControlVelocity_R=0;
		}
		return ControlVelocity_R; //返回速度控制值
}
void PID_motor_L(int TargetVelocity)
{
		PWM_L=Velocity_FeedbackControl_L(TargetVelocity,n_l);
		moter_set(moter_L,PWM_L);
}
void PID_motor_R(int TargetVelocity)
{
		PWM_R=Velocity_FeedbackControl_R(TargetVelocity,n_r);
		moter_set(moter_R,PWM_R);
}
void PID_control()
{
	if(PID_state_Volocity)
	{
		PID_motor_L(PID_Target_L);
		PID_motor_R(PID_Target_R);
	}
}
#endif