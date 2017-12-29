 
#include "common.h"
#include "ftm.h"
#include <math.h>
#include "uart.h"
#include "adc.h"
int fabss(int n);
float Gyro_Now,angle_offset_vertical,Gyro_DIR,Gyro_DIRR;//��һ������ٶȺͽǶ�
float g_fGyroscopeAngleIntegral, angle_now ;
volatile float     MMA7361 ,ENC03,ENC03_DIR,gyro_offset,gyro_DIR;//ad����Ƕȣ�ad������ٶȣ�  
volatile float  Speed_L , Speed_R , speed_Start , Speed_L_Last , Speed_R_Last , PWM_angle , PWM_speed; 
              //�����ٶ�,�����ٶ� , ��������ѹ ��                                 PD��ѹ    ��PI��ѹ
extern float DirectionOut;
extern float SpeedControlOut; //PID������ݵ�ǰ��������Ŀ�õ�ƫ��
int i;

float P_ANGLE=1370;//1570
float D_ANGLE=22;//15
float P_DIRR=0;//35;
float P_DIRL=0;//51.5,44
float MOTOR_DEAD_VAL_L=0;//������ѹ��
float MOTOR_DEAD_VAL_R=0;//180;//������ѹ��180
//*******************�˲�*************//
//�����һ����ٶȺͽǶȵõ��ںϽ��ٶȺͽǶ�
float Rate;//�ںϽ��ٶ�
float angle ; //�ںϽǶ�
float DT=0.005;//0.0065.0.007
float GRAVITY_ADJUST_TIME_CONSTANT=1;//0.1,0.2  1.2
void QingHua_AngleCalaulate(float G_angle,float Gyro)
{
    float fDeltaValue;
    angle_now = g_fGyroscopeAngleIntegral;   
    fDeltaValue = (G_angle - angle_now) / GRAVITY_ADJUST_TIME_CONSTANT;  
    g_fGyroscopeAngleIntegral += (Gyro + fDeltaValue) * DT;               
}

//**************�������˲�*********************//
/*
float angle, angle_dot;         //�ⲿ��Ҫ���õı���
//-------------------------------------------------------
// 0.00015     //0.0001
const float Q_angle=0.001, Q_gyro=0.001, R_angle=0.1, dt=0.005;
//0.0001         //0.00015        //1.2
//ע�⣺dt��ȡֵΪkalman�˲�������ʱ��;         //0.8
static float P[2][2] = {
    { 1, 0 },
    { 0, 1 }
};

static float Pdot[4] ={0,0,0,0};

static const char C_0 = 1;

static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
//-------------------------------------------------------
void Kalman_Filter(float angle_m,float gyro_m)          //gyro_m:gyro_measure
{
    angle+=(gyro_m-q_bias) * dt;
    
    Pdot[0]=Q_angle - P[0][1] - P[1][0];
    Pdot[1]=- P[1][1];
    Pdot[2]=- P[1][1];
    Pdot[3]=Q_gyro;
    
    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;
    
    
    angle_err = angle_m - angle;
    
    

    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    
    E = R_angle + C_0 * PCt_0;
    
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
    
    
    angle   += K_0 * angle_err;
    q_bias  += K_1 * angle_err;
    angle_dot = gyro_m-q_bias;
}*/

//��һ�����㲢�õ���ʵ�Ƕ�ֵ//
void AD_Calculate(void)
{
	// ENC03 ad���ٶ�
	// MMA7361 ad�Ƕ�
	// Gyro_Now��һ���ٶ�
	// angle_offset_vertical��һ�Ƕ�
	
		MMA7361=ad_once(ADC0, SE13, ADC_12bit);
		ENC03=ad_once(ADC0, SE12, ADC_12bit);
                ENC03_DIR=ad_once(ADC0, SE17, ADC_12bit);
	  Gyro_Now = (gyro_offset - ENC03 ) * 0.2 ;//0.2,0.3
          Gyro_DIRR =  -(gyro_DIR - ENC03_DIR ) * 0.1 ;
    angle_offset_vertical = (1257- MMA7361) *0.13;//��С��ǰ  1257
    
    //if(Gyro_DIR>=0) Gyro_DIR=Gyro_DIRR*P_DIRR;
  //  else Gyro_DIR=Gyro_DIRR*P_DIRL;
   //if(angle_offset_vertical > 90)angle_offset_vertical = 90;
  // if(angle_offset_vertical < -90)angle_offset_vertical = -90;
	//Kalman_Filter(angle_offset_vertical,Gyro_Now) ;
        QingHua_AngleCalaulate(angle_offset_vertical,Gyro_Now) ;
}
 



void Speed_Calculate( float angle, float Gyro_Now)
 {
      PWM_angle = angle_now * P_ANGLE + Gyro_Now * D_ANGLE ;//angle�����ںϽǶ�    Rate�ںϽ��ٶ�     
      speed_Start=PWM_angle+SpeedControlOut;//-fabss(Gyro_DIR);
     
      
 

 Speed_L = speed_Start+DirectionOut;//g_fDirectionOut
 Speed_R = speed_Start-DirectionOut;
 /************************************/
 if(Speed_L > 9900) Speed_L = 9900;
 if(Speed_L < -9900) Speed_L = -9900;
 if(Speed_R > 9900) Speed_R = 9900;
 if(Speed_R < -9900) Speed_R = -9900;

	 /*********************************/
    if(Speed_L > 0)    
        Speed_L_Last = 10000 - Speed_L;
    else
        Speed_L_Last = -10000 - Speed_L;

    if(Speed_R > 0)     
        Speed_R_Last = 10000 - Speed_R;
    else
        Speed_R_Last = -10000 - Speed_R;
 
 /***********PWM����******************/
 if(Speed_L >= 0) 
{
FTM_PWM_Duty(FTM0, CH0, 10000);
FTM_PWM_Duty(FTM0, CH1,Speed_L_Last - MOTOR_DEAD_VAL_L); //??????
}
else
{
FTM_PWM_Duty(FTM0, CH1,10000);
FTM_PWM_Duty(FTM0, CH0,-Speed_L_Last - MOTOR_DEAD_VAL_L); //??????
 }

 if(Speed_R >= 0) 
 
{
FTM_PWM_Duty(FTM0, CH2,10000);
FTM_PWM_Duty(FTM0, CH3,Speed_R_Last - MOTOR_DEAD_VAL_R); //??????
 }
 else
 {
FTM_PWM_Duty(FTM0, CH3,10000);
FTM_PWM_Duty(FTM0, CH2,-Speed_R_Last - MOTOR_DEAD_VAL_R); //??????
 }

     if(angle_now>33||angle_now<-11)
     {FTM_PWM_Duty(FTM0, CH0, 10000);
      FTM_PWM_Duty(FTM0, CH1, 10000);
      FTM_PWM_Duty(FTM0, CH2, 10000);
      FTM_PWM_Duty(FTM0, CH3, 10000);
      DisableInterrupts;}
}

 
