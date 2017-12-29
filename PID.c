/******************** (C) COPYRIGHT 2012-2013 岱默科技 DEMOK*********
 * 文件名         ： PID.c
 * 描述           ： 欧姆龙编码器PID控制程序
 *
 * 实验平台       ： 岱默科技DEMOK Kinetis开发板
 * 作者           ： 岱默科技DEMOK Kinetis开发小组

 * 淘宝店铺       ： http://shop60443799.taobao.com/
 * 技术交流邮箱   ： 1030923155@qq.com 
 * 技术交流QQ群   ： 103360642

 * 最后修订时间    ：2013-05-14
 * 修订内容        ：无
**********************************************************************************/

#include "common.h"
float e_1,e_2;

#define ts 0.5//采样时间0.005
//#define Ki 0
//#define Kd 0//40
extern s16 CurrentVelocity;
//float Kp=8;//7
 float Kp;
float Ti=0;//0.06
float Td=0;//0.22
//s16 SetPoint=-30;//-30
extern s16 SetPoint;
s16 iError;

int SpeedControlPeriod;
float A=0,B=0,C=0;
float SpeedControlOutNew,SpeedControlOutOld,SpeedControlOut,SpeedControlIntegral,Velocity,Velocity_1,a;    //当前误差
float fDelta=0; 

void SpeedControl(void)
{
       CurrentVelocity=FTM1_CNT-FTM2_CNT;
       CurrentVelocity=CurrentVelocity/2;       //读取两个电极脉冲计数平均值  
       FTM1_CNT= 0;                //清计数
       FTM2_CNT=0;
       
       Velocity=0.5*CurrentVelocity+0.5*Velocity_1;
       Velocity_1=Velocity;
/* 
    A=Kp*(1+ts/Ti+Td/ts);
    B=-Kp*(1+2*Td/ts);
    C=Kp*Td/ts;
   iError = SetPoint -  Velocity;            //增量计算   
   SpeedControlOutOld = SpeedControlOutNew;  
   SpeedControlOutNew=Kp*(iError-e_1)+Ki*iError+Kd*(iError-2*e_1+e_2);; 
    e_2=e_1;  
    e_1=iError;
   */
    float fP, fI,fD,Ip,E,EC,Kpp; 
    
   
    
    fDelta = SetPoint - Velocity; 
    ;
   // else if (fDelta<100||fDelta>-100) Kp=1.25*Kp;
  if (fDelta<-300)  Ip=0;
  else if (fDelta<-100) Ip=1;
  else Ip=1.2;
  if (fDelta>0) Kp=1.7*Kp,Ip=1;
  /*  if (fDelta>100||fDelta<-100) Ip=0;
    else if (fDelta>75||fDelta<-75) Ip=0.25;
    else if(fDelta>50||fDelta<-50) Ip=0.5;
    else if(fDelta>25||fDelta<-25) Ip=0.75;
    else Ip=1;*/
    
   
    fP = fDelta * Kp; 
    fI = fDelta * Ti*Ip;
    SpeedControlIntegral += fI;
    if (fDelta>60||fDelta<-60)  Td=0;
    fD = (fDelta-e_1)*Td;
    if (SpeedControlIntegral<=-1000) SpeedControlIntegral=-1000;
    if (SpeedControlIntegral>=900) SpeedControlIntegral=900; 
   SpeedControlOutOld = SpeedControlOutNew; 
   SpeedControlOutNew = fP + SpeedControlIntegral + fD;   
   e_1=fDelta;
   
 //  if (fDelta<-700) {if(SpeedControlOutNew<-15000) SpeedControlOutNew=-15000;}
  if(SpeedControlOutNew<=-15000) SpeedControlOutNew=-15000;
}
//u(k)=u(k-1)+Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
 //iIncpid =Kp*(iError-e_1)+Ki*iError+Kd*(iError-2*e_1+e_2);
void SpeedControlOutput(void)
{
   float fValue; 
   fValue = SpeedControlOutNew - SpeedControlOutOld;
   SpeedControlOut = fValue * (SpeedControlPeriod +1)/100+SpeedControlOutOld;
}