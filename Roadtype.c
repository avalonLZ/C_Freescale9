#include "common.h"
#include "ftm.h"
#include "gpio.h"
#include  "delay.h"
u16 stop_count=0;
extern s16 SetPoint,speed;
extern float Kp,Td,Ti,Dir_P,Dir_D,temerror,P_DIRR,P_DIRL,Pp,fDelta,Dir_D,Dir_P;
extern u8 IsStartLine;
extern int enableisstartline,RightStableNumbers,LeftStableNumbers,StableNumbers; 



void Stop2()
{
  if(IsStartLine==1) 
  {
    delayms(150);
 FTM_PWM_Duty(FTM0, CH0, 10000);
  FTM_PWM_Duty(FTM0, CH1, 10000);
  FTM_PWM_Duty(FTM0, CH2, 10000);
  FTM_PWM_Duty(FTM0, CH3, 10000);
  DisableInterrupts;
 /*  SetPoint=0;*/
  }
}

extern int boma1,boma2,boma3,StartPos,EndPos;
void Boma()
{
     if(gpio_get(PORTA, 8)==0) boma2=0;
     else boma2=1;
     if(gpio_get(PORTA, 7)==0) boma1=0;
     else boma1=1;
     if(gpio_get(PORTE, 28)==0) boma3=0;
     else boma3=1;
   /*  
      
     if(boma1==1&&boma2==0)
     {
       speed=-300;
       
       StartPos=36;//37 38
       EndPos=40;//43   42
       
       P_DIRR=0;//27
       P_DIRL=0;//28
       
     //if(fDelta>350||fDelta<-350) Kp=40;
     Kp=30;//56.5;//14     16.5         55
     Ti=2;//4.5;//1.5;//9;6    1.6/4     4
     Td=50;//18;//;
       
       Pp=12;//10
     }*/
 /*    
     if(boma1==0&&boma2==0)
     {
       speed=-500;
       
       StartPos=37;  //32 
       EndPos=41;   //36
       
       P_DIRR=30;//23;//27
       P_DIRL=0;//25;//28
      
        //if(fDelta>350||fDelta<-350) Kp=40; 
     Kp=40;//56.5;//14     16.5         55   95
     Ti=2;//4.5;//1.5;//9;6    1.6/4     4
     Td=0;//18;//;
     Dir_D=-40;
       Pp=13;//12
     }
     */
  /*        
     if(boma1==1&&boma2==1)
     {
       speed=-700;
       
       StartPos=33;//34
       EndPos=41;//38
       
       P_DIRR=35;//20;//77;//36  60
       P_DIRL=0;//17;//38;//40  37
       
        //if(fDelta>350||fDelta<-350) Kp=40;
     Kp=60;//56.5;//14     16.5         55
     Ti=2;//4.5;//1.5;//9;6    1.6/4     4
     Td=0;//18;//;
       Dir_D=-50; 
       Pp=14;
     }*/
     if(boma3==0) enableisstartline=0; 
         
     if(boma1==0&&boma2==1)
     {
       speed=-1270;
       
       StartPos=32;//31
       EndPos=36;//35
       
     //  P_DIRR=0;//20;//77;//36  60
     //  P_DIRL=0;//17;//38;//40  37
      Dir_P=46;//40
     Dir_D=-18; //-35
     Pp=50;//45
        //if(fDelta>350||fDelta<-350) Kp=40;
     Kp=21;//56.5;//14     16.5         55
     Ti=2;//4.5;//1.5;//9;6    1.6/4     4
     Td=0;//18;//;
     }
     
     if(boma1==1&&boma2==1)
     {
       speed=-900;
       
       StartPos=36;//28  32  37
       EndPos=40;//42  40     41
       
     //  P_DIRR=0;//20;//77;//36  60
     //  P_DIRL=0;//17;//38;//40  37
     Dir_P=40;//220  120 350  80   40
     Dir_D=-18; //-50 -25  -42   -55  -18
     Pp=50;//80                        50
        //if(fDelta>350||fDelta<-350) Kp=40;
     Kp=20;//56.5;//14     16.5         55
     Ti=2;//4.5;//1.5;//9;6    1.6/4     4
     Td=0;//18;//;
     
     if((RightStableNumbers<=17)&&(LeftStableNumbers<=17)&&(StableNumbers<=17))
     { 
         temerror=32;//28
     
      // if (temerror<0)  temerror=-40;
     }
    }
     
    
     if(boma1==1&&boma2==0)
     {
      
       speed=-1100;
       
       StartPos=34;//34
       EndPos=38;//37
       
     //  P_DIRR=0;//20;//77;//36  60
     //  P_DIRL=0;//17;//38;//40  37
     Dir_P=47;//220  120 350  80
     Dir_D=-18; //-50 -25  -42   -55
     Pp=50;//80
        //if(fDelta>350||fDelta<-350) Kp=40;
     Kp=21;//56.5;//14     16.5         55
     Ti=2;//4.5;//1.5;//9;6    1.6/4     4
     Td=0;//18;//;
     }
     
      if(boma1==0&&boma2==0)
     {
       speed=-600;
       
       StartPos=39;//34
       EndPos=43;//38
       
     //  P_DIRR=0;//20;//77;//36  60
     //  P_DIRL=0;//17;//38;//40  37
     Dir_P=40;  //40
        //if(fDelta>350||fDelta<-350) Kp=40;
     Kp=17;//56.5;//14     16.5         55
     Ti=2;//4.5;//1.5;//9;6    1.6/4     4
     Td=0;//18;//;
       Dir_D=-10;//-35 
       Pp=45;//45
       
     /*  if((RightStableNumbers<=13)&&(LeftStableNumbers<=13)&&(StableNumbers<=13))
      { 
     //  if (temerror>0)  temerror=30;
         temerror=-23;
     }*/
     }
}


void Stop()
{   
   
   
  if(stop_count<=300)
  {  
     stop_count++;
      Kp=0;
      Ti=0;
      Dir_D=0;
    //temerror=0;
    // SetPoint=0;
  }
   else if (stop_count>300)
   {
     Boma();
   
   }
  
}