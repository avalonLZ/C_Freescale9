#include "common.h"
#include "isr.h"
float Dir_P=200;//35,40  8
float Dir_D=-30;//25,20  20
float Dir_I=0;
float fValue1=0,fValue0=0,fDValue=0,temerror0=0,temerror1=0,Pp=0;
float  midline=0,Midline=0;
int fabss(int n);
extern float DirectionOut=0,DirectionOutNew=0,DirectionOutOld=0,Gyro_DIRR;
extern int DirectionControlPeriod=0;
extern int SubBasePoint,SetPoint;
extern float temerror,value;
extern unsigned char LineType[V];
/*
void DirectionControl()
{

 
  
      DirectionOutOld=DirectionOutNew;
                      
      fValue1=temerror;//value;
      if (temerror<3&&temerror>-3)  Dir_P=0.9*Dir_P;
    //  if (temerror<2&&temerror>-2)  Dir_D=1*Dir_D;
     // else Dir_D=1*Dir_D;
    //  else if (temerror<5||temerror>-5)  Dir_D=1.2*Dir_D;
     // else if (temerror<15||temerror>-15)  Dir_D=1*Dir_D;
      fDValue = (float)(fValue1-fValue0);
    
      fValue0=fValue1;
   
      DirectionOutNew = fValue1*Dir_P + Gyro_DIRR*Dir_D;
     
      DirectionOut=DirectionOutNew;                  

     
    }      


*/


 ///////////////////////模糊PID//////////////////////////////
#define EMAX    3    
#define EMIN    -3   
#define ECMAX    3     
#define ECMIN    -3 
#define emax    50    
#define emin    -50   
#define ecmax    100     
#define ecmin    -100
 
float temerror_p,temerror_d,temerror_i,Dir_DD;
float e,ec;
int KP=0;
  



        
int Prule[7][7]={ 
//误差变化率 -3,-2,-1, 0, 1, 2, 3     // 误差     
              6,5, 4, 3, 2, 1, 0,   //   -3   
              5,5, 4, 3, 2, 0,-1,   //   -2 
              4,4, 3, 2, 0,-2,-2,   //   -1 
              3,3, 2, 0,-2,-3,-3,   //    0 
              2,2, 0,-2,-3,-4,-4,   //    1 
              1,0,-2,-3,-4,-5,-5,   //    2 
              0,-1,-2,-3,-4,-5,-6      //  3
                 };
/*  
int Prule[7][7]={ 
//误差变化率 -3,-2,-1, 0, 1, 2, 3     // 误差     
               -6,-6,-6,-5,-5,-5,-4,   //   -3      
               -5,-4,-4,-3,-2,-2,-1,   //   -2    
               -4,-3,-2,-1, 0, 1, 2,   //   -1       
               -4,-3,-1, 0, 1, 3, 4,   //    0      
               -2,-1, 0, 1, 2, 3, 4,   //    1     
                1, 2, 2, 3, 4, 4, 5,   //    2     
                4, 5, 5, 5, 6, 6, 6   //    3
                 };


int Irule[7][7]={ 
// 误差变化率 -3,-2,-1, 0, 1, 2, 3     // 误差     
              -6,-5,-4,-3,-2,-1, 0,   //   -3   
              -6,-5,-4,-3,-2,-1, 1,   //   -2 
              -5,-4,-3,-2, 0, 1, 2,   //   -1 
              -4,-3,-2, 0, 2, 3, 4,   //    0 
              -2,-2, 0, 2, 2, 4, 5,   //    1 
              -1,-1, 1, 3, 3, 5, 6,   //    2 
               0, 0, 2, 4, 4, 5, 6      //  3
                 };
int Drule[7][7]={ 
//误差变化率 -3,-2,-1, 0, 1, 2, 3     // 误差     
              2,-2,-6,-6,-6,-4, 2,   //   -3   
              1,-2,-5,-4,-4,-2, 1,   //   -2 
              0,-1,-4,-3,-2,-1, 0,   //   -1 
              0,-1,-2,-2,-1,-1, 0,   //    0 
              0, 0, 0, 0, 0, 0, 0,   //    1 
              5, 2, 2, 2, 1, 1, 5,   //    2 
              6, 4, 4, 4, 2, 2, 6      //  3
                 };
  
*/     
//////////////////////////////////////////////////////////////////  
  float ke,kec;
    float E,EC;  
  /////////////////////////////模糊PID//////////////////////////////                        
   /**********************************************************/
 
void DirectionControl()   
{ 
   
   
   
    
     DirectionOutOld=DirectionOutNew;             
     temerror1=temerror0;
     temerror0=temerror;

     e=temerror0;
     ec=temerror0-temerror1;
     //ke=(EMAX-EMIN)/(emax-emin);
     //kec=(ECMAX-ECMIN)/(ecmax-ecmin);
     
     E=e/10;//(EMAX+EMIN)/2+ke*(e-(emax+emin)/2);   6
     EC=ec;//(ECMAX+ECMIN)/2+kec*(ec-(ecmax+ecmin)/2);
/*     
       if(E>=5.5)
     E=6;
     else if(E>=4.5)
     E=5;
     else if(E>=3.5)
     E=4;
     elae if(E>=2.5)
     E=3;
     else if(E>=1.5)
     E=2;
     else if(E>=0.5)
     E=1;
     else if(E>=-0.5)
     E=0;
     else if(E>=-1.5)
     E=-1;
     else if(E>=-2.5)
     E=-2;
     else if(E>=-3.5)
     E=-3;
      else if(E>=-4.5)
     E=4;
     else if(E>=-5.5)
     E=-5;
     else 
     E=-6;
 */
     if(E>=2.5)
     E=3;
     else if(E>=1.5)
     E=2;
     else if(E>=0.5)
     E=1;
     else if(E>=-0.5)
     E=0;
     else if(E>=-1.5)
     E=-1;
     else if(E>=-2.5)
     E=-2;
     else
     E=-3;
     
    if(EC>=2.5)
     EC=3;
     else if(EC>=1.5)
     EC=2;
     else if(EC>=0.5)
     EC=1;
     else if(EC>=-0.5)
     EC=0;
     else if(EC>=-1.5)
     EC=-1;
     else if(EC>=-2.5)
     EC=-2;
     else
     EC=-3;
    
    // KP=fabss(E);
     KP=fabss(Prule[(int)(E+3)][(int)(EC+3)]);
 //    KI=Irule[(int)(E+3)][(int)(EC+3)];
 //    KD=Drule[(int)(E+3)][(int)(EC+3)];

     //if (temerror0>1) {if (LineType[42]==1)  temerror0= 15;}
    // if (temerror0<-1) {if (LineType[42]==1)  temerror0= -15;}
     temerror_p=Dir_P+Pp*KP;
     
     if (temerror0>0) Dir_DD=Dir_D*1.4;
     else Dir_DD=Dir_D;
     DirectionOutNew =temerror_p*temerror0+Dir_DD*Gyro_DIRR;     
     DirectionOut=DirectionOutNew;            

}  



void DirectionControlOutput()
{
  float fValue;
  fValue=DirectionOutNew-DirectionOutOld;
  DirectionOut=(fValue)*(DirectionControlPeriod+1)/10+DirectionOutOld;
  }
