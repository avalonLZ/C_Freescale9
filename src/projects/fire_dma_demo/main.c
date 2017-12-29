
#include "common.h"
#include "include.h"
#include "isr.h"

extern int enableisstartline;
extern float OutData[4],gyro_offset,gyro_DIR;
extern volatile s16 CurrentVelocity = 0;         //�������� 10ms������Ŀ  ��isr.c������

extern u8 FieldCounter,IsStartLine; 

int CountBlack,boma2,boma1,boma3;
extern s16 stop_count;
void OutPut_Data(void);
void AD_Calculate(void);
void Speed_Calculate(float angle,float Rate);
void Stop2();
extern int count,podaocount,podao;

   void main()
{   int i=0,j=0,k=0;
   
   DisableInterrupts;       //��ֹ���ж�
   V_Cnt=0;                                    //�м���
   Is_SendPhoto=0;                             //�Ӵ��ڷ���ͼ��
    
    uart_init(UART4, 9600);                         
    adc_init(ADC0, SE12);                  
    adc_init(ADC0, SE13);
    adc_init(ADC0, SE17);
    FTM1_QUAD_Iint();
    FTM2_QUAD_Iint();
    FTM_PWM_init(FTM0, CH0, 8000, 0);            //PTC1       1000Hz
    FTM_PWM_init(FTM0, CH1, 8000, 0);            //PTC2       1000Hz
    FTM_PWM_init(FTM0, CH2, 8000, 0);            //PTC3       1000Hz
    FTM_PWM_init(FTM0, CH3, 8000, 0);            //PTC4       1000Hz
    gpio_init (PORTA, 7, 0, 0);
    gpio_init (PORTA, 8, 0, 0);
    gpio_init (PORTE, 28, 0, 0);
    pit_init_ms(PIT0, 1);
    
    delayms(100);
    for(k=0;k<20;k++)
    {
      gyro_offset+=ad_once(ADC0, SE12, ADC_12bit);
      gyro_DIR+=ad_once(ADC0, SE17, ADC_12bit);
    }
    gyro_offset=gyro_offset/20;
    gyro_DIR=gyro_DIR/20;
    
    delayms(1500);
      /*********************************************************************************/
    exti_init(PORTA,9,rising_down);            //���жϣ�PORTA29 �˿��ⲿ�жϳ�ʼ�� �������ش����жϣ��ڲ�����
    disable_irq(87);                            //���жϹر�
    exti_init(PORTB,4,falling_down);            //���жϣ�PORTB0 �˿��ⲿ�жϳ�ʼ�� ���½��ش����жϣ��ڲ�����
    /*************************************************************************************/
   // pll_init(PLL50);
   // LCD_Init();
    EnableInterrupts;	    
    //LCD_clear();	
    while(1)
    { 
     enableisstartline++;
    // if(podao==1)             podaocount++;
     if(enableisstartline<1100) IsStartLine=0;
     else
      {  
         enableisstartline=1100;
         CheckStartLine(); 
         Stop2();
      }

      
      Stop();
      
       
       
  
      
   
      
   
     // OutData[0]=Gyro_DIRR;//k1;//SubBasePoint;
      OutData[1]= boma1;
      OutData[2]=boma2;
      OutData[3]=boma3;
      OutPut_Data(); 
    
    }
}
