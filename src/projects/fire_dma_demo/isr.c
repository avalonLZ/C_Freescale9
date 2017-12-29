/******************** (C) COPYRIGHT 2011 �Ĭ�Ƽ�DEMOK Kinetis����С��**************
 * �ļ���         �� isr.c
 * ����           �� �жϴ�������
 *
 * ʵ��ƽ̨       �� �Ĭ�Ƽ�DEMOK Kinetis������
 * ����           �� �Ĭ�Ƽ�DEMOK Kinetis����С��

 * �Ա�����       �� http://shop60443799.taobao.com/
 * ������������   �� 1030923155@qq.com 
 * ��������QQȺ   �� 103360642

 * ����޶�ʱ��    ��2012-11-7
 * �޶�����        ����
**********************************************************************************/



#include "common.h"
#include "include.h"
#include "isr.h"
extern volatile s16 CurrentVelocity;         //��������
extern float Gyro_Now,angle_offset_vertical;
extern float angle,speed_Start,Rate;
extern volatile float  MMA7361 ,ENC03; 
extern int SpeedControlPeriod,DirectionControlPeriod;

//extern float P_ANGLE,D_ANGLE,Kp,Ti;
//extern float DirectionOut;
extern s16 SetPoint;
int PIT_CNT,SpeedControlCount,DirectionControlCount,PulsePeriod,flag10ms;
int fabss(int n)
{
	if(n < 0)return (-1)*n;
	else return n;	
}
#define MAX(a,b)            (((a) > (b)) ? (a) : (b))
#define Min(a,b)            (((a) < (b)) ? (a) : (b)
int b;
int otsu (unsigned char *image, int rows, int cols, int x0, int y0, int dx, int dy, int vvv);
extern int BlackLineData[V];
void AD_Calculate(void);
void Speed_Calculate(float angle,float Gyro_Now);
void DirectionControl(void);
void SpeedControlOutput(void);
void DirectionControlOutput(void);
void SpeedControl(void);
void HandleImg();
void hdgs();
void CheckStartLine();
void lvbo (unsigned char ImageData[][70]);
void GetImageParam();
volatile u8  pit_flag = 0;
volatile u32 dma_int_count = 0;//DMA���������ʼ��

/******************************************************************************/
unsigned char Pix_Data[V][H] = {0};                                     //�ɼ����� ���е�ͼ������
unsigned char Pix_Data1[V][Hx] = {0}; 
unsigned char Pix_Data2[V][Hx] = {0};
unsigned char V_Cnt=16;                                                  //�вɼ�����
unsigned char Is_SendPhoto=0;                                           //ͼ���ͱ�־
u8 FieldCounter=0;                                           //�ж���ż��
int enableisstartline=0;

void PIT0_IRQHandler(void)
{
    DisableInterrupts;           //��ֹ���ж�
   
    PIT_CNT++;     
    
    SpeedControlPeriod ++; 
    SpeedControlOutput();     //�ٶȿ��ƣ�ÿ1ms��һ�㣬100msΪһ����      
     
    DirectionControlPeriod ++;
    DirectionControlOutput(); //�Ƕȿ��ƣ�ÿ1ms��һ�㣬10msΪһ����
   
    
     if(PIT_CNT==1)            //С���Ƕȼ���
    {      
      AD_Calculate();  
    }
    
    else if(PIT_CNT==2)            //���PWM
    {  
       Speed_Calculate(angle,Gyro_Now); 
    }
    else if(PIT_CNT==3)            //С���ٶȼ��� 
    { 
       SpeedControlCount ++; 
       if(SpeedControlCount >= 20) //�������100ms��20*5��
        { 
          //enableisstartline++;
          SpeedControl(); 
          SpeedControlCount = 0; 
          SpeedControlPeriod = 0; 
         // LCD_Write_String(4, 4,"xmu");
        }      
    } 
    else if(PIT_CNT==4)             //������ȡ
    {  
      DirectionControlCount ++;
      if(DirectionControlCount >= 2) //�������20ms��4*5��
      { 
        
 
          DirectionControl();
          DirectionControlCount = 0;
          DirectionControlPeriod = 0; 
      }
    }
    else if(PIT_CNT>=5)
    {            
      PIT_CNT=0;
      PulsePeriod++;
      if(PulsePeriod>=2)
      {  
       PulsePeriod=0;
     
     
      }
    } 
    PIT_Flag_Clear(PIT0);       //���жϱ�־λ     
    EnableInterrupts;	        //�����ж�
}





//��������ʱ�����
/*************************************************************************
*  �������ƣ�DMA_CH4_Handler
*  ����˵����DMAͨ��4���жϷ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺
*  ��    ע��
*************************************************************************/
void DMA_CH4_Handler(void)
{
    DMA_IRQ_CLEAN(DMA_CH4);                                 //���ͨ�������жϱ�־λ    (���������ٴν����ж�)
    DMA_DIS(DMA_CH4);                                       //�ɼ���H�����ݺ�������DMA�жϣ�ֹͣDMA���䡣���ж��д�DMA����
    if(V_Cnt == 220)
    Is_SendPhoto = 1;
}

/************************************************************************/
//�������
//���жϴ�����
void PORTA_IRQHandler()
{
    if((PORTA_ISFR & (1<<9)))                              //PTA29�����ж�,���ж�
    {
      PORTA_ISFR |= (1<<9);                                //д1���жϱ�־λ
      /**************�û�����**************************************************/
      if((++V_Cnt)%4 == 0)                                  //�жϸ��������Ƿ���Ҫ,�����Լ���Ҫ�������Լ��趨�б�����
      {
        DMA_EN(DMA_CH4);                                    //ʹ��ͨ��CHn Ӳ������
      }
                           //�����������ж���Ҫ�������Ƿ��Ѿ��ɼ��꣬��������һ����־λ
      /****************
      *����Ҳ���Խ������ݴ���
      *****************/
             
      
      /************************************************************************/
    }
}

//���жϴ�����
void PORTB_IRQHandler()
{
    if(PORTB_ISFR & (1<<4))                                 //PTB0�����ж�,���ж�
    {
      PORTB_ISFR |= (1<<4);                                 //д1���жϱ�־λ   
      /**************�û�����**************************************************/
      //��ż���б�
      FieldCounter++;                        
      if(FieldCounter>= 2)
      {
        FieldCounter = 0;
      }
      //�泡      �ɼ�ͼ��
      if(FieldCounter == 0)
      {
        DMA_PORTx2BUFF_Init (DMA_CH4, (void *)&PTD_BYTE0_IN, Pix_Data, PTC0, DMA_BYTE1, H, DMA_rising_keepon); 
        //DMAͨ��4��ʼ����PTC0�����ش���DMA���䣬Դ��ַΪPTD_BYTE0_IN��Ŀ�ĵ�ַΪ��Pix_Data ��ÿ�δ���1Byte������H�κ�ֹͣ���䣬Ŀ�ĵ�ַ���ֲ��䣬�ر�ͨ��CHn Ӳ������
        enable_irq(87);                                     //ʹ��PORTA�жϣ����������жϣ�PORTA��ISR�жϺ�Ϊ87
        V_Cnt=16;                                            //�вɼ���������
      }
      //ż��
       else
      {
        DMA_DIS(DMA_CH4);                                   //ֹͣDMA����
        disable_irq(87);                                    //�ر�PORTA�жϣ����ر����жϣ�PORTA��ISR�жϺ�Ϊ87
       hsz ();
       b=otsu(&Pix_Data1[0][0],51,70,0,0,70,51,0);
       lvbo (Pix_Data1);      
       HandleImg();    
       GetImageParam();
       
      } 
   
      /************************************************************************/
    }
}
/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�FTM2_IRQHandler
*  ����˵����FTM2���벶׽�жϷ�����
*  ����˵������
*  �������أ���
*  ��    ע�����ź���Ҫ�����Լ���ʼ�����޸ģ��ο����еĴ�������Լ��Ĺ���
*
*************************************************************************/

void FTM2_IRQHandler()
{

  u8 s = FTM2_STATUS;             //��ȡ��׽�ͱȽ�״̬  All CHnF bits can be checked using only one read of STATUS.
    u8 CHn;
    FTM2_STATUS = 0x00;             //���жϱ�־λ

    CHn = 0;
    if( s & (1 << CHn) )
    {
        FTM_IRQ_DIS(FTM2, CHn);     //��ֹ���벶׽�ж�
        /*     �û�����       */
    


        /*********************/
        //�����������￪�����벶׽�жϣ�������main�����������Ҫ������
        //ͨ�� CH0��CH1��Ch2��Ch3 ���˲���
        FTM_IRQ_EN(FTM2, CHn); //�������벶׽�ж�
        //delayms(10);        //��Ϊ������ź�������̲��ȶ������״���������벶׽�����������ʱ
        //�����ǵ��жϲ�Ӧ�ù�����ʱ�����Կ����벶׽�жϾͷ���main�����������Ҫ������
    }
}



/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�USART1_IRQHandler
*  ����˵��������1 �ж� ���� ������
*  ����˵������
*  �������أ���
*
*************************************************************************/
void USART1_IRQHandler(void)
{
    uint8 ch;

    DisableInterrupts;		    //�����ж�

    //����һ���ֽ����ݲ��ط�
    ch = uart_getchar (UART1);      //���յ�һ������
    uart_sendStr  (UART1, "\n�㷢�͵�����Ϊ��");
    uart_putchar (UART1, ch);       //�ͷ��ͳ�ȥ

    EnableInterrupts;		    //�����ж�
}

/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�SysTick_Handler
*  ����˵����ϵͳ�δ�ʱ���жϷ�����
*  ����˵������
*  �������أ���
*
*************************************************************************/
void SysTick_Handler(void)
{
    //    OSIntEnter();
    //    OSTimeTick();
    //    OSIntExit();
}


/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�HardFault_Handler
*  ����˵����Ӳ���Ϸ��жϷ�����
*  ����˵������
*  �������أ���
*
*************************************************************************/
void HardFault_Handler(void)
{
    while (1)
    {
        printf("\n****Ӳ���Ϸô���!!!*****\r\n\n");
    }
}

/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�PendSV_Handler
*  ����˵����PendSV��������ϵͳ���ã��жϷ�����
*  ����˵������
*  �������أ���
*
*************************************************************************/
void PendSV_Handler(void)
{
}





/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�FTM0_IRQHandler
*  ����˵����FTM0���벶׽�жϷ�����
*  ����˵������
*  �������أ���
*  ��    ע�����ź���Ҫ�����Լ���ʼ�����޸ģ��ο����еĴ�������Լ��Ĺ���
*
*************************************************************************/
void FTM0_IRQHandler()
{


}

/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�FTM1_IRQHandler
*  ����˵����FTM1���벶׽�жϷ�����
*  ����˵������
*  �������أ���
*  ��    ע�����ź���Ҫ�����Լ���ʼ�����޸ģ��ο����еĴ�������Լ��Ĺ���
*
*************************************************************************/

void FTM1_IRQHandler()
{

  
}






