/******************** (C) COPYRIGHT 2011 岱默科技DEMOK Kinetis开发小组**************
 * 文件名         ： isr.c
 * 描述           ： 中断处理例程
 *
 * 实验平台       ： 岱默科技DEMOK Kinetis开发板
 * 作者           ： 岱默科技DEMOK Kinetis开发小组

 * 淘宝店铺       ： http://shop60443799.taobao.com/
 * 技术交流邮箱   ： 1030923155@qq.com 
 * 技术交流QQ群   ： 103360642

 * 最后修订时间    ：2012-11-7
 * 修订内容        ：无
**********************************************************************************/



#include "common.h"
#include "include.h"
#include "isr.h"
extern volatile s16 CurrentVelocity;         //用来计数
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
volatile u32 dma_int_count = 0;//DMA传输次数初始化

/******************************************************************************/
unsigned char Pix_Data[V][H] = {0};                                     //采集Ｖ行 Ｈ列的图像数据
unsigned char Pix_Data1[V][Hx] = {0}; 
unsigned char Pix_Data2[V][Hx] = {0};
unsigned char V_Cnt=16;                                                  //行采集计数
unsigned char Is_SendPhoto=0;                                           //图像发送标志
u8 FieldCounter=0;                                           //判断奇偶场
int enableisstartline=0;

void PIT0_IRQHandler(void)
{
    DisableInterrupts;           //禁止总中断
   
    PIT_CNT++;     
    
    SpeedControlPeriod ++; 
    SpeedControlOutput();     //速度控制，每1ms加一点，100ms为一周期      
     
    DirectionControlPeriod ++;
    DirectionControlOutput(); //角度控制，每1ms加一点，10ms为一周期
   
    
     if(PIT_CNT==1)            //小车角度计算
    {      
      AD_Calculate();  
    }
    
    else if(PIT_CNT==2)            //电机PWM
    {  
       Speed_Calculate(angle,Gyro_Now); 
    }
    else if(PIT_CNT==3)            //小车速度计算 
    { 
       SpeedControlCount ++; 
       if(SpeedControlCount >= 20) //如果大于100ms（20*5）
        { 
          //enableisstartline++;
          SpeedControl(); 
          SpeedControlCount = 0; 
          SpeedControlPeriod = 0; 
         // LCD_Write_String(4, 4,"xmu");
        }      
    } 
    else if(PIT_CNT==4)             //中线提取
    {  
      DirectionControlCount ++;
      if(DirectionControlCount >= 2) //如果大于20ms（4*5）
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
    PIT_Flag_Clear(PIT0);       //清中断标志位     
    EnableInterrupts;	        //开总中断
}





//正常运行时程序段
/*************************************************************************
*  函数名称：DMA_CH4_Handler
*  功能说明：DMA通道4的中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：
*  备    注：
*************************************************************************/
void DMA_CH4_Handler(void)
{
    DMA_IRQ_CLEAN(DMA_CH4);                                 //清除通道传输中断标志位    (这样才能再次进入中断)
    DMA_DIS(DMA_CH4);                                       //采集完H个数据后进入这个DMA中断，停止DMA传输。行中断中打开DMA传输
    if(V_Cnt == 220)
    Is_SendPhoto = 1;
}

/************************************************************************/
//晓克添加
//行中断处理函数
void PORTA_IRQHandler()
{
    if((PORTA_ISFR & (1<<9)))                              //PTA29触发中断,行中断
    {
      PORTA_ISFR |= (1<<9);                                //写1清中断标志位
      /**************用户任务**************************************************/
      if((++V_Cnt)%4 == 0)                                  //判断该行数据是否需要,根据自己需要的行数自己设定判别条件
      {
        DMA_EN(DMA_CH4);                                    //使能通道CHn 硬件请求
      }
                           //可以在这里判断需要的数据是否已经采集完，可以设置一个标志位
      /****************
      *这里也可以进行数据处理
      *****************/
             
      
      /************************************************************************/
    }
}

//场中断处理函数
void PORTB_IRQHandler()
{
    if(PORTB_ISFR & (1<<4))                                 //PTB0触发中断,场中断
    {
      PORTB_ISFR |= (1<<4);                                 //写1清中断标志位   
      /**************用户任务**************************************************/
      //奇偶场判别
      FieldCounter++;                        
      if(FieldCounter>= 2)
      {
        FieldCounter = 0;
      }
      //奇场      采集图像
      if(FieldCounter == 0)
      {
        DMA_PORTx2BUFF_Init (DMA_CH4, (void *)&PTD_BYTE0_IN, Pix_Data, PTC0, DMA_BYTE1, H, DMA_rising_keepon); 
        //DMA通道4初始化，PTC0上升沿触发DMA传输，源地址为PTD_BYTE0_IN，目的地址为：Pix_Data ，每次传输1Byte，传输H次后停止传输，目的地址保持不变，关闭通道CHn 硬件请求
        enable_irq(87);                                     //使能PORTA中断，即开启行中断，PORTA的ISR中断号为87
        V_Cnt=16;                                            //行采集计数清零
      }
      //偶场
       else
      {
        DMA_DIS(DMA_CH4);                                   //停止DMA传输
        disable_irq(87);                                    //关闭PORTA中断，即关闭行中断，PORTA的ISR中断号为87
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
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：FTM2_IRQHandler
*  功能说明：FTM2输入捕捉中断服务函数
*  参数说明：无
*  函数返回：无
*  备    注：引脚号需要根据自己初始化来修改，参考现有的代码添加自己的功能
*
*************************************************************************/

void FTM2_IRQHandler()
{

  u8 s = FTM2_STATUS;             //读取捕捉和比较状态  All CHnF bits can be checked using only one read of STATUS.
    u8 CHn;
    FTM2_STATUS = 0x00;             //清中断标志位

    CHn = 0;
    if( s & (1 << CHn) )
    {
        FTM_IRQ_DIS(FTM2, CHn);     //禁止输入捕捉中断
        /*     用户任务       */
    


        /*********************/
        //不建议在这里开启输入捕捉中断，而是在main函数里根据需要来开启
        //通道 CH0、CH1、Ch2、Ch3 有滤波器
        FTM_IRQ_EN(FTM2, CHn); //开启输入捕捉中断
        //delayms(10);        //因为输入的信号跳变过程不稳定，容易触发多次输入捕捉，所以添加延时
        //但考虑到中断不应该过长延时，所以开输入捕捉中断就放在main函数里，根据需要来开启
    }
}



/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：USART1_IRQHandler
*  功能说明：串口1 中断 接收 服务函数
*  参数说明：无
*  函数返回：无
*
*************************************************************************/
void USART1_IRQHandler(void)
{
    uint8 ch;

    DisableInterrupts;		    //关总中断

    //接收一个字节数据并回发
    ch = uart_getchar (UART1);      //接收到一个数据
    uart_sendStr  (UART1, "\n你发送的数据为：");
    uart_putchar (UART1, ch);       //就发送出去

    EnableInterrupts;		    //开总中断
}

/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：SysTick_Handler
*  功能说明：系统滴答定时器中断服务函数
*  参数说明：无
*  函数返回：无
*
*************************************************************************/
void SysTick_Handler(void)
{
    //    OSIntEnter();
    //    OSTimeTick();
    //    OSIntExit();
}


/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：HardFault_Handler
*  功能说明：硬件上访中断服务函数
*  参数说明：无
*  函数返回：无
*
*************************************************************************/
void HardFault_Handler(void)
{
    while (1)
    {
        printf("\n****硬件上访错误!!!*****\r\n\n");
    }
}

/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：PendSV_Handler
*  功能说明：PendSV（可悬起系统调用）中断服务函数
*  参数说明：无
*  函数返回：无
*
*************************************************************************/
void PendSV_Handler(void)
{
}





/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：FTM0_IRQHandler
*  功能说明：FTM0输入捕捉中断服务函数
*  参数说明：无
*  函数返回：无
*  备    注：引脚号需要根据自己初始化来修改，参考现有的代码添加自己的功能
*
*************************************************************************/
void FTM0_IRQHandler()
{


}

/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：FTM1_IRQHandler
*  功能说明：FTM1输入捕捉中断服务函数
*  参数说明：无
*  函数返回：无
*  备    注：引脚号需要根据自己初始化来修改，参考现有的代码添加自己的功能
*
*************************************************************************/

void FTM1_IRQHandler()
{

  
}






