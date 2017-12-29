/**************************************************************************
 ** �������ڣ�  2012-11-2                                                 *
 ** �ļ���  ��  isr.h                                                     *
 ** �汾    ��	                                                          *
 ** ƽ̨    ��  ����������MK60DN512ZVLQ10��Сϵͳ��                       *
 ** �����б�	                                                          *
 ** ����������  �жϷ��������¶���                                      *          
 ** ������  ��  ���������� yullion.taobao.com                                                *
 ** �޸���ʷ��                                                            *
 ** @��Ȩ���У� ���������� yullion.taobao.com                                               *
 ** �Ա��꣺    yullion.taobao.com                                        *
 **************************************************************************/

/*******************************************************************************************
��ӭ�����Ա���  ����������  yullion.taobao.com
K60�Ա�����     http://item.taobao.com/item.htm?spm=a1z10.1.w4.18.rMwvaU&id=21039172293
********************************************************************************************/

#ifndef __ISR_H
#define __ISR_H 1

#include  "include.h"

/***************************���¶����ж�������**********************************
 *  ��ȡ��Ĭ�ϵ��ж�����Ԫ�غ궨��       #undef  VECTOR_xxx
 *  �����¶��嵽�Լ���д���жϺ���       #define VECTOR_xxx    xxx_IRQHandler
 *  ���磺
 *       #undef  VECTOR_003
 *       #define VECTOR_003    HardFault_Handler    ���¶���Ӳ���Ϸ��жϷ�����
 *
 *       extren void  HardFault_Handler(void);      ����������Ȼ����isr.c�ﶨ��
 *******************************************************************************/

//#undef  VECTOR_016
//#define VECTOR_016    DMA_CH0_Handler

#undef  VECTOR_020
#define VECTOR_020    DMA_CH4_Handler

#undef  VECTOR_084
#define VECTOR_084    PIT0_IRQHandler     //���¶���84���ж�ΪPIT0_IRQHandler�ж�

//#undef  VECTOR_085
//#define VECTOR_085    PIT1_IRQHandler     //���¶���84���ж�ΪPIT0_IRQHandler�ж�

/********************************************************************************/
#undef  VECTOR_103
#define VECTOR_103    PORTA_IRQHandler    //���¶���103���ж�ΪPORTA_IRQHandler�ж�

#undef  VECTOR_104
#define VECTOR_104    PORTB_IRQHandler    //���¶���104���ж�ΪPORTB_IRQHandler�ж�  
/********************************************************************************/
#define H 320                             //�ɼ���ͼ�������
#define V 51                              //�ɼ���ͼ�������
#define Hx 70
#define PHOTO_SIZE H*V
#define TIAOSHI                           //�����ڵ����ã����ô��ڵ���ʱע�͵�����伴��

extern unsigned char Pix_Data[V][H];       //�ɼ�50�� 200�е�ͼ������      //����ע��
extern unsigned char V_Cnt;                      //�вɼ�����
extern unsigned char Is_SendPhoto;               //ͼ���ͱ�־



extern void DMA_CH0_Handler(void);
extern void DMA_CH4_Handler(void);              //DMAͨ��4���жϷ�����
extern void PIT0_IRQHandler();            //PIT0 ��ʱ�жϷ�����
extern void PIT1_IRQHandler();            //PIT0 ��ʱ�жϷ�����
extern void PORTB_IRQHandler();            //PORTB �жϷ�����
extern void PORTA_IRQHandler();            //PORTA �жϷ�����

#endif
