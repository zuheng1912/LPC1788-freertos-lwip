/**********************************************************************
�ļ�����GPIO_Buzzer.c
��  �ܣ�����������������3��LED�����������˸
Ŀ  �ģ��ó�ѧ���ܿ��ٵ��˽�GPIO���÷���ϵͳ�δ��жϣ�SysTick�����÷���
��  �ߣ�OPENMCU
��  �ڣ�2014��9��26��
**********************************************************************/
#include "lpc177x_8x_gpio.h"
#include "lpc177x_8x_clkpwr.h"
#include "bsp.h"
#include "lpc177x_8x_uart.h"
#include "stdio.h"
#include "debug_frmwrk.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "client.h"
 
#define START_TASK_PRIO 1 //�������ȼ�
#define START_STK_SIZE 128 //�����ջ��С
TaskHandle_t StartTask_Handler; //������
void start_task(void *pvParameters); //������

#define LED_TASK_PRIO 2 //�������ȼ�
#define LED_STK_SIZE 50 //�����ջ��С
TaskHandle_t LEDTask_Handler; //������
void led_task(void *p_arg); //������

TaskHandle_t LED1Task_Handler; //������
void led1_task(void *p_arg); //������




volatile unsigned long SysTickCnt;
extern void Uart_Init(void);
extern int fputc(int ch, FILE *f);

extern void TCPIP_Init(void);  //���ڳ�ʼ��lwip
 
void SysTick_Handler (void);
void Delay (unsigned long tick);
void init (void);


/*********************************************************************
��������	SysTick_Handler
��	�ܣ�	ϵͳ�δ��жϺ��� 1ms ��һ���жϡ�
��	����	��
��	�أ� 	��
 **********************************************************************/
extern void xPortSysTickHandler(void);
void SysTick_Handler (void)
{
  //	SysTickCnt++;
	
	#if (INCLUDE_xTaskGetSchedulerState == 1 )
 if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
 #endif /* INCLUDE_xTaskGetSchedulerState */
 xPortSysTickHandler();
 #if (INCLUDE_xTaskGetSchedulerState == 1 )
	 
 }
 #endif /* INCLUDE_xTaskGetSchedulerState */
}
/***********************************************************************
��������	Delay
��	�ܣ�	��ʱ����
��  ����	tick tick��ֵ������ʱ���ٸ��δ�һ���δ�Ϊ 1ms.
��  �أ�	��
***********************************************************************/
void Delay (unsigned long tick)
{
	unsigned long systickcnt;

	systickcnt = SysTickCnt;
	while ((SysTickCnt - systickcnt) < tick);
}
/**********************************************************************
��������init
��  �ܣ���ʼ��ϵͳʱ�ӵδ�GPIO�ķ���
��  ������
��  �أ���
**********************************************************************/
void init (void)
{
	uint32_t cclk = CLKPWR_GetCLK(CLKPWR_CLKTYPE_CPU);//���Ƶ��120000000Hz
	/*����ϵͳʱ�ӵδ��ж� 1ms*/
	SysTick_Config(cclk/1000);
	/*����GPIO�ķ���Ϊ���*/
	GPIO_SetDir(2, 1<<27, 1);//�������ܽ�����Ϊ���
	GPIO_SetDir(BRD_LED_2_CONNECTED_PORT, BRD_LED_2_CONNECTED_MASK, GPIO_DIRECTION_OUTPUT);
	GPIO_SetDir(BRD_LED_3_CONNECTED_PORT, BRD_LED_3_CONNECTED_MASK, GPIO_DIRECTION_OUTPUT);
	GPIO_SetDir(BRD_LED_4_CONNECTED_PORT, BRD_LED_4_CONNECTED_MASK, GPIO_DIRECTION_OUTPUT);
}
/*********************************************************************
��������main
��	�ܣ���������
��  ������
��  �أ���
*********************************************************************/
int main(void)
{
	init ();
	Uart_Init();//���ڳ�ʼ��

   xTaskCreate((TaskFunction_t )start_task,  /* ������ں��� */
                        (const char*    )"AppTaskCreate",/* �������� */
                        (uint16_t       )START_STK_SIZE,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )1, /* ��������ȼ� */
                        (TaskHandle_t*  )&StartTask_Handler);/* ������ƿ�ָ�� */ 	

  vTaskStartScheduler();											


}


static void led_task(void* parameter)
{
	uint8_t value = 0;
	while (1)
	{
	//	GPIO_OutputValue(2, 1<<27, value);//������
		GPIO_OutputValue(BRD_LED_1_CONNECTED_PORT, BRD_LED_2_CONNECTED_MASK, value);
		GPIO_OutputValue(BRD_LED_1_CONNECTED_PORT, BRD_LED_3_CONNECTED_MASK, value);
		value = !value;
		vTaskDelay(500);
	}   
}

void start_task(void *pvParameters)
{
	  BaseType_t xReturn = pdPASS;
	
	  TCPIP_Init();
	
	 client_init();
	
	  printf("��������ʾ�����巢�����ݵ�������\n\n");
  
    printf("��������ģ�����£�\n\t ����<--����-->·��<--����-->������\n\n");
  
    printf("ʵ����ʹ��TCPЭ�鴫�����ݣ�������ΪTCP Server����������ΪTCP Client\n\n");
  
    printf("�����̵�IP��ַ����User/arch/sys_arch.h�ļ����޸�\n\n");
  
    printf("�����̲ο�<<LwIPӦ��ʵս����ָ��>>��16�� ʹ�� Socket �ӿڱ��\n\n");
	   
	
	  taskENTER_CRITICAL();           //�����ٽ���
	  xReturn = xTaskCreate((TaskFunction_t )led_task, /* ������ں��� */
                        (const char*    )"led_Task",/* �������� */
                        (uint16_t       )50,   /* ����ջ��С */
                        (void*          )NULL,	/* ������ں������� */
                        (UBaseType_t    )2,	    /* ��������ȼ� */
                        (TaskHandle_t*  )&LEDTask_Handler);/* ������ƿ�ָ�� */
  if(pdPASS == xReturn)
    printf("Create led_task sucess...\r\n");
	
		  xReturn = xTaskCreate((TaskFunction_t )led1_task, /* ������ں��� */
                        (const char*    )"led1_Task",/* �������� */
                        (uint16_t       )50,   /* ����ջ��С */
                        (void*          )NULL,	/* ������ں������� */
                        (UBaseType_t    )3,	    /* ��������ȼ� */
                        (TaskHandle_t*  )&LED1Task_Handler);/* ������ƿ�ָ�� */
  if(pdPASS == xReturn)
    printf("Create led1_task sucess...\r\n");
	
	                                                             
	
	vTaskDelete(StartTask_Handler); //ɾ��AppTaskCreate����
  
  taskEXIT_CRITICAL();            //�˳��ٽ���
}

static void led1_task(void* parameter)
{
	uint8_t value = 0;
	while (1)
	{
		GPIO_OutputValue(2, 1<<27, value);//������
		GPIO_OutputValue(BRD_LED_1_CONNECTED_PORT, BRD_LED_4_CONNECTED_MASK, value);
		value = !value;
		vTaskDelay(100);
	}   
}

void HardFault_Handler()
{
	char a;
	a = 0;
}




