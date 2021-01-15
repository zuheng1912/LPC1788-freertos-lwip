/**********************************************************************
文件名；GPIO_Buzzer.c
功  能；蜂鸣器发出声音和3个LED发光二极管闪烁
目  的；让初学者能快速的了解GPIO的用法和系统滴答中断（SysTick）的用法。
作  者；OPENMCU
日  期；2014年9月26日
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
 
#define START_TASK_PRIO 1 //任务优先级
#define START_STK_SIZE 128 //任务堆栈大小
TaskHandle_t StartTask_Handler; //任务句柄
void start_task(void *pvParameters); //任务函数

#define LED_TASK_PRIO 2 //任务优先级
#define LED_STK_SIZE 50 //任务堆栈大小
TaskHandle_t LEDTask_Handler; //任务句柄
void led_task(void *p_arg); //任务函数

TaskHandle_t LED1Task_Handler; //任务句柄
void led1_task(void *p_arg); //任务函数




volatile unsigned long SysTickCnt;
extern void Uart_Init(void);
extern int fputc(int ch, FILE *f);

extern void TCPIP_Init(void);  //用于初始化lwip
 
void SysTick_Handler (void);
void Delay (unsigned long tick);
void init (void);


/*********************************************************************
函数名；	SysTick_Handler
功	能；	系统滴答中断函数 1ms 进一次中断。
参	数；	无
返	回； 	无
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
函数名；	Delay
功	能；	延时函数
参  数；	tick tick的值就是延时多少个滴答一个滴答为 1ms.
返  回；	无
***********************************************************************/
void Delay (unsigned long tick)
{
	unsigned long systickcnt;

	systickcnt = SysTickCnt;
	while ((SysTickCnt - systickcnt) < tick);
}
/**********************************************************************
函数名；init
功  能；初始化系统时钟滴答，GPIO的方向。
参  数；无
返  回；无
**********************************************************************/
void init (void)
{
	uint32_t cclk = CLKPWR_GetCLK(CLKPWR_CLKTYPE_CPU);//获得频率120000000Hz
	/*设置系统时钟滴答中断 1ms*/
	SysTick_Config(cclk/1000);
	/*设置GPIO的方向为输出*/
	GPIO_SetDir(2, 1<<27, 1);//蜂鸣器管脚设置为输出
	GPIO_SetDir(BRD_LED_2_CONNECTED_PORT, BRD_LED_2_CONNECTED_MASK, GPIO_DIRECTION_OUTPUT);
	GPIO_SetDir(BRD_LED_3_CONNECTED_PORT, BRD_LED_3_CONNECTED_MASK, GPIO_DIRECTION_OUTPUT);
	GPIO_SetDir(BRD_LED_4_CONNECTED_PORT, BRD_LED_4_CONNECTED_MASK, GPIO_DIRECTION_OUTPUT);
}
/*********************************************************************
函数名；main
功	能；主处理函数
参  数；无
返  回；无
*********************************************************************/
int main(void)
{
	init ();
	Uart_Init();//串口初始化

   xTaskCreate((TaskFunction_t )start_task,  /* 任务入口函数 */
                        (const char*    )"AppTaskCreate",/* 任务名字 */
                        (uint16_t       )START_STK_SIZE,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )1, /* 任务的优先级 */
                        (TaskHandle_t*  )&StartTask_Handler);/* 任务控制块指针 */ 	

  vTaskStartScheduler();											


}


static void led_task(void* parameter)
{
	uint8_t value = 0;
	while (1)
	{
	//	GPIO_OutputValue(2, 1<<27, value);//蜂鸣器
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
	
	  printf("本例程演示开发板发送数据到服务器\n\n");
  
    printf("网络连接模型如下：\n\t 电脑<--网线-->路由<--网线-->开发板\n\n");
  
    printf("实验中使用TCP协议传输数据，电脑作为TCP Server，开发板作为TCP Client\n\n");
  
    printf("本例程的IP地址均在User/arch/sys_arch.h文件中修改\n\n");
  
    printf("本例程参考<<LwIP应用实战开发指南>>第16章 使用 Socket 接口编程\n\n");
	   
	
	  taskENTER_CRITICAL();           //进入临界区
	  xReturn = xTaskCreate((TaskFunction_t )led_task, /* 任务入口函数 */
                        (const char*    )"led_Task",/* 任务名字 */
                        (uint16_t       )50,   /* 任务栈大小 */
                        (void*          )NULL,	/* 任务入口函数参数 */
                        (UBaseType_t    )2,	    /* 任务的优先级 */
                        (TaskHandle_t*  )&LEDTask_Handler);/* 任务控制块指针 */
  if(pdPASS == xReturn)
    printf("Create led_task sucess...\r\n");
	
		  xReturn = xTaskCreate((TaskFunction_t )led1_task, /* 任务入口函数 */
                        (const char*    )"led1_Task",/* 任务名字 */
                        (uint16_t       )50,   /* 任务栈大小 */
                        (void*          )NULL,	/* 任务入口函数参数 */
                        (UBaseType_t    )3,	    /* 任务的优先级 */
                        (TaskHandle_t*  )&LED1Task_Handler);/* 任务控制块指针 */
  if(pdPASS == xReturn)
    printf("Create led1_task sucess...\r\n");
	
	                                                             
	
	vTaskDelete(StartTask_Handler); //删除AppTaskCreate任务
  
  taskEXIT_CRITICAL();            //退出临界区
}

static void led1_task(void* parameter)
{
	uint8_t value = 0;
	while (1)
	{
		GPIO_OutputValue(2, 1<<27, value);//蜂鸣器
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




