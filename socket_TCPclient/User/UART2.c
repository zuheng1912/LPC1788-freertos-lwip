
#include "LPC177x_8x.h"
#include "lpc177x_8x_uart.h"
#include "lpc177x_8x_pinsel.h"
#include "lpc177x_8x_gpio.h"
#include "stdio.h"


/*************************************************************************************
函数功能:获取字符函数
参数:无
返回值:获取一个字符
*************************************************************************************/
int UART2_GetChar (void) 
{

  	return (LPC_UART2->RBR);
}

/*************************************************************************************
函数功能:发送一个字节函数
参数:一个字节的数据
返回值:
*************************************************************************************/
int UART2_SendByte (int ucData)
{
	while (!(LPC_UART2->LSR & 0x20));           /*读bit5 0 为有数据，1 为发送完成*/
    return (LPC_UART2->THR =ucData);
}
/*************************************************************************************
函数功能:发送字符串函数
参数:字符串数据
返回值:无
*************************************************************************************/
void UART2_SendString (unsigned char *s) 
{
  	while (*s != 0) 
	{
   		UART2_SendByte(*s++);
	}
}



/*************************************************************************************
函数功能:获取字符函数
参数:无
返回值:获取一个字符
*************************************************************************************/
int UART3_GetChar (void) 
{

  	return (LPC_UART3->RBR);
}

/*************************************************************************************
函数功能:发送一个字节函数
参数:一个字节的数据
返回值:
*************************************************************************************/
int UART3_SendByte (int ucData)
{
	while (!(LPC_UART3->LSR & 0x20));           /*读bit5 0 为有数据，1 为发送完成*/
    return (LPC_UART3->THR =ucData);
}

/*************************************************************************************
函数功能:发送字符串函数
参数:字符串数据
返回值:无
*************************************************************************************/
void UART3_SendString (unsigned char *s) 
{
  	while (*s != 0) 
	{
   		UART3_SendByte(*s++);
	}
}
/*********************************************************************************
串口初始化函数

**********************************************************************************/
void Uart_Init(void)
{
    
    UART_CFG_Type UARTConfigStruct;
   //串口2的管脚初始化
    PINSEL_ConfigPin(0,10,1);
    PINSEL_ConfigPin(0,11,1);

	  UARTConfigStruct.Baud_rate=115200;
	  UARTConfigStruct.Databits=UART_DATABIT_8;
   	UARTConfigStruct.Parity=UART_PARITY_NONE;
	  UARTConfigStruct.Stopbits=UART_STOPBIT_1;
    UART_ConfigStructInit(&UARTConfigStruct);//初始化结构体
    UART_Init(UART_2, &UARTConfigStruct);//初始化串口2
	
	 //串口3的管脚初始化
	  PINSEL_ConfigPin(0,2,2);
    PINSEL_ConfigPin(0,3,2);
	  UARTConfigStruct.Baud_rate=115200;
	  UARTConfigStruct.Databits=UART_DATABIT_8;
   	UARTConfigStruct.Parity=UART_PARITY_NONE;
	  UARTConfigStruct.Stopbits=UART_STOPBIT_1;
    UART_ConfigStructInit(&UARTConfigStruct);//初始化结构体
    UART_Init(UART_3, &UARTConfigStruct);//初始化串口3
		

    UART_TxCmd(UART_2, ENABLE);//使能发送
    UART_TxCmd(UART_3, ENABLE);//使能发送
		 
    UART_IntConfig(UART_2, UART_INTCFG_RBR, ENABLE);//使能发送中断
    UART_IntConfig(UART_2, UART_INTCFG_RLS, ENABLE);//使能中断线的状态
    NVIC_SetPriority(UART2_IRQn, 31);//设置中断优先级
    NVIC_EnableIRQ(UART2_IRQn);//使能串口2中断


}



//int main(void)
//{
//   
//	Uart_Init();//串口初始化
//	printf("*******************fuck you***********************\r\n");
//	printf("*******************usb转串口**********************\r\n");
//	while(1){
//		
//		;
//		
//		
//	  }

//}



/*********************************************************************
中断服务函数


**********************************************************************/
uint16_t flag=0; 
char res;
void UART2_IRQHandler(){
	

    flag  = LPC_UART2->IIR;                          //读取(清除)中断中断状态
    flag &= 0x0F;		  
    if((flag == 0X04)&&(LPC_UART2->LSR & 0X01))    //如果是接收中断，并且有数据 没有数据就会自动清理该寄存器
		{  
     res=UART2_GetChar();                       //读取数据 
	   UART2_SendByte(res);
  
		
  }
		
}

int fputc(int ch, FILE *f)  
{     
       UART_SendByte(UART_2, ch);    //如果想换成其他UART输出，可修改UART_0         

      while (UART_CheckBusy(UART_2)); //发送是否完成
       return (ch);  
}  
	


