
#include "LPC177x_8x.h"
#include "lpc177x_8x_uart.h"
#include "lpc177x_8x_pinsel.h"
#include "lpc177x_8x_gpio.h"
#include "stdio.h"


/*************************************************************************************
��������:��ȡ�ַ�����
����:��
����ֵ:��ȡһ���ַ�
*************************************************************************************/
int UART2_GetChar (void) 
{

  	return (LPC_UART2->RBR);
}

/*************************************************************************************
��������:����һ���ֽں���
����:һ���ֽڵ�����
����ֵ:
*************************************************************************************/
int UART2_SendByte (int ucData)
{
	while (!(LPC_UART2->LSR & 0x20));           /*��bit5 0 Ϊ�����ݣ�1 Ϊ�������*/
    return (LPC_UART2->THR =ucData);
}
/*************************************************************************************
��������:�����ַ�������
����:�ַ�������
����ֵ:��
*************************************************************************************/
void UART2_SendString (unsigned char *s) 
{
  	while (*s != 0) 
	{
   		UART2_SendByte(*s++);
	}
}



/*************************************************************************************
��������:��ȡ�ַ�����
����:��
����ֵ:��ȡһ���ַ�
*************************************************************************************/
int UART3_GetChar (void) 
{

  	return (LPC_UART3->RBR);
}

/*************************************************************************************
��������:����һ���ֽں���
����:һ���ֽڵ�����
����ֵ:
*************************************************************************************/
int UART3_SendByte (int ucData)
{
	while (!(LPC_UART3->LSR & 0x20));           /*��bit5 0 Ϊ�����ݣ�1 Ϊ�������*/
    return (LPC_UART3->THR =ucData);
}

/*************************************************************************************
��������:�����ַ�������
����:�ַ�������
����ֵ:��
*************************************************************************************/
void UART3_SendString (unsigned char *s) 
{
  	while (*s != 0) 
	{
   		UART3_SendByte(*s++);
	}
}
/*********************************************************************************
���ڳ�ʼ������

**********************************************************************************/
void Uart_Init(void)
{
    
    UART_CFG_Type UARTConfigStruct;
   //����2�Ĺܽų�ʼ��
    PINSEL_ConfigPin(0,10,1);
    PINSEL_ConfigPin(0,11,1);

	  UARTConfigStruct.Baud_rate=115200;
	  UARTConfigStruct.Databits=UART_DATABIT_8;
   	UARTConfigStruct.Parity=UART_PARITY_NONE;
	  UARTConfigStruct.Stopbits=UART_STOPBIT_1;
    UART_ConfigStructInit(&UARTConfigStruct);//��ʼ���ṹ��
    UART_Init(UART_2, &UARTConfigStruct);//��ʼ������2
	
	 //����3�Ĺܽų�ʼ��
	  PINSEL_ConfigPin(0,2,2);
    PINSEL_ConfigPin(0,3,2);
	  UARTConfigStruct.Baud_rate=115200;
	  UARTConfigStruct.Databits=UART_DATABIT_8;
   	UARTConfigStruct.Parity=UART_PARITY_NONE;
	  UARTConfigStruct.Stopbits=UART_STOPBIT_1;
    UART_ConfigStructInit(&UARTConfigStruct);//��ʼ���ṹ��
    UART_Init(UART_3, &UARTConfigStruct);//��ʼ������3
		

    UART_TxCmd(UART_2, ENABLE);//ʹ�ܷ���
    UART_TxCmd(UART_3, ENABLE);//ʹ�ܷ���
		 
    UART_IntConfig(UART_2, UART_INTCFG_RBR, ENABLE);//ʹ�ܷ����ж�
    UART_IntConfig(UART_2, UART_INTCFG_RLS, ENABLE);//ʹ���ж��ߵ�״̬
    NVIC_SetPriority(UART2_IRQn, 31);//�����ж����ȼ�
    NVIC_EnableIRQ(UART2_IRQn);//ʹ�ܴ���2�ж�


}



//int main(void)
//{
//   
//	Uart_Init();//���ڳ�ʼ��
//	printf("*******************fuck you***********************\r\n");
//	printf("*******************usbת����**********************\r\n");
//	while(1){
//		
//		;
//		
//		
//	  }

//}



/*********************************************************************
�жϷ�����


**********************************************************************/
uint16_t flag=0; 
char res;
void UART2_IRQHandler(){
	

    flag  = LPC_UART2->IIR;                          //��ȡ(���)�ж��ж�״̬
    flag &= 0x0F;		  
    if((flag == 0X04)&&(LPC_UART2->LSR & 0X01))    //����ǽ����жϣ����������� û�����ݾͻ��Զ�����üĴ���
		{  
     res=UART2_GetChar();                       //��ȡ���� 
	   UART2_SendByte(res);
  
		
  }
		
}

int fputc(int ch, FILE *f)  
{     
       UART_SendByte(UART_2, ch);    //����뻻������UART��������޸�UART_0         

      while (UART_CheckBusy(UART_2)); //�����Ƿ����
       return (ch);  
}  
	


