#include "usart.h"	  


//重定向 printf 函数，使其输出到 USART1 串口
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数  
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef' d in stdio.h. */ 
FILE __stdout;       
//定义 sys exit()以避免使用半主机模式
int _sys_exit(int x) 
{ 
	x = x; 
} 


//重定义fputc函数
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送，直到发送完毕
	USART1->DR = (u8) ch;   //将字符 ch 写入数据寄存器，从而通过 USART1 发送出去
	return ch;  //数据类型为 无符号8位整数
}
#endif 




//end
////////////////////////////////////////////////////////////////
/* USART1_TX   GPIOA.9 */
/* USART1_RX	  GPIOA.10??? */
void Uart1_Init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;
	
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分
    mantissa<<=4;
	mantissa+=fraction; 
	
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟
	RCC->APB2ENR|=1<<14;  //使能串口时钟
	
	GPIOA->CRH&=0XFFFFF00F;//IO状态设置
	GPIOA->CRH|=0X000008B0;//IO状态设置
		  
	RCC->APB2RSTR|=1<<14;   //复位串口1
	RCC->APB2RSTR&=~(1<<14);//停止复位
	
	//波特率设置
 	USART1->BRR=mantissa; // 波特率设置USART1->CR11=0X200C://1位停止，无校验
	USART1->CR1|=0X200C;  //1位停止，无校验位，
	
/*******************************************************************
************************************************************************/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 使能接收中断
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

//    NVIC_EnableIRQ(USART1_IRQn); // 使能USART1中断
/*******************************************************************
************************************************************************/
}


void Uart3_Init(u32 pclk1,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;
	
	temp=(float)(pclk1*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分
    mantissa<<=4;
	mantissa+=fraction; 
	
	RCC->APB2ENR|=1<<3;   //使能GPIOB口时钟
	RCC->APB1ENR |= 1 << 18; // 使能 USART3 时钟
	
	GPIOB->CRH &= 0xFFFF00FF; // 清除 GPIOB10 和 GPIOB11 的配置
	GPIOB->CRH |= 0x00008B00; // 配置 GPIOB10 为复用推挽输出，GPIOB11 为浮空输入
		  
	RCC->APB1RSTR |= 1 << 18;  // 复位 USART3
	RCC->APB1RSTR &= ~(1 << 18); // 停止复位
	
	//波特率设置
 	USART3->BRR=mantissa; // 波特率设置USART1->CR11=0X200C://1位停止，无校验
	USART3->CR1|=0X200C;  //1位停止，无校验位，
}



void uart_init( u32 bound)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // 1. 使能 USART1 和 GPIOA 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    // 2. 配置 GPIOA 引脚 (PA9: TX, PA10: RX)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	
	////配置GPIOB引脚 PB10 PB11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 浮空输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);



    // 3. USART1 配置
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
	
	
	////USART3配置
	USART_Init(USART3, &USART_InitStructure);

    // 4. 使能 USART1
    USART_Cmd(USART1, ENABLE);
	USART_Cmd(USART3, ENABLE);
}

/******
******/
//
// @简介：使用串口发送多个字节的数据
// 
// @参数 USARTx：串口名称，如USART1, USART2, USART3 ...
// @参数 pData : 要发送的数据（数组）
// @参数 Size  ：要发送数据的数量，单位是字节
//
__weak void My_USART_SendBytes(USART_TypeDef *USARTx, const uint8_t *pData, uint16_t Size)
{
	if(Size == 0) return;
	
	for(uint16_t i=0; i < Size; i++)
	{
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
		
		USART_SendData(USARTx, pData[i]);
	}
	
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
}



//
// @简介：通过串口发送字符串
// 
// @参数 USARTx：串口名称，如USART1, USART2, USART3 ...
// @参数 Str   ：要发送的字符串
//
void My_USART_SendString(USART_TypeDef *USARTx, const char *Str)
{
	My_USART_SendBytes(USARTx, (const uint8_t *)Str, strlen(Str));
}




// @简介：通过串口读取一字节的数据
// 
// @参数 USARTx  ：串口名称，如USART1, USART2, USART3 ...
// 
// @返回值：读取到的字节
//
uint8_t My_USART_ReceiveByte(USART_TypeDef *USARTx)
{
	while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET);
	
	return USART_ReceiveData(USARTx);
}



