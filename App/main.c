

//注意:在禁用JTAG接后后可这样恢复:
//1.设置KEIL为JTAG或SW下载模式
//2.给板上电,按下板上的复位按键不放,点击KEIL下的FLASH下载按钮,使JTAG或SW下载工具检测完成接口后立刻放开,
//此目的在于争取IC在上电的时候有控制权,使在未进入禁用JTAG或SW生效的时候取可控制IC进行烧录

#include "stm32f10x.h"




void Init_All_Periph(void)
{
	int i;
	
	RCC_Configuration();	
	NVIC_Configuration();

	ComInit(COM1,115200);
	usart1TxDMAInit();
}


int main(void)
{  
	Init_All_Periph();
	
	while(1);
	
}





#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
  while (1)
  {
  }
}
#endif





