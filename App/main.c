

//ע��:�ڽ���JTAG�Ӻ��������ָ�:
//1.����KEILΪJTAG��SW����ģʽ
//2.�����ϵ�,���°��ϵĸ�λ��������,���KEIL�µ�FLASH���ذ�ť,ʹJTAG��SW���ع��߼����ɽӿں����̷ſ�,
//��Ŀ��������ȡIC���ϵ��ʱ���п���Ȩ,ʹ��δ�������JTAG��SW��Ч��ʱ��ȡ�ɿ���IC������¼

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





