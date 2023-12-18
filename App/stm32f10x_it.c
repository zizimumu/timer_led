
#include "stm32f10x_it.h"
#include "app.h"

static uint32_t gTimerCount;


void NMI_Handler(void)
	{
	}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
	{
	}

uint32_t GetTimer(void)
{
	return gTimerCount;

}





extern RTC_IRQ_FUNC g_rtc_irqhandler;

void RTC_IRQHandler(void)
{							    
	if(RTC->CRL&0x0001)//�����ж�
	{							

	}
	if(RTC->CRL&0x0002)//�����ж�
	{
		RTC->CRL&=~(0x0002);//�������ж�
	} 				  

	if(g_rtc_irqhandler){
		g_rtc_irqhandler(NULL);
		g_rtc_irqhandler = NULL;
		
	}
	
	RTC->CRL&=0X0FFA;         //�������������жϱ�־
	while(!(RTC->CRL&(1<<5)));//�ȴ�RTC�Ĵ����������	
	
	//GPIO_ResetBits(LED3_GPIO,LED3_PIN);	   							 	   	 
}


void RTCAlarm_IRQHandler(void)
{
	if(g_rtc_irqhandler){
		g_rtc_irqhandler(NULL);
		g_rtc_irqhandler = NULL;
		
	}

	//GPIO_ResetBits(LED2_GPIO,LED2_PIN);


	EXTI_ClearITPendingBit(EXTI_Line17);
}

void TIM3_IRQHandler(void)   //TIM3�ж�
{
//	int16_t tmp;

	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		
		//DebugPrint("value is \r\n");
		//tmp = SPI_Read(0x31);
		//PrintInt(tmp,0,0);

		
		gTimerCount++;


/*
		tmp = ReadAxisX();
		PrintInt(tmp,0,1);
		
		tmp = ReadAxisY();
		PrintInt(tmp,0,1);
		
		tmp = ReadAxisZ();
		PrintInt(tmp,0,0);
*/
		__disable_irq();
		
		step_counter();
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );
		__enable_irq();


	
	}

}

void EXTI9_5_IRQHandler(void)
{
	/****  key test *************
	delay_ms(10); 
	if( (GPIOB->IDR & GPIO_Pin_7) == 0){
		if(Handflag == 1){
			GPIO_SetBits(GPIOC,GPIO_Pin_1);
			Handflag = 0;	
		}
		else {
			GPIO_ResetBits(GPIOC,GPIO_Pin_1);
			Handflag = 1;	
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line7);
	****  key test *************/	
}
void EXTI1_IRQHandler(void)
{
/*
	AFEXX_processData();	
	EXTI_ClearITPendingBit(EXTI_Line1);
*/
	__disable_irq();
	ADXL_IntProcess();
	EXTI_ClearITPendingBit(EXTI_Line1);
	__enable_irq();



}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
