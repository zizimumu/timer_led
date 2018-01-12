
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

void RTC_IRQHandler(void)
{							    
	if(RTC->CRL&0x0001)//秒钟中断
	{							
		//RTC_Get();//更新时间 
		DebugPrint("RTC timer:");
#if 0		
		PrintInt(gRTCTimer.w_year,0,1);
		PrintInt(gRTCTimer.w_month,0,1);
		PrintInt(gRTCTimer.w_date,0,1);
		PrintInt(gRTCTimer.hour,0,1);
		PrintInt(gRTCTimer.min,0,1);
		PrintInt(gRTCTimer.sec,0,0);
#endif
	}
	if(RTC->CRL&0x0002)//闹钟中断
	{
		RTC->CRL&=~(0x0002);//清闹钟中断
	} 				  								 
	RTC->CRL&=0X0FFA;         //清除溢出，秒钟中断标志
	while(!(RTC->CRL&(1<<5)));//等待RTC寄存器操作完成		   							 	   	 
}

void TIM3_IRQHandler(void)   //TIM3中断
{
//	int16_t tmp;

	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
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
