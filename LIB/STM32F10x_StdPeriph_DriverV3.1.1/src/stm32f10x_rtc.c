/**
  ******************************************************************************
  * @file    stm32f10x_rtc.c
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    06/19/2009
  * @brief   This file provides all the RTC firmware functions.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_rtc.h"
#include <stdio.h>
/** @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */

/** @defgroup RTC 
  * @brief RTC driver modules
  * @{
  */

/** @defgroup RTC_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */

/** @defgroup RTC_Private_Defines
  * @{
  */

#define CRL_CNF_Set      ((uint16_t)0x0010)      /*!< Configuration Flag Enable Mask */
#define CRL_CNF_Reset    ((uint16_t)0xFFEF)      /*!< Configuration Flag Disable Mask */
#define RTC_LSB_Mask     ((uint32_t)0x0000FFFF)  /*!< RTC LSB Mask */
#define PRLH_MSB_Mask    ((uint32_t)0x000F0000)  /*!< RTC Prescaler MSB Mask */

/************************** self define start *************************/
u8 const _tableWeek[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //月修正数据表	  

const u8 _monTable[12]={31,28,31,30,31,30,31,31,30,31,30,31};

// tm gRTCTimer; // 时钟结构体 

/************************** self define end *************************/

/**
  * @brief  Enables or disables the specified RTC interrupts.
  * @param  RTC_IT: specifies the RTC interrupts sources to be enabled or disabled.
  *   This parameter can be any combination of the following values:
  *     @arg RTC_IT_OW: Overflow interrupt
  *     @arg RTC_IT_ALR: Alarm interrupt
  *     @arg RTC_IT_SEC: Second interrupt
  * @param  NewState: new state of the specified RTC interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void RTC_ITConfig(uint16_t RTC_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RTC_IT(RTC_IT));  
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    RTC->CRH |= RTC_IT;
  }
  else
  {
    RTC->CRH &= (uint16_t)~RTC_IT;
  }
}

/**
  * @brief  Enters the RTC configuration mode.
  * @param  None
  * @retval None
  */
void RTC_EnterConfigMode(void)
{
  /* Set the CNF flag to enter in the Configuration Mode */
  RTC->CRL |= CRL_CNF_Set;
}

/**
  * @brief  Exits from the RTC configuration mode.
  * @param  None
  * @retval None
  */
void RTC_ExitConfigMode(void)
{
  /* Reset the CNF flag to exit from the Configuration Mode */
  RTC->CRL &= CRL_CNF_Reset;
}

/**
  * @brief  Gets the RTC counter value.
  * @param  None
  * @retval RTC counter value.
  */
uint32_t RTC_GetCounter(void)
{
  uint16_t tmp = 0;
  tmp = RTC->CNTL;
  return (((uint32_t)RTC->CNTH << 16 ) | tmp) ;
}

/**
  * @brief  Sets the RTC counter value.
  * @param  CounterValue: RTC counter new value.
  * @retval None
  */
void RTC_SetCounter(uint32_t CounterValue)
{ 
  RTC_EnterConfigMode();
  /* Set RTC COUNTER MSB word */
  RTC->CNTH = CounterValue >> 16;
  /* Set RTC COUNTER LSB word */
  RTC->CNTL = (CounterValue & RTC_LSB_Mask);
  RTC_ExitConfigMode();
}

/**
  * @brief  Sets the RTC prescaler value.
  * @param  PrescalerValue: RTC prescaler new value.
  * @retval None
  */
void RTC_SetPrescaler(uint32_t PrescalerValue)
{
  /* Check the parameters */
  assert_param(IS_RTC_PRESCALER(PrescalerValue));
  
  RTC_EnterConfigMode();
  /* Set RTC PRESCALER MSB word */
  RTC->PRLH = (PrescalerValue & PRLH_MSB_Mask) >> 16;
  /* Set RTC PRESCALER LSB word */
  RTC->PRLL = (PrescalerValue & RTC_LSB_Mask);
  RTC_ExitConfigMode();
}

/**
  * @brief  Sets the RTC alarm value.
  * @param  AlarmValue: RTC alarm new value.
  * @retval None
  */
void RTC_SetAlarm(uint32_t AlarmValue)
{  
  RTC_EnterConfigMode();
  /* Set the ALARM MSB word */
  RTC->ALRH = AlarmValue >> 16;
  /* Set the ALARM LSB word */
  RTC->ALRL = (AlarmValue & RTC_LSB_Mask);
  RTC_ExitConfigMode();
}

/**
  * @brief  Gets the RTC divider value.
  * @param  None
  * @retval RTC Divider value.
  */
uint32_t RTC_GetDivider(void)
{
  uint32_t tmp = 0x00;
  tmp = ((uint32_t)RTC->DIVH & (uint32_t)0x000F) << 16;
  tmp |= RTC->DIVL;
  return tmp;
}

/**
  * @brief  Waits until last write operation on RTC registers has finished.
  * @note   This function must be called before any write to RTC registers.
  * @param  None
  * @retval None
  */
void RTC_WaitForLastTask(void)
{
  /* Loop until RTOFF flag is set */
  while ((RTC->CRL & RTC_FLAG_RTOFF) == (uint16_t)RESET)
  {
  }
}

/**
  * @brief  Waits until the RTC registers (RTC_CNT, RTC_ALR and RTC_PRL)
  *   are synchronized with RTC APB clock.
  * @note   This function must be called before any read operation after an APB reset
  *   or an APB clock stop.
  * @param  None
  * @retval None
  */
void RTC_WaitForSynchro(void)
{
  /* Clear RSF flag */
  RTC->CRL &= (uint16_t)~RTC_FLAG_RSF;
  /* Loop until RSF flag is set */
  while ((RTC->CRL & RTC_FLAG_RSF) == (uint16_t)RESET)
  {
  }
}

/**
  * @brief  Checks whether the specified RTC flag is set or not.
  * @param  RTC_FLAG: specifies the flag to check.
  *   This parameter can be one the following values:
  *     @arg RTC_FLAG_RTOFF: RTC Operation OFF flag
  *     @arg RTC_FLAG_RSF: Registers Synchronized flag
  *     @arg RTC_FLAG_OW: Overflow flag
  *     @arg RTC_FLAG_ALR: Alarm flag
  *     @arg RTC_FLAG_SEC: Second flag
  * @retval The new state of RTC_FLAG (SET or RESET).
  */
FlagStatus RTC_GetFlagStatus(uint16_t RTC_FLAG)
{
  FlagStatus bitstatus = RESET;
  
  /* Check the parameters */
  assert_param(IS_RTC_GET_FLAG(RTC_FLAG)); 
  
  if ((RTC->CRL & RTC_FLAG) != (uint16_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Clears the RTC抯 pending flags.
  * @param  RTC_FLAG: specifies the flag to clear.
  *   This parameter can be any combination of the following values:
  *     @arg RTC_FLAG_RSF: Registers Synchronized flag. This flag is cleared only after
  *                        an APB reset or an APB Clock stop.
  *     @arg RTC_FLAG_OW: Overflow flag
  *     @arg RTC_FLAG_ALR: Alarm flag
  *     @arg RTC_FLAG_SEC: Second flag
  * @retval None
  */
void RTC_ClearFlag(uint16_t RTC_FLAG)
{
  /* Check the parameters */
  assert_param(IS_RTC_CLEAR_FLAG(RTC_FLAG)); 
    
  /* Clear the coressponding RTC flag */
  RTC->CRL &= (uint16_t)~RTC_FLAG;
}

/**
  * @brief  Checks whether the specified RTC interrupt has occured or not.
  * @param  RTC_IT: specifies the RTC interrupts sources to check.
  *   This parameter can be one of the following values:
  *     @arg RTC_IT_OW: Overflow interrupt
  *     @arg RTC_IT_ALR: Alarm interrupt
  *     @arg RTC_IT_SEC: Second interrupt
  * @retval The new state of the RTC_IT (SET or RESET).
  */
ITStatus RTC_GetITStatus(uint16_t RTC_IT)
{
  ITStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_RTC_GET_IT(RTC_IT)); 
  
  bitstatus = (ITStatus)(RTC->CRL & RTC_IT);
  if (((RTC->CRH & RTC_IT) != (uint16_t)RESET) && (bitstatus != (uint16_t)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Clears the RTC抯 interrupt pending bits.
  * @param  RTC_IT: specifies the interrupt pending bit to clear.
  *   This parameter can be any combination of the following values:
  *     @arg RTC_IT_OW: Overflow interrupt
  *     @arg RTC_IT_ALR: Alarm interrupt
  *     @arg RTC_IT_SEC: Second interrupt
  * @retval None
  */
void RTC_ClearITPendingBit(uint16_t RTC_IT)
{
  /* Check the parameters */
  assert_param(IS_RTC_IT(RTC_IT));  
  
  /* Clear the coressponding RTC pending bit */
  RTC->CRL &= (uint16_t)~RTC_IT;
}


/*******************************self define start ************************************/   
	   
u8 Is_Leap_Year(u16 year)
{			  
	if(year%4==0) //必须能被4整除
		{ 
		if(year%100==0) 
			{ 
			if(year%400==0)return 1;//如果以00结尾,还要能被400整除 	   
			else return 0;   
			}else return 1;   
		}else return 0;	
}	

//获得现在是星期几
//功能描述:输入公历日期得到星期(只允许1901-2099年)
//输入参数：公历年月日 
//返回值：星期号																						 
u8 RTC_Get_Week(u16 year,u8 month,u8 day)
{	
	u16 temp2;
	u8 yearH,yearL;
	
	yearH=year/100;	yearL=year%100; 
	// 如果为21世纪,年份数加100  
	if (yearH>19)yearL+=100;
	// 所过闰年数只算1900年之后的  
	temp2=yearL+yearL/4;
	temp2=temp2%7; 
	temp2=temp2+day+_tableWeek[month-1];
	if (yearL%4==0&&month<3)temp2--;
	return(temp2%7);
} 

void setRTC_NVIC(void )
{
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	
	NVIC_InitStructure.NVIC_IRQChannel =RTC_IRQn;			
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = RTC_NVIC_SUBPRI;					
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								
		
	NVIC_Init(&NVIC_InitStructure); 	
	
#if 1
	  NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
  NVIC_Init(&NVIC_InitStructure); 
	
	EXTI_ClearITPendingBit(EXTI_Line17); 
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;	//设置按键所有的外部线路
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;			//设外外部中断模式:EXTI线路为中断请求
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //外部中断触发沿选择:设置输入线路下降沿为中断请求
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
#endif	
}

u8 RTC_UserInit(void)
{
	
	uint32_t temp=0;
	
	//check if it is the first time init
	if (BKP_ReadBackupRegister(BKP_DR3) != 0x55aa){	 			
	//if (1){	
		printf("rtc need reconfig ...\r\n");
		/* Enable PWR and BKP clocks */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);	
		
		/* Allow access to BKP Domain ,PWR_CR register*/
		PWR_BackupAccessCmd(ENABLE);	
		

#if 1
		/* Reset Backup Domain ,RCC_BDCR register*/
		//BKP_DeInit();	
		
		/* Enable LSE */
		RCC_LSEConfig(RCC_LSE_ON);	
		/* Wait till LSE is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET){	
			temp++;
		}
		if(temp>=72000000)	return 1;    
		/* Select LSE as RTC Clock Source */
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);		  
		/* Enable RTC Clock */
		RCC_RTCCLKCmd(ENABLE);	
		/* Wait for RTC registers synchronization */
		RTC_WaitForSynchro();		
		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();	
		/* Enable the RTC Second */
		RTC_ITConfig(RTC_IT_ALR, ENABLE);	
		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();	
		/* Set RTC prescaler: set RTC period to 1sec */
		/* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
		RTC_SetPrescaler(32767);
		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();	
		//RTC_Set(2013,12,17,14,4,55); 
		BKP_WriteBackupRegister(BKP_DR3, 0x55aa);	
		
#endif		

	
	
	}
	else
	{
		/* Enable PWR and BKP clocks */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);	
		
		/* Allow access to BKP Domain ,PWR_CR register*/
		PWR_BackupAccessCmd(ENABLE);	
		
		//printf("\rNo need to configure RTC....");
		/* Wait for RTC registers synchronization */
		//RTC_WaitForSynchro();	//等待最近一次对RTC寄存器的写操作完成
		
		/* Enable the RTC Second */
		RTC_ITConfig(RTC_IT_ALR, ENABLE);	
		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();	//等待最近一次对RTC寄存器的写操作完成
	}    				     
	
	/* Clear reset flags */
	RCC_ClearFlag();	//清除RCC的复位标志位
		
	setRTC_NVIC();
	return 0; //ok
}



 			   
//设置时钟
//把输入的时钟转换为秒钟
//以1970年1月1日为基准
//1970~2099年为合法年份
//返回值:0,成功;其他:错误代码.
//月份数据表											 
u8 RTC_Set(u16 syear,u8 smon,u8 sday,u8 hour,u8 min,u8 sec)
{
	u16 t;
	u32 seccount=0;
	if(syear<1970||syear>2099)return 1;	   
	for(t=1970;t<syear;t++){
		if(Is_Leap_Year(t))seccount+=31622400;//闰年的秒钟数
		else seccount+=31536000;			  //平年的秒钟数
	}
	smon-=1;
	for(t=0;t<smon;t++){
		seccount+=(u32)_monTable[t]*86400;//月份秒钟数相加
		if(Is_Leap_Year(syear)&&t==1)seccount+=86400;//闰年2月份增加一天的秒钟数	   
	}
	seccount+=(u32)(sday-1)*86400;
	seccount+=(u32)hour*3600;
	seccount+=(u32)min*60;	 
	seccount+=sec;
											    
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();	
	/* Change the current time */
	RTC_SetCounter(seccount);	
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();	
	return 0;	    
}

//得到当前的时间
//返回值:0,成功;其他:错误代码.
u8 RTC_Get(tm *timer)
{
	static u16 daycnt=0;
	u32 timecount=0; 
	u32 temp=0;
	u16 temp1=0;	  
	   
	timecount=RTC->CNTH;//得到计数器中的值(秒钟数)
	timecount<<=16;
	timecount+=RTC->CNTL;			 

	temp=timecount/86400;   //得到天数(秒钟数对应的)
	// if(daycnt!=temp)//超过一天了
	if(1)
	{	  
		daycnt=temp;
		temp1=1970;	//从1970年开始
		while(temp>=365)
		{				 
			if(Is_Leap_Year(temp1))//是闰年
			{
				if(temp>=366)temp-=366;//闰年的秒钟数
				else {temp1++;break;}  
			}
			else temp-=365;	  //平年 
			temp1++;  
		}   
		timer->w_year=temp1;//得到年份
		temp1=0;
		while(temp>=28)//超过了一个月
		{
			if(Is_Leap_Year(timer->w_year)&&temp1==1)//当年是不是闰年/2月份
			{
				if(temp>=29)temp-=29;//闰年的秒钟数
				else break; 
			}
			else 
			{
				if(temp>=_monTable[temp1])temp-=_monTable[temp1];//平年
				else break;
			}
			temp1++;  
		}
		timer->w_month=temp1+1;//得到月份
		timer->w_date=temp+1;  //得到日期 
	}
	temp=timecount%86400;     //得到秒钟数   	   
	timer->hour=temp/3600;     //小时
	timer->min=(temp%3600)/60; //分钟	
	timer->sec=(temp%3600)%60; //秒钟
	timer->week=RTC_Get_Week(timer->w_year,timer->w_month,timer->w_date);//获取星期   
	return 0;
}

unsigned int get_sec_between(tm *current,tm *dest)
{
	unsigned int sec;
	int hour,min;
	
	if(dest->hour > current->hour ||  (dest->hour == current->hour&&dest->min >= current->min)){
		hour = dest->hour - current->hour;
		min = (int)dest->min - (int)current->min;
		min = hour*60+min;
		
		if(min <=0)
			min = 24*60;
	}
	else{
		min = 24*60-(current->hour*60+current->min) + dest->hour*60+dest->min;
	}
	return (min*60);
	
	
}
volatile RTC_IRQ_FUNC g_rtc_irqhandler = NULL;

void register_rtc_irq_hander(RTC_IRQ_FUNC handler)
{
	g_rtc_irqhandler = handler;
}


void RTC_SetAlarm_user(tm *dest,RTC_IRQ_FUNC handler)
{
		tm current;
		unsigned int sec;
	
		g_rtc_irqhandler = handler;
		RTC_Get(&current);
	
		sec = get_sec_between(&current,dest) + current.sec;
		printf("alarm will wake up after %d s,%02d:%02d\r\n",sec,dest->hour,dest->min);
	
	
		sec = RTC_GetCounter()+sec;
		
		RTC_SetAlarm(sec);
		RTC_WaitForLastTask();
}







/*******************************self define end ************************************/  

