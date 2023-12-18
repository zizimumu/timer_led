#include "stm32f10x.h"

#include "app.h"
#include <stdio.h>
#include "stm32f10x_usart.h"


#define UART_INPUT_WAIT 120000 // 120s
// #define USE_STOP_MODE

#ifndef USE_STOP_MODE
// #define TIMER_COUNT 36000  //1Khz
#define TIMER_COUNT 2000  // 18K

#else
#define TIMER_COUNT 16000  //500hz
#endif

#define MAX_PWM_COUNT  TIMER_COUNT

void PWM_Configuration(void)
	{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟使能
		
			/*GPIOB Configuration: TIM3 channel2*/	  //设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		
		
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	//不分频。PWM频率=72000000/900=80Khz
	/* ---------------------------------------------------------------
	TIM3CLK 即PCLK1=36MHz
	TIM3 Configuration: generate 1 PWM signals :
    TIM3CLK = 36 MHz, Prescaler = 0x0, TIM3 counter clock = 36 MHz
    TIM3 ARR Register = 900 => TIM3 Frequency = TIM3 counter clock/(ARR + 1)
    TIM3 Frequency = 36 KHz.
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 
	TIM3CLK = 36 MHz, Prescaler = 0, TIM3 counter clock = 36MHz
	--------------------------------------------------------------- */
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = TIMER_COUNT; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =0; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	/* Output Compare Active Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIMx在CCR2上的预装载寄存器
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设


	TIM_SetCompare2(TIM3,0);
	}

void LedInit(void )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(LED1_CLOCK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = LED1_PIN|LED2_PIN|LED3_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED1_GPIO, &GPIO_InitStructure);

	GPIO_SetBits(LED1_GPIO,LED1_PIN);
	GPIO_SetBits(LED2_GPIO,LED2_PIN);
	GPIO_SetBits(LED3_GPIO,LED3_PIN);
}


void config_pwm_gpio_pin(void )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA,GPIO_Pin_7);
}


void config_pwm_pin(void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟使能
		
			/*GPIOB Configuration: TIM3 channel2*/	  //设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}



void pwm_gpio_trigle(void)
{
	static unsigned int st = 0;
	if(st & 0x1)
		GPIO_SetBits(GPIOA,GPIO_Pin_7);
	else
		GPIO_ResetBits(GPIOA,GPIO_Pin_7);
	
	st++;
}




void led_trigle(uint16_t led )
{
	static unsigned int on = 0;
	if(on & 0x1)
		GPIO_SetBits(LED1_GPIO,led);
	else
		GPIO_ResetBits(LED1_GPIO,led);
	
	on++;
}

void Init_All_Periph(void)
{
	RCC_ClocksTypeDef RCC_ClocksStatus;
	
	//use clock default,8Mhz
#ifndef USE_STOP_MODE
	RCC_Configuration();
#endif	
	NVIC_Configuration();

	ComInit(COM1,9600);
	
	printf("com init done\r\n");
	//usart1TxDMAInit();
	delay_init( );
	
	//TIM3_UserConfiguration();

	LedInit();
	
	printf("rtc init ...\r\n");
	RTC_UserInit();
	
	printf("pwm init ...\r\n");
	PWM_Configuration();

	
	RCC_GetClocksFreq(&RCC_ClocksStatus);
	printf("system clock : sysclk %d ,hclk %d ,APB11 %d ,APB2 %d \r\n", \
					RCC_ClocksStatus.SYSCLK_Frequency,RCC_ClocksStatus.HCLK_Frequency, \
					RCC_ClocksStatus.PCLK1_Frequency,RCC_ClocksStatus.PCLK2_Frequency);

}



extern void main_loop(void );
tm timer;

int g_user_alarm_hour = 0;
int g_user_alarm_min = 0;
int g_user_pwm_min = 0;

#define USR_DEFINE_HOUR g_user_alarm_hour
#define USR_DEFINE_MIN g_user_alarm_min
#define USR_DEFINE_PWM_COUNT g_user_pwm_min  //the min from 0 to Max
#define USR_DEFINE_PWM_HOUR 1			//the hour from on to off


void pwm_start_out(void)
{
	unsigned int sec,step,step2,counter;
	int stat = 0;
	uint16_t pwmval,period;
	
	sec = USR_DEFINE_PWM_COUNT*60;
	

	
	
	// first step, just set the step to 1 to slow down the adjustment since eys is sensitive about the dim light
	// the duty width should >= MOS upslope + downslope, SL3400 is 2us@50Khz
	pwmval = 5; // TIMER_COUNT * 1/ 100;
	counter = 0;
	step = 1;

	printf("first step\r\n");
	while(1){
			TIM_SetCompare2(TIM3,pwmval);
			pwmval += step;

			delay_s(30);
			counter += 30;

			// printf("ccounter %d, pwmval %d\r\n",counter,pwmval);
			if(counter >= (sec/2) || pwmval >= TIMER_COUNT  )
				break;
			
			//led_trigle(LED1_PIN);
	}
	
	// second step, fast the adjustment
	period = 30;
	if(pwmval < TIMER_COUNT)
		step =  (TIMER_COUNT - pwmval) / (sec/2/period);
	else
		return;
	
	printf("second step\r\n");
	step++;
	
	while(1){
			TIM_SetCompare2(TIM3,pwmval);
			pwmval += step;

			delay_s(period);


			if(pwmval >= TIMER_COUNT )
				break;
			
			//led_trigle(LED1_PIN);
	}	
}


volatile int pwm_start = 0,pwm_stop = 0;

void pwm_start_irqhandler(void *arg)
{
	pwm_start = 1;
	//__DSB();
	//GPIO_ResetBits(LED2_GPIO,LED2_PIN);
}

void pwm_stop_irqhandler(void *arg)
{
	pwm_stop = 1;
	//__DSB();
}
	
	

int main(void)
{  
	
	unsigned int sec,cc = 0;
	uint16_t back;
	tm timer_dest;
	Init_All_Periph();

	g_user_alarm_hour = 6;
	g_user_alarm_min = 20;
	g_user_pwm_min = 20;

	
	
	
	//pwm_start_out();

	
	
	
	
	
	
	
	
	
	
	printf("system startimg...\r\n");
	back = BKP_ReadBackupRegister(BKP_DR1);
	if(back != 0){
		g_user_alarm_hour = ((back &0xff00)>>8)%24;
		g_user_alarm_min = (back &0xff)%60;
	}	
	back = BKP_ReadBackupRegister(BKP_DR2);
	if(back != 0){
		g_user_pwm_min = (back & 0xff)%60;
	}
	
	back = BKP_ReadBackupRegister(BKP_DR4);
	
	
	
	
	RTC_Get(&timer);

	printf("last flag value is 0x%x\r\n",back);
	printf("current timer is :\r\n");
	printf("	%d	%02d %02d,%02d:%02d:%02d\r\n",timer.w_year,timer.w_month,timer.w_date,timer.hour,timer.min,timer.sec);
	printf("user set timer is %d:%d ,pwm mini %d\r\n",USR_DEFINE_HOUR,USR_DEFINE_MIN,USR_DEFINE_PWM_COUNT);
	
	timer_dest.hour = USR_DEFINE_HOUR;
	timer_dest.min = USR_DEFINE_MIN;
	RTC_SetAlarm_user(&timer_dest,pwm_start_irqhandler);
	
	if(usartCharGet_timeout(UART_INPUT_WAIT)){
		printf("\r\nno user stop,step into normal mode\r\n");
		while(1){
					
			//RTC_Get(&timer);

			//delay_ms(500);
			//led_trigle(LED3_PIN);

			if(pwm_start){
					RTC_Get(&timer);

					printf("pwm startimg...\r\n");
					printf("current timer is :\r\n");
					printf("	%d	%02d %02d,%02d:%02d:%02d\r\n",timer.w_year,timer.w_month,timer.w_date,timer.hour,timer.min,timer.sec);
				
						

					pwm_start_out();
					pwm_start = 0;
					

					// trigger LED for last alarm
					config_pwm_gpio_pin();
					cc=0;
					while(cc < 300){
							delay_s(1);
							pwm_gpio_trigle();
							cc++;
					}
					
					
					GPIO_SetBits(GPIOA,GPIO_Pin_7);
					delay_s(3600);
					
					/*
					RTC_Get(&timer);
					timer_dest.hour = timer.hour+USR_DEFINE_PWM_HOUR;
					timer_dest.min = timer.min;
					RTC_SetAlarm_user(&timer_dest,pwm_stop_irqhandler);
					*/

					printf("pwm end...\r\n");


					// resume to PWM configuration
					config_pwm_pin();
					pwm_stop = 1;
				
			}
			
			if(pwm_stop){
					printf("shutdown the LED ...\r\n");
					TIM_SetCompare2(TIM3,0);
					pwm_stop = 0;
				
					timer_dest.hour = USR_DEFINE_HOUR;
					timer_dest.min = USR_DEFINE_MIN;
					RTC_SetAlarm_user(&timer_dest,pwm_start_irqhandler);
			}
			
			printf("\r\n\r\ngo to sleep\r\n");
#ifdef USE_STOP_MODE
			
			PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI); // when resum from stop mode,the system clock is HSI 8Mhz
#else
			__WFI();
#endif
			//GPIO_ResetBits(LED1_GPIO,LED1_PIN);
			//delay_ms(10);
			printf("system resum\r\n");
			
			//BKP_WriteBackupRegister(BKP_DR4, (pwm_start<<8) | pwm_stop);
			//PWR_EnterSTANDBYMode();
			
			
		}
	}
	else{
		 //GPIO_ResetBits(LED2_GPIO,LED2_PIN);
		main_loop();
	}

	
}





#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
  while (1)
  {
  }
}
#endif
