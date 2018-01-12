#include "stm32f10x.h"

#include "app.h"
#include <stdio.h>

#define TIMER_COUNT 36000  //1Khz

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
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIMx在CCR2上的预装载寄存器
	
	TIM_ARRPreloadConfig(TIM3, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
	}

void LedInit(void )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(LED1_CLOCK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = LED1_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED1_GPIO, &GPIO_InitStructure);

	GPIO_SetBits(LED1_GPIO,LED1_PIN);
}

void led_trigle(void)
{
	static unsigned int on = 0;
	if(on & 0x1)
		GPIO_SetBits(LED1_GPIO,LED1_PIN);
	else
		GPIO_ResetBits(LED1_GPIO,LED1_PIN);
	
	on++;
}

void Init_All_Periph(void)
{
	RCC_Configuration();	
	NVIC_Configuration();

	ComInit(COM1,115200);
	//usart1TxDMAInit();
	delay_init(72);
	
	//TIM3_UserConfiguration();

	LedInit();
	RTC_UserInit();



}



extern void main_loop(void );
tm timer;
#define USR_DEFINE_HOUR 6
#define USR_DEFINE_MIN 20
#define USR_DEFINE_PWM_COUNT 10
#define USR_DEFINE_PWM_HOUR 1


void pwm_start_out(void)
{
	unsigned int sec,step,pwmval,cnt;
	
	sec = USR_DEFINE_PWM_COUNT*60;
	step = TIMER_COUNT/sec;
	pwmval = 0;
	while(1){
			TIM_SetCompare2(TIM3,pwmval);
			pwmval += step;
			delay_ms(1000);
			if(pwmval > TIMER_COUNT)
				break;
			
			led_trigle();
	}
}


int main(void)
{  
	int pwm_start = 0,pwm_stop = 0;
	
	
	Init_All_Periph();


	printf("system startimg...\r\n");
	printf("user set timer is %d:%d\r\n",USR_DEFINE_HOUR,USR_DEFINE_MIN);
	while(1){
		main_loop();
		
		
		RTC_Get(&timer);
		
		if((timer.hour == USR_DEFINE_HOUR) && \
			  (timer.min == USR_DEFINE_MIN) ){
				pwm_start = 1;
				pwm_stop = 0;
		}
		if(pwm_start){
				printf("pwm startimg...\r\n");
				pwm_start_out();
				pwm_start = 0;
				printf("pwm end...\r\n");
		}
		
		if((timer.hour - USR_DEFINE_HOUR == USR_DEFINE_PWM_HOUR )&& (pwm_stop == 0)){
				printf("shutdown the LED ...\r\n");
				TIM_SetCompare2(TIM3,0);
				pwm_stop = 1;
		}
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
