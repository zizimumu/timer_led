#include "Readline.h"
#include <stdio.h>
#include <string.h>
#include "stm32f10x_rtc.h"
#include "stm32f10x.h"
 
 
int  test_function(struct cmd_tbl_s *tst_tbl,int a,int b,char *buf[])
{
	int i;

	printf("arg count %d\n\r avgv: ",b);

	for(i=0;i<b;i++)
	printf(" %s",buf[i]);

	return 1;
}

int  rtc_function(struct cmd_tbl_s *tst_tbl,int a,int b,char *buf[])
{
	int argc = b;
	tm timer;
	char tmp[32];
	char *src;
	unsigned int year,month,data,hour,min,temp;
	if(argc<2){
			printf("arg err,please use as fellow\r\n");
			printf("    rtc get/set");
			return 0;
	}
	if(strcmp(buf[1],"get") == 0){
		RTC_Get(&timer);
		printf("current timer is :\r\n");
		printf("	%d	%02d %02d,%02d:%02d:%02d\r\n",timer.w_year,timer.w_month,timer.w_date,timer.hour,timer.min,timer.sec);
	}
	else{
		if((strcmp(buf[1],"set") == 0) && \
			(argc == 3) && \
			(strlen(buf[2]) == 12 )){
				src = buf[2];
				memcpy(tmp,src,12);
				tmp[12] = 0;
				
				sscanf(&tmp[10],"%d",&min);
				tmp[10] = 0;
				sscanf(&tmp[8],"%d",&hour);
				tmp[8] = 0;
				sscanf(&tmp[6],"%d",&data);
				tmp[6] = 0;
				sscanf(&tmp[4],"%d",&month);
				tmp[4] = 0;
				sscanf(&tmp[0],"%d",&year);
				//tmp[10] = 0;
				printf("setting	%d	%02d %02d,%02d:%02d\r\n",year,month,data,hour,min);

				RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);	
				
				/* Allow access to BKP Domain ,PWR_CR register*/
				PWR_BackupAccessCmd(ENABLE);	
				
				/* Reset Backup Domain ,RCC_BDCR register*/
				BKP_DeInit();	
				
				/* Enable LSE */
				RCC_LSEConfig(RCC_LSE_ON);	
				/* Wait till LSE is ready */
				while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET){	
					temp++;
				}
				//if(temp>=72000000)	return 1;    
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

				RTC_Set(year,month,data,hour,min,0);
		}
		else{
			printf("rtc set param err,e.g. 201211131314\r\n");
		}
	}
	return 1;
}


int  pwm_set(struct cmd_tbl_s *tst_tbl,int a,int b,char *buf[])
{
	int argc = b;
	unsigned int duty = 0;

	if(argc != 2){
		printf("pwm_set param err,please use e.g.  pwm 50\r\n");
		return -1;
	}
	
#define TIMER_COUNT 36000

	sscanf(buf[1],"%d",&duty);
	if(duty > 100){
		printf("pwm_set param err,please use e.g.  pwm 50\r\n");
		return -1;
	}
	printf("setting pwm duty to percent %d\r\n",duty);
	
	duty = (TIMER_COUNT*duty)/100;
	TIM_SetCompare2(TIM3,duty);

	return 1;
}
int  alarm_set(struct cmd_tbl_s *tst_tbl,int a,int b,char *buf[])
{
	int argc = b;
  int pwm_min,hour,min;
	uint16_t back;
	char tmp[16];

	if(argc != 3){
		printf("alarm_set param err,please use e.g.  alarm 0620 10\r\n");
		return -1;
	}
	memcpy(tmp,buf[1],4);
	tmp[4] = 0;
	sscanf(&tmp[2],"%d",&min);
	tmp[2] = 0;
	sscanf(&tmp[0],"%d",&hour);
	
	sscanf(buf[2],"%d",&pwm_min);

	if(hour <24 && hour >=0 && min <60 && min >= 0 && pwm_min <= 60){
		back = (uint16_t)hour<<8 | (uint16_t)min;
		
		printf("alarm setting is %d:%d, pwm min is %d\r\n",hour,min,pwm_min);
		BKP_WriteBackupRegister(BKP_DR1, back);	
		BKP_WriteBackupRegister(BKP_DR2, pwm_min);	
	}
	else{
		printf("param err\r\n");
	}
	
	


	return 1;
}



 int  reset(struct cmd_tbl_s *tst_tbl,int a,int b,char *buf[])
 {
	
	 return 1;
 }


 cmd_tbl_t gCmd_array[] = {
 {"test",1,1,test_function,"just for test"},
 {"rtc",1,1,rtc_function,"rtc get/set 201201021112"},
 {"pwm",1,1,pwm_set,"pwm set"},
 {"alarm",1,1,alarm_set,"alarm set"},
 {"reset",1,1,reset,"system reset"}
};




void print_invalide_cmd(void)
{
//	cmd_tbl_t *cmdtp;
	int tablen;

	for (tablen = 0;tablen < sizeof(gCmd_array)/sizeof(gCmd_array[0]);
	     tablen++) {
		 s_putstring(gCmd_array[tablen].name);
		 s_putstring("\r\n");
	}
}
cmd_tbl_t *find_cmd (const char *cmd)
{
	cmd_tbl_t *cmdtp;
	int i;

	if (!cmd)
		return NULL;

	for (i = 0;i < sizeof(gCmd_array)/sizeof(gCmd_array[0]);i++) {
		cmdtp = &gCmd_array[i];
		if (strcmp(cmd, cmdtp->name) == 0) {
				return cmdtp;	/* full match */
		}
	}
	return NULL;	/* not found or ambiguous command */

}

