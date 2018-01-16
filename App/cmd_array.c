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
	unsigned int year,month,data,hour,min;
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


 
 cmd_tbl_t gCmd_array[] = {
 {"test",1,1,test_function,"just for test"},
 {"rtc",1,1,rtc_function,"rtc function"},
 {"pwm",1,1,pwm_set,"pwm set"}
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

