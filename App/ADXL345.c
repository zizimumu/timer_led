
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "ADXL345.h"
#include "app.h"


#include "delay.h"
#include<math.h>


#define ADXL_CS_PIN 	GPIO_Pin_2
#define ADXL_GPIO 		GPIOA
#define ADXL_CLOCK 	RCC_APB2Periph_GPIOA

#define ADXL_CS_HIGH()	 ADXL_GPIO->BSRR = ADXL_CS_PIN
#define ADXL_CS_LOW()	 ADXL_GPIO->BRR = ADXL_CS_PIN



#define VALID_STEP 3

#define READ_TIMES 5
#define TIMEWINDOW_MIN 2   		//时间窗，×0.02s＝0.2s
#define TIMEWINDOW_MAX 20		//时间窗，×0.02s＝2s


#define FALL_INACTIVE_THREASH 		4
#define FALL_INACTIVE_TIME 			5


#define INTERVAL_FALL2ACTIVE		10  //200ms
#define VPP_THREASH		100  //200ms
#define SAMPLE_COUNTE		60  
#define LONG_TIME_SIT_THREASH    	50
#define LONG_TIME_SIT_ALARM    	10



#define LED1_ON()   GPIO_ResetBits(LED1_GPIO,LED1_PIN)
#define LED1_OFF()   GPIO_SetBits(LED1_GPIO,LED1_PIN)





int32_t _axisTmpVal_X[READ_TIMES];
int32_t _axisTmpVal_Y[READ_TIMES];
int32_t _axisTmpVal_Z[READ_TIMES];


							
int32_t _axisResult[3];	
int32_t _axisResultOldSit[3];	
int32_t _axisResultNewSit[3];	


int32_t gStepCount;	
int32_t gLongTimeSitCount;	

int32_t gStepValid[3];	

uint32_t gSepOldTimer[3];	
uint32_t gSepNewTimer[3];
uint32_t gSampeCounter;

uint32_t gFallTimer;

uint8_t gFallstate = STATE_NON;

uint32_t gLedOn;



int16_t _array0[3]={1,1,1};
int16_t _array1[3]={1,1,1};
int16_t _array2[3]={0,0,0};
int16_t _array3[3]={0,0,0};

int16_t _max[3]={0,0,0};
int16_t _min[3]={0x0fff,0x0fff,0x0fff};

int16_t _dc[3]={500,500,500};
int16_t _vpp[3]={30,30,30};	
int16_t  _precision[3]={0x0fff,0x0fff,0x0fff};	
int16_t _old_fixed[3];
int16_t _new_fixed[3];

enum AXIS_CHANEL {
	X_AXIS = 0,
	Y_AXIS,
	Z_AXIS
	};
#include "x_sourceChar.c"
#include "y_sourceChar.c"
#include "z_sourceChar.c"

int gImitateCount;
void ADXL_IO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(	ADXL_CLOCK, ENABLE );	
	   

	GPIO_InitStructure.GPIO_Pin = ADXL_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ADXL_GPIO, &GPIO_InitStructure);
	
	ADXL_CS_HIGH();

}

void SPI_Wrt(u8 addr,u8 data) //地址格式：0xFF 数据格式：0xFF
{

  SPI_UserReadWriteByte(addr);

  SPI_UserReadWriteByte(data);
 

}
u8 SPI_Read(u8 addr) //地址格式：0xFF 数据格式：0xFF
{
	u8 value;
	addr = addr | 0x80;
	 SPI_UserReadWriteByte(addr);

	value = SPI_UserReadWriteByte(0xff);


	return value;
}
int16_t ReadAxisX(void)
{
	u8 tmp;
	u16 value;
	
	value =SPI_Read(0x33);
	value = value <<8;
	tmp = SPI_Read(0x32);
	value = value | tmp;
	
	return ((int16_t)value);
}
int16_t ReadAxisY(void)

{
	u8 tmp;
	u16 value;
	
	value =SPI_Read(0x35);
	value = value <<8;
	tmp = SPI_Read(0x34);
	value = value | tmp;
	
	return ((int16_t)value );

}
int16_t ReadAxisZ(void)

{
	u8 tmp;
	u16 value;
	
	value =SPI_Read(0x37);
	value = value <<8;
	tmp = SPI_Read(0x36);
	value = value | tmp;
	
	return ((int16_t)value );

}

u8 ADXL_GetIntState(void)
{
	return SPI_Read(0x30);
}
void ADXL_EnbleActiveInt(void)
{
	 SPI_Wrt(0x2E,0x10);
}



void ADXL_EnbleInactiveInt(void)
{
	 SPI_Wrt(0x2E,0x08);
}

void ADXL_EnbleFreeFallInt(void)
{
	 SPI_Wrt(0x2E,0x04);
}
void ADXL_EnbleFreeFall_InactInt(void)
{
	 SPI_Wrt(0x2E,0x0c);
}

void InitInactivePara(u8 threash,u8 inactTime )
{
	SPI_Wrt(0x25,threash);  //inactive thresh	(62.5mg/LSB) unsigned
	SPI_Wrt(0x26,inactTime);  //TIME_INACT  (1s/LSB)
}

void SetInactiveACModel()
{
	//SPI_Wrt(0x27,0x70);  //disable inactive
	SPI_Wrt(0x27,0x7f);  //all axis enable ,a
}
void SetInactiveDCModel()
{
	//SPI_Wrt(0x27,0x70);  //disable inactive
	SPI_Wrt(0x27,0x77);  //all axis enable 
}


void ProcessFreeFall(void)
{
	DebugPrint("fall detected\r\n");
	GPIO_ResetBits(LED1_GPIO,LED1_PIN);
}
void ProcessFreeFallEmerg(void)
{
	DebugPrint("fall detected,and emergece occur\r\n");
	GPIO_ResetBits(LED2_GPIO,LED2_PIN);
}

// 中断优先级要小于定时器读取中断
void ADXL_IntProcess(void)
{
	u8 status;
	int32_t interval;
	status = ADXL_GetIntState();
	if(status&FLAG_FREE_FALL){
		if(STATE_NON == gFallstate){
			gFallstate = STATE_FALL;
			gFallTimer = GetTimer();
			ADXL_EnbleActiveInt();
		}
	}
	else if(status&FLAG_ACTIVE){
		if(STATE_FALL == gFallstate){
			gFallstate = STATE_ACTIVE;
			interval = GetTimer()- gFallTimer ;
			if((1 < interval)&&(interval <= INTERVAL_FALL2ACTIVE)){  // 200ms 


				InitInactivePara(FALL_INACTIVE_THREASH,FALL_INACTIVE_TIME);
				ADXL_EnbleInactiveInt();
				gFallstate = STATE_INACTIVE;
				ProcessFreeFall();
			}
			else {
				gFallstate = STATE_NON;
				ADXL_EnbleFreeFallInt();
			}
		}
	}
	else if(status&FLAG_INACTIVE){
		if(STATE_INACTIVE == gFallstate){
			ADXL_EnbleFreeFallInt();
			ProcessFreeFallEmerg();
			gFallstate = STATE_NON;
		}
		else{

		}
	}
	else {}

	DebugPrint("ADXL interrupt\r\n");
	
}

void ADXL_Init_SPI(void)
{

	DebugPrint("ADXL_Init start\r\n");

	SPI_UserInit();
	ADXL_IO_Init();

	
	ADXL_CS_LOW();
		
	SPI_Wrt(0x31,0x0B);  //4-wire SPI,inerrupt active high ,13 bit resolution, right align,rang : 16g

	SPI_Wrt(0x2D,0x20);  //set to standby model

	SPI_Wrt(0x1E,0x00);  //X轴误差补偿; (15.6mg/LSB)
	SPI_Wrt(0x1F,0x00);  //Y轴误差补偿; (15.6mg/LSB)
	SPI_Wrt(0x20,0x00);  //Z轴误差补偿; (15.6mg/LSB)

	SPI_Wrt(0x21,0x00);  //敲击延时0:禁用; (1.25ms/LSB)
	SPI_Wrt(0x22,0x00);  //检测第一次敲击后的延时0:禁用; (1.25ms/LSB)
	SPI_Wrt(0x23,0x00);  //敲击窗口0:禁用; (1.25ms/LSB)

	SPI_Wrt(0x24,0x20);  //active thresh  (62.5mg/LSB) unsigned   ->. 2g
	//SPI_Wrt(0x25,0x02);  //inactive thresh  (62.5mg/LSB) unsigned
	//SPI_Wrt(0x26,0x05);  //TIME_INACT  (1s/LSB)
	SPI_Wrt(0x27,0x77);  //CT_INACT_CTL  DC model and all axis enable;  active detecd: logically OR  ;inactive detect: logically AND
	SPI_Wrt(0x28,0x0C);  //free fall thresh  (62.5mg/LSB) -> 0.75g
	SPI_Wrt(0x29,0x06);  //free fall time (5ms/LSB)         -> 30ms
	SPI_Wrt(0x2A,0x80);  //

	SPI_Wrt(0x2C,0x0A);  //100Hz data rate
	  //开启Link,测量功能;关闭自动休眠,休眠,唤醒功能
	//SPI_Wrt(0x2E,0x04);  //  0x1C    enble active ,inactive ,free fall inerrupt
	SPI_Wrt(0x2F,0xe0);  // select to INT1 pin
	
	SPI_Wrt(0x38,0x00);  //FIFO模式设定,Stream模式，触发连接INT1,31级样本缓冲

	
	InitInactivePara(FALL_INACTIVE_THREASH,FALL_INACTIVE_TIME);
	//SetInactiveACModel();
	ADXL_EnbleFreeFallInt();


	SPI_Wrt(0x2D,0x28);//link ,mearsure
	
	DebugPrint("ADXL_Init end\r\n");
}

void ADXL_Init_I2C(void)
{
	I2c_UserInit();
	
	//I2C_WriteByte(0xA6,BW_RATE,0x08);   //速率设定为12.5 参考pdf13页
	//I2C_WriteByte(0xA6,POWER_CTL,0x08);   //电源工作方式设置
//	I2C_WriteByte(0xA6,DATA_FORMAT,0x03);	
}


void GetAxisVal(void)

{
	
/*	
	_axisResult[X_AXIS] = x_sourceChar[gImitateCount];
	_axisResult[Y_AXIS] = y_sourceChar[gImitateCount];
	_axisResult[Z_AXIS] = z_sourceChar[gImitateCount];
	gImitateCount++;
	//DebugPrint("count is ");
	//PrintInt(gImitateCount,0,0);
	if(gImitateCount == (sizeof(x_sourceChar)/2) ){
		DebugPrint("imitage end\r\n");
		while(1){
			
		}
	}

*/


	int16_t itmp;
//	int16_t max_x = 0,max_y = 0,max_z = 0;
//	int16_t min_x = 0x0fff,min_y = 0x0fff,min_z = 0x0fff;
	int32_t x_totalVal = 0,y_totalVal = 0,z_totalVal = 0;
	
	for(itmp=0;itmp < READ_TIMES;itmp ++){
		_axisTmpVal_X[itmp]= ReadAxisX();
		_axisTmpVal_Y[itmp]= ReadAxisY();
		_axisTmpVal_Z[itmp]= ReadAxisZ();


		#if 0 
		if(_axisTmpVal_X[itmp] > max_x)
			max_x = _axisTmpVal_X[itmp];
		if(_axisTmpVal_Y[itmp] > max_y)
			max_y = _axisTmpVal_Y[itmp];
		if(_axisTmpVal_Z[itmp] > max_z)
			max_z = _axisTmpVal_Z[itmp];
		
		if(_axisTmpVal_X[itmp] < min_x)
			min_x = _axisTmpVal_X[itmp];
		if(_axisTmpVal_Y[itmp] < min_y)
			min_y = _axisTmpVal_Y[itmp];
		if(_axisTmpVal_Z[itmp] < min_z)
			min_z = _axisTmpVal_Z[itmp];
		#endif

		x_totalVal += _axisTmpVal_X[itmp];
		y_totalVal += _axisTmpVal_Y[itmp];
		z_totalVal += _axisTmpVal_Z[itmp];
	}	
	_axisResult[X_AXIS] = (x_totalVal) / (READ_TIMES);//(x_totalVal - max_x - min_x ) / (READ_TIMES -2);
	_axisResult[Y_AXIS] = (y_totalVal ) / (READ_TIMES);//(y_totalVal - max_y - min_y ) / (READ_TIMES -2);
	_axisResult[Z_AXIS] = (z_totalVal ) / (READ_TIMES);//(z_totalVal - max_z - min_z ) / (READ_TIMES -2);


	
}

void TimeWindow(u8 axis)
{
	gSepOldTimer[axis] = gSepNewTimer[axis];
	gSepNewTimer[axis] = GetTimer();

	if( (gSepNewTimer[axis]-gSepOldTimer[axis]>10)&&(gSepNewTimer[axis]-gSepOldTimer[axis]<100)){ // 0.2s -- 2s

		gStepValid[axis]++;
		if(gStepValid[axis] > VALID_STEP) {
			gStepCount += gStepValid[axis];
			gStepValid[axis] = 0;
			DebugPrint("step valid \r\n");
			PrintInt(gStepCount,0,0);
			LCD_WriteInt(gStepCount,0,1,0);
		}
		
	}
	else 
		gStepValid[axis] = 0;
	
}


void step_counter(void)
{
	unsigned char jtemp;	
	uint32_t timer;
	GetAxisVal();

	timer = GetTimer();
/*	
	if((timer % 50) == 0){
		gLongTimeSitCount = 0;
		_axisResultOldSit[X_AXIS]  = _axisResultNewSit[X_AXIS] ;
		_axisResultOldSit[Y_AXIS]  = _axisResultNewSit[Y_AXIS] ;
		_axisResultOldSit[Z_AXIS]  = _axisResultNewSit[Z_AXIS] ;

		_axisResultNewSit[X_AXIS] = _axisResult[X_AXIS];
		_axisResultNewSit[Y_AXIS] = _axisResult[Y_AXIS];
		_axisResultNewSit[Z_AXIS] = _axisResult[Z_AXIS];
		if((abs(_axisResultOldSit[X_AXIS] - _axisResultNewSit[X_AXIS]) > LONG_TIME_SIT_PREC)&&
			(abs(_axisResultOldSit[Y_AXIS] - _axisResultNewSit[Y_AXIS]) > LONG_TIME_SIT_PREC)&&
			(abs(_axisResultOldSit[Z_AXIS] - _axisResultNewSit[Z_AXIS]) > LONG_TIME_SIT_PREC) ){
			gLongTimeSitCount++;
			if(gLongTimeSitCount > LONG_TIME_SIT_MAX_TIMER){ //10s
				gLongTimeSitCount = 0;
				DebugPrint("sit long time\r\n");
			}
		}
		else {
			gLongTimeSitCount = 0;
		}
	}
*/
	
	for(jtemp=X_AXIS;jtemp<=Z_AXIS;jtemp++){

		_array3[jtemp]=_array2[jtemp];
		_array2[jtemp]=_array1[jtemp];
		_array1[jtemp]=_array0[jtemp];

		_array0[jtemp]=_axisResult[jtemp];
		_axisResult[jtemp]=_array0[jtemp]+_array1[jtemp]+_array2[jtemp]+_array3[jtemp];
	 	_axisResult[jtemp]=_axisResult[jtemp]/4;
		
		if (_axisResult[jtemp]>_max[jtemp])               {_max[jtemp]=_axisResult[jtemp];}
		if (_axisResult[jtemp]<_min[jtemp])               {_min[jtemp]=_axisResult[jtemp];}
	}


	gSampeCounter++;


	//----------------------------------计算动态门限和动态精度-----------------------//
	if (gSampeCounter == SAMPLE_COUNTE){               
	  	gSampeCounter=0;



		DebugPrint("vpp is\r\n");
		for(jtemp=X_AXIS;jtemp<=Z_AXIS;jtemp++){
			_vpp[jtemp]=_max[jtemp]-_min[jtemp];
	    	_dc[jtemp]=_min[jtemp]+(_vpp[jtemp]/2);

			//PrintInt(_max[jtemp],0,1);PrintInt(_min[jtemp],0,0);
			PrintInt(_vpp[jtemp],0,0);
			
			_max[jtemp]=0;
	    	_min[jtemp]=0x0fff;

			if (_vpp[jtemp]>=VPP_THREASH){
				_precision[jtemp]=_vpp[jtemp]/8; //8  _vpp[jtemp]/32
				gLongTimeSitCount = 0;
			}

			else {
				_precision[jtemp]= 0x0fff;
				//gLongTimeSitCount = 0;
			}

		}
		if((_vpp[X_AXIS] < LONG_TIME_SIT_THREASH)
			&&(_vpp[Y_AXIS] < LONG_TIME_SIT_THREASH)
			&&(_vpp[Z_AXIS] < LONG_TIME_SIT_THREASH) ){

			gLongTimeSitCount++;
			if(LONG_TIME_SIT_ALARM == gLongTimeSitCount){
				DebugPrint("long time sit\r\n");
				gLongTimeSitCount = 0;
				
				gLedOn = 1;
				
			}
		}
		if(gLedOn){
			LED1_ON();
			gLedOn = 0;
		}
		else {
			LED1_OFF();
		}
	}

	//--------------------------线性移位寄存器--------------------------------------

	for(jtemp=X_AXIS;jtemp<=Z_AXIS;jtemp++){
		_old_fixed[jtemp]=_new_fixed[jtemp];

		if (_axisResult[jtemp]>=_new_fixed[jtemp]){   
	 		if((_axisResult[jtemp]-_new_fixed[jtemp])>=_precision[jtemp]){
				_new_fixed[jtemp]=_axisResult[jtemp];
			}
		}
		if (_axisResult[jtemp]<_new_fixed[jtemp]){   
	   		if((_new_fixed[jtemp]-_axisResult[jtemp])>=_precision[jtemp]){
				_new_fixed[jtemp]=_axisResult[jtemp];
			}
		}
	}

	//------------------------- 动态门限判决 ----------------------------------
	if ((_vpp[X_AXIS]> _vpp[Y_AXIS])&&(_vpp[X_AXIS]> _vpp[Z_AXIS])){
		if ((_old_fixed[X_AXIS]>_dc[X_AXIS])&&(_new_fixed[X_AXIS]<_dc[X_AXIS])){
			DebugPrint("xaxis find step\r\n");
			TimeWindow(X_AXIS);
			//STEPS=STEPS+1;
		} 
	}
	else if ((_vpp[Y_AXIS]> _vpp[X_AXIS])&&(_vpp[Y_AXIS]> _vpp[Z_AXIS])){
		if ((_old_fixed[Y_AXIS]>_dc[Y_AXIS])&&(_new_fixed[Y_AXIS]<_dc[Y_AXIS])){
			DebugPrint("yaxis find step\r\n");
			TimeWindow(Y_AXIS);
			//STEPS=STEPS+1;
		}
	}
	else if ((_vpp[Z_AXIS]> _vpp[Y_AXIS])&&(_vpp[Z_AXIS]> _vpp[X_AXIS])){
		if ((_old_fixed[Z_AXIS]>_dc[Z_AXIS])&&(_new_fixed[Z_AXIS]<_dc[Z_AXIS])){
			DebugPrint("zaxis find step\r\n");
			TimeWindow(Z_AXIS);
			//STEPS=STEPS+1;
		}
	}



	if( (STATE_FALL == gFallstate)||(STATE_ACTIVE == gFallstate)||(STATE_INACTIVE== gFallstate) ){
		if(GetTimer()- gFallTimer >= 300 ){  //6s 

			ADXL_EnbleFreeFallInt();
			gFallstate = STATE_NON;
		}
	}

}



