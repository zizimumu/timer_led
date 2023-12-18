
#ifndef _ADXL_H
#define _ADXL_H

#define FLAG_DATA_RREADY  	0x80
#define FLAG_SINGLE_TAP		0x40
#define FLAG_DOUBLE_TAP		0x20
#define FLAG_ACTIVE			0x10
#define FLAG_INACTIVE		0x08
#define FLAG_FREE_FALL		0x04
#define FLAG_WATER_MARK		0x02
#define FLAG_OVER_RUN		0x01


enum FREE_FALL_STATE {
	STATE_NON = 0,
	STATE_FALL,
	STATE_ACTIVE,
	STATE_INACTIVE
		
};




void ADXL_Init_SPI(void);
void ADXL_Init_I2C(void);

int16_t ReadAxisX(void);
int16_t ReadAxisY(void);
int16_t ReadAxisZ(void);

u8 SPI_Read(u8 addr);
u8 ADXL_GetIntState(void);


void step_counter(void);
void ADXL_IntProcess(void);




#endif
