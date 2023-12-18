#ifndef _AFE4400_H
#define _AFE4400_H

#include "stm32f10x.h"

/*********************
AFE4400 Register Address Definitions
*********************/

#define CONTROL0                0x00
#define REDSTARTCOUNT           0x01
#define REDENDCOUNT             0x02
#define REDLEDSTARTCOUNT        0x03
#define REDLEDENDCOUNT          0x04
#define AMBREDSTARTCOUNT        0x05
#define AMBREDENDCOUNT          0x06
#define IRSTARTCOUNT            0x07
#define IRENDCOUNT              0x08
#define IRLEDSTARTCOUNT         0x09
#define IRLEDENDCOUNT           0x0A
#define AMBIRSTARTCOUNT         0x0B
#define AMBIRENDCOUNT           0x0C
#define REDCONVSTART            0x0D
#define REDCONVEND              0x0E
#define AMBREDCONVSTART         0x0F
#define AMBREDCONVEND           0x10
#define IRCONVSTART             0x11
#define IRCONVEND               0x12
#define AMBIRCONVSTART          0x13
#define AMBIRCONVEND            0x14
#define ADCRESETSTCOUNT0        0x15
#define ADCRESETENDCOUNT0       0x16
#define ADCRESETSTCOUNT1        0x17
#define ADCRESETENDCOUNT1       0x18
#define ADCRESETSTCOUNT2        0x19
#define ADCRESETENDCOUNT2       0x1A
#define ADCRESETSTCOUNT3        0x1B
#define ADCRESETENDCOUNT3       0x1C
#define PRPCOUNT                0x1D
#define CONTROL1                0x1E
#define TIAGAIN                 0x20
#define TIA_AMB_GAIN            0x21
#define LEDCNTRL                0x22
#define CONTROL2                0x23
#define MODE                    0x25
#define ALARM                   0x29
#define REDVALUE                0x2A
#define AMBREDVALUE             0x2B
#define IRVALUE                 0x2C
#define AMBIRVALUE              0x2D
#define REDMINUSAMBREDVALUE     0x2E
#define IRMINUSAMBIRVALUE       0x2F
#define DIAGNOSTICS             0x30

void AFEXX_IO_Init(void);
void AFEXX_processData(void );
void initAFE4400(void);


#endif 

