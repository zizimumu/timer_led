#ifndef _USB_CORE_H

#define _USB_CORE_H

#include <stm32f10x_lib.h>

typedef union
{
  u16 w;
  struct BW
  {
    u8 byte0;
    u8 byte1;
  }
  bw;
} u16_u8;

typedef struct _DEVICE_INFO
{
  u8 USBbmRequestType;       /* bmRequestType */
  u8 USBbRequest;            /* bRequest */
  u16_u8 USBwValues;         /* wValue */
  u16_u8 USBwIndexs;         /* wIndex */
  u16_u8 USBwLengths;        /* wLength */
}SETUP_DATA;


typedef enum _STANDARD_REQUESTS
{
  GET_STATUS = 0,
  CLEAR_FEATURE,
  RESERVED1,
  SET_FEATURE,
  RESERVED2,
  SET_ADDRESS,
  GET_DESCRIPTOR,
  SET_DESCRIPTOR,
  GET_CONFIGURATION,
  SET_CONFIGURATION,
  GET_INTERFACE,
  SET_INTERFACE,
  TOTAL_sREQUEST,  /* Total number of Standard request */
  SYNCH_FRAME = 12
} STANDARD_REQUESTS;

#define REQUEST_TYPE_MASK 0x60

/************request type *********/
#define STAND_REQUEST  0x00
#define DEV_CLASS_REQUEST  0x20
#define VENDOR_REQUEST  0x40


/************request descriptor type  *********/
#define DEV_DESCRIPTOR  0x01
#define CONFIG_DESCRIPTOR  0x02
#define STRING_DESCRIPTOR  0x03
#define INTERF_DESCRIPTOR  0x04
#define HID_CLASS_DESCRIPTOR  0x21
#define HID_REPORT_DESCRIPTOR  0x22

/************get status type  *********/
#define GET_DEV_STATUS 0x80
#define GET_INTERF_STATUS 0x81
#define GET_ENDP_STATUS 0x82

void Setup_Process(u8 * RecBbuf);
void In_Processs(void );
void Out_Process(void );


extern u16 g_Endp0_RxState;
extern u16 g_Endp0_TxState;


#endif

