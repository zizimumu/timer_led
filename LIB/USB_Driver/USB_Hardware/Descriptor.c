#include <stm32f10x_lib.h>

#include "Descriptor.h"
/*
** HID MOUSE DESCRIPTOR DECLARATIONS
*/
const unsigned char    get_status_device_table[ 2] = 
{
 0x00, 0x00 /* always return both bytes zero */
};
const unsigned char    get_status_interface_table[2] = 
{
 0x00, 0x00 /* always return both bytes zero */
};
const unsigned char    get_status_endpoint_table[2] = 
{
 0x00, 0x00 /* always return both bytes zero */
};


const unsigned char Conf_Des_All[LEN_CONFIG_ALL_DESCRIPTOR] =
{
	
 /* config_desc_table */
 0x09, 		/* length of descriptor (9 bytes) */
 0x02, 		/* descriptor type (CONFIGURATION) */
 0x19, 0x00,/* total length of descriptor (bytes) */
 0x01, 		/* number of interfaces to configure (1) */
 0x01, 		/* configuration value (1) */
 0x00, 		/* configuration string index  */
 0xE0, 		/* configuration attributes (bus powered, remote wakeup) */
 0x64, 		/* maximum power (set at 100, change this value for your specific optics)*/

 
  /* interface descriptor */
 0x09, 		/* length of descriptor (9 bytes) */
 0x04, 		/* descriptor type (INTERFACE) */
 0x00, 		/* interface number (0) */
 0x00, 		/* alternate setting (0) */
 0x01, 		/* number of endpoints (2) */
 0xff, 		/* interface class () */
 0x00, 		/* interface sub-class (HID use no sub-class ,but use :1=BOOT, 0=no boot) */
 0x00, 		/* interface protocol (2..defined by USB spec) */
 0x00 , 		/* interface string index (not supported) */


 
/* Endpoint1_Descriptor[] */

 0x07, 		/* descriptor length (7 bytes) */
 0x05, 		/* descriptor type (ENDPOINT) */
 0x81, 		/* endpoint address (IN endpoint, endpoint 1) */
 0x03, 		/* endpoint attributes (interrupt) */
 0x40, 0x00, /* maximum packet size (64 bytes) */
 0x1a,		/* polling interval (10ms) */
 
 /* Endpoint2_Descriptor[] */


// 0x07, 		/* descriptor length (7 bytes) */
// 0x05, 		/* descriptor type (ENDPOINT) */
// 0x02, 		/* endpoint address (OUT endpoint, endpoint 2) */
 //0x03, 		/* endpoint attributes (interrupt) */
 //0x40, 0x00, /* maximum packet size (64 bytes) */
 //0x1a 		/* polling interval (10ms) */
 
};
const unsigned char Endpoint_Descriptor[7] = 
{
 0x07, 		/* descriptor length (7 bytes) */
 0x05, 		/* descriptor type (ENDPOINT) */
 0x81, 		/* endpoint address (IN endpoint, endpoint 1) */
 0x03, 		/* endpoint attributes (interrupt) */
 0x40, 0x00, /* maximum packet size (4 bytes) */
 0x0a  		/* polling interval (10ms) */
};


const unsigned char  Interface_Descriptor[9] = 
{
 0x09, 		/* length of descriptor (9 bytes) */
 0x04, 		/* descriptor type (INTERFACE) */
 0x00, 		/* interface number (0) */
 0x00, 		/* alternate setting (0) */
 0x01, 		/* number of endpoints (2) */
 0xff, 		/* interface class (self define) */
 0x00, 		/* interface sub-class (HID use no sub-class ,but use :1=BOOT, 0=no boot) */
 0x00, 		/* interface protocol (if sub-class =0, it 0,other  1  keboard , 2. mouse) */
 0x00  		/* interface string index (not supported) */
};

const unsigned char  config_desc_table[9] = 
{
 0x09, 		/* length of descriptor (9 bytes) */
 0x02, 		/* descriptor type (CONFIGURATION) */
 0x19, 0x00,/* total length of descriptor (bytes) */
 0x01, 		/* number of interfaces to configure (1) */
 0x01, 		/* configuration value (1) */
 0x00, 		/* configuration string index  */
 0xA0, 		/* configuration attributes (bus powered, remote wakeup) */
 0x64 		/* maximum power (set at 100, change this value for your specific optics)*/
};

const unsigned char  device_desc_table[18] = 
{
    0x12,                       /*bLength */
    0x01, /*bDescriptorType*/
    0x00,                       /*bcdUSB */
    0x02,
    0x00,                       /*bDeviceClass*/
    0x00,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    0x40,                       /*bMaxPacketSize40*/
		0x11,						/*idVendor (0x0483)*/
		0x11,
		0x78,						/*idProduct = 0x5710*/
		0x56,

    0x00,                       /*bcdDevice rel. 2.00*/
    0x02,
    1,                          /*Index of string descriptor describing
                                                  manufacturer */
    2,                          /*Index of string descriptor describing
                                                 product*/
    3,                          /*Index of string descriptor describing the
                                                 device serial number */
    0x01                        /*bNumConfigurations*/
};


/* String Descriptors */
const unsigned char  USBStringLanguageDescription[4] = 
{
 0x04, 		/* Length */
 0x03, 		/* Type (3=string) */
 0x09, 		/* Language: English */
 0x04 		/* Sub-language: Default */
};

const unsigned char  USBStringDescription1[12] = 
{
	12,3,'M',0,'y',0,'U',0,'S',0,'B',0
};
const unsigned char  USBStringDescription2[12] = 
{
	12,3,'M',0,'y',0,'U',0,'S',0,'B',0
};

const unsigned char  USBStringDescription3[12] = 
{
	12,3,'M',0,'y',0,'U',0,'S',0,'B',0
};

