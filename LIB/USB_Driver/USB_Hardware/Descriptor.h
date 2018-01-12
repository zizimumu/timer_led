#ifndef _DESCRIPTOR_H

#define _DESCRIPTOR_H


#define LEN_DEV_DESCRIPTOR 				0x12
#define LEN_CONFIG_DESCRIPTOR 			0x09
#define LEN_INTERF_DESCRIPTOR 			0x09
#define LEN_CONFIG_ALL_DESCRIPTOR 		0x19
#define LEN_HID_REPORT_DESCRIPTOR 		0x1b
#define LEN_ENDP_DESCRIPTOR 			0x07
#define LEN_CLASS_DESCRIPTOR 			0x09

#define LEN_STRING_LANG_DESCRIPTOR 		0x04
#define LEN_STRING1_DESCRIPTOR 			0x0c
#define LEN_STRING2_DESCRIPTOR 			0x0c
#define LEN_STRING3_DESCRIPTOR 			0x0c

#define LEN_STATUS						0x02

 extern const unsigned char    get_status_device_table[ 2];
 extern const unsigned char    get_status_interface_table[2];
 extern const unsigned char    get_status_endpoint_table[2];
 
 extern const unsigned char  hid_report_desc_table[27];
 extern const unsigned char Conf_Des_All[LEN_CONFIG_ALL_DESCRIPTOR];
 extern const unsigned char Endpoint_Descriptor[7];
 extern const unsigned char Class_Descriptor[9];
 extern const unsigned char  Interface_Descriptor[9];
 extern const unsigned char  config_desc_table[9];
 extern const unsigned char  device_desc_table[18];
 extern const unsigned char  USBStringLanguageDescription[4];
 extern const unsigned char  USBStringDescription1[12];
 extern const unsigned char  USBStringDescription2[12];
 extern const unsigned char  USBStringDescription3[12];


 #endif

