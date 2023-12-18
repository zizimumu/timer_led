#include "USB_Core.h"
#include "Descriptor.h"
#include "USB_Reg.h"
#include "Usart.h"

#define OK 0x01
#define NG 0x00

static SETUP_DATA g_Setup_Data;
static u8 g_Interface =0;
static u8 g_Config =0;
u8 g_Rev_Buf[10];


u16 g_Endp0_RxState=0;
u16 g_Endp0_TxState=0;
u8 g_Ctr_Event =0;
#define Send0LengthData() { _SetEPTxCount(ENDP0, 0); \
    						_SetEPTxStatus(ENDP0, EP_TX_VALID); }


void UserToPMABufferCopy(u8 *pbUsrBuf, u16 wPMABufAddr, u16 wNBytes)
{
  u32 n = (wNBytes + 1) >> 1;   /* n = (wNBytes + 1) / 2 */
  u32 i, temp1, temp2;
  u16 *pdwVal;
  pdwVal = (u16 *)(wPMABufAddr * 2 + PMAAddr);
  for (i = n; i != 0; i--)
  {
    temp1 = (u16) * pbUsrBuf;
    pbUsrBuf++;
    temp2 = temp1 | (u16) * pbUsrBuf << 8;
    *pdwVal++ = temp2;
    pdwVal++;
    pbUsrBuf++;
  }
}



void Get_Status()
{
	u8 *DataBuffer=0;
	u16 Length = 0;
	if(GET_DEV_STATUS == g_Setup_Data.USBbmRequestType)
	{
		DataBuffer = (u8 *)get_status_device_table;
		Length = LEN_STATUS;
	}
	else if(GET_INTERF_STATUS== g_Setup_Data.USBbmRequestType)
	{
		DataBuffer = (u8 *)get_status_interface_table;
		Length = LEN_STATUS;
	}
	else if(GET_INTERF_STATUS== g_Setup_Data.USBbmRequestType)
	{
		DataBuffer =(u8 *) get_status_endpoint_table;
		Length = LEN_STATUS;

	}
	else 
		return ;
	
	UserToPMABufferCopy(DataBuffer, GetEPTxAddr(ENDP0), Length);
    SetEPTxCount(ENDP0, Length);	


	g_Endp0_TxState = EP_TX_VALID;
	g_Endp0_RxState = EP_RX_VALID; 
}
void Set_Addr(void )
{
//	u8 addr =g_Setup_Data.USBwValues.bw.byte0;
//	SetDADDR(addr | DADDR_EF); /* set device address and enable function */
	
	Send0LengthData();


	g_Endp0_TxState = EP_TX_VALID;
	g_Endp0_RxState = EP_RX_VALID; 


	
}
void Set_Interface(void )
{
	if(g_Setup_Data.USBwValues.bw.byte0 != 0)
		g_Interface= 1;
	else
		g_Interface= 0;
	Send0LengthData();	

	g_Endp0_TxState = EP_TX_VALID;
	g_Endp0_RxState = EP_RX_VALID; 
}
void Get_Interface(void )
{
	u8 DataBuffer=g_Interface;
	u16 Length = 1;	
	
	UserToPMABufferCopy(&DataBuffer, GetEPTxAddr(ENDP0), Length);
    SetEPTxCount(ENDP0, Length);

	g_Endp0_TxState = EP_TX_VALID;
	g_Endp0_RxState = EP_RX_VALID; 
}

void Set_Config(void )
{
	if(g_Setup_Data.USBwIndexs.bw.byte0 != 0)
		g_Config= 1;
	else
		g_Config= 0;
	Send0LengthData();		

	g_Endp0_TxState = EP_TX_VALID;
	g_Endp0_RxState = EP_RX_VALID; 
}

void Get_Config(void )
{
	u8 DataBuffer;
	u16 Length = 1;	

	if( 0x00 == g_Config )
		g_Config =0x01;

	DataBuffer =g_Config;
	
	UserToPMABufferCopy(&DataBuffer, GetEPTxAddr(ENDP0), Length);
    SetEPTxCount(ENDP0, Length);


	g_Endp0_TxState = EP_TX_VALID;
	g_Endp0_RxState = EP_RX_VALID; 
}


void Get_Descriptor(void )
{
	u8 *DataBuffer=0;
	u16 Length = 0;

#ifdef Debug
	uart_send_byte(g_Setup_Data.USBwValues.bw.byte1);
#endif

	
	if(DEV_DESCRIPTOR == g_Setup_Data.USBwValues.bw.byte1)
	{
		DataBuffer = (u8 *)device_desc_table;
		Length = LEN_DEV_DESCRIPTOR;
	}
	else if (CONFIG_DESCRIPTOR == g_Setup_Data.USBwValues.bw.byte1)
	{

		if(LEN_CONFIG_DESCRIPTOR == g_Setup_Data.USBwLengths.bw.byte0)	//only get config desc
		{
			DataBuffer = (u8 *)config_desc_table;
			Length = LEN_INTERF_DESCRIPTOR;
		}
		// get all config desc
		else  
		{
			DataBuffer = (u8 *)Conf_Des_All;
			Length = LEN_CONFIG_ALL_DESCRIPTOR;
		}
		
	}
	else if(STRING_DESCRIPTOR == g_Setup_Data.USBwValues.bw.byte1)
	{
		if(0x00 == g_Setup_Data.USBwValues.bw.byte0)				/* USBStringLanguageDescription*/
		{
			DataBuffer = (u8 *)USBStringLanguageDescription;
			Length = LEN_STRING_LANG_DESCRIPTOR;
		}
		else if(0x01 == g_Setup_Data.USBwValues.bw.byte0)
		{

			DataBuffer = (u8 *)USBStringDescription1;
			Length = LEN_STRING1_DESCRIPTOR;
		}
		else if (0x02 == g_Setup_Data.USBwValues.bw.byte0)
		{
			DataBuffer = (u8 *)USBStringDescription2;
			Length = LEN_STRING2_DESCRIPTOR;

		}
		else if (0x03 == g_Setup_Data.USBwValues.bw.byte0)
		{
			DataBuffer = (u8 *)USBStringDescription3;
			Length = LEN_STRING3_DESCRIPTOR;

		}
		else{}
	}
	else if(INTERF_DESCRIPTOR == g_Setup_Data.USBwValues.bw.byte1)
	{
		DataBuffer = (u8 *)Interface_Descriptor;
		Length = LEN_INTERF_DESCRIPTOR;	
	}
/*
	else if(HID_CLASS_DESCRIPTOR == g_Setup_Data.USBwValues.bw.byte1)
	{
		DataBuffer = (u8 *)Class_Descriptor;
		Length = LEN_CLASS_DESCRIPTOR;	
	}

	else if(HID_REPORT_DESCRIPTOR == g_Setup_Data.USBwValues.bw.byte1)
	{
		DataBuffer = (u8 *)hid_report_desc_table;
		Length = LEN_HID_REPORT_DESCRIPTOR;

	}
*/
	else 
	{
		return ;
	}
	
	UserToPMABufferCopy(DataBuffer, GetEPTxAddr(ENDP0), Length);
    SetEPTxCount(ENDP0, Length);

	g_Endp0_RxState = EP_RX_NAK ;
	g_Endp0_TxState = EP_TX_VALID;

	SetEPRxCount(ENDP0,0x40);
	
}
void  Stand_Request_Process(void )
{

	switch(g_Setup_Data.USBbRequest)
	{
		case GET_STATUS:
			Get_Status();
			break;
		case CLEAR_FEATURE:
			break;
		case SET_FEATURE:
			break;
		case SET_ADDRESS:
			Set_Addr();
			break;
		case GET_DESCRIPTOR:					//get descriptor
			Get_Descriptor();
			break;
		case SET_DESCRIPTOR:
			break;
		case GET_CONFIGURATION:
			Get_Config();
			break;
		case SET_CONFIGURATION:
			Set_Config();
			break ;
		case GET_INTERFACE:
			Get_Interface();
			break;
		case SET_INTERFACE:
			Set_Interface();
			break;
		default:
			break;
				
	}
}
void Setup_Process(u8 * RecBbuf)
{
	u8 Request_Type =0;
	u8 *p_tmp=RecBbuf;
	g_Setup_Data.USBbmRequestType = *RecBbuf++;
	g_Setup_Data.USBbRequest = *RecBbuf++;
	RecBbuf+=2;
	g_Setup_Data.USBwValues.bw.byte0 =  *RecBbuf++;
	g_Setup_Data.USBwValues.bw.byte1 =  *RecBbuf++;
	RecBbuf+=2;
	g_Setup_Data.USBwIndexs.bw.byte0 =  *RecBbuf++;
	g_Setup_Data.USBwIndexs.bw.byte1 =  *RecBbuf++;
	RecBbuf+=2;
	g_Setup_Data.USBwLengths.bw.byte0 =  *RecBbuf++;
	g_Setup_Data.USBwLengths.bw.byte1 =  *RecBbuf;


	g_Ctr_Event = OK;

	Request_Type =(REQUEST_TYPE_MASK & g_Setup_Data.USBbmRequestType);
	if( Request_Type ==STAND_REQUEST )							   //USB standard requst
	{

	   Stand_Request_Process();
	}
	else if( Request_Type ==DEV_CLASS_REQUEST )	
	{
		
	}
	else if(Request_Type == VENDOR_REQUEST )
	{
		
	}
	else{}							

}

void In_Processs(void )
{
	

	
	u8 addr;
	if(SET_ADDRESS ==  g_Setup_Data.USBbRequest)
	{
		addr = g_Setup_Data.USBwValues.bw.byte0;
		SetDADDR(addr | DADDR_EF); /* set device address and enable function */
	}
	else{
		if(SET_CONFIGURATION == g_Setup_Data.USBbRequest)
		{
			//g_USB_Dev_Status = OK;
		}
	}
	
	g_Endp0_RxState = EP_RX_VALID;
	g_Endp0_TxState = EP_TX_VALID;
}

void Out_Process(void )
{
	//Send0LengthData();
	unsigned char * pBuf ;
	u8 i=0;
	u16 rev_count=0;
	g_Endp0_RxState = EP_RX_STALL;
	g_Endp0_TxState = EP_TX_STALL;


/*	if(g_Ctr_Event == NG)    		//host send data to device 
	{
		pBuf = PMAAddr + (u8 *)(_GetEPRxAddr(ENDP0) * 2);
		rev_count  = (u16)*_pEPRxCount(0x00);
		if(rev_count>10)
			rev_count = 10;
		for(i=0;i<(rev_count/2+rev_count%2);)
		{
			g_Rev_Buf[i] = *pBuf;
			g_Rev_Buf[i+1] = *(pBuf+1);
				pBuf +=4;
		}
		uart_send_buf(g_Rev_Buf,10);
		
	}
	g_Ctr_Event = NG;

	*/
}
