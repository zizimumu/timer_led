#include "stm32f10x_lib.h"
#include "USB_Reg.h"
#include "USB_Core.h"
#include "Usart.h"
#include "USB_Config.h"





void CTR_LP(void )
{
	u16 istr = 0;
	u32 wEPVal=0;
	u8 endpoint=0;
	u8 *pBuf=0;

	while(((istr = _GetISTR()) & ISTR_CTR) != 0){

	
		_SetISTR((u16)CLR_CTR); 					/* clear CTR flag  why? */
		endpoint = (u8)(istr & ISTR_EP_ID);
		if( ENDP0 == endpoint)	  						// endpoint 0
		{
			_SetEPRxStatus(ENDP0, EP_RX_NAK);
      		_SetEPTxStatus(ENDP0, EP_TX_NAK);

			if ((istr & ISTR_DIR) == 0)	 			// IN
			{
		
				_ClearEP_CTR_TX(ENDP0);
				In_Processs();
				_SetEPRxStatus(ENDP0, g_Endp0_RxState);
          		_SetEPTxStatus(ENDP0, g_Endp0_TxState);
			}
			else   									// OUT or SETUP
			{
				wEPVal = _GetENDPOINT(ENDP0);
				if ((wEPVal &EP_SETUP) != 0)		// setup  event
 		        {

          			_ClearEP_CTR_RX(ENDP0);	
					pBuf = PMAAddr + (u8 *)(_GetEPRxAddr(ENDP0) * 2); /* addr of reciver buf */		
          			Setup_Process(pBuf);
          			/* before terminate set Tx & Rx status */
          			_SetEPRxStatus(ENDP0, g_Endp0_RxState);
          			_SetEPTxStatus(ENDP0, g_Endp0_TxState);
          			return;
        		}
       		    else 								// Out event
        		{

					_ClearEP_CTR_RX(ENDP0);
					Out_Process();					
					_SetEPRxStatus(ENDP0, g_Endp0_RxState);
					_SetEPTxStatus(ENDP0, g_Endp0_TxState);

        		}

			}
		}
		else  if (ENDP1 == endpoint )	   				// endpoint 1
		{
			   
		      wEPVal = _GetENDPOINT(0x01);
		      if ((wEPVal & EP_CTR_RX) != 0)
		      {
		        _ClearEP_CTR_RX(ENDP1);
				//uart_send_byte(0x67);

		      } 
		      if ((wEPVal & EP_CTR_TX) != 0)
		      {
		      	//uart_send_byte(0x66);
		        _ClearEP_CTR_TX(ENDP1);
		      }

		}
		else if(ENDP2 == endpoint )					// endpoint 2
		{
			wEPVal = _GetENDPOINT(ENDP2);	
			if ((wEPVal & EP_CTR_RX) != 0)
			{


			
			  _ClearEP_CTR_RX(ENDP2);
			  _SetEPRxStatus(ENDP2, EP_RX_VALID);
			
			} 
			if ((wEPVal & EP_CTR_TX) != 0)
			{
			  _ClearEP_CTR_TX(ENDP2);
			}
		}
	}	
}



/*******************************************************************************
* Function Name  : USB_Istr
* Description    : STR events interrupt service routine
* Input          :
* Output         :
* Return         :
*******************************************************************************/
void USB_Istr(void)
{	
  u16 istr = 0;

  istr = _GetISTR();

  if (istr & ISTR_RESET)
  {
    _SetISTR((u16)CLR_RESET);
    USB_Reset();

  }
  else if (istr & ISTR_DOVR)
  {
    _SetISTR((u16)CLR_DOVR);

  }
  else if (istr & ISTR_ERR)
  {

    _SetISTR((u16)CLR_ERR);

  }
  else if (istr & ISTR_WKUP )
  {
    _SetISTR((u16)CLR_WKUP);

  }
  else if (istr & ISTR_SUSP)
  {
    _SetISTR((u16)CLR_SUSP);

  }
  else if (istr & ISTR_SOF )
  {
    _SetISTR((u16)CLR_SOF);
  }
  else if (istr & ISTR_ESOF )
  {
    _SetISTR((u16)CLR_ESOF);

  }
  else if (istr & ISTR_CTR)
  {
    CTR_LP();
  }
  else 
  {}

} 
