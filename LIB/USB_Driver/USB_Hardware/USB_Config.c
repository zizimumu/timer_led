#include <stm32f10x_lib.h>

#include "USB_Config.h"
#include "sys.h"
#include "USB_Reg.h"


//����USBʱ��,USBclk=48Mhz
void Set_USBClock(void)
{
 	RCC->CFGR&=~(1<<22); //USBclk=PLLclk/1.5=48Mhz	    
	RCC->APB1ENR|=1<<23; //USBʱ��ʹ��					 
}

//USB�ж�����
void USB_Interrupts_Config(void )
{
  
	EXTI->IMR|=1<<18;//  ������18�ϵ��ж�
 	EXTI->RTSR|=1<<18;//line 18���¼��������ش���	 
	MY_NVIC_Init(1,0,USB_LP_CAN_RX0_IRQChannel,2);//��2�����ȼ���֮ 
	MY_NVIC_Init(0,0,USBWakeUp_IRQChannel,2);     //��2�����ȼ����	 	 
}



void  PowerOn(void )
{
  u16 wRegVal;
  u16 	wInterrupt_Mask;				   

  wRegVal = CNTR_FRES;
  _SetCNTR(wRegVal);	 
  wInterrupt_Mask = 0;
  
  _SetCNTR(wInterrupt_Mask);
  _SetISTR(0);												 /*** Clear pending interrupts ***/
  
  wInterrupt_Mask =CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM;	/*** Set interrupt mask ***/
  _SetCNTR(wInterrupt_Mask);	

  
  _SetISTR(0);               								/* clear pending interrupts */
  wInterrupt_Mask = IMR_MSK;
  _SetCNTR(wInterrupt_Mask); 								/* set interrupts mask */

}
void SetEndpAddr(u8 EndpNum)
{
	u8 i;
	for (i = 0; i < EndpNum; i++)
  	{
    _SetEPAddress((u8)i, (u8)i);
  	} 
}

void USB_Reset(void )
{


   	 SetBTABLE(BTABLE_ADDRESS);
	 // Initialize Endpoint 0 
	 SetEPType(ENDP0, EP_CONTROL);
	 
	 SetEPTxStatus(ENDP0, EP_TX_STALL);
	 
	 SetEPRxAddr(ENDP0, ENDP0_RXADDR);
	 SetEPTxAddr(ENDP0, ENDP0_TXADDR);
	 Clear_Status_Out(ENDP0);
	 SetEPRxCount(ENDP0, ENDP0_MAX_PACKET_SIZE);


	 //SetEPRxValid(ENDP0);
	 SetEPRxStatus(ENDP0, EP_RX_VALID);
	 SetEPTxStatus(ENDP0, EP_TX_STALL);
	 
	 // Initialize Endpoint 1 
	 SetEPType(ENDP1, EP_INTERRUPT);
	 SetEPTxAddr(ENDP1, ENDP1_TXADDR);
	 SetEPTxCount(ENDP1, ENDP0_MAX_PACKET_SIZE);
	 SetEPRxStatus(ENDP1, EP_RX_DIS);
	 SetEPTxStatus(ENDP1, EP_TX_VALID);


	 // Initialize Endpoint 2 
	 SetEPType(ENDP2, EP_INTERRUPT);
	 SetEPRxAddr(ENDP2, ENDP2_RXADDR);
	 SetEPRxCount(ENDP2, ENDP0_MAX_PACKET_SIZE);
	 SetEPRxStatus(ENDP2, EP_RX_VALID);
	 SetEPTxStatus(ENDP2, EP_TX_DIS);

	
	 SetEndpAddr(3);			//number of endpoint 3
	
	 SetDADDR(0x00 | DADDR_EF); // set device address and enable function 
	
}

void USB_Init(void )
{
	PowerOn();


/*
    SetBTABLE(BTABLE_ADDRESS);
	// Initialize Endpoint 0 
  	SetEPType(ENDP0, EP_CONTROL);
    SetEPTxStatus(ENDP0, EP_TX_STALL);
  	SetEPRxAddr(ENDP0, ENDP0_RXADDR);
	SetEPTxAddr(ENDP0, ENDP0_TXADDR);
  	Clear_Status_Out(ENDP0);
  	SetEPRxCount(ENDP0, ENDP0_MAX_PACKET_SIZE);
  	SetEPRxValid(ENDP0);
	
	// Initialize Endpoint 1 
	SetEPType(ENDP1, EP_INTERRUPT);
  	SetEPTxAddr(ENDP1, ENDP1_TXADDR);
  	SetEPTxCount(ENDP1, 4);
  	SetEPRxStatus(ENDP1, EP_RX_DIS);
  	SetEPTxStatus(ENDP1, EP_TX_NAK);

	SetEndpAddr(2);

	SetDADDR(0x00 | DADDR_EF); // set device address and enable function 

*/
}
