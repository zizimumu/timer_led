
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_usart.h"
//#include "stm32f10x_rcc.h"
#include "stdio.h"

//#include "stm32f10x_dma.h"


#define CR1_UE_Set                ((uint16_t)0x2000)  /*!< USART Enable Mask */
#define CR1_UE_Reset              ((uint16_t)0xDFFF)  /*!< USART Disable Mask */

#define CR1_WAKE_Mask             ((uint16_t)0xF7FF)  /*!< USART WakeUp Method Mask */

#define CR1_RWU_Set               ((uint16_t)0x0002)  /*!< USART mute mode Enable Mask */
#define CR1_RWU_Reset             ((uint16_t)0xFFFD)  /*!< USART mute mode Enable Mask */
#define CR1_SBK_Set               ((uint16_t)0x0001)  /*!< USART Break Character send Mask */
#define CR1_CLEAR_Mask            ((uint16_t)0xE9F3)  /*!< USART CR1 Mask */
#define CR2_Address_Mask          ((uint16_t)0xFFF0)  /*!< USART address Mask */

#define CR2_LINEN_Set              ((uint16_t)0x4000)  /*!< USART LIN Enable Mask */
#define CR2_LINEN_Reset            ((uint16_t)0xBFFF)  /*!< USART LIN Disable Mask */

#define CR2_LBDL_Mask             ((uint16_t)0xFFDF)  /*!< USART LIN Break detection Mask */
#define CR2_STOP_CLEAR_Mask       ((uint16_t)0xCFFF)  /*!< USART CR2 STOP Bits Mask */
#define CR2_CLOCK_CLEAR_Mask      ((uint16_t)0xF0FF)  /*!< USART CR2 Clock Mask */

#define CR3_SCEN_Set              ((uint16_t)0x0020)  /*!< USART SC Enable Mask */
#define CR3_SCEN_Reset            ((uint16_t)0xFFDF)  /*!< USART SC Disable Mask */

#define CR3_NACK_Set              ((uint16_t)0x0010)  /*!< USART SC NACK Enable Mask */
#define CR3_NACK_Reset            ((uint16_t)0xFFEF)  /*!< USART SC NACK Disable Mask */

#define CR3_HDSEL_Set             ((uint16_t)0x0008)  /*!< USART Half-Duplex Enable Mask */
#define CR3_HDSEL_Reset           ((uint16_t)0xFFF7)  /*!< USART Half-Duplex Disable Mask */

#define CR3_IRLP_Mask             ((uint16_t)0xFFFB)  /*!< USART IrDA LowPower mode Mask */
#define CR3_CLEAR_Mask            ((uint16_t)0xFCFF)  /*!< USART CR3 Mask */

#define CR3_IREN_Set              ((uint16_t)0x0002)  /*!< USART IrDA Enable Mask */
#define CR3_IREN_Reset            ((uint16_t)0xFFFD)  /*!< USART IrDA Disable Mask */
#define GTPR_LSB_Mask             ((uint16_t)0x00FF)  /*!< Guard Time Register LSB Mask */
#define GTPR_MSB_Mask             ((uint16_t)0xFF00)  /*!< Guard Time Register MSB Mask */
#define IT_Mask                   ((uint16_t)0x001F)  /*!< USART Interrupt Mask */



/*************************self define *******************************/
//STM32F103RB 3 uart
#define USART_DMA_EN 0

#if USART_DMA_EN

#define DBG_COM USART1
#define USART_SEND_LEN  512
unsigned char usartDCMSendBuf[USART_SEND_LEN];
unsigned int g_DMASendLenTotal = 0;
unsigned int g_DMASendLenTotal_Next = 0;


#endif


GPIO_TypeDef * COM_IO_PORT[3] = {GPIOA,GPIOA,GPIOB};
const uint16_t COM_TX_PIN[3] = {GPIO_Pin_9,GPIO_Pin_2,GPIO_Pin_10};
const uint16_t COM_RX_PIN[3] = {GPIO_Pin_10,GPIO_Pin_3,GPIO_Pin_11};
USART_TypeDef *  COM_USART[3] = {USART1,USART2,USART3};
IRQn_Type  COM_IRQn[2] = {USART1_IRQn,USART2_IRQn};
const uint32_t  COM_RCC_CLOCK[3] = {RCC_APB2Periph_GPIOA,RCC_APB2Periph_GPIOA,RCC_APB2Periph_GPIOB};






/****************************************************************/

/**
  * @}
  */

/** @defgroup USART_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USART_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup USART_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup USART_Private_Functions
  * @{
  */

/**
  * @brief  Deinitializes the USARTx peripheral registers to their default reset values.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values: USART1, USART2, USART3, UART4 or UART5.
  * @retval None
  */
void USART_DeInit(USART_TypeDef* USARTx)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));

  if (USARTx == USART1)
  {
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);
  }
  else if (USARTx == USART2)
  {
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE);
  }
  else if (USARTx == USART3)
  {
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, DISABLE);
  }    
  else if (USARTx == UART4)
  {
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4, DISABLE);
  }    
  else
  {
    if (USARTx == UART5)
    { 
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART5, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART5, DISABLE);
    }
  }
}

/**
  * @brief  Initializes the USARTx peripheral according to the specified
  *   parameters in the USART_InitStruct .
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure
  *   that contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct)
{
  uint32_t tmpreg = 0x00, apbclock = 0x00;
  uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
  uint32_t usartxbase = 0;
  RCC_ClocksTypeDef RCC_ClocksStatus;
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_BAUDRATE(USART_InitStruct->USART_BaudRate));  
  assert_param(IS_USART_WORD_LENGTH(USART_InitStruct->USART_WordLength));
  assert_param(IS_USART_STOPBITS(USART_InitStruct->USART_StopBits));
  assert_param(IS_USART_PARITY(USART_InitStruct->USART_Parity));
  assert_param(IS_USART_MODE(USART_InitStruct->USART_Mode));
  assert_param(IS_USART_HARDWARE_FLOW_CONTROL(USART_InitStruct->USART_HardwareFlowControl));
  /* The hardware flow control is available only for USART1, USART2 and USART3 */
  if (USART_InitStruct->USART_HardwareFlowControl != USART_HardwareFlowControl_None)
  {
    assert_param(IS_USART_123_PERIPH(USARTx));
  }

  usartxbase = (uint32_t)USARTx;

/*---------------------------- USART CR2 Configuration -----------------------*/
  tmpreg = USARTx->CR2;
  /* Clear STOP[13:12] bits */
  tmpreg &= CR2_STOP_CLEAR_Mask;
  /* Configure the USART Stop Bits, Clock, CPOL, CPHA and LastBit ------------*/
  /* Set STOP[13:12] bits according to USART_StopBits value */
  tmpreg |= (uint32_t)USART_InitStruct->USART_StopBits;
  
  /* Write to USART CR2 */
  USARTx->CR2 = (uint16_t)tmpreg;

/*---------------------------- USART CR1 Configuration -----------------------*/
  tmpreg = USARTx->CR1;
  /* Clear M, PCE, PS, TE and RE bits */
  tmpreg &= CR1_CLEAR_Mask;
  /* Configure the USART Word Length, Parity and mode ----------------------- */
  /* Set the M bits according to USART_WordLength value */
  /* Set PCE and PS bits according to USART_Parity value */
  /* Set TE and RE bits according to USART_Mode value */
  tmpreg |= (uint32_t)USART_InitStruct->USART_WordLength | USART_InitStruct->USART_Parity |
            USART_InitStruct->USART_Mode;
  /* Write to USART CR1 */
  USARTx->CR1 = (uint16_t)tmpreg;

/*---------------------------- USART CR3 Configuration -----------------------*/  
  tmpreg = USARTx->CR3;
  /* Clear CTSE and RTSE bits */
  tmpreg &= CR3_CLEAR_Mask;
  /* Configure the USART HFC -------------------------------------------------*/
  /* Set CTSE and RTSE bits according to USART_HardwareFlowControl value */
  tmpreg |= USART_InitStruct->USART_HardwareFlowControl;
  /* Write to USART CR3 */
  USARTx->CR3 = (uint16_t)tmpreg;

/*---------------------------- USART BRR Configuration -----------------------*/
  /* Configure the USART Baud Rate -------------------------------------------*/
  RCC_GetClocksFreq(&RCC_ClocksStatus);
  if (usartxbase == USART1_BASE)
  {
    apbclock = RCC_ClocksStatus.PCLK2_Frequency;
  }
  else
  {
    apbclock = RCC_ClocksStatus.PCLK1_Frequency;
  }
  /* Determine the integer part */
  integerdivider = ((0x19 * apbclock) / (0x04 * (USART_InitStruct->USART_BaudRate)));
  tmpreg = (integerdivider / 0x64) << 0x04;
  /* Determine the fractional part */
  fractionaldivider = integerdivider - (0x64 * (tmpreg >> 0x04));
  tmpreg |= ((((fractionaldivider * 0x10) + 0x32) / 0x64)) & ((uint8_t)0x0F);
  /* Write to USART BRR */
  USARTx->BRR = (uint16_t)tmpreg;
}

/**
  * @brief  Fills each USART_InitStruct member with its default value.
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure
  *   which will be initialized.
  * @retval None
  */
void USART_StructInit(USART_InitTypeDef* USART_InitStruct)
{
  /* USART_InitStruct members default value */
  USART_InitStruct->USART_BaudRate = 9600;
  USART_InitStruct->USART_WordLength = USART_WordLength_8b;
  USART_InitStruct->USART_StopBits = USART_StopBits_1;
  USART_InitStruct->USART_Parity = USART_Parity_No ;
  USART_InitStruct->USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStruct->USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
}

/**
  * @brief  Initializes the USARTx peripheral Clock according to the 
  *   specified parameters in the USART_ClockInitStruct .
  * @param  USARTx: where x can be 1, 2, 3 to select the USART peripheral.
  * @param  USART_ClockInitStruct: pointer to a USART_ClockInitTypeDef
  *   structure that contains the configuration information for the specified 
  *   USART peripheral.  
  * @note The Smart Card mode is not available for UART4 and UART5.
  * @retval None
  */
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct)
{
  uint32_t tmpreg = 0x00;
  /* Check the parameters */
  assert_param(IS_USART_123_PERIPH(USARTx));
  assert_param(IS_USART_CLOCK(USART_ClockInitStruct->USART_Clock));
  assert_param(IS_USART_CPOL(USART_ClockInitStruct->USART_CPOL));
  assert_param(IS_USART_CPHA(USART_ClockInitStruct->USART_CPHA));
  assert_param(IS_USART_LASTBIT(USART_ClockInitStruct->USART_LastBit));
  
/*---------------------------- USART CR2 Configuration -----------------------*/
  tmpreg = USARTx->CR2;
  /* Clear CLKEN, CPOL, CPHA and LBCL bits */
  tmpreg &= CR2_CLOCK_CLEAR_Mask;
  /* Configure the USART Clock, CPOL, CPHA and LastBit ------------*/
  /* Set CLKEN bit according to USART_Clock value */
  /* Set CPOL bit according to USART_CPOL value */
  /* Set CPHA bit according to USART_CPHA value */
  /* Set LBCL bit according to USART_LastBit value */
  tmpreg |= (uint32_t)USART_ClockInitStruct->USART_Clock | USART_ClockInitStruct->USART_CPOL | 
                 USART_ClockInitStruct->USART_CPHA | USART_ClockInitStruct->USART_LastBit;
  /* Write to USART CR2 */
  USARTx->CR2 = (uint16_t)tmpreg;
}

/**
  * @brief  Fills each USART_ClockInitStruct member with its default value.
  * @param  USART_ClockInitStruct: pointer to a USART_ClockInitTypeDef
  *   structure which will be initialized.
  * @retval None
  */
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct)
{
  /* USART_ClockInitStruct members default value */
  USART_ClockInitStruct->USART_Clock = USART_Clock_Disable;
  USART_ClockInitStruct->USART_CPOL = USART_CPOL_Low;
  USART_ClockInitStruct->USART_CPHA = USART_CPHA_1Edge;
  USART_ClockInitStruct->USART_LastBit = USART_LastBit_Disable;
}

/**
  * @brief  Enables or disables the specified USART peripheral.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USARTx peripheral.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected USART by setting the UE bit in the CR1 register */
    USARTx->CR1 |= CR1_UE_Set;
  }
  else
  {
    /* Disable the selected USART by clearing the UE bit in the CR1 register */
    USARTx->CR1 &= CR1_UE_Reset;
  }
}

/**
  * @brief  Enables or disables the specified USART interrupts.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IT: specifies the USART interrupt sources to be enabled or disabled.
  *   This parameter can be one of the following values:
  *     @arg USART_IT_CTS:  CTS change interrupt (not available for UART4 and UART5)
  *     @arg USART_IT_LBD:  LIN Break detection interrupt
  *     @arg USART_IT_TXE:  Tansmit Data Register empty interrupt
  *     @arg USART_IT_TC:   Transmission complete interrupt
  *     @arg USART_IT_RXNE: Receive Data register not empty interrupt
  *     @arg USART_IT_IDLE: Idle line detection interrupt
  *     @arg USART_IT_PE:   Parity Error interrupt
  *     @arg USART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @param  NewState: new state of the specified USARTx interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState)
{
  uint32_t usartreg = 0x00, itpos = 0x00, itmask = 0x00;
  uint32_t usartxbase = 0x00;
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_CONFIG_IT(USART_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  /* The CTS interrupt is not available for UART4 and UART5 */
  if (USART_IT == USART_IT_CTS)
  {
    assert_param(IS_USART_123_PERIPH(USARTx));
  }   
  
  usartxbase = (uint32_t)USARTx;

  /* Get the USART register index */
  usartreg = (((uint8_t)USART_IT) >> 0x05);

  /* Get the interrupt position */
  itpos = USART_IT & IT_Mask;
  itmask = (((uint32_t)0x01) << itpos);
    
  if (usartreg == 0x01) /* The IT is in CR1 register */
  {
    usartxbase += 0x0C;
  }
  else if (usartreg == 0x02) /* The IT is in CR2 register */
  {
    usartxbase += 0x10;
  }
  else /* The IT is in CR3 register */
  {
    usartxbase += 0x14; 
  }
  if (NewState != DISABLE)
  {
    *(__IO uint32_t*)usartxbase  |= itmask;
  }
  else
  {
    *(__IO uint32_t*)usartxbase &= ~itmask;
  }
}

/**
  * @brief  Enables or disables the USART’s DMA interface.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3 or UART4.  
  * @param  USART_DMAReq: specifies the DMA request.
  *   This parameter can be any combination of the following values:
  *     @arg USART_DMAReq_Tx: USART DMA transmit request
  *     @arg USART_DMAReq_Rx: USART DMA receive request
  * @param  NewState: new state of the DMA Request sources.
  *   This parameter can be: ENABLE or DISABLE.
  * @note The DMA mode is not available for UART5.  
  * @retval None
  */
void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_1234_PERIPH(USARTx));
  assert_param(IS_USART_DMAREQ(USART_DMAReq));  
  assert_param(IS_FUNCTIONAL_STATE(NewState)); 
  if (NewState != DISABLE)
  {
    /* Enable the DMA transfer for selected requests by setting the DMAT and/or
       DMAR bits in the USART CR3 register */
    USARTx->CR3 |= USART_DMAReq;
  }
  else
  {
    /* Disable the DMA transfer for selected requests by clearing the DMAT and/or
       DMAR bits in the USART CR3 register */
    USARTx->CR3 &= (uint16_t)~USART_DMAReq;
  }
}

/**
  * @brief  Sets the address of the USART node.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_Address: Indicates the address of the USART node.
  * @retval None
  */
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_ADDRESS(USART_Address)); 
    
  /* Clear the USART address */
  USARTx->CR2 &= CR2_Address_Mask;
  /* Set the USART address node */
  USARTx->CR2 |= USART_Address;
}

/**
  * @brief  Selects the USART WakeUp method.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_WakeUp: specifies the USART wakeup method.
  *   This parameter can be one of the following values:
  *     @arg USART_WakeUp_IdleLine: WakeUp by an idle line detection
  *     @arg USART_WakeUp_AddressMark: WakeUp by an address mark
  * @retval None
  */
void USART_WakeUpConfig(USART_TypeDef* USARTx, uint16_t USART_WakeUp)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_WAKEUP(USART_WakeUp));
  
  USARTx->CR1 &= CR1_WAKE_Mask;
  USARTx->CR1 |= USART_WakeUp;
}

/**
  * @brief  Determines if the USART is in mute mode or not.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART mute mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState)); 
  
  if (NewState != DISABLE)
  {
    /* Enable the USART mute mode  by setting the RWU bit in the CR1 register */
    USARTx->CR1 |= CR1_RWU_Set;
  }
  else
  {
    /* Disable the USART mute mode by clearing the RWU bit in the CR1 register */
    USARTx->CR1 &= CR1_RWU_Reset;
  }
}

/**
  * @brief  Sets the USART LIN Break detection length.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_LINBreakDetectLength: specifies the LIN break detection length.
  *   This parameter can be one of the following values:
  *     @arg USART_LINBreakDetectLength_10b: 10-bit break detection
  *     @arg USART_LINBreakDetectLength_11b: 11-bit break detection
  * @retval None
  */
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, uint16_t USART_LINBreakDetectLength)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_LIN_BREAK_DETECT_LENGTH(USART_LINBreakDetectLength));
  
  USARTx->CR2 &= CR2_LBDL_Mask;
  USARTx->CR2 |= USART_LINBreakDetectLength;  
}

/**
  * @brief  Enables or disables the USART’s LIN mode.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART LIN mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the LIN mode by setting the LINEN bit in the CR2 register */
    USARTx->CR2 |= CR2_LINEN_Set;
  }
  else
  {
    /* Disable the LIN mode by clearing the LINEN bit in the CR2 register */
    USARTx->CR2 &= CR2_LINEN_Reset;
  }
}

/**
  * @brief  Transmits single data through the USARTx peripheral.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  Data: the data to transmit.
  * @retval None
  */
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_DATA(Data)); 
    
  /* Transmit Data */
  USARTx->DR = (Data & (uint16_t)0x01FF);
}

/**
  * @brief  Returns the most recent received data by the USARTx peripheral.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @retval The received data.
  */
uint16_t USART_ReceiveData(USART_TypeDef* USARTx)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  
  /* Receive Data */
  return (uint16_t)(USARTx->DR & (uint16_t)0x01FF);
}

/**
  * @brief  Transmits break characters.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @retval None
  */
void USART_SendBreak(USART_TypeDef* USARTx)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  
  /* Send break characters */
  USARTx->CR1 |= CR1_SBK_Set;
}

/**
  * @brief  Sets the specified USART guard time.
  * @param  USARTx: where x can be 1, 2 or 3 to select the USART peripheral.
  * @param  USART_GuardTime: specifies the guard time.
  * @note The guard time bits are not available for UART4 and UART5.   
  * @retval None
  */
void USART_SetGuardTime(USART_TypeDef* USARTx, uint8_t USART_GuardTime)
{    
  /* Check the parameters */
  assert_param(IS_USART_123_PERIPH(USARTx));
  
  /* Clear the USART Guard time */
  USARTx->GTPR &= GTPR_LSB_Mask;
  /* Set the USART guard time */
  USARTx->GTPR |= (uint16_t)((uint16_t)USART_GuardTime << 0x08);
}

/**
  * @brief  Sets the system clock prescaler.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_Prescaler: specifies the prescaler clock.  
  * @note   The function is used for IrDA mode with UART4 and UART5.
  * @retval None
  */
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler)
{ 
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  
  /* Clear the USART prescaler */
  USARTx->GTPR &= GTPR_MSB_Mask;
  /* Set the USART prescaler */
  USARTx->GTPR |= USART_Prescaler;
}

/**
  * @brief  Enables or disables the USART’s Smart Card mode.
  * @param  USARTx: where x can be 1, 2 or 3 to select the USART peripheral.
  * @param  NewState: new state of the Smart Card mode.
  *   This parameter can be: ENABLE or DISABLE.     
  * @note The Smart Card mode is not available for UART4 and UART5. 
  * @retval None
  */
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_123_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the SC mode by setting the SCEN bit in the CR3 register */
    USARTx->CR3 |= CR3_SCEN_Set;
  }
  else
  {
    /* Disable the SC mode by clearing the SCEN bit in the CR3 register */
    USARTx->CR3 &= CR3_SCEN_Reset;
  }
}

/**
  * @brief  Enables or disables NACK transmission.
  * @param  USARTx: where x can be 1, 2 or 3 to select the USART peripheral. 
  * @param  NewState: new state of the NACK transmission.
  *   This parameter can be: ENABLE or DISABLE.  
  * @note The Smart Card mode is not available for UART4 and UART5.
  * @retval None
  */
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_123_PERIPH(USARTx));  
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the NACK transmission by setting the NACK bit in the CR3 register */
    USARTx->CR3 |= CR3_NACK_Set;
  }
  else
  {
    /* Disable the NACK transmission by clearing the NACK bit in the CR3 register */
    USARTx->CR3 &= CR3_NACK_Reset;
  }
}

/**
  * @brief  Enables or disables the USART’s Half Duplex communication.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART Communication.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the Half-Duplex mode by setting the HDSEL bit in the CR3 register */
    USARTx->CR3 |= CR3_HDSEL_Set;
  }
  else
  {
    /* Disable the Half-Duplex mode by clearing the HDSEL bit in the CR3 register */
    USARTx->CR3 &= CR3_HDSEL_Reset;
  }
}

/**
  * @brief  Configures the USART’s IrDA interface.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IrDAMode: specifies the IrDA mode.
  *   This parameter can be one of the following values:
  *     @arg USART_IrDAMode_LowPower
  *     @arg USART_IrDAMode_Normal
  * @retval None
  */
void USART_IrDAConfig(USART_TypeDef* USARTx, uint16_t USART_IrDAMode)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_IRDA_MODE(USART_IrDAMode));
    
  USARTx->CR3 &= CR3_IRLP_Mask;
  USARTx->CR3 |= USART_IrDAMode;
}

/**
  * @brief  Enables or disables the USART’s IrDA interface.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the IrDA mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
    
  if (NewState != DISABLE)
  {
    /* Enable the IrDA mode by setting the IREN bit in the CR3 register */
    USARTx->CR3 |= CR3_IREN_Set;
  }
  else
  {
    /* Disable the IrDA mode by clearing the IREN bit in the CR3 register */
    USARTx->CR3 &= CR3_IREN_Reset;
  }
}

/**
  * @brief  Checks whether the specified USART flag is set or not.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_FLAG: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg USART_FLAG_CTS:  CTS Change flag (not available for UART4 and UART5)
  *     @arg USART_FLAG_LBD:  LIN Break detection flag
  *     @arg USART_FLAG_TXE:  Transmit data register empty flag
  *     @arg USART_FLAG_TC:   Transmission Complete flag
  *     @arg USART_FLAG_RXNE: Receive data register not empty flag
  *     @arg USART_FLAG_IDLE: Idle Line detection flag
  *     @arg USART_FLAG_ORE:  OverRun Error flag
  *     @arg USART_FLAG_NE:   Noise Error flag
  *     @arg USART_FLAG_FE:   Framing Error flag
  *     @arg USART_FLAG_PE:   Parity Error flag
  * @retval The new state of USART_FLAG (SET or RESET).
  */
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_FLAG(USART_FLAG));
  /* The CTS flag is not available for UART4 and UART5 */
  if (USART_FLAG == USART_FLAG_CTS)
  {
    assert_param(IS_USART_123_PERIPH(USARTx));
  }  
  
  if ((USARTx->SR & USART_FLAG) != (uint16_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Clears the USARTx's pending flags.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_FLAG: specifies the flag to clear.
  *   This parameter can be any combination of the following values:
  *     @arg USART_FLAG_CTS:  CTS Change flag (not available for UART4 and UART5).
  *     @arg USART_FLAG_LBD:  LIN Break detection flag.
  *     @arg USART_FLAG_TC:   Transmission Complete flag.
  *     @arg USART_FLAG_RXNE: Receive data register not empty flag.
  *   
  * @note
  *   - PE (Parity error), FE (Framing error), NE (Noise error), ORE (OverRun 
  *     error) and IDLE (Idle line detected) flags are cleared by software 
  *     sequence: a read operation to USART_SR register (USART_GetFlagStatus()) 
  *     followed by a read operation to USART_DR register (USART_ReceiveData()).
  *   - RXNE flag can be also cleared by a read to the USART_DR register 
  *     (USART_ReceiveData()).
  *   - TC flag can be also cleared by software sequence: a read operation to 
  *     USART_SR register (USART_GetFlagStatus()) followed by a write operation
  *     to USART_DR register (USART_SendData()).
  *   - TXE flag is cleared only by a write to the USART_DR register 
  *     (USART_SendData()).
  * @retval None
  */
void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_CLEAR_FLAG(USART_FLAG));
  /* The CTS flag is not available for UART4 and UART5 */
  if ((USART_FLAG & USART_FLAG_CTS) == USART_FLAG_CTS)
  {
    assert_param(IS_USART_123_PERIPH(USARTx));
  } 
   
  USARTx->SR = (uint16_t)~USART_FLAG;
}

/**
  * @brief  Checks whether the specified USART interrupt has occurred or not.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IT: specifies the USART interrupt source to check.
  *   This parameter can be one of the following values:
  *     @arg USART_IT_CTS:  CTS change interrupt (not available for UART4 and UART5)
  *     @arg USART_IT_LBD:  LIN Break detection interrupt
  *     @arg USART_IT_TXE:  Tansmit Data Register empty interrupt
  *     @arg USART_IT_TC:   Transmission complete interrupt
  *     @arg USART_IT_RXNE: Receive Data register not empty interrupt
  *     @arg USART_IT_IDLE: Idle line detection interrupt
  *     @arg USART_IT_ORE:  OverRun Error interrupt
  *     @arg USART_IT_NE:   Noise Error interrupt
  *     @arg USART_IT_FE:   Framing Error interrupt
  *     @arg USART_IT_PE:   Parity Error interrupt
  * @retval The new state of USART_IT (SET or RESET).
  */
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT)
{
  uint32_t bitpos = 0x00, itmask = 0x00, usartreg = 0x00;
  ITStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_GET_IT(USART_IT));
  /* The CTS interrupt is not available for UART4 and UART5 */ 
  if (USART_IT == USART_IT_CTS)
  {
    assert_param(IS_USART_123_PERIPH(USARTx));
  }   
  
  /* Get the USART register index */
  usartreg = (((uint8_t)USART_IT) >> 0x05);
  /* Get the interrupt position */
  itmask = USART_IT & IT_Mask;
  itmask = (uint32_t)0x01 << itmask;
  
  if (usartreg == 0x01) /* The IT  is in CR1 register */
  {
    itmask &= USARTx->CR1;
  }
  else if (usartreg == 0x02) /* The IT  is in CR2 register */
  {
    itmask &= USARTx->CR2;
  }
  else /* The IT  is in CR3 register */
  {
    itmask &= USARTx->CR3;
  }
  
  bitpos = USART_IT >> 0x08;
  bitpos = (uint32_t)0x01 << bitpos;
  bitpos &= USARTx->SR;
  if ((itmask != (uint16_t)RESET)&&(bitpos != (uint16_t)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  
  return bitstatus;  
}

/**
  * @brief  Clears the USARTx’s interrupt pending bits.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IT: specifies the interrupt pending bit to clear.
  *   This parameter can be one of the following values:
  *     @arg USART_IT_CTS:  CTS change interrupt (not available for UART4 and UART5)
  *     @arg USART_IT_LBD:  LIN Break detection interrupt
  *     @arg USART_IT_TC:   Transmission complete interrupt. 
  *     @arg USART_IT_RXNE: Receive Data register not empty interrupt.
  *   
  * @note
  *   - PE (Parity error), FE (Framing error), NE (Noise error), ORE (OverRun 
  *     error) and IDLE (Idle line detected) pending bits are cleared by 
  *     software sequence: a read operation to USART_SR register 
  *     (USART_GetITStatus()) followed by a read operation to USART_DR register 
  *     (USART_ReceiveData()).
  *   - RXNE pending bit can be also cleared by a read to the USART_DR register 
  *     (USART_ReceiveData()).
  *   - TC pending bit can be also cleared by software sequence: a read 
  *     operation to USART_SR register (USART_GetITStatus()) followed by a write 
  *     operation to USART_DR register (USART_SendData()).
  *   - TXE pending bit is cleared only by a write to the USART_DR register 
  *     (USART_SendData()).
  * @retval None
  */
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT)
{
  uint16_t bitpos = 0x00, itmask = 0x00;
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_CLEAR_IT(USART_IT));
  /* The CTS interrupt is not available for UART4 and UART5 */
  if (USART_IT == USART_IT_CTS)
  {
    assert_param(IS_USART_123_PERIPH(USARTx));
  }   
  
  bitpos = USART_IT >> 0x08;
  itmask = ((uint16_t)0x01 << (uint16_t)bitpos);
  USARTx->SR = (uint16_t)~itmask;
}


/************************* self define**************************************/
void ComInit(COM_NUM COM,uint32_t USART_BaudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	
	RCC_APB2PeriphClockCmd(COM_RCC_CLOCK[COM] | RCC_APB2Periph_AFIO, ENABLE);		/* Enable GPIO clock */
	
	if (COM1 == COM)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	}
	else
	{
		RCC_APB1PeriphClockCmd((uint32_t)0x00010000<<COM, ENABLE);
	}	
	
	GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];				/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(COM_IO_PORT[COM], &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];				/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		
	GPIO_Init(COM_IO_PORT[COM], &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = USART_BaudRate;		/* USART configuration */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(COM_USART[COM], &USART_InitStructure);	
	
	USART_Cmd(COM_USART[COM], ENABLE);			/* Enable USART */
}

void setUsartRXI(COM_NUM COM )
{
	NVIC_InitTypeDef NVIC_InitStructure;
	COM_USART[COM]->CR1|=(uint32_t)USART_FLAG_RXNE; 
	
	NVIC_InitStructure.NVIC_IRQChannel = COM_IRQn[COM];			
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = UART_NVIC_SUBPRI;					
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								
		
	NVIC_Init(&NVIC_InitStructure); 	
}



#if USART_DMA_EN
void usart1TxDMAInit(void )
{
	/* USART1 TX DMA4 Channel Config */
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel4); 

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR; 
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)usartDCMSendBuf;  
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; 
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  				//memory addr increase
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
	DMA_Init(DMA1_Channel4, &DMA_InitStructure); 

	USART_Cmd(USART1, DISABLE);
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); 
	USART_Cmd(USART1, ENABLE);
}


void DMAPrint(char * str)
{
	unsigned int len = 0;
	if( 0 == str)
		return ;
	len = strlen(str);

	
	USART1->CR3 &= (uint16_t)~USART_DMAReq_Tx;			// disable the  USART DMA sent 
	USART1->CR1 &= CR1_UE_Reset;						// disable the USART
	DMA1_Channel4->CCR &= CCR_ENABLE_Reset;				//disable DMA

	if( 0 == DMA1_Channel4->CNDTR )  					// DMA complete
	{
		strcpy((char *)usartDCMSendBuf,str);
		DMA1_Channel4->CNDTR = len;
		
		g_DMASendLenTotal = len;
		
		DMA1_Channel4->CMAR = (u32)usartDCMSendBuf;
		USART1->CR3 |= (uint16_t)USART_DMAReq_Tx;		//enable the  USART DMA sent 
		USART1->CR1 |= CR1_UE_Set;						//enable the  USART  sent 
		DMA1->IFCR |=  (DMA1_FLAG_TE4|DMA1_FLAG_TC4|DMA1_FLAG_HT4 );//must clear the flag
		DMA1_Channel4->CCR |= CCR_ENABLE_Set;			//enable DMA
	}
	else
	{
		
		g_DMASendLenTotal_Next = len+ DMA1_Channel4->CNDTR;
		if(g_DMASendLenTotal + len > USART_SEND_LEN)
		{
			USART1->CR1 |= CR1_UE_Set;
			usartStrSend("usart DMA send buf over!\r\n");
			return ;
		}
		strcpy((char *)&usartDCMSendBuf[g_DMASendLenTotal],str);
		g_DMASendLenTotal += len;
		
		DMA1_Channel4->CNDTR= g_DMASendLenTotal_Next;
		DMA1_Channel4->CMAR = (u32)usartDCMSendBuf + g_DMASendLenTotal - g_DMASendLenTotal_Next;


		USART1->CR3 |= (uint16_t)USART_DMAReq_Tx;
		USART1->CR1 |= CR1_UE_Set;
		DMA1->IFCR |=  (DMA1_FLAG_TE4|DMA1_FLAG_TC4|DMA1_FLAG_HT4 );
		DMA1_Channel4->CCR |= CCR_ENABLE_Set;
	}	
}

#endif

#if 1
#pragma import(__use_no_semihosting)                
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
_sys_exit(int x) 
{ 
	x = x; 
} 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 


void usartCharSend(unsigned char charData)
{      
	while((USART1->SR&0X40)==0);
	USART1->DR = charData;      
}

unsigned char usartCharGet(void)
{      
	while((USART1->SR&0X20)==0);
	return (unsigned char)(USART1->DR & 0x00ff);      
}


#define WAITT_KEY_TIMER 5000   //ms

extern void delay_ms(u16 nms);

unsigned char usartCharGet_timeout(unsigned int ms)
{      
	unsigned int cnt = 0;
	
	printf("press any KEY to enter cmd mode\r\n");
	
	USART1->SR = 0x00;
	while((USART1->SR&0X20)==0){
			delay_ms(1);
			cnt++;
			if(cnt >= ms)
				return 1;
	}
	return 0;    
}

void usartBytesSend(unsigned char *charData,unsigned char len)
{   
	unsigned int i = 0;
	while(i != len){
 		usartCharSend(charData[i]);
		i++;
	}   
}

void usartStrSend(char * str)
{
	unsigned int i = 0;
	while(str[i] != 0){
 		usartCharSend(str[i]);
		i++;
	}
}

/***********************************
uint32_t sendData  : limit is 32bit
unsigned char rate : 0 -decimal  1-hex
unsigned char style: 0 - end with \r\n  1- end with ' '
429 496 729 5

**************************************/
void PrintInt(int32_t sendValue,unsigned char rate,unsigned char style)
{
	unsigned char hexTmp[11];
	unsigned char decTmp[13];
	unsigned char i,value;
	uint32_t tmpValue,sendData;

	sendData = sendValue;
	if(sendValue < 0){
		DebugPrint("-");
		sendData = 0xffffffff - (uint32_t)sendValue +1;
	}
	if( 1 == rate){
		for(i=0;i<8;i++){
			value =(unsigned char)(( 0xf0000000&( sendData<<(i*4) ) )>>28 );
			if((0 <= value )&&(value <= 9))
				hexTmp[i] = value + 0x30;
			else if((10 <= value)&&(value <= 15))
				hexTmp[i] = value + 55;
		}
		if(style == 0){
			hexTmp[8] = '\r';
			hexTmp[9] = '\n';
			hexTmp[10] = 0;
		}
		else if(style == 1){
			hexTmp[8] = ' ';
			hexTmp[9] = 0;
		}

		i = 0;
		while(hexTmp[i] == '0'){
			i++;
			if(i == 8){
				DebugPrint((char *)&hexTmp[7]);
				return ;
			}
		}
		DebugPrint((char *)&hexTmp[i]);
	}
	else {
		tmpValue = sendData;
		for(i=0;i<10;i++){
			decTmp[9-i] = tmpValue%10 + 0x30;
			tmpValue = tmpValue/10;
		}
		if(style == 0){
			decTmp[10] = '\r';
			decTmp[11] = '\n';
			decTmp[12] = 0;
		}
		else {
			decTmp[10] = ' ';
			decTmp[11] = 0;
		}
		i = 0;
		while(decTmp[i] == '0'){
			i++;
			if(i == 10){
				DebugPrint((char *)&decTmp[9]);
				return ;
			}

		}
		DebugPrint((char *)&decTmp[i]);

	}

}

void SendToScope(unsigned char  *array)
{
/*
	uint16_t IX,IY,CRC_value;
	unsigned char CRC_L,CRC_H;
	
	CRC_value = 0xffff;

	for(IX = 0; IX < 8; IX++){
		CRC_value = CRC_value ^(uint16_t)array[IX];
		
		for(IY = 0;IY <8;IY++){
			if((CRC_value&0x0001) != 0) CRC_value = (CRC_value >>1)^0xa001;
			else CRC_value = CRC_value >> 1;
		}
	}
	CRC_L = (unsigned char)(CRC_value );
	CRC_H = (unsigned char)(CRC_value >> 8);	 



	usartBytesSend(array,8);
	
	usartCharSend(CRC_L);
	usartCharSend(CRC_H);
*/
	unsigned char i ,tmp;
	for(i = 0;i<8;i++)
		tmp +=array[i];	

	usartBytesSend(array,8);
	usartCharSend(tmp);
	
}

