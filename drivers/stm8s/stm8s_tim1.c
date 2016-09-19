#include "stm8s_tim1.h"

/** @addtogroup STM8S_StdPeriph_Driver
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void TI1_Config(uint8_t TIM1_ICPolarity, uint8_t TIM1_ICSelection,
                       uint8_t TIM1_ICFilter);
static void TI2_Config(uint8_t TIM1_ICPolarity, uint8_t TIM1_ICSelection,
                       uint8_t TIM1_ICFilter);
static void TI3_Config(uint8_t TIM1_ICPolarity, uint8_t TIM1_ICSelection,
                       uint8_t TIM1_ICFilter);
static void TI4_Config(uint8_t TIM1_ICPolarity, uint8_t TIM1_ICSelection,
                       uint8_t TIM1_ICFilter);
/**
  * @brief  Configure the TI1 as Input.
  * @param  TIM1_ICPolarity  The Input Polarity.
  *         This parameter can be one of the following values:
  *                       - TIM1_ICPOLARITY_FALLING
  *                       - TIM1_ICPOLARITY_RISING
  * @param  TIM1_ICSelection specifies the input to be used.
  *         This parameter can be one of the following values:
  *                       - TIM1_ICSELECTION_DIRECTTI: TIM1 Input 1 is selected to
  *                         be connected to IC1.
  *                       - TIM1_ICSELECTION_INDIRECTTI: TIM1 Input 1 is selected to
  *                         be connected to IC2.
  * @param  TIM1_ICFilter Specifies the Input Capture Filter.
  *         This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
static void TI1_Config(uint8_t TIM1_ICPolarity,
                       uint8_t TIM1_ICSelection,
                       uint8_t TIM1_ICFilter)
{
  /* Disable the Channel 1: Reset the CCE Bit */
  TIM1->CCER1 &= (uint8_t)(~TIM1_CCER1_CC1E);
  
  /* Select the Input and set the filter */
  TIM1->CCMR1 = (uint8_t)((uint8_t)(TIM1->CCMR1 & (uint8_t)(~(uint8_t)( TIM1_CCMR_CCxS | TIM1_CCMR_ICxF ))) | 
                          (uint8_t)(( (TIM1_ICSelection)) | ((uint8_t)( TIM1_ICFilter << 4))));
  
  /* Select the Polarity */
  if (TIM1_ICPolarity != TIM1_ICPOLARITY_RISING)
  {
    TIM1->CCER1 |= TIM1_CCER1_CC1P;
  }
  else
  {
    TIM1->CCER1 &= (uint8_t)(~TIM1_CCER1_CC1P);
  }
  
  /* Set the CCE Bit */
  TIM1->CCER1 |=  TIM1_CCER1_CC1E;
}

/**
  * @brief  Configure the TI2 as Input.
  * @param  TIM1_ICPolarity  The Input Polarity.
  *         This parameter can be one of the following values:
  *                       - TIM1_ICPOLARITY_FALLING
  *                       - TIM1_ICPOLARITY_RISING
  * @param  TIM1_ICSelection specifies the input to be used.
  *         This parameter can be one of the following values:
  *                       - TIM1_ICSELECTION_DIRECTTI: TIM1 Input 2 is selected to
  *                         be connected to IC2.
  *                       - TIM1_ICSELECTION_INDIRECTTI: TIM1 Input 2 is selected to
  *                         be connected to IC1.
  * @param  TIM1_ICFilter Specifies the Input Capture Filter.
  *         This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
static void TI2_Config(uint8_t TIM1_ICPolarity,
                       uint8_t TIM1_ICSelection,
                       uint8_t TIM1_ICFilter)
{
  /* Disable the Channel 2: Reset the CCE Bit */
  TIM1->CCER1 &=  (uint8_t)(~TIM1_CCER1_CC2E);
  
  /* Select the Input and set the filter */
  TIM1->CCMR2  = (uint8_t)((uint8_t)(TIM1->CCMR2 & (uint8_t)(~(uint8_t)( TIM1_CCMR_CCxS | TIM1_CCMR_ICxF ))) 
                           | (uint8_t)(( (TIM1_ICSelection)) | ((uint8_t)( TIM1_ICFilter << 4))));
  /* Select the Polarity */
  if (TIM1_ICPolarity != TIM1_ICPOLARITY_RISING)
  {
    TIM1->CCER1 |= TIM1_CCER1_CC2P;
  }
  else
  {
    TIM1->CCER1 &= (uint8_t)(~TIM1_CCER1_CC2P);
  }
  /* Set the CCE Bit */
  TIM1->CCER1 |=  TIM1_CCER1_CC2E;
}

/**
  * @brief  Configure the TI3 as Input.
  * @param  TIM1_ICPolarity  The Input Polarity.
  *         This parameter can be one of the following values:
  *                       - TIM1_ICPOLARITY_FALLING
  *                       - TIM1_ICPOLARITY_RISING
  * @param  TIM1_ICSelection specifies the input to be used.
  *         This parameter can be one of the following values:
  *                       - TIM1_ICSELECTION_DIRECTTI: TIM1 Input 3 is selected to
  *                         be connected to IC3.
  *                       - TIM1_ICSELECTION_INDIRECTTI: TIM1 Input 3 is selected to
  *                         be connected to IC4.
  * @param  TIM1_ICFilter Specifies the Input Capture Filter.
  *         This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
static void TI3_Config(uint8_t TIM1_ICPolarity,
                       uint8_t TIM1_ICSelection,
                       uint8_t TIM1_ICFilter)
{
  /* Disable the Channel 3: Reset the CCE Bit */
  TIM1->CCER2 &=  (uint8_t)(~TIM1_CCER2_CC3E);
  
  /* Select the Input and set the filter */
  TIM1->CCMR3 = (uint8_t)((uint8_t)(TIM1->CCMR3 & (uint8_t)(~(uint8_t)( TIM1_CCMR_CCxS | TIM1_CCMR_ICxF))) 
                          | (uint8_t)(( (TIM1_ICSelection)) | ((uint8_t)( TIM1_ICFilter << 4))));
  
  /* Select the Polarity */
  if (TIM1_ICPolarity != TIM1_ICPOLARITY_RISING)
  {
    TIM1->CCER2 |= TIM1_CCER2_CC3P;
  }
  else
  {
    TIM1->CCER2 &= (uint8_t)(~TIM1_CCER2_CC3P);
  }
  /* Set the CCE Bit */
  TIM1->CCER2 |=  TIM1_CCER2_CC3E;
}

/**
  * @brief  Configure the TI4 as Input.
  * @param  TIM1_ICPolarity  The Input Polarity.
  *         This parameter can be one of the following values:
  *                       - TIM1_ICPOLARITY_FALLING
  *                       - TIM1_ICPOLARITY_RISING
  * @param  TIM1_ICSelection specifies the input to be used.
  *         This parameter can be one of the following values:
  *                       - TIM1_ICSELECTION_DIRECTTI: TIM1 Input 4 is selected to
  *                         be connected to IC4.
  *                       - TIM1_ICSELECTION_INDIRECTTI: TIM1 Input 4 is selected to
  *                         be connected to IC3.
  * @param  TIM1_ICFilter Specifies the Input Capture Filter.
  *         This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
static void TI4_Config(uint8_t TIM1_ICPolarity,
                       uint8_t TIM1_ICSelection,
                       uint8_t TIM1_ICFilter)
{
  /* Disable the Channel 4: Reset the CCE Bit */
  TIM1->CCER2 &=  (uint8_t)(~TIM1_CCER2_CC4E);
  
  /* Select the Input and set the filter */
  TIM1->CCMR4 = (uint8_t)((uint8_t)(TIM1->CCMR4 & (uint8_t)(~(uint8_t)( TIM1_CCMR_CCxS | TIM1_CCMR_ICxF )))
                          | (uint8_t)(( (TIM1_ICSelection)) | ((uint8_t)( TIM1_ICFilter << 4))));
  
  /* Select the Polarity */
  if (TIM1_ICPolarity != TIM1_ICPOLARITY_RISING)
  {
    TIM1->CCER2 |= TIM1_CCER2_CC4P;
  }
  else
  {
    TIM1->CCER2 &= (uint8_t)(~TIM1_CCER2_CC4P);
  }
  
  /* Set the CCE Bit */
  TIM1->CCER2 |=  TIM1_CCER2_CC4E;
}

/**
  * @brief  Sets the TIM1 Input Capture 1 prescaler.
  * @param   TIM1_IC1Prescaler specifies the Input Capture prescaler new value
  * This parameter can be one of the following values:
  *                       - TIM1_ICPSC_DIV1: no prescaler
  *                       - TIM1_ICPSC_DIV2: capture is done once every 2 events
  *                       - TIM1_ICPSC_DIV4: capture is done once every 4 events
  *                       - TIM1_ICPSC_DIV8: capture is done once every 8 events
  * @retval None
  */
void TIM1_SetIC1Prescaler(TIM1_ICPSC_TypeDef TIM1_IC1Prescaler)
{
  /* Reset the IC1PSC Bits */ /* Set the IC1PSC value */
  TIM1->CCMR1 = (uint8_t)((uint8_t)(TIM1->CCMR1 & (uint8_t)(~TIM1_CCMR_ICxPSC)) 
                          | (uint8_t)TIM1_IC1Prescaler);
}

/**
  * @brief  Sets the TIM1 Input Capture 2 prescaler.
  * @param   TIM1_IC2Prescaler specifies the Input Capture prescaler new value
  * This parameter can be one of the following values:
  *                       - TIM1_ICPSC_DIV1: no prescaler
  *                       - TIM1_ICPSC_DIV2: capture is done once every 2 events
  *                       - TIM1_ICPSC_DIV4: capture is done once every 4 events
  *                       - TIM1_ICPSC_DIV8: capture is done once every 8 events
  * @retval None
  */
void TIM1_SetIC2Prescaler(TIM1_ICPSC_TypeDef TIM1_IC2Prescaler)
{
  /* Reset the IC1PSC Bits */ /* Set the IC1PSC value */
  TIM1->CCMR2 = (uint8_t)((uint8_t)(TIM1->CCMR2 & (uint8_t)(~TIM1_CCMR_ICxPSC))
                          | (uint8_t)TIM1_IC2Prescaler);
}

/**
  * @brief  Sets the TIM1 Input Capture 3 prescaler.
  * @param   TIM1_IC3Prescaler specifies the Input Capture prescaler new value
  * This parameter can be one of the following values:
  *                       - TIM1_ICPSC_DIV1: no prescaler
  *                       - TIM1_ICPSC_DIV2: capture is done once every 2 events
  *                       - TIM1_ICPSC_DIV4: capture is done once every 4 events
  *                       - TIM1_ICPSC_DIV8: capture is done once every 8 events
  * @retval None
  */
void TIM1_SetIC3Prescaler(TIM1_ICPSC_TypeDef TIM1_IC3Prescaler)
{
  /* Reset the IC1PSC Bits & Set the IC1PSC value */
  TIM1->CCMR3 = (uint8_t)((uint8_t)(TIM1->CCMR3 & (uint8_t)(~TIM1_CCMR_ICxPSC)) | 
                          (uint8_t)TIM1_IC3Prescaler);
}

/**
  * @brief  Sets the TIM1 Input Capture 4 prescaler.
  * @param  TIM1_IC4Prescaler specifies the Input Capture prescaler new value
  *         This parameter can be one of the following values:
  *                       - TIM1_ICPSC_DIV1: no prescaler
  *                       - TIM1_ICPSC_DIV2: capture is done once every 2 events
  *                       - TIM1_ICPSC_DIV4: capture is done once every 4 events
  *                       - TIM1_ICPSC_DIV8: capture is done once every 8 events
  * @retval None
  */
void TIM1_SetIC4Prescaler(TIM1_ICPSC_TypeDef TIM1_IC4Prescaler)
{
  /* Reset the IC1PSC Bits &  Set the IC1PSC value */
  TIM1->CCMR4 = (uint8_t)((uint8_t)(TIM1->CCMR4 & (uint8_t)(~TIM1_CCMR_ICxPSC)) |
                          (uint8_t)TIM1_IC4Prescaler);
}

/**
  * @brief  Initializes the TIM1 peripheral according to the specified parameters.
  * @param  TIM1_Channel specifies the input capture channel from TIM1_Channel_TypeDef.
  * @param  TIM1_ICPolarity specifies the Input capture polarity from  
  *         TIM1_ICPolarity_TypeDef .
  * @param  TIM1_ICSelection specifies the Input capture source selection from 
  *         TIM1_ICSelection_TypeDef.
  * @param  TIM1_ICPrescaler specifies the Input capture Prescaler from
  *         TIM1_ICPSC_TypeDef.
  * @param  TIM1_ICFilter specifies the Input capture filter value.
  * @retval None
  */
void TIM1_ICInit(TIM1_Channel_TypeDef TIM1_Channel,
                 TIM1_ICPolarity_TypeDef TIM1_ICPolarity,
                 TIM1_ICSelection_TypeDef TIM1_ICSelection,
                 TIM1_ICPSC_TypeDef TIM1_ICPrescaler,
                 uint8_t TIM1_ICFilter)
{
  if (TIM1_Channel == TIM1_CHANNEL_1)
  {
    /* TI1 Configuration */
    TI1_Config((uint8_t)TIM1_ICPolarity,
               (uint8_t)TIM1_ICSelection,
               (uint8_t)TIM1_ICFilter);
    /* Set the Input Capture Prescaler value */
    TIM1_SetIC1Prescaler(TIM1_ICPrescaler);
  }
  else if (TIM1_Channel == TIM1_CHANNEL_2)
  {
    /* TI2 Configuration */
    TI2_Config((uint8_t)TIM1_ICPolarity,
               (uint8_t)TIM1_ICSelection,
               (uint8_t)TIM1_ICFilter);
    /* Set the Input Capture Prescaler value */
    TIM1_SetIC2Prescaler(TIM1_ICPrescaler);
  }
  else if (TIM1_Channel == TIM1_CHANNEL_3)
  {
    /* TI3 Configuration */
    TI3_Config((uint8_t)TIM1_ICPolarity,
               (uint8_t)TIM1_ICSelection,
               (uint8_t)TIM1_ICFilter);
    /* Set the Input Capture Prescaler value */
    TIM1_SetIC3Prescaler(TIM1_ICPrescaler);
  }
  else
  {
    /* TI4 Configuration */
    TI4_Config((uint8_t)TIM1_ICPolarity,
               (uint8_t)TIM1_ICSelection,
               (uint8_t)TIM1_ICFilter);
    /* Set the Input Capture Prescaler value */
    TIM1_SetIC4Prescaler(TIM1_ICPrescaler);
  }
}

/**
  * @brief  Enables or disables the TIM1 peripheral.
  * @param  NewState new state of the TIM1 peripheral.
  *         This parameter can be ENABLE or DISABLE.
  * @retval None
  */
void TIM1_Cmd(FunctionalState NewState)
{
  /* set or Reset the CEN Bit */
  if (NewState != DISABLE)
  {
    TIM1->CR1 |= TIM1_CR1_CEN;
  }
  else
  {
    TIM1->CR1 &= (uint8_t)(~TIM1_CR1_CEN);
  }
}

/**
  * @brief  Clears the TIM1s pending flags.
  * @param  TIM1_FLAG specifies the flag to clear.
  *         This parameter can be one of the following values:
  *                       - TIM1_FLAG_UPDATE: TIM1 update Flag
  *                       - TIM1_FLAG_CC1: TIM1 Capture Compare 1 Flag
  *                       - TIM1_FLAG_CC2: TIM1 Capture Compare 2 Flag
  *                       - TIM1_FLAG_CC3: TIM1 Capture Compare 3 Flag
  *                       - TIM1_FLAG_CC4: TIM1 Capture Compare 4 Flag
  *                       - TIM1_FLAG_COM: TIM1 Commutation Flag
  *                       - TIM1_FLAG_TRIGGER: TIM1 Trigger Flag
  *                       - TIM1_FLAG_BREAK: TIM1 Break Flag
  *                       - TIM1_FLAG_CC1OF: TIM1 Capture Compare 1 overcapture Flag
  *                       - TIM1_FLAG_CC2OF: TIM1 Capture Compare 2 overcapture Flag
  *                       - TIM1_FLAG_CC3OF: TIM1 Capture Compare 3 overcapture Flag
  *                       - TIM1_FLAG_CC4OF: TIM1 Capture Compare 4 overcapture Flag
  * @retval None.
  */
void TIM1_ClearFlag(TIM1_FLAG_TypeDef TIM1_FLAG)
{
  /* Clear the flags (rc_w0) clear this bit by writing 0. Writing 1 has no effect*/
  TIM1->SR1 = (uint8_t)(~(uint8_t)(TIM1_FLAG));
  TIM1->SR2 = (uint8_t)((uint8_t)(~((uint8_t)((uint16_t)TIM1_FLAG >> 8))) & 
                        (uint8_t)0x1E);
}

/**
  * @brief  Gets the TIM1 Input Capture 1 value.
  * @param  None
  * @retval Capture Compare 1 Register value.
  */
uint16_t TIM1_GetCapture1(void)
{
  /* Get the Capture 1 Register value */
  
  uint16_t tmpccr1 = 0;
  uint8_t tmpccr1l=0, tmpccr1h=0;
  
  tmpccr1h = TIM1->CCR1H;
  tmpccr1l = TIM1->CCR1L;
  
  tmpccr1 = (uint16_t)(tmpccr1l);
  tmpccr1 |= (uint16_t)((uint16_t)tmpccr1h << 8);
  /* Get the Capture 1 Register value */
  return (uint16_t)tmpccr1;
}


