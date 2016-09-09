#include "stm8s_clk.h"

CONST uint8_t HSIDivFactor[4] = {1, 2, 4, 8}; /*!< Holds the different HSI Divider factors */


/**
  * @brief  Output the selected clock on a dedicated I/O pin.
  * @param   CLK_CCO : Specifies the clock source.
  * This parameter can be any of the  @ref CLK_Output_TypeDef enumeration.
  * @retval None
  * @par Required preconditions:
  * The dedicated I/O pin must be set at 1 in the corresponding Px_CR1 register \n
  * to be set as input with pull-up or push-pull output.
  */
void CLK_CCOConfig(CLK_Output_TypeDef CLK_CCO)
{
  /* Clears of the CCO type bits part */
  CLK->CCOR &= (uint8_t)(~CLK_CCOR_CCOSEL);
  
  /* Selects the source provided on cco_ck output */
  CLK->CCOR |= (uint8_t)CLK_CCO;
  
  /* Enable the clock output */
  CLK->CCOR |= CLK_CCOR_CCOEN;
}

/**
  * @brief  Configures the HSI and CPU clock dividers.
  * @param   ClockPrescaler Specifies the HSI or CPU clock divider to apply.
  * @retval None
  */
void CLK_SYSCLKConfig(CLK_Prescaler_TypeDef CLK_Prescaler)
{
  if (((uint8_t)CLK_Prescaler & (uint8_t)0x80) == 0x00) /* Bit7 = 0 means HSI divider */
  {
    CLK->CKDIVR &= (uint8_t)(~CLK_CKDIVR_HSIDIV);
    CLK->CKDIVR |= (uint8_t)((uint8_t)CLK_Prescaler & (uint8_t)CLK_CKDIVR_HSIDIV);
  }
  else /* Bit7 = 1 means CPU divider */
  {
    CLK->CKDIVR &= (uint8_t)(~CLK_CKDIVR_CPUDIV);
    CLK->CKDIVR |= (uint8_t)((uint8_t)CLK_Prescaler & (uint8_t)CLK_CKDIVR_CPUDIV);
  }
}



/**
  * @brief  Enables or disablle the Configurable Clock Output (CCO).
  * @param   NewState : New state of CCEN bit (CCO register).
  * This parameter can be any of the @ref FunctionalState enumeration.
  * @retval None
  */
void CLK_CCOCmd(FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Set CCOEN bit */
    CLK->CCOR |= CLK_CCOR_CCOEN;
  }
  else
  {
    /* Reset CCOEN bit */
    CLK->CCOR &= (uint8_t)(~CLK_CCOR_CCOEN);
  }
}

/**
  * @brief  Configures the HSI clock dividers.
  * @param   HSIPrescaler : Specifies the HSI clock divider to apply.
  * This parameter can be any of the @ref CLK_Prescaler_TypeDef enumeration.
  * @retval None
  */
void CLK_HSIPrescalerConfig(CLK_Prescaler_TypeDef HSIPrescaler)
{
  /* Clear High speed internal clock prescaler */
  CLK->CKDIVR &= (uint8_t)(~CLK_CKDIVR_HSIDIV);
  
  /* Set High speed internal clock prescaler */
  CLK->CKDIVR |= (uint8_t)HSIPrescaler;
}

/**
  * @brief  This function returns the frequencies of different on chip clocks.
  * @param  None
  * @retval the master clock frequency
  */
uint32_t CLK_GetClockFreq(void)
{
  uint32_t clockfrequency = 0;
  CLK_Source_TypeDef clocksource = CLK_SOURCE_HSI;
  uint8_t tmp = 0, presc = 0;
  
  /* Get CLK source. */
  clocksource = (CLK_Source_TypeDef)CLK->CMSR;
  
  if (clocksource == CLK_SOURCE_HSI)
  {
    tmp = (uint8_t)(CLK->CKDIVR & CLK_CKDIVR_HSIDIV);
    tmp = (uint8_t)(tmp >> 3);
    presc = HSIDivFactor[tmp];
    clockfrequency = HSI_VALUE / presc;
  }
  else if ( clocksource == CLK_SOURCE_LSI)
  {
    clockfrequency = LSI_VALUE;
  }
  else
  {
    clockfrequency = HSE_VALUE;
  }
  
  return((uint32_t)clockfrequency);
}
