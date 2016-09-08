#include "stm8s_clk.h"

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
  /* check the parameters */
  assert_param(IS_CLK_OUTPUT_OK(CLK_CCO));
  
  /* Clears of the CCO type bits part */
  CLK->CCOR &= (uint8_t)(~CLK_CCOR_CCOSEL);
  
  /* Selects the source provided on cco_ck output */
  CLK->CCOR |= (uint8_t)CLK_CCO;
  
  /* Enable the clock output */
  CLK->CCOR |= CLK_CCOR_CCOEN;
}

/**
  * @brief  Enables or disablle the Configurable Clock Output (CCO).
  * @param   NewState : New state of CCEN bit (CCO register).
  * This parameter can be any of the @ref FunctionalState enumeration.
  * @retval None
  */
void CLK_CCOCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONALSTATE_OK(NewState));
  
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
