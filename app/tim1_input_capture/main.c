#include "stm8s.h"
#include "printf.h"

uint32_t TIM1ClockFreq = 2000000;
uint32_t LSIClockFreq = 0;
uint16_t ICValue1 = 0;
uint16_t ICValue2 =0;

static void TIM1_Config(void);
void putchar(char c);

void main(void)
{
	unsigned long int d;

  /* Connect LSI to COO pin*/ /* PC4: CCO */
  GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);
  CLK_CCOConfig(CLK_OUTPUT_LSI);
  CLK_CCOCmd(ENABLE);

  /* TIM1 configuration */
  TIM1_Config();
    
  /* Compute LSI clock frequency */
  LSIClockFreq = (8 * TIM1ClockFreq) / (ICValue2 - ICValue1);
  
	/* for debug */
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
	UART1_DeInit();
	UART1_Init((uint32_t)115200, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
	             UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
	printf("Masure Clock: %u %u %u\n", LSIClockFreq, ICValue1, ICValue2);

	// Configure pins
	PE_DDR = 0x20;
	PE_CR1 = 0x20;
	do {
		PE_ODR ^= 0x20;
		for(d = 0; d < LSIClockFreq; d++) { }
	} while(1);
}

void putchar(char c)
{
	UART1_SendData8(c);
	while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
}

/**
  * @brief  Configure TIM1 to to capture the internal clock source (LSI)
  * @param  None
  * @retval None
  */
static void TIM1_Config(void)
{
  TIM1_ICInit( TIM1_CHANNEL_1, TIM1_ICPOLARITY_FALLING, TIM1_ICSELECTION_DIRECTTI,
               TIM1_ICPSC_DIV8, 0x0);
  
  /* Enable TIM1 */
  TIM1_Cmd(ENABLE);

  /* Clear CC1 Flag*/
  TIM1_ClearFlag(TIM1_FLAG_CC1);
  
  /* wait a capture on CC1 */
  while((TIM1->SR1 & TIM1_FLAG_CC1) != TIM1_FLAG_CC1);
  /* Get CCR1 value*/
  ICValue1 = TIM1_GetCapture1();
  TIM1_ClearFlag(TIM1_FLAG_CC1);
  
  /* wait a capture on cc1 */
  while((TIM1->SR1 & TIM1_FLAG_CC1) != TIM1_FLAG_CC1);
  /* Get CCR1 value*/
  ICValue2 = TIM1_GetCapture1();
  TIM1_ClearFlag(TIM1_FLAG_CC1);
}
