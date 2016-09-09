#include "stm8s.h"

/**
  * @addtogroup TIM1_Input_Capture
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t TIM1ClockFreq = 2000000;
__IO uint32_t LSIClockFreq = 0;
uint16_t ICValue1 =0, ICValue2 =0;
/* Private function prototypes -----------------------------------------------*/
static void TIM1_Config(void);
static int putchar(char c);
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
	//unsigned long int d;

	/* for debug */
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
	UART1_DeInit();
	UART1_Init((uint32_t)115200, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
	              UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);

  /* Connect LSI to COO pin*/ /* PC4: CCO */
  GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);
  CLK_CCOConfig(CLK_OUTPUT_LSI);
  CLK_CCOCmd(ENABLE);

  /* TIM1 configuration -----------------------------------------*/
  TIM1_Config();
    
  /* Compute LSI clock frequency */
  LSIClockFreq = (8 * TIM1ClockFreq) / (ICValue2 - ICValue1);
  
	// Configure pins
	//PE_DDR = 0x20;
	//PE_CR1 = 0x20;
	// Loop
	//do {
	//	PE_ODR ^= 0x20;
	//	for(d = 0; d < 255000; d++) { }
	//	//for(d = 0; d < LSIClockFreq; d++) { }
	//} while(1);
	putchar('H');
	putchar('e');
	putchar('L');
	putchar('L');
	putchar('o');
	putchar('\n');
}

static int putchar(char c)
{
	UART1_SendData8(c);
	while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
	return c;
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
