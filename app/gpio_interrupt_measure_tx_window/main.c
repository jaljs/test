#include "stm8s.h"
#include "printf.h"

void putchar(char c);
static void GPIO_Config(void);

#define GPIO_TX_WINDOW_PORT_EXTERNAL_INTERRUPTS (4)
#define GPIO_TX_WINDOW_PORT (GPIOB)
#define GPIO_TX_WINDOW_PIN (GPIO_PIN_7)
#define GPIO_TX_WINDOW_TEST_CLK_PIN (GPIO_PIN_6)
#define GPIO_LED_PORT (GPIOE)
#define GPIO_LED_PIN  (GPIO_PIN_5)

void putchar(char c);

void main(void)
{
	unsigned long int i;
  /* Connect LSI to COO pin*/ /* PC4: CCO */
	GPIO_Config();

	EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_ONLY);
	EXTI_SetTLISensitivity(EXTI_TLISENSITIVITY_RISE_ONLY);

	/* Enable Interrupt */
	__asm__("rim\n");

	/* for debug */
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
	UART1_DeInit();
	UART1_Init((uint32_t)115200, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
	             UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);

	GPIO_Init(GPIO_TX_WINDOW_PORT, GPIO_TX_WINDOW_TEST_CLK_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
	while(1) {
		GPIO_WriteReverse(GPIO_TX_WINDOW_PORT, GPIO_TX_WINDOW_TEST_CLK_PIN);
		for (i = 0; i < 250000; i++) {}
	}
}

void putchar(char c)
{
	UART1_SendData8(c);
	while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);
}

static void GPIO_Config(void)
{
	/* Enable GPIO TX WINDOW Interrupt */
	GPIO_Init(GPIO_TX_WINDOW_PORT, GPIO_TX_WINDOW_PIN, GPIO_MODE_IN_FL_IT);

	/* LED */
	GPIO_Init(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
}

/**
  * @brief  External Interrupt PORTB Interrupt routine
  * @param  None
  * @retval None
  */
void gpio_isr(void) __interrupt(GPIO_TX_WINDOW_PORT_EXTERNAL_INTERRUPTS)
{
  if (GPIO_ReadInputData(GPIO_TX_WINDOW_PORT) & GPIO_TX_WINDOW_PIN) {
    GPIO_WriteReverse(GPIO_LED_PORT, GPIO_LED_PIN);
  }
}
