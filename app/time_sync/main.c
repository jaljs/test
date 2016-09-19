#include "stm8l15x.h"
#include "printf.h"
#include "stm8l15x_usart.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_clk.h"

#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN  (GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7)

void putchar(char c);
void Delay(uint32_t nCount);

void CLK_Config(void);
void GPIO_Config(void);
void USART_Config(void);

void main(void)
{
  CLK_Config();
  USART_Config();
	GPIO_Config();

	while(1) {
    GPIO_ToggleBits(GPIO_LED_PORT, GPIO_LED_PIN);
    Delay(0x80000);
	}
}

void putchar(char c)
{
	USART_SendData8(USART1, c);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

void Delay(uint32_t nCount)
{
  while (nCount != 0) {
    nCount--;
  }
}

static void GPIO_Config(void)
{
	/* LED */
	GPIO_Init(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_Mode_Out_PP_Low_Fast);
}

static void CLK_Config(void)
{
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
}

static void USART_Config(void)
{
	USART_DeInit(USART1);
	USART_Init(
    USART1,
    (uint32_t)115200,
    USART_WordLength_8b,
    USART_StopBits_1,
    USART_Parity_No,
    USART_Mode_Rx | USART_Mode_Tx
  );
}
