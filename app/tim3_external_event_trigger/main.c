#include "stm8l15x.h"

#include "stm8l15x_usart.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_clk.h"
#include "stm8l15x_tim3.h"
#include "stm8l15x_syscfg.h"

#include <stdio.h>

#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN  (GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7)

static void Delay (uint16_t nCount);
static void USART_Config(void);
void putchar(char c);

static void CLK_Config(void)
{
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
  CLK_SYSCLKSourceSwitchCmd(ENABLE);
  CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSI);
  while (CLK_GetSYSCLKSource() != CLK_SYSCLKSource_HSI) {}

  CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, ENABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE);
}

static void TIM3_Config(void)
{
  TIM3_TimeBaseInit(TIM3_Prescaler_1, TIM3_CounterMode_Up, 0xFFFF);
  TIM3_TIxExternalClockConfig(TIM3_TIxExternalCLK1Source_TI1, TIM3_ICPolarity_Rising, 0);
  TIM3_Cmd(ENABLE);
}

void main(void)
{
  CLK_Config();
  GPIO_Init(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_Mode_Out_PP_Low_Fast);
  TIM3_Config();
  USART_Config();
  while (1)
  {
    printf("cnt: %04x\n", TIM3_GetCounter());
    GPIO_ToggleBits(GPIO_LED_PORT, GPIO_LED_PIN);
  }
}

void Delay(uint16_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}

void putchar(char c)
{
  USART_SendData8(USART1, c);
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
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
