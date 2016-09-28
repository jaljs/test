#include "stm8l15x.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_clk.h"
#include "stm8l15x_tim2.h"
#include "stm8l15x_tim3.h"
#include "stm8l15x_itc.h"

#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN  (GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7)

#define GPIO_TX_WIN_OUT_PORT GPIOB
#define GPIO_TX_WIN_OUT_PIN GPIO_Pin_4

static void Delay(uint32_t nCount);
static void enable_interrupts(void);

static void CLK_Config(void);
static void GPIO_Config(void);
static void TIM2_Config(void);
static void TIM3_Config(void);
static void gpio_tx_win_out_pin_generate_one_pulse(uint16_t micro_second);

static void initialize(void);

void main(void)
{
  CLK_Config();
  GPIO_Config();
  TIM2_Config();
  TIM3_Config();

  initialize();

  enable_interrupts();
  while(1) {}
}

static void Delay(uint32_t nCount)
{
  while (nCount != 0) {
    nCount--;
  }
}

static void enable_interrupts(void)
{
  __asm__("rim\n");
}

static void initialize(void)
{
  /* Set TIM3 generate pulse routine to the highest priority */
  ITC_SetSoftwarePriority(TIM3_CC_IRQn, ITC_PriorityLevel_3);
}

static void GPIO_Config(void)
{
  /* LED */
  GPIO_Init(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_Mode_Out_PP_Low_Fast);

  /* Tx WIN out */
  GPIO_Init(GPIO_TX_WIN_OUT_PORT, GPIO_TX_WIN_OUT_PIN, GPIO_Mode_Out_PP_Low_Fast);
}

static void CLK_Config(void)
{
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
  CLK_SYSCLKSourceSwitchCmd(ENABLE);
  CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSI);

  /* Enable TIM2 clock */
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, ENABLE);

  /* Enable TIM3 clock */
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, ENABLE);
}

static void gpio_tx_win_out_pin_generate_one_pulse(uint16_t micro_second)
{
  TIM3_SetCounter(0);
  TIM3_SetCompare1(16*micro_second);
  GPIO_SetBits(GPIO_TX_WIN_OUT_PORT, GPIO_TX_WIN_OUT_PIN);
  TIM3_Cmd(ENABLE);
}

static void TIM2_Config(void)
{
  /* 16M/16 = 1MHz
     1/1MHz = 1us
     21000 = 21ms
   */
  TIM2_TimeBaseInit(TIM2_Prescaler_16, TIM2_CounterMode_Up, 21000);

  /* Clear TIM2 update flag */
  TIM2_ClearFlag(TIM2_FLAG_Update);

  /* Enable update interrupt */
  TIM2_ITConfig(TIM2_IT_Update, ENABLE);

  /* ENABLE TIM2 */
  TIM2_Cmd(ENABLE);
}

static void TIM3_Config(void)
{
  TIM3_TimeBaseInit(TIM3_Prescaler_1, TIM3_CounterMode_Up, 0xFFFF);

  TIM3_SelectOnePulseMode(TIM3_OPMode_Single);

  /* Clear TIM3 update flag */
  TIM3_ClearFlag(TIM3_FLAG_CC1);

  /* Enable TIM3 CC1 interrupt */
  TIM3_ITConfig(TIM3_IT_CC1, ENABLE);

  /* DISABLE TIM3 (Will Use Later) */
  TIM3_Cmd(DISABLE);
}

void tim2_upd_ovf(void) __interrupt(19)
{
  gpio_tx_win_out_pin_generate_one_pulse(1000);
  GPIO_ToggleBits(GPIO_LED_PORT, GPIO_LED_PIN);
  TIM2_ClearITPendingBit(TIM2_IT_Update);
}

void tim3_cc_usart3(void) __interrupt(22)
{
  GPIO_ResetBits(GPIO_TX_WIN_OUT_PORT, GPIO_TX_WIN_OUT_PIN);
  TIM3_ClearITPendingBit(TIM3_IT_CC1);
}
