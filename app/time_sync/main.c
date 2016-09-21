#include "stdio.h"

#include "stm8l15x.h"
#include "stm8l15x_usart.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_clk.h"
#include "stm8l15x_tim1.h"
#include "stm8l15x_tim2.h"
#include "stm8l15x_tim3.h"
#include "stm8l15x_exti.h"
#include "stm8l15x_itc.h"

#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN  (GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7)
#define TIM2_PERIOD 0xFFFF

#define GPIO_TX_WIN_PORT GPIOB
#define GPIO_TX_WIN_IN GPIO_Pin_7
#define GPIO_TX_WIN_DISCOVERY_WIN_EVENT GPIO_Pin_6
#define GPIO_TX_WIN_TIME_SYNC_OUT GPIO_Pin_5
#define GPIO_TX_WIN_FIXOUT GPIO_Pin_4

enum {
  State_Calc_Falling_Edge_1 = 1,
  State_Calc_Falling_Edge_2,
  State_Calc_Falling_Edge_3,
};

static uint16_t sys_clk_high;

static uint8_t state;
static uint32_t last_clk_diff;
static uint32_t last_clk_count;

void putchar(char c);

static void Delay(uint32_t nCount);
static void enable_interrupts(void);
static uint32_t clk_diff(uint32_t current, uint32_t last);

static void CLK_Config(void);
static void GPIO_Config(void);
static void USART_Config(void);
static void TIM1_Config(void);
static void TIM2_Config(void);
static void TIM3_Config(void);
static void GPIOB_Pin_6_generate_one_pulse(uint16_t micro_second);
static void GPIOB_Pin_4_generate_one_pulse(uint16_t micro_second);

static void initialize(void);

void main(void)
{
  CLK_Config();
  USART_Config();
	GPIO_Config();
  TIM1_Config();
  TIM2_Config();
  TIM3_Config();

  initialize();

  enable_interrupts();
  while(1) {
    Delay(0x80000);
    printf("clk:%04x%04x\n", sys_clk_high, TIM2_GetCounter());
  }
}

void putchar(char c)
{
	USART_SendData8(USART1, c);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
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
  sys_clk_high = 0;
  state = State_Calc_Falling_Edge_1;

  /* Set Find Discovery Win interrupt to the highest priority */
  ITC_SetSoftwarePriority(EXTI7_IRQn, ITC_PriorityLevel_3);

  /* Set TIM1 generate pulse routine to the lowest priority */
  ITC_SetSoftwarePriority(TIM1_CC_IRQn, ITC_PriorityLevel_2);

  /* Set TIM3 generate pulse routine to the lowest priority */
  ITC_SetSoftwarePriority(TIM3_CC_IRQn, ITC_PriorityLevel_2);
}

static uint32_t clk_diff(uint32_t current, uint32_t last)
{
  if (current > last)
    return current - last;
  else
    return TIM2_PERIOD - last + current;
}

static void GPIO_Config(void)
{
	/* LED */
	GPIO_Init(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_Mode_Out_PP_Low_Fast);

  /* Discovery WIN Event */
  GPIO_Init(GPIO_TX_WIN_PORT, GPIO_TX_WIN_DISCOVERY_WIN_EVENT, GPIO_Mode_Out_PP_Low_Fast);

  /* Tx WIN Fix */
  GPIO_Init(GPIO_TX_WIN_PORT, GPIO_TX_WIN_FIXOUT, GPIO_Mode_Out_PP_Low_Fast);

  /* Tx WIN Input (Interrupt) */
  GPIO_Init(GPIO_TX_WIN_PORT, GPIO_TX_WIN_IN, GPIO_Mode_In_FL_IT);
  EXTI_SetPinSensitivity(EXTI_Pin_7, EXTI_Trigger_Falling);
}

static void CLK_Config(void)
{
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
  CLK_SYSCLKSourceSwitchCmd(ENABLE);
  CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSE);

  /* Enable TIM1 clock */
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM1, ENABLE);

  /* Enable TIM2 clock */
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, ENABLE);

  /* Enable TIM3 clock */
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, ENABLE);

  /* Enable USART clock */
  CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE);
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

#define TIM1_Prescaler_1 0

static void TIM1_Config(void)
{
  TIM1_TimeBaseInit(TIM1_Prescaler_1, TIM1_CounterMode_Up, 0xFFFF, 0);

  TIM1_SelectOnePulseMode(TIM1_OPMode_Single);

  /* Clear TIM1 update flag */
  TIM1_ClearFlag(TIM1_FLAG_CC1);

  /* Enable TIM1 CC1 interrupt */
  TIM1_ITConfig(TIM1_IT_CC1, ENABLE);

  /* DISABLE TIM1 (Will Use Later) */
  TIM1_Cmd(DISABLE);
}

static void GPIOB_Pin_6_generate_one_pulse(uint16_t micro_second)
{
  TIM1_SetCounter(0);
  TIM1_SetCompare1(16*micro_second);
  GPIO_SetBits(GPIO_TX_WIN_PORT, GPIO_TX_WIN_DISCOVERY_WIN_EVENT);
  TIM1_Cmd(ENABLE);
}

static void GPIOB_Pin_4_generate_one_pulse(uint16_t micro_second)
{
  TIM3_SetCounter(0);
  TIM3_SetCompare1(16*micro_second);
  GPIO_SetBits(GPIO_TX_WIN_PORT, GPIO_TX_WIN_FIXOUT);
  TIM3_Cmd(ENABLE);
}

static void TIM2_Config(void)
{
  /* 16M/16 = 1MHz
     1/1MHz = 1us
   */
  TIM2_TimeBaseInit(TIM2_Prescaler_16, TIM2_CounterMode_Up, TIM2_PERIOD);

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

void exti4_isr(void) __interrupt(12)
{
}

/* EXTI7 GPIOB Pin_7 -> Tx Win Input */
void exti7_isr(void) __interrupt(15)
{
  uint32_t curr_clk_diff;
  uint32_t clk_count;

  GPIOB_Pin_4_generate_one_pulse(1000); /* 1ms */

  clk_count = TIM2_GetCounter();
  switch(state) {
  case State_Calc_Falling_Edge_1:
    state = State_Calc_Falling_Edge_2;
    break;
  case State_Calc_Falling_Edge_2:
    last_clk_diff = clk_diff(clk_count, last_clk_count);
    state = State_Calc_Falling_Edge_3;
    break;
  case State_Calc_Falling_Edge_3:
    /*
       100 -> 100us
    */
    curr_clk_diff = clk_diff(clk_count, last_clk_count);
    if (curr_clk_diff > last_clk_diff && ((curr_clk_diff - last_clk_diff) > 100)) {
       GPIO_ToggleBits(GPIO_LED_PORT, GPIO_LED_PIN);

       /* Trigger Discovery Event (4ms) */
       GPIOB_Pin_6_generate_one_pulse(4000);

       state = State_Calc_Falling_Edge_1;
       clk_count = 0;
    } else if (curr_clk_diff < last_clk_diff && ((last_clk_diff - curr_clk_diff) > 100)) {
      state = State_Calc_Falling_Edge_1;
      clk_count = 0;
    } else {
      last_clk_diff = clk_diff(clk_count, last_clk_count);
    }
    break;
  }

  /* Always update last_clk_count */
  last_clk_count = clk_count;

  EXTI_ClearITPendingBit(EXTI_IT_Pin7);
}

void tim1_cc_capture(void) __interrupt(24)
{
  GPIO_ResetBits(GPIO_TX_WIN_PORT, GPIO_TX_WIN_DISCOVERY_WIN_EVENT);
  TIM1_ClearITPendingBit(TIM1_IT_CC1);
}

void tim2_upd_ovf(void) __interrupt(19)
{
  sys_clk_high++; 
  TIM2_ClearITPendingBit(TIM2_IT_Update);
}

void tim3_cc_usart3(void) __interrupt(22)
{
  GPIO_ResetBits(GPIO_TX_WIN_PORT, GPIO_TX_WIN_FIXOUT);
  TIM3_ClearITPendingBit(TIM3_IT_CC1);
}
