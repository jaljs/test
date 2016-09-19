#include "stm8l15x.h"
#include "printf.h"
#include "stm8l15x_usart.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_clk.h"
#include "stm8l15x_tim2.h"

#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN  (GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7)
#define TIM2_PERIOD 1249

#define GPIO_TX_WIN_PORT GPIOB
#define GPIO_TX_WIN_IN GPIO_Pin_7
#define GPIO_TX_WIN_DISCOVERY_WIN_EVENT GPIO_Pin_6
#define GPIO_TX_WIN_TIME_SYNC_OUT GPIO_Pin_5

enum {
  STATE_TRIG_TX_WIN_FL_EG_1 = 1,
  STATE_TRIG_TX_WIN_FL_EG_2,
  STATE_TRIG_TX_WIN_FL_EG_3,
};

static uint32_t sys_clk_count;
static uint8_t state_tx_win;

void putchar(char c);

static void Delay(uint32_t nCount);
static void enable_interrupts(void);

static void CLK_Config(void);
static void GPIO_Config(void);
static void USART_Config(void);
static void TIM2_Config(void);

static void initialize(void);

void main(void)
{
  CLK_Config();
  USART_Config();
	GPIO_Config();
  TIM2_Config();

  initialize();

  enable_interrupts();

	while(1) {
    Delay(0x80000);
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
  sys_clk_count = 0;
  state_tx_win = STATE_TRIG_TX_WIN_FL_EG_1;
}

static void GPIO_Config(void)
{
	/* LED */
	GPIO_Init(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_Mode_Out_PP_Low_Fast);
}

static void CLK_Config(void)
{
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);

  /* Enable TIM2 clock */
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, ENABLE);
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

static void TIM2_Config(void)
{
  /* 16M/128 = 125000HZ
     TIM2_PERIOD = 0.01s * 125000 - 1 = 1249
   */
  TIM2_TimeBaseInit(TIM2_Prescaler_128, TIM2_CounterMode_Up, TIM2_PERIOD);

  /* Clear TIM2 update flag */
  TIM2_ClearFlag(TIM2_FLAG_Update);

  /* Enable update interrupt */
  TIM2_ITConfig(TIM2_IT_Update, ENABLE);

  /* Enable TIM2 */
  TIM2_Cmd(ENABLE);
}

void exti4_isr(void) __interrupt(12)
{
}

void exti7_isr(void) __interrupt(15)
{

}

void tim2_upd_ovf(void) __interrupt(19)
{
  sys_clk_count++; 
  if (sys_clk_count == 100) {
    GPIO_ToggleBits(GPIO_LED_PORT, GPIO_LED_PIN);
    sys_clk_count = 0;
  }
  TIM2_ClearITPendingBit(TIM2_IT_Update);
}
