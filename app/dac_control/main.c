#include "stm8l15x.h"

#include "stm8l15x_usart.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_clk.h"
#include "stm8l15x_tim1.h"
#include "stm8l15x_tim2.h"
#include "stm8l15x_tim3.h"
#include "stm8l15x_exti.h"
#include "stm8l15x_itc.h"
#include "stm8l15x_dac.h"
#include "stm8l15x_syscfg.h"

#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN  (GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7)

static void Delay (uint16_t nCount);
static void USART_Config(void);

static void DAC_Config(void)
{
  /* DAC Channel1 Config: 12bit right ----------------------------------------*/
  /* DAC deinitialize */
  DAC_DeInit();
  
  /* Fill DAC Init param DAC_Trigger_T4_TRGO and  DAC Channel1 Init */
  DAC_Init(DAC_Channel_1, DAC_Trigger_None, DAC_OutputBuffer_Disable);
  SYSCFG_RIDeInit();
  // RI->IOSR2 = RI_IOSR2_CH14E;
  //RI->IOSR1 = RI_IOSR1_CH13E;
   //RI->IOSR3 = RI_IOSR3_CH15E;
  //SYSCFG_RIIOSwitchConfig(RI_IOSwitch_13, ENABLE);
  //SYSCFG_RIIOSwitchConfig(RI_IOSwitch_14, ENABLE);
  SYSCFG_RIIOSwitchConfig(RI_IOSwitch_15, ENABLE);
  RI->IOCMR1 = 0x10;
  /* Enable DAC Channel1 */
  DAC_Cmd(DAC_Channel_1, ENABLE);
  
  /* Enable DMA for DAC Channel1 */
  DAC_DMACmd(DAC_Channel_1, DISABLE);
}

static void CLK_Config(void)
{
  /* Enable Clocks of DAC, TIM4 and DMA1 */
  CLK_PeripheralClockConfig(CLK_Peripheral_DAC, ENABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_COMP, ENABLE);
}

void main(void)
{
  uint16_t i, j;
  uint8_t step;
  CLK_Config();
  GPIO_Init(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_Mode_Out_PP_Low_Fast);
  DAC_Config();
  i = 0;
  step = 33;
  while (1)
  {
#if 0
    /* Toggle LEDs LD1..LD4 */
    for (i = 1862; i <= 3103;i+=1) {
      GPIO_ToggleBits(GPIO_LED_PORT, GPIO_LED_PIN);
      DAC_SetChannel1Data(DAC_Align_12b_R, i);
      Delay(0x8000);
    }
    for (i = 3103; i > 1862 ;i-=1) {
      GPIO_ToggleBits(GPIO_LED_PORT, GPIO_LED_PIN);
      DAC_SetChannel1Data(DAC_Align_12b_R, i);
      Delay(0x8000);
    }
#else
    // Ref: 3.27V
    // for (i = 0; i <= 12; i++) {
    //   GPIO_ToggleBits(GPIO_LED_PORT, GPIO_LED_PIN);
    //   DAC_SetChannel1Data(DAC_Align_12b_R, 1878 + 200 - i*step);
    //   for (j = 0;j < 10;j++)
    //     Delay(0xFFFF);
    // }

    DAC_SetChannel1Data(DAC_Align_12b_R, 1878);
    while (1);
#endif
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
