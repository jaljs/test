#include "stm8l15x.h"

#include "stm8l15x_usart.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_clk.h"
#include "stm8l15x_tim1.h"
#include "stm8l15x_syscfg.h"
#include "stm8l15x_dac.h"

#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN  (GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7)

#define UART_CMD_LED_SET_ERROR    'A'
#define UART_CMD_LED_SET_DSM      'B'
#define UART_CMD_LED_SET_FG_UP    'C'
#define UART_CMD_QUERY_ONLINE     'D'
#define UART_CMD_QUERY_DSM_CHANGE 'E'
#define UART_CMD_READ_FREQ_OFFSET 'F'
#define UART_CMD_READ_DEVICE_TYPE 'G'

#define GROUND '1'
#define SKY '0'

enum {
  UAV_DEVICE_TYPE_GROUND = 1,
  UAV_DEVICE_TYPE_SKY,
  UAV_DEVICE_TYPE_UNKNOWN,
};

static void Delay (uint32_t nCount);
static void putchar(uint8_t c);
static uint8_t getchar(void);

static void send_cmd(char cmd);
static int cmd_query_online(void);
static int cmd_set_led(char cmd);
static uint16_t cmd_get_freq_offset(void);

static void CLK_Config(void);
static void GPIO_Config(void);
static void USART_Config(void);
static void DAC_Config(void);
static void TIM1_Config(void);

static void Delay(uint32_t nCount)
{
  while (nCount != 0) {
    nCount--;
  }
}

static void Delay_100ms()
{
  Delay(0x19000);
}

static void putchar(uint8_t c)
{
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
  USART_SendData8(USART1, c);
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

static uint8_t getchar(void)
{
  int c = 0;
  uint32_t failed_loop;

  failed_loop = 0;
  /* Loop until the Read data register flag is SET */
  while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET) {
//    failed_loop++;
//    if (failed_loop > 0xFFFF)
//      return 'E';
  }
  c = USART_ReceiveData8(USART1);
  return (c);
}

static void send_cmd(char cmd)
{
  putchar('^');
  putchar(cmd);
  putchar('$');
}

static int cmd_query_dsm_change(void)
{
  uint16_t val1, val2;
  val1 = TIM1_GetCounter();
  Delay_100ms();
  Delay_100ms();
  Delay_100ms();
  Delay_100ms();
  Delay_100ms();
  Delay_100ms();
  Delay_100ms();
  val2 = TIM1_GetCounter();
  if (val1 == val2)
    return 0;
  else
    return 1;
}

static int cmd_query_online(void)
{
  char ch;
  send_cmd(UART_CMD_QUERY_ONLINE);
  ch = getchar();
  if (ch == '1')
    return 1;
  else
    return 0;
}

static int cmd_read_device_type(void)
{
  char ch;
  send_cmd(UART_CMD_READ_DEVICE_TYPE);
  ch = getchar();
  if (ch == GROUND)
    return UAV_DEVICE_TYPE_GROUND;
  else if (ch == SKY)
    return UAV_DEVICE_TYPE_SKY;
  else
    return UAV_DEVICE_TYPE_UNKNOWN;
}

static int cmd_set_led(char cmd)
{
  char ch;
  send_cmd(cmd);
  ch = getchar();
  if (ch == 'O')
    return 1;
  else
    return 0;
}

static uint16_t cmd_get_freq_offset(void)
{
  uint8_t val1, val2;
  val1 = getchar();
  val2 = getchar();
  return (val1 << 8) | val2;
}

static void CLK_Config(void)
{
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
  CLK_SYSCLKSourceSwitchCmd(ENABLE);
  CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSI);
  while (CLK_GetSYSCLKSource() != CLK_SYSCLKSource_HSI) {}

  /* Enable TIM1 clock */
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM1, ENABLE);

  /* Enable USART clock */
  CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE);

  /* Enable Clocks of DAC */
  CLK_PeripheralClockConfig(CLK_Peripheral_DAC, ENABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_COMP, ENABLE);
}

static void GPIO_Config(void)
{
  /* LED */
  GPIO_Init(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_Mode_Out_PP_Low_Fast);
}

static void USART_Config(void)
{
  //GPIO_Init(GPIOA, GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3, GPIO_Mode_Out_PP_Low_Fast);
  GPIO_Init(GPIOA, GPIO_Pin_0 | GPIO_Pin_2, GPIO_Mode_Out_PP_Low_Fast);
  GPIO_Init(GPIOA, GPIO_Pin_3, GPIO_Mode_In_FL_No_IT);
  SYSCFG_REMAPPinConfig(REMAP_Pin_USART1TxRxPortA, ENABLE);
  SYSCFG_REMAPPinConfig(REMAP_Pin_USART1Clk, ENABLE);
  USART_DeInit(USART1);
  USART_Init(
    USART1,
    (uint32_t)9600,
    USART_WordLength_8b,
    USART_StopBits_1,
    USART_Parity_No,
    USART_Mode_Rx | USART_Mode_Tx
  );
}

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

#define TIM1_Prescaler_1 0

static void TIM1_Config(void)
{
  TIM1_TimeBaseInit(TIM1_Prescaler_1, TIM1_CounterMode_Up, 0xFFFF, 0);
  //TIM1_ETRClockMode2Config(TIM1_ExtTRGPSC_OFF, TIM1_ExtTRGPolarity_NonInverted, 0);
  //TIM1_SelectInputTrigger(TIM1_TRGSelection_ETRF);
  TIM1_TIxExternalClockConfig(TIM1_TIxExternalCLK1Source_TI1, TIM1_ICPolarity_Rising, 0);
  TIM1_Cmd(ENABLE);
}

void main(void)
{
  uint16_t current_freq_offset;
  uint8_t index_list[13];
  int i, j;
  CLK_Config();
  for (i = 0; i < 50; i++)
    Delay_100ms();
  GPIO_Config();
  USART_Config();
  DAC_Config();
  TIM1_Config();
  DAC_SetChannel1Data(DAC_Align_12b_R, 1878);
//  while (cmd_read_device_type() == UAV_DEVICE_TYPE_UNKNOWN); /* wait until c8800 uart initialized */
  
//  if (cmd_read_device_type() == UAV_DEVICE_TYPE_GROUND) {
  while (1)
  {
  #if 1
    if (cmd_query_online() == 0) {
      int dsm_changed_times;
      cmd_set_led(UART_CMD_LED_SET_DSM);
      dsm_changed_times = 0;
      while (1) {
        //if (cmd_query_dsm_change() == 0 && cmd_query_dsm_change() == 0 && cmd_query_dsm_change() == 0) {
        if (cmd_query_dsm_change() == 0) {
          for (i = 0; i <= 12; i++)
               index_list[i] = 0;

          j = 0;
          for (i = 0; i <= 12; i++) {
            uint16_t val1, val2;
            // Ref: 3.27V
            // Minial Voltage Move 0.15Hz
            // 5Hz / 0.15 = 33.3333
            // 1878 = 1.5 * 4095 / 3.27
            // 200 = 33.3333 * 6
            DAC_SetChannel1Data(DAC_Align_12b_R, 1878 + 200 - i*33);

            val1 = TIM1_GetCounter();
            Delay_100ms();
            Delay_100ms();
            val2 = TIM1_GetCounter();
            if (val1 != val2) {
               index_list[j++] = i;
            }
          }

          if (j == 0) {
            // recover to 1.5V
            DAC_SetChannel1Data(DAC_Align_12b_R, 1878);
          } else {
            // find middle index
            i = index_list[j/2];
            DAC_SetChannel1Data(DAC_Align_12b_R, 1878 + 200 - i*33);
          }
        } else {
          dsm_changed_times++; 
          if (dsm_changed_times > 3)
            break;
        }
      }
    } else {
      cmd_set_led(UART_CMD_LED_SET_ERROR);
    }
   #else
   // Delay_100ms();
   // GPIO_ToggleBits(GPIO_LED_PORT, GPIO_LED_PIN);
   DAC_SetChannel1Data(DAC_Align_12b_R, 1878);
   //DAC_SetChannel1Data(DAC_Align_12b_R, 1944);
   //cmd_set_led(UART_CMD_LED_SET_DSM);
   //cmd_query_online();
   #endif
  }
  //}
}
