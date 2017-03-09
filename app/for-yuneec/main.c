#include "stm8l15x.h"
#include "stm8l15x_usart.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_clk.h"
#include "stm8l15x_tim1.h"
#include "stm8l15x_tim2.h"
#include "stm8l15x_tim3.h"
#include "stm8l15x_tim4.h"
#include "stm8l15x_exti.h"
#include "stm8l15x_itc.h"
#include "stm8l15x_dac.h"
#include "stm8l15x_syscfg.h"
#include "version.h"

#if defined(LED)
#define GPIO_LED_PORT (GPIOA)
#define GPIO_LED_PIN  (GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7)
#endif
#define TIM2_PERIOD 0xFFFF

#define GPIO_TX_WIN_IN_PORT GPIOB
#define GPIO_TX_WIN_IN_PIN GPIO_Pin_1

#define GPIO_TX_WIN_OUT_PORT GPIOC // for credo demo board
#define GPIO_TX_WIN_OUT_PIN GPIO_Pin_6
//#define GPIO_TX_WIN_OUT_PORT GPIOB // for yuneec
//#define GPIO_TX_WIN_OUT_PIN GPIO_Pin_3

#define GPIO_TX_WIN_OUT2_PORT GPIOA
#define GPIO_TX_WIN_OUT2_PIN GPIO_Pin_5

enum {
  State_Calc_Falling_Edge_1 = 1,
  State_Calc_Falling_Edge_2,
  State_Calc_Falling_Edge_3,
};

static uint16_t sys_clk_high;
static uint32_t failed_loop = 333;

static uint8_t state;
static uint32_t last_clk_diff;
static uint32_t last_clk_count;
static uint32_t current_tx_window_measure_time = 0;
static uint32_t current_discovery_window_measure_time = 0;
static int uart_ready = 0;
static int gnd_tx_signal_triggered = 0;
static uint32_t dis_win_clk = 0;
static uint32_t last_dis_win_clk = 0;
static uint32_t period_clk = 0;
static uint32_t last_tx_trigger_time = 0;

static void Delay(uint32_t nCount);
static void enable_interrupts(void);
static void disable_interrupts(void);
static uint32_t clk_diff(uint32_t current, uint32_t last);
static void DAC_Config(void);

static uint8_t lockbit = 0;
static void lock(void);
static void unlock(void);
static uint8_t trylock(void);
static void CLK_Config(void);
static void GPIO_Config(void);
static void TIM2_Config(void);
static void TIM3_Config(void);
static void TIM4_Config(void);
static void gpio_tx_win_out_generate_one_pulse();

static void initialize(void);

static const char cb64[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/*
** encodeblock
**
** encode 3 8-bit binary bytes as 4 '6-bit' characters
*/
static void encodeblock( unsigned char *in, unsigned char *out, int len )
{
    out[0] = (unsigned char) cb64[ (int)(in[0] >> 2) ];
    out[1] = (unsigned char) cb64[ (int)(((in[0] & 0x03) << 4) | ((in[1] & 0xf0) >> 4)) ];
    out[2] = (unsigned char) (len > 1 ? cb64[ (int)(((in[1] & 0x0f) << 2) | ((in[2] & 0xc0) >> 6)) ] : '=');
    out[3] = (unsigned char) (len > 2 ? cb64[ (int)(in[2] & 0x3f) ] : '=');
}

/******************************************/
/* Auto Freq Offest */
#define UART_CMD_LED_SET_ERROR    'A'
#define UART_CMD_LED_SET_DSM      'B'
#define UART_CMD_LED_SET_FG_UP    'C'
#define UART_CMD_QUERY_ONLINE     'D'
#define UART_CMD_QUERY_DSM_CHANGE 'E'
#define UART_CMD_READ_FREQ_OFFSET 'F'
#define UART_CMD_READ_DEVICE_TYPE 'G'
#define UART_CMD_REPORT_DBA       'H'
#define UART_CMD_REPORT_MCU_VER   'I'

#define GROUND '1'
#define SKY '0'

enum {
  UAV_DEVICE_TYPE_GROUND = 1,
  UAV_DEVICE_TYPE_SKY,
  UAV_DEVICE_TYPE_UNKNOWN,
};

static void Delay_100ms();
static void putchar(uint8_t c);
static uint8_t getchar(void);

static void send_cmd(char cmd);
static int cmd_query_online(void);
static int cmd_set_led(char cmd);
static uint16_t cmd_get_freq_offset(void);
static void cmd_report_tx_discovery_window_measure_time(void);

static void lock(void)
{
	lockbit = 1;
}

static void unlock(void)
{
	lockbit = 0;
}

static uint8_t trylock(void)
{
	if (lockbit == 0) {
		lockbit = 1;
		return 1;
	}
	return 0;
}

static void Delay_100ms()
{
  Delay(0x19000);
}

static void USART_Config(void);
static void TIM1_Config(void);

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

#define TIM1_Prescaler_1 0

static void TIM1_Config(void)
{
  TIM1_TimeBaseInit(TIM1_Prescaler_1, TIM1_CounterMode_Up, 0xFFFF, 0);

	// PD3
	//GPIO_Init(GPIOD, GPIO_Pin_3, GPIO_Mode_In_FL_No_IT);
  TIM1_ETRClockMode2Config(TIM1_ExtTRGPSC_OFF, TIM1_ExtTRGPolarity_NonInverted, 0);
  TIM1_SelectInputTrigger(TIM1_TRGSelection_ETRF);

	// PD2
  //TIM1_TIxExternalClockConfig(TIM1_TIxExternalCLK1Source_TI1, TIM1_ICPolarity_Rising, 0);
  TIM1_Cmd(ENABLE);
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

  failed_loop = 0;
  /* Loop until the Read data register flag is SET */
  while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET) {
			failed_loop++;
			if (failed_loop > 0xFFFF)
      	return 'E';
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
  if (cmd_set_led(UART_CMD_LED_SET_DSM) == 2)
		return 2;
  val1 = TIM1_GetCounter();
  Delay_100ms();
  Delay_100ms();
  Delay_100ms();
  Delay_100ms();
  Delay_100ms();
  Delay_100ms();
  Delay_100ms();
  val2 = TIM1_GetCounter();
  if (val1 == val2) {
    return 0;
  } else {
    return 1;
	}
}

static int cmd_query_online(void)
{
  char ch;
  send_cmd(UART_CMD_QUERY_ONLINE);
  ch = getchar();
  if (ch == 'E')
    return 2;
  else if (ch == '1')
		return 1;
	else
    return 0;
}

static void cmd_report_tx_discovery_window_measure_time(void)
{
	uint32_t t, d;
	unsigned char in[8];
  char ch;
	int i;

	lock();
	putchar('^');
	putchar(UART_CMD_REPORT_DBA);
	//t = failed_loop;
	t = current_tx_window_measure_time;
	d = current_discovery_window_measure_time;

	in[0] = (t >> 24) & 0xFF;
	in[1] = (t >> 16) & 0xFF;
	in[2] = (t >> 8) & 0xFF;
	in[3] = (t) & 0xFF;
	in[4] = (d >> 24) & 0xFF;
	in[5] = (d >> 16) & 0xFF;
	in[6] = (d >> 8) & 0xFF;
	in[7] = (d) & 0xFF;

	/* base64 encodec */
	for (i = 0; i < 8; i+=3) {
		unsigned char t_in[3];
		unsigned char t_out[4];
		t_in[0] = in[i];
		t_in[1] = i+1 >= 8? 0: in[i+1];
		t_in[2] = i+2 >= 8? 0: in[i+2];
		encodeblock(t_in, t_out, 3);
		putchar(t_out[0]);
		putchar(t_out[1]);
		putchar(t_out[2]);
		putchar(t_out[3]);
	}

	putchar('$');
	unlock();
  ch = getchar();
}

static void cmd_report_mcu_version(void)
{
	uint32_t ver;
	int i;
	unsigned char in[4];
  char ch;
	putchar('^');
	putchar(UART_CMD_REPORT_MCU_VER);
	ver = MCU_VERSION_HASH;
	in[0] = (ver >> 24) & 0xFF;
	in[1] = (ver >> 16) & 0xFF;
	in[2] = (ver >> 8) & 0xFF;
	in[3] = (ver) & 0xFF;
	/* base64 encodec */
	for (i = 0; i < 4; i+=3) {
		unsigned char t_in[3];
		unsigned char t_out[4];
		t_in[0] = in[i];
		t_in[1] = i+1 >= 4? 0: in[i+1];
		t_in[2] = i+1 >= 4? 0: in[i+2];
		encodeblock(t_in, t_out, 3);
		putchar(t_out[0]);
		putchar(t_out[1]);
		putchar(t_out[2]);
		putchar(t_out[3]);
	}
	putchar('$');
  ch = getchar();
}

static int device_type = UAV_DEVICE_TYPE_UNKNOWN;

static int cmd_read_device_type(void)
{
  char ch;

  if (device_type != UAV_DEVICE_TYPE_UNKNOWN)
    return device_type;

  send_cmd(UART_CMD_READ_DEVICE_TYPE);
  ch = getchar();
	if (ch == 'E') {
		return UAV_DEVICE_TYPE_UNKNOWN;
	} else if (ch == GROUND) {
		uart_ready = 1;
    device_type = UAV_DEVICE_TYPE_GROUND;
    disable_interrupts();
		EXTI_SetPinSensitivity(EXTI_Pin_1, EXTI_Trigger_Rising);
    enable_interrupts();
    return UAV_DEVICE_TYPE_GROUND;
  } else if (ch == SKY) {
		uart_ready = 1;
    device_type = UAV_DEVICE_TYPE_SKY;
    return UAV_DEVICE_TYPE_SKY;
  } else {
    return UAV_DEVICE_TYPE_UNKNOWN;
  }
}

static int cmd_set_led(char cmd)
{
  char ch;
  send_cmd(cmd);
  ch = getchar();
	if (ch == 'E')
		return 2;
  else if (ch == 'O')
    return 1;
  else
    return 0;
}

static uint16_t cmd_get_freq_offset(void)
{
  uint8_t val1, val2;
	send_cmd('F');
  val1 = getchar();
  val2 = getchar();
	if (val1 == 'E' || val2 == 'E')
		return 0xFFFF;
	else
  	return (val1 << 8) | val2;
}

/******************************************/

int16_t get_c8800_freq_offset()
{
	int is_negative;
	uint16_t u = cmd_get_freq_offset();
	uint32_t x;
	if (u == 0xFFFF)
		return 0xFFFF;
	
	is_negative = 0;
	if (u & (1 << 14))
		is_negative = 1;
	
	if (is_negative)
		u = 0x3fff - (u & 0x3fff);
	else
		u = u & 0x3fff;
	
	x = u;
	x = x * 1000 / 16 * 25 / 1000;
	x = x / 15;
	if (is_negative)
		return x; /* if freq offset is negative, we should +x */
	else
		return -x;
}

void main(void)
{
  uint32_t current_voltage_value = 1878;
	uint32_t last_freq_offset_voltage = current_voltage_value;
	//uint32_t first_bootstrap = 1;
  int i;
  CLK_Config();

  /* GPIO PD0 (MASTER/SLAVE) */
  GPIO_Init(GPIOD, GPIO_Pin_0, GPIO_Mode_Out_PP_High_Fast);

  //for (i = 0; i < 50; i++)
   // Delay_100ms();
  GPIO_Config();
  USART_Config();
  DAC_Config();
  TIM1_Config();
  TIM2_Config();
  TIM3_Config();
  TIM4_Config();
  initialize();
  DAC_SetChannel1Data(DAC_Align_12b_R, current_voltage_value);

  enable_interrupts();

  while (1)
  {
  #if 1
    /* skip sky auto freq offset */
  	Delay_100ms();
  	Delay_100ms();
		if (uart_ready) cmd_report_tx_discovery_window_measure_time();
  	Delay_100ms();
  	Delay_100ms();
		if (uart_ready) cmd_report_mcu_version();
  	Delay_100ms();
    if (cmd_read_device_type() != UAV_DEVICE_TYPE_GROUND) continue;

    if (cmd_query_online() == 0) {

      int dsm_changed_times;
      dsm_changed_times = 0;
      while (1) {
				int val = cmd_query_dsm_change();
        if (val == 0) {
          for (i = 0; i <= 12; i++) {
            uint16_t val1, val2;
						uint16_t next_voltage;
            // Ref: 3.27V
            // Minial Voltage Move 0.15Hz
            // 5Hz / 0.15 = 33.3333
            // 1878 = 1.5 * 4095 / 3.27
            // 200 = 33.3333 * 6
				//		if (first_bootstrap) {
							next_voltage = 1878 + 200 - i*33;
				//		} else {
				//			next_voltage = last_freq_offset_voltage + (i % 2 == 0? (i/2) * 33 : -1 * (i+1)/2 * 33);
				//			next_voltage = 1878 + (i % 2 == 0? (i/2) * 33 : -1 * (i+1)/2 * 33);
				//		}
						current_voltage_value = next_voltage;
            DAC_SetChannel1Data(DAC_Align_12b_R, current_voltage_value);

            val1 = TIM1_GetCounter();
            Delay_100ms();
            Delay_100ms();
            Delay_100ms();
            val2 = TIM1_GetCounter();
            if (val1 != val2) {
						 	int16_t u = get_c8800_freq_offset();
						 	if (u == 0xFFFF)
						 		continue;
						 	current_voltage_value = current_voltage_value + u;
							last_freq_offset_voltage = current_voltage_value;
						 	DAC_SetChannel1Data(DAC_Align_12b_R, current_voltage_value);
						 	break;
            }
          }
        } else if (val == 1) {
          dsm_changed_times++; 
          if (dsm_changed_times > 3) {
				//		first_bootstrap = 0;
      			cmd_set_led(UART_CMD_LED_SET_ERROR);
            break;
					}
        } else {
					/* UART ERROR: break directly */
					break;
				}
      }
    } else {
			int16_t u = get_c8800_freq_offset();
			if (u != 0xFFFF && u != 0) {
				current_voltage_value = current_voltage_value + u;
				last_freq_offset_voltage = current_voltage_value;
				DAC_SetChannel1Data(DAC_Align_12b_R, current_voltage_value);
			}
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

static void disable_interrupts(void)
{
  __asm__("sim\n");
}

static void initialize(void)
{
  sys_clk_high = 0;
  state = State_Calc_Falling_Edge_1;

  /* Set Find Discovery Win interrupt to the highest priority */
  ITC_SetSoftwarePriority(EXTI1_IRQn, ITC_PriorityLevel_1);

  /* Set TIM3 generate pulse routine to the lowest priority */
  ITC_SetSoftwarePriority(TIM2_UPD_OVF_TRG_BRK_IRQn, ITC_PriorityLevel_3);

  /* Set TIM3 generate pulse routine to the lowest priority */
  ITC_SetSoftwarePriority(TIM3_CC_IRQn, ITC_PriorityLevel_2);

	/* Set RF ATTENNA switch routine to the lowest priority */
  ITC_SetSoftwarePriority(EXTI4_IRQn, ITC_PriorityLevel_3);
}

static uint32_t clk_diff(uint32_t current, uint32_t last)
{
  if (current > last)
    return current - last;
  else
    return TIM2_PERIOD - last + current;
}

static uint32_t clk_diff2(uint32_t current, uint32_t last)
{
  if (current > last)
    return current - last;
  else
    return last - current;
}

static void GPIO_Config(void)
{
	#if defined(LED)
  /* LED */
  GPIO_Init(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_Mode_Out_PP_Low_Fast);
	#endif

  /* Tx WIN Output */
  GPIO_Init(GPIO_TX_WIN_OUT_PORT, GPIO_TX_WIN_OUT_PIN, GPIO_Mode_Out_PP_Low_Fast);

	/* Tx WIN Output2 */
  GPIO_Init(GPIO_TX_WIN_OUT2_PORT, GPIO_TX_WIN_OUT2_PIN, GPIO_Mode_Out_PP_Low_Fast);

  /* Tx WIN Input (Interrupt) */
  GPIO_Init(GPIO_TX_WIN_IN_PORT, GPIO_TX_WIN_IN_PIN, GPIO_Mode_In_FL_IT);
  EXTI_SetPinSensitivity(EXTI_Pin_1, EXTI_Trigger_Falling);

	/* RF ATTENNA switch (C8800 -> MCU) */
  GPIO_Init(GPIOA, GPIO_Pin_4, GPIO_Mode_In_PU_IT);
  EXTI_SetPinSensitivity(EXTI_Pin_4, EXTI_Trigger_Rising_Falling);

	/* RF ATTENNA PB2 Low: RF1 High: RF2 */
  GPIO_Init(GPIOB, GPIO_Pin_2, GPIO_Mode_Out_PP_Low_Fast);
}

static void CLK_Config(void)
{
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
  CLK_SYSCLKSourceSwitchCmd(ENABLE);
  CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSI);

  while (CLK_GetSYSCLKSource() != CLK_SYSCLKSource_HSI) {}

  /* Enable TIM1 clock */
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM1, ENABLE);

  /* Enable TIM2 clock */
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, ENABLE);

  /* Enable TIM3 clock */
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, ENABLE);

  /* Enable TIM4 CLK */
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);

  /* Enable USART clock */
  CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE);

  /* Enable Clocks of DAC */
  CLK_PeripheralClockConfig(CLK_Peripheral_DAC, ENABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_COMP, ENABLE);
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

static uint32_t trig_counter = 0;
static uint32_t curr_trig_clk = 0;
static uint32_t period_counter = 0;

static void gpio_tx_win_out_generate_one_pulse()
{
  uint32_t tmp_clk;
  trig_counter++;
  GPIO_ResetBits(GPIO_TX_WIN_OUT_PORT, GPIO_TX_WIN_OUT_PIN);
  GPIO_ResetBits(GPIO_TX_WIN_OUT2_PORT, GPIO_TX_WIN_OUT2_PIN);
  curr_trig_clk = ((uint32_t)sys_clk_high << 16 | TIM2_GetCounter());

  TIM4_SetCounter(62); /* loss */
  TIM4_Cmd(ENABLE);

  if (uart_ready == 1 && gnd_tx_signal_triggered == 1 && cmd_read_device_type() == UAV_DEVICE_TYPE_GROUND) {
    if (trig_counter >= 1 && trig_counter <= 8) {
      if (trig_counter == 2 && (clk_diff2(curr_trig_clk, dis_win_clk) < period_clk)) {
        last_dis_win_clk = dis_win_clk;
        period_counter = 0;
      }
      tmp_clk = last_dis_win_clk + period_clk * period_counter + trig_counter * current_tx_window_measure_time;
      TIM3_SetCounter(0);
      TIM3_SetCompare1(tmp_clk - curr_trig_clk);
      TIM3_Cmd(ENABLE);
    } else if (trig_counter == 9) {
    trig_counter = 0;
      tmp_clk = last_dis_win_clk + period_clk * period_counter + period_clk;
      TIM3_SetCounter(0);
      TIM3_SetCompare1(tmp_clk - curr_trig_clk);
      TIM3_Cmd(ENABLE);
      trig_counter = 0;
      period_counter++;
    }
  }
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
  /* 16M/16 = 1MHz
     1/1MHz = 1us
   */
  TIM3_TimeBaseInit(TIM3_Prescaler_16, TIM3_CounterMode_Up, 0xFFFF);

  TIM3_SelectOnePulseMode(TIM3_OPMode_Single);

  /* Clear TIM3 update flag */
  TIM3_ClearFlag(TIM3_FLAG_CC1);

  /* Enable TIM3 CC1 interrupt */
  TIM3_ITConfig(TIM3_IT_CC1, ENABLE);

  /* DISABLE TIM3 (Will Use Later) */
  TIM3_Cmd(DISABLE);
}

#define TIM4_PERIOD       124

static void TIM4_Config(void)
{
  /* TIM4 configuration:
   - TIM4CLK is set to 16 MHz, the TIM4 Prescaler is equal to 128 so the TIM1 counter
   clock used is 16 MHz / 128 = 125 000 Hz
  - With 125 000 Hz we can generate time base:
      max time base is 2.048 ms if TIM4_PERIOD = 255 --> (255 + 1) / 125000 = 2.048 ms
      min time base is 0.016 ms if TIM4_PERIOD = 1   --> (  1 + 1) / 125000 = 0.016 ms
  - In this example we need to generate a time base equal to 1 ms
   so TIM4_PERIOD = (0.001 * 125000 - 1) = 124 */

  /* Time base configuration */
  TIM4_TimeBaseInit(TIM4_Prescaler_128, TIM4_PERIOD);

  TIM4_SelectOnePulseMode(TIM4_OPMode_Single);

  /* Clear TIM4 update flag */
  TIM4_ClearFlag(TIM4_FLAG_Update);

  /* Enable update interrupt */
  TIM4_ITConfig(TIM4_IT_Update, ENABLE);

  TIM4_Cmd(DISABLE);
}

uint8_t update_tx_discovery_measure_time(uint32_t t, uint32_t d)
{
	uint8_t valid = 0;
	uint32_t tmp;
	if (trylock() == 1) {
   	/* TODO Trigger Discovery Event (4ms) */
	  if ((t > 27000 && t < 28000) && (d > 33000 && d < 34000)) {
      dis_win_clk = (((uint32_t)sys_clk_high << 16) | TIM2_GetCounter());
			if (current_tx_window_measure_time == 0) {
	 			current_tx_window_measure_time = t;
			} else {
				tmp = current_tx_window_measure_time + t;
				current_tx_window_measure_time = tmp / 2;
			}

			if (current_discovery_window_measure_time == 0) {
	 			current_discovery_window_measure_time = d;
			} else {
	 			tmp = current_discovery_window_measure_time + d;
				current_discovery_window_measure_time = tmp / 2;
			}

      /* update dis_win_clk every time */
      period_clk = current_tx_window_measure_time * 8 + current_discovery_window_measure_time;

      if (uart_ready == 1 && gnd_tx_signal_triggered == 0 && cmd_read_device_type() == UAV_DEVICE_TYPE_GROUND) {
        /* for gnd only trigger once in isr */
        gnd_tx_signal_triggered = 1;
        last_dis_win_clk = dis_win_clk;
        gpio_tx_win_out_generate_one_pulse();
      }
			valid = 1;
		}
	  unlock();
	}
	return valid;
}

static uint8_t exti1_lock = 0;

/* EXTI1 GPIOB Pin_1 -> Tx Win Input */
void exti1_isr(void) __interrupt(9)
{
  uint32_t curr_clk_diff;
  uint32_t clk_count;

	exti1_lock = 1;

  if (uart_ready == 1 && cmd_read_device_type() == UAV_DEVICE_TYPE_SKY) {
    gpio_tx_win_out_generate_one_pulse();
  }

  clk_count = TIM2_GetCounter();
  last_tx_trigger_time = ((uint32_t)sys_clk_high << 16) | clk_count;

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
    if (curr_clk_diff > last_clk_diff && ((curr_clk_diff - last_clk_diff) > 1000)) {
			 #if defined(LED)
       GPIO_ToggleBits(GPIO_LED_PORT, GPIO_LED_PIN);
			 #endif

			 update_tx_discovery_measure_time(last_clk_diff, curr_clk_diff);

       state = State_Calc_Falling_Edge_1;
       clk_count = 0;
    } else if (curr_clk_diff < last_clk_diff && ((last_clk_diff - curr_clk_diff) > 1000)) {
  		//update_tx_discovery_measure_time(curr_clk_diff, last_clk_count);
      state = State_Calc_Falling_Edge_1;
      clk_count = 0;
    } else {
      last_clk_diff = clk_diff(clk_count, last_clk_count);
    }
    break;
  }

  /* Always update last_clk_count */
  last_clk_count = clk_count;

  EXTI_ClearITPendingBit(EXTI_IT_Pin1);
}

/* EXTI4 GPIOA Pin_4 */
void exti4_isr(void) __interrupt(12)
{
	BitStatus bit = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4);
	if (bit == 0) {
		/* ATTENNA 1 */
		GPIO_ResetBits(GPIOB, GPIO_Pin_2);
	} else {
		/* ATTENNA 2 */
		GPIO_SetBits(GPIOB, GPIO_Pin_2);
	}

  EXTI_ClearITPendingBit(EXTI_IT_Pin4);
}

void tim2_upd_ovf(void) __interrupt(19)
{
  sys_clk_high++; 
  TIM2_ClearITPendingBit(TIM2_IT_Update);
}

void tim3_cc_usart3(void) __interrupt(22)
{
  gpio_tx_win_out_generate_one_pulse();
  TIM3_ClearITPendingBit(TIM3_IT_CC1);
}

static uint32_t loss_delay = 0;

void tim4_upd_ovf_trg(void) __interrupt(25)
{
  if (loss_delay == 0) {
    if (clk_diff2(curr_trig_clk, last_tx_trigger_time) < 1000) {
      /* tx trigger */
      GPIO_SetBits(GPIO_TX_WIN_OUT_PORT, GPIO_TX_WIN_OUT_PIN);
      GPIO_SetBits(GPIO_TX_WIN_OUT2_PORT, GPIO_TX_WIN_OUT2_PIN);
    } else {
      loss_delay = 1;
      /* loss delay 1ms */
      TIM4_SetCounter(0); /* loss */
      TIM4_Cmd(ENABLE);
    }
  } else {
    GPIO_SetBits(GPIO_TX_WIN_OUT_PORT, GPIO_TX_WIN_OUT_PIN);
    GPIO_SetBits(GPIO_TX_WIN_OUT2_PORT, GPIO_TX_WIN_OUT2_PIN);
    loss_delay = 0;
  }
  TIM4_ClearITPendingBit(TIM4_IT_Update);
}
