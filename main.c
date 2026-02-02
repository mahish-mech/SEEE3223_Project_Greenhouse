#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>

/* ===================== REQUIRED FOR HAL_Delay ===================== */
void SysTick_Handler(void) { HAL_IncTick(); }
void SystemClock_Config(void) {}

/* ===================== USER SETTINGS ===================== */
#define TEMP_THRESHOLD_C      32
#define TEMP_HYSTERESIS_C      1   // alarm can re-enable when temp <= 31

/* Servo travel (your confirmed good values) */
#define SERVO_CLOSED_US        2100
#define SERVO_OPEN_US         900

/* Alarm beep pattern */
#define ALARM_BEEP_ON_MS       200
#define ALARM_BEEP_PERIOD_MS  1000

/* ===================== CONFIRMED BOARD PINS ===================== */
/* Buttons (active HIGH) */
#define SW_PORT   GPIOA
#define SW1_PIN   GPIO_PIN_2   // PANIC
#define SW2_PIN   GPIO_PIN_3   // RESET

/* External buzzer module on PA0 (ACTIVE-LOW) */
#define BUZ_PORT  GPIOA
#define BUZ_PIN   GPIO_PIN_0
#define BUZ_ON()   HAL_GPIO_WritePin(BUZ_PORT, BUZ_PIN, GPIO_PIN_RESET)
#define BUZ_OFF()  HAL_GPIO_WritePin(BUZ_PORT, BUZ_PIN, GPIO_PIN_SET)

/* DHT11 */
#define DHT_PORT  GPIOA
#define DHT_PIN   GPIO_PIN_1

/* Servo: PA9 = TIM1_CH2 */
#define SERVO_GPIO_PORT GPIOA
#define SERVO_GPIO_PIN  GPIO_PIN_9
#define SERVO_GPIO_AF   GPIO_AF1_TIM1

/* LCD I2C1: PB8/PB9 */
I2C_HandleTypeDef hi2c1;
static uint8_t lcd_addr = 0x00;   // 8-bit address (7-bit<<1)

/* PCF8574 mapping */
#define LCD_RS  (1U<<0)
#define LCD_EN  (1U<<2)
#define LCD_BL  (1U<<3)

/* Servo timer */
TIM_HandleTypeDef htim1;

/* ===================== DWT microsecond delay (for DHT11) ===================== */
static void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = (SystemCoreClock / 1000000U) * us;
  while ((DWT->CYCCNT - start) < ticks) { }
}

/* ===================== GPIO INIT ===================== */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gi = {0};

  /* Buttons: active-high => PULLDOWN */
  gi.Pin = SW1_PIN | SW2_PIN;
  gi.Mode = GPIO_MODE_INPUT;
  gi.Pull = GPIO_PULLDOWN;
  gi.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SW_PORT, &gi);

  /* Buzzer output */
  gi.Pin = BUZ_PIN;
  gi.Mode = GPIO_MODE_OUTPUT_PP;
  gi.Pull = GPIO_NOPULL;
  gi.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZ_PORT, &gi);
  BUZ_OFF();

  /* DHT default input pull-up */
  gi.Pin = DHT_PIN;
  gi.Mode = GPIO_MODE_INPUT;
  gi.Pull = GPIO_PULLUP;
  gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DHT_PORT, &gi);
}

/* ===================== I2C1 MSP INIT ===================== */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef gi = {0};
    gi.Pin = GPIO_PIN_8 | GPIO_PIN_9;    // PB8=SCL, PB9=SDA
    gi.Mode = GPIO_MODE_AF_OD;
    gi.Pull = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gi);
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  (void)HAL_I2C_Init(&hi2c1);
}

static uint8_t I2C_FindLCD(void)
{
  uint8_t common[] = {0x27, 0x3F};
  for (uint32_t i = 0; i < sizeof(common); i++)
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(common[i] << 1), 2, 50) == HAL_OK)
      return common[i];

  for (uint8_t a = 0x08; a <= 0x77; a++)
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 2, 20) == HAL_OK)
      return a;

  return 0;
}

/* ===================== LCD ===================== */
static void lcd_write4(uint8_t nibble, uint8_t rs)
{
  uint8_t data = (uint8_t)(LCD_BL | (rs ? LCD_RS : 0) | (nibble << 4));
  uint8_t buf[2] = { (uint8_t)(data | LCD_EN), (uint8_t)(data & (uint8_t)~LCD_EN) };
  (void)HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, buf, 2, 50);
  HAL_Delay(1);
}
static void lcd_send(uint8_t val, uint8_t rs)
{
  lcd_write4((val >> 4) & 0x0F, rs);
  lcd_write4(val & 0x0F, rs);
}
static void lcd_cmd(uint8_t cmd) { lcd_send(cmd, 0); }
static void lcd_data(uint8_t d)  { lcd_send(d, 1); }
static void lcd_clear(void) { lcd_cmd(0x01); HAL_Delay(2); }
static void lcd_put_cur(uint8_t row, uint8_t col)
{
  lcd_cmd((row == 0) ? (uint8_t)(0x80 + col) : (uint8_t)(0xC0 + col));
}
static void lcd_print(const char *s) { while (*s) lcd_data((uint8_t)*s++); }

static void lcd_init(void)
{
  HAL_Delay(50);
  lcd_write4(0x03, 0); HAL_Delay(5);
  lcd_write4(0x03, 0); HAL_Delay(5);
  lcd_write4(0x03, 0); HAL_Delay(5);
  lcd_write4(0x02, 0); HAL_Delay(1);

  lcd_cmd(0x28);
  lcd_cmd(0x0C);
  lcd_cmd(0x06);
  lcd_clear();
}

/* ===================== SERVO (TIM1 CH2 on PA9) ===================== */
static void MX_TIM1_Init_ForServo(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef gi = {0};
  gi.Pin = SERVO_GPIO_PIN;
  gi.Mode = GPIO_MODE_AF_PP;
  gi.Pull = GPIO_NOPULL;
  gi.Speed = GPIO_SPEED_FREQ_HIGH;
  gi.Alternate = SERVO_GPIO_AF;
  HAL_GPIO_Init(SERVO_GPIO_PORT, &gi);

  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  uint32_t timclk = pclk2;
  if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) timclk = 2 * pclk2;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = (timclk / 1000000U) - 1U;   // 1us tick
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000 - 1;                     // 50Hz
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;

  (void)HAL_TIM_PWM_Init(&htim1);

  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = SERVO_CLOSED_US;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;

  (void)HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

static void Servo_SetUS(uint16_t us)
{
  if (us < 600) us = 600;
  if (us > 2400) us = 2400;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, us);
}

/* ===================== DHT11 ===================== */
static void DHT_SetOutput(void)
{
  GPIO_InitTypeDef gi = {0};
  gi.Pin = DHT_PIN;
  gi.Mode = GPIO_MODE_OUTPUT_PP;
  gi.Pull = GPIO_NOPULL;
  gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DHT_PORT, &gi);
}
static void DHT_SetInput(void)
{
  GPIO_InitTypeDef gi = {0};
  gi.Pin = DHT_PIN;
  gi.Mode = GPIO_MODE_INPUT;
  gi.Pull = GPIO_PULLUP;
  gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DHT_PORT, &gi);
}
static uint8_t DHT_Wait(GPIO_PinState state, uint32_t timeout_us)
{
  uint32_t t = 0;
  while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) != state)
  {
    if (t++ >= timeout_us) return 0;
    delay_us(1);
  }
  return 1;
}
static uint8_t DHT11_Read(uint8_t *tempC, uint8_t *hum)
{
  uint8_t data[5] = {0};

  DHT_SetOutput();
  HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET);
  HAL_Delay(18);
  HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);
  delay_us(30);
  DHT_SetInput();

  if (!DHT_Wait(GPIO_PIN_RESET, 120)) return 0;
  if (!DHT_Wait(GPIO_PIN_SET,   120)) return 0;
  if (!DHT_Wait(GPIO_PIN_RESET, 120)) return 0;

  for (int i = 0; i < 40; i++)
  {
    if (!DHT_Wait(GPIO_PIN_SET, 100)) return 0;
    delay_us(35);
    GPIO_PinState s = HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN);

    data[i/8] <<= 1;
    if (s == GPIO_PIN_SET) data[i/8] |= 1;

    if (!DHT_Wait(GPIO_PIN_RESET, 140)) return 0;
  }

  uint8_t sum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
  if (sum != data[4]) return 0;

  *hum   = data[0];
  *tempC = data[2];
  return 1;
}

/* ===================== APP ===================== */
typedef enum { MODE_NORMAL=0, MODE_ALARM=1, MODE_PANIC=2 } SystemMode;

static void LCD_Show(uint8_t t, uint8_t h, SystemMode mode, uint8_t dht_ok, uint8_t reset_override)
{
  if (lcd_addr == 0) return;

  char line1[17], line2[17];

  if (!dht_ok)
  {
    snprintf(line1, sizeof(line1), "DHT FAIL");
    snprintf(line2, sizeof(line2), "Check PA1");
  }
  else
  {
    snprintf(line1, sizeof(line1), "T:%2uC  H:%2u%%", t, h);

    if (mode == MODE_PANIC) snprintf(line2, sizeof(line2), "MODE: PANIC ");
    else if (reset_override) snprintf(line2, sizeof(line2), "RESET OVERRIDE");
    else if (mode == MODE_ALARM) snprintf(line2, sizeof(line2), "MODE: ALARM ");
    else snprintf(line2, sizeof(line2), "MODE: NORMAL");
  }

  lcd_put_cur(0,0); lcd_print("                ");
  lcd_put_cur(1,0); lcd_print("                ");
  lcd_put_cur(0,0); lcd_print(line1);
  lcd_put_cur(1,0); lcd_print(line2);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init_ForServo();
  DWT_Init();

  /* LCD */
  uint8_t addr7 = I2C_FindLCD();
  if (addr7 != 0)
  {
    lcd_addr = (uint8_t)(addr7 << 1);
    lcd_init();
    lcd_put_cur(0,0); lcd_print("Smart Greenhouse");
    lcd_put_cur(1,0); lcd_print("Starting...");
    HAL_Delay(500);
  }
  else
  {
    lcd_addr = 0; // run without LCD if not found
  }

  /* Initial safe state */
  SystemMode mode = MODE_NORMAL;
  SystemMode prev_mode = MODE_NORMAL;

  uint8_t reset_override = 0;   // when 1, force NORMAL regardless of temp
  Servo_SetUS(SERVO_CLOSED_US);
  BUZ_OFF();

  GPIO_PinState sw1_prev = GPIO_PIN_RESET;
  GPIO_PinState sw2_prev = GPIO_PIN_RESET;

  uint32_t last_read_ms = 0;
  uint32_t alarm_cycle_start_ms = 0;

  uint8_t tempC = 0, hum = 0;
  uint8_t dht_ok = 0;

  while (1)
  {
    /* ---- Buttons ---- */
    GPIO_PinState sw1 = HAL_GPIO_ReadPin(SW_PORT, SW1_PIN);
    GPIO_PinState sw2 = HAL_GPIO_ReadPin(SW_PORT, SW2_PIN);

    /* PANIC */
    if (sw1_prev == GPIO_PIN_RESET && sw1 == GPIO_PIN_SET)
    {
      mode = MODE_PANIC;
      reset_override = 0;              // panic cancels override
      Servo_SetUS(SERVO_OPEN_US);
      BUZ_ON();                        // continuous
    }

    /* RESET: force NORMAL no matter temp */
    if (sw2_prev == GPIO_PIN_RESET && sw2 == GPIO_PIN_SET)
    {
      mode = MODE_NORMAL;
      reset_override = 1;              // latch override
      BUZ_OFF();
      Servo_SetUS(SERVO_CLOSED_US);
    }

    sw1_prev = sw1;
    sw2_prev = sw2;

    /* ---- Read DHT every 1s ---- */
    if ((HAL_GetTick() - last_read_ms) >= 1000)
    {
      last_read_ms = HAL_GetTick();
      dht_ok = DHT11_Read(&tempC, &hum);

      /* If reset override is active, keep NORMAL until it cools down enough */
      if (reset_override && dht_ok)
      {
        if (tempC <= (TEMP_THRESHOLD_C - TEMP_HYSTERESIS_C))
          reset_override = 0; // allow auto again
      }

      /* Auto control only if NOT panic and NOT reset override */
      if (mode != MODE_PANIC && !reset_override && dht_ok)
      {
        if (tempC >= TEMP_THRESHOLD_C) mode = MODE_ALARM;
        else if (tempC <= (TEMP_THRESHOLD_C - TEMP_HYSTERESIS_C)) mode = MODE_NORMAL;
      }

      /* Apply outputs for NORMAL immediately on each read */
      if (mode == MODE_NORMAL)
      {
        BUZ_OFF();
        Servo_SetUS(SERVO_CLOSED_US);
      }
      else if (mode == MODE_PANIC)
      {
        Servo_SetUS(SERVO_OPEN_US);
        BUZ_ON();
      }
      else /* MODE_ALARM */
      {
        Servo_SetUS(SERVO_OPEN_US);
      }

      LCD_Show(tempC, hum, mode, dht_ok, reset_override);
    }

    /* ---- Enter/Exit ALARM handling ---- */
    if (mode != prev_mode)
    {
      if (mode == MODE_ALARM)
      {
        alarm_cycle_start_ms = HAL_GetTick(); // start beep cycle when entering alarm
      }
      if (prev_mode == MODE_ALARM && mode != MODE_PANIC)
      {
        BUZ_OFF(); // leaving alarm -> ensure buzzer off (unless panic)
      }
      prev_mode = mode;
    }

    /* ---- Alarm beep pattern (runs continuously while in ALARM) ---- */
    if (mode == MODE_ALARM)
    {
      uint32_t now = HAL_GetTick();
      uint32_t dt = now - alarm_cycle_start_ms;

      if (dt >= ALARM_BEEP_PERIOD_MS)
      {
        alarm_cycle_start_ms = now;
        dt = 0;
      }

      if (dt < ALARM_BEEP_ON_MS) BUZ_ON();
      else BUZ_OFF();
    }

    HAL_Delay(10);
  }
}
