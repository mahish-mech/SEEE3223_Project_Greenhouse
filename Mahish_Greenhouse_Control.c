/* ===============================================================================
   PROJECT: Smart Greenhouse System
   MAHISH (LEAD): System Logic, State Machine, GPIO Init, Main Loop
   =============================================================================== */
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>

/* ===================== REQUIRED FOR HAL_Delay ===================== */
void SysTick_Handler(void) { HAL_IncTick(); }
void SystemClock_Config(void) {}

/* ===================== USER SETTINGS ===================== */
#define TEMP_THRESHOLD_C      32
#define TEMP_HYSTERESIS_C      1    // alarm can re-enable when temp <= 31
#define ALARM_BEEP_ON_MS       200
#define ALARM_BEEP_PERIOD_MS  1000

/* ===================== CONFIRMED BOARD PINS ===================== */
#define SW_PORT   GPIOA
#define SW1_PIN   GPIO_PIN_2   // PANIC
#define SW2_PIN   GPIO_PIN_3   // RESET

#define BUZ_PORT  GPIOA
#define BUZ_PIN   GPIO_PIN_0
#define BUZ_ON()   HAL_GPIO_WritePin(BUZ_PORT, BUZ_PIN, GPIO_PIN_RESET)
#define BUZ_OFF()  HAL_GPIO_WritePin(BUZ_PORT, BUZ_PIN, GPIO_PIN_SET)

#define DHT_PORT  GPIOA
#define DHT_PIN   GPIO_PIN_1

#define SERVO_GPIO_PORT GPIOA
#define SERVO_GPIO_PIN  GPIO_PIN_9
#define SERVO_GPIO_AF   GPIO_AF1_TIM1

/* ===================== GPIO INIT (MAHISH) ===================== */
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

/* ===================== MAIN APPLICATION (MAHISH) ===================== */
typedef enum { MODE_NORMAL=0, MODE_ALARM=1, MODE_PANIC=2 } SystemMode;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  
  // NOTE: Other members must insert their Init functions here (MX_I2C1, MX_TIM1, DWT)
  // For now, we assume they will be added.
  
  /* Initial safe state */
  SystemMode mode = MODE_NORMAL;
  SystemMode prev_mode = MODE_NORMAL;
  uint8_t reset_override = 0;   
  
  // Initial Output States
  BUZ_OFF();
  
  GPIO_PinState sw1_prev = GPIO_PIN_RESET;
  GPIO_PinState sw2_prev = GPIO_PIN_RESET;
  uint32_t last_read_ms = 0;
  uint32_t alarm_cycle_start_ms = 0;
  uint8_t tempC = 0, hum = 0;
  uint8_t dht_ok = 0;

  while (1)
  {
    /* ---- Buttons Logic ---- */
    GPIO_PinState sw1 = HAL_GPIO_ReadPin(SW_PORT, SW1_PIN);
    GPIO_PinState sw2 = HAL_GPIO_ReadPin(SW_PORT, SW2_PIN);

    /* PANIC MODE LOGIC */
    if (sw1_prev == GPIO_PIN_RESET && sw1 == GPIO_PIN_SET) {
      mode = MODE_PANIC;
      reset_override = 0;           
      BUZ_ON();                     
    }

    /* RESET LOGIC */
    if (sw2_prev == GPIO_PIN_RESET && sw2 == GPIO_PIN_SET) {
      mode = MODE_NORMAL;
      reset_override = 1;           
      BUZ_OFF();
    }
    sw1_prev = sw1;
    sw2_prev = sw2;

    /* ---- System Heartbeat (Every 1s) ---- */
    if ((HAL_GetTick() - last_read_ms) >= 1000)
    {
      last_read_ms = HAL_GetTick();
      
      // CALL MEMBER 2 FUNCTION HERE: dht_ok = DHT11_Read(&tempC, &hum);
      
      /* Logic: If reset override active, check if cooled down */
      if (reset_override && dht_ok) {
        if (tempC <= (TEMP_THRESHOLD_C - TEMP_HYSTERESIS_C))
          reset_override = 0; 
      }

      /* Logic: Auto Control */
      if (mode != MODE_PANIC && !reset_override && dht_ok) {
        if (tempC >= TEMP_THRESHOLD_C) mode = MODE_ALARM;
        else if (tempC <= (TEMP_THRESHOLD_C - TEMP_HYSTERESIS_C)) mode = MODE_NORMAL;
      }

      /* Actuator Control */
      if (mode == MODE_NORMAL) {
        BUZ_OFF();
        // CALL MEMBER 4 FUNCTION: Servo_SetUS(SERVO_CLOSED_US);
      }
      else if (mode == MODE_PANIC) {
        // CALL MEMBER 4 FUNCTION: Servo_SetUS(SERVO_OPEN_US);
        BUZ_ON();
      }
      else { /* ALARM */
        // CALL MEMBER 4 FUNCTION: Servo_SetUS(SERVO_OPEN_US);
      }

      // CALL MEMBER 3 FUNCTION: LCD_Show(tempC, hum, mode, dht_ok, reset_override);
    }

    /* ---- Alarm Beep Logic ---- */
    if (mode != prev_mode) {
      if (mode == MODE_ALARM) alarm_cycle_start_ms = HAL_GetTick();
      if (prev_mode == MODE_ALARM && mode != MODE_PANIC) BUZ_OFF();
      prev_mode = mode;
    }

    if (mode == MODE_ALARM) {
      uint32_t dt = HAL_GetTick() - alarm_cycle_start_ms;
      if (dt >= ALARM_BEEP_PERIOD_MS) { alarm_cycle_start_ms = HAL_GetTick(); dt = 0; }
      if (dt < ALARM_BEEP_ON_MS) BUZ_ON(); else BUZ_OFF();
    }
    HAL_Delay(10);
  }
}
