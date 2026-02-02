/* ===================== Shabiq : SERVO MOTOR (PWM) ===================== */
#define SERVO_CLOSED_US        2100
#define SERVO_OPEN_US          900
TIM_HandleTypeDef htim1;

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
