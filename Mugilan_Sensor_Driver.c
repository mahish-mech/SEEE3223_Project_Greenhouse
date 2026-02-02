/* ===================== Mugilan : DHT11 SENSOR & TIMING ===================== */
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
  while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) != state) {
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
  if (!DHT_Wait(GPIO_PIN_SET,    120)) return 0;
  if (!DHT_Wait(GPIO_PIN_RESET, 120)) return 0;

  for (int i = 0; i < 40; i++) {
    if (!DHT_Wait(GPIO_PIN_SET, 100)) return 0;
    delay_us(35);
    GPIO_PinState s = HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN);
    data[i/8] <<= 1;
    if (s == GPIO_PIN_SET) data[i/8] |= 1;
    if (!DHT_Wait(GPIO_PIN_RESET, 140)) return 0;
  }
  uint8_t sum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
  if (sum != data[4]) return 0;
  *hum    = data[0];
  *tempC = data[2];
  return 1;
}
