/* ===================== Ali : LCD DISPLAY (I2C) ===================== */
I2C_HandleTypeDef hi2c1;
static uint8_t lcd_addr = 0x00;
#define LCD_RS  (1U<<0)
#define LCD_EN  (1U<<2)
#define LCD_BL  (1U<<3)

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1) {
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
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(common[i] << 1), 2, 50) == HAL_OK) return common[i];
  return 0;
}

static void lcd_write4(uint8_t nibble, uint8_t rs)
{
  uint8_t data = (uint8_t)(LCD_BL | (rs ? LCD_RS : 0) | (nibble << 4));
  uint8_t buf[2] = { (uint8_t)(data | LCD_EN), (uint8_t)(data & (uint8_t)~LCD_EN) };
  (void)HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, buf, 2, 50);
  HAL_Delay(1);
}
static void lcd_send(uint8_t val, uint8_t rs) { lcd_write4((val >> 4) & 0x0F, rs); lcd_write4(val & 0x0F, rs); }
static void lcd_cmd(uint8_t cmd) { lcd_send(cmd, 0); }
static void lcd_data(uint8_t d)  { lcd_send(d, 1); }
static void lcd_clear(void) { lcd_cmd(0x01); HAL_Delay(2); }
static void lcd_put_cur(uint8_t row, uint8_t col) { lcd_cmd((row == 0) ? (uint8_t)(0x80 + col) : (uint8_t)(0xC0 + col)); }
static void lcd_print(const char *s) { while (*s) lcd_data((uint8_t)*s++); }

static void lcd_init(void)
{
  HAL_Delay(50);
  lcd_write4(0x03, 0); HAL_Delay(5);
  lcd_write4(0x03, 0); HAL_Delay(5);
  lcd_write4(0x03, 0); HAL_Delay(5);
  lcd_write4(0x02, 0); HAL_Delay(1);
  lcd_cmd(0x28); lcd_cmd(0x0C); lcd_cmd(0x06); lcd_clear();
}

static void LCD_Show(uint8_t t, uint8_t h, int mode, uint8_t dht_ok, uint8_t reset_override)
{
  if (lcd_addr == 0) return;
  char line1[17], line2[17];
  if (!dht_ok) { snprintf(line1, 16, "DHT FAIL"); snprintf(line2, 16, "Check PA1"); }
  else {
    snprintf(line1, 16, "T:%2uC  H:%2u%%", t, h);
    if (mode == 2) snprintf(line2, 16, "MODE: PANIC ");
    else if (reset_override) snprintf(line2, 16, "RESET OVERRIDE");
    else if (mode == 1) snprintf(line2, 16, "MODE: ALARM ");
    else snprintf(line2, 16, "MODE: NORMAL");
  }
  lcd_put_cur(0,0); lcd_print(line1);
  lcd_put_cur(1,0); lcd_print(line2);
}
