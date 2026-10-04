#include "OLED.h"

#include "ti_msp_dl_config.h"

#define OLED_I2C_ADDR_WRITE (0x78U)

#define OLED_GPIO_PORT      GPIOA
#define OLED_SCL_PIN        DL_GPIO_PIN_8
#define OLED_SDA_PIN        DL_GPIO_PIN_9
#define OLED_SCL_IOMUX      IOMUX_PINCM19
#define OLED_SDA_IOMUX      IOMUX_PINCM20

typedef struct {
    char ch;
    uint8_t data[5];
} OLED_Font5x7;

static const OLED_Font5x7 gFont5x7[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'K', {0x7F, 0x08, 0x14, 0x22, 0x41}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'N', {0x7F, 0x02, 0x04, 0x08, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'Y', {0x07, 0x08, 0x70, 0x08, 0x07}},
};

static void oled_delay(void)
{
    delay_cycles(120);
}

static void oled_pin_low(uint32_t pin, uint32_t iomux)
{
    DL_GPIO_clearPins(OLED_GPIO_PORT, pin);
    DL_GPIO_initDigitalOutputFeatures(iomux, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_DRIVE_STRENGTH_LOW,
        DL_GPIO_HIZ_DISABLE);
    DL_GPIO_enableOutput(OLED_GPIO_PORT, pin);
}

static void oled_pin_release(uint32_t iomux)
{
    uint32_t pin = (iomux == OLED_SCL_IOMUX) ? OLED_SCL_PIN : OLED_SDA_PIN;

    DL_GPIO_disableOutput(OLED_GPIO_PORT, pin);
    DL_GPIO_initDigitalInputFeatures(iomux, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

static void oled_scl_low(void)
{
    oled_pin_low(OLED_SCL_PIN, OLED_SCL_IOMUX);
}

static void oled_scl_high(void)
{
    oled_pin_release(OLED_SCL_IOMUX);
}

static void oled_sda_low(void)
{
    oled_pin_low(OLED_SDA_PIN, OLED_SDA_IOMUX);
}

static void oled_sda_high(void)
{
    oled_pin_release(OLED_SDA_IOMUX);
}

static void oled_i2c_start(void)
{
    oled_sda_high();
    oled_scl_high();
    oled_delay();
    oled_sda_low();
    oled_delay();
    oled_scl_low();
}

static void oled_i2c_stop(void)
{
    oled_sda_low();
    oled_delay();
    oled_scl_high();
    oled_delay();
    oled_sda_high();
    oled_delay();
}

static void oled_i2c_send_byte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8U; i++) {
        if ((byte & 0x80U) != 0U) {
            oled_sda_high();
        } else {
            oled_sda_low();
        }

        oled_delay();
        oled_scl_high();
        oled_delay();
        oled_scl_low();
        byte <<= 1;
    }

    oled_sda_high();
    oled_delay();
    oled_scl_high();
    oled_delay();
    oled_scl_low();
}

static void oled_write_byte(uint8_t control, uint8_t data)
{
    oled_i2c_start();
    oled_i2c_send_byte(OLED_I2C_ADDR_WRITE);
    oled_i2c_send_byte(control);
    oled_i2c_send_byte(data);
    oled_i2c_stop();
}

static void oled_write_command(uint8_t command)
{
    oled_write_byte(0x00U, command);
}

static void oled_write_data(uint8_t data)
{
    oled_write_byte(0x40U, data);
}

static void oled_set_cursor(uint8_t row, uint8_t column)
{
    oled_write_command((uint8_t) (0xB0U | (row & 0x07U)));
    oled_write_command((uint8_t) (0x10U | ((column >> 4U) & 0x0FU)));
    oled_write_command((uint8_t) (column & 0x0FU));
}

static const uint8_t *oled_find_font(char ch)
{
    for (uint32_t i = 0; i < (sizeof(gFont5x7) / sizeof(gFont5x7[0])); i++) {
        if (gFont5x7[i].ch == ch) {
            return gFont5x7[i].data;
        }
    }

    return gFont5x7[0].data;
}

void OLED_Init(void)
{
    DL_GPIO_enablePower(GPIOA);
    delay_cycles(POWER_STARTUP_DELAY);

    oled_scl_high();
    oled_sda_high();

    for (uint32_t i = 0; i < 50000U; i++) {
        __NOP();
    }

    oled_write_command(0xAEU);
    oled_write_command(0xD5U);
    oled_write_command(0x80U);
    oled_write_command(0xA8U);
    oled_write_command(0x3FU);
    oled_write_command(0xD3U);
    oled_write_command(0x00U);
    oled_write_command(0x40U);
    oled_write_command(0xA1U);
    oled_write_command(0xC8U);
    oled_write_command(0xDAU);
    oled_write_command(0x12U);
    oled_write_command(0x81U);
    oled_write_command(0xCFU);
    oled_write_command(0xD9U);
    oled_write_command(0xF1U);
    oled_write_command(0xDBU);
    oled_write_command(0x30U);
    oled_write_command(0xA4U);
    oled_write_command(0xA6U);
    oled_write_command(0x8DU);
    oled_write_command(0x14U);
    oled_write_command(0xAFU);
    OLED_Clear();
}

void OLED_Clear(void)
{
    for (uint8_t row = 0; row < 8U; row++) {
        oled_set_cursor(row, 0);
        for (uint8_t column = 0; column < 128U; column++) {
            oled_write_data(0x00U);
        }
    }
}

void OLED_ShowChar(uint8_t row, uint8_t column, char ch)
{
    const uint8_t *font = oled_find_font(ch);
    uint8_t x = (uint8_t) (column * 6U);

    if ((row > 7U) || (x > 122U)) {
        return;
    }

    oled_set_cursor(row, x);
    for (uint8_t i = 0; i < 5U; i++) {
        oled_write_data(font[i]);
    }
    oled_write_data(0x00U);
}

void OLED_ShowString(uint8_t row, uint8_t column, const char *string)
{
    while ((*string != '\0') && (column < 21U)) {
        OLED_ShowChar(row, column, *string);
        column++;
        string++;
    }
}

void OLED_ShowNum(uint8_t row, uint8_t column, uint32_t number, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++) {
        uint32_t divisor = 1U;

        for (uint8_t j = 0; j < (uint8_t) (length - i - 1U); j++) {
            divisor *= 10U;
        }

        OLED_ShowChar(row, (uint8_t) (column + i),
            (char) ('0' + ((number / divisor) % 10U)));
    }
}

static int32_t oled_float_to_tenths(float value)
{
    if (value >= 0.0f) {
        return (int32_t) (value * 10.0f + 0.5f);
    }

    return (int32_t) (value * 10.0f - 0.5f);
}

void OLED_ShowSignedFloat1(uint8_t row, uint8_t column, float value)
{
    int32_t scaled = oled_float_to_tenths(value);

    if (scaled < 0) {
        OLED_ShowChar(row, column, '-');
        scaled = -scaled;
    } else {
        OLED_ShowChar(row, column, ' ');
    }

    if (scaled > 9999) {
        scaled = 9999;
    }

    OLED_ShowNum(row, (uint8_t) (column + 1U), (uint32_t) (scaled / 10), 3);
    OLED_ShowChar(row, (uint8_t) (column + 4U), '.');
    OLED_ShowNum(row, (uint8_t) (column + 5U), (uint32_t) (scaled % 10), 1);
}
