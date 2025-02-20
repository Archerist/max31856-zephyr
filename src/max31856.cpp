#include "max31856.h"
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

struct spi_config config = {
    .frequency = 500000,
    .operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_MODE_CPHA,
    .slave = 0,
    .cs = NULL,
};

max31856::max31856(const device *spi_dev, const device *gpio_dev, gpio_pin_t cs_pin, uint8_t CR0_config, uint8_t CR1_config)
{
    max31856::spi_dev = spi_dev;
    max31856::gpio_dev = gpio_dev;
    max31856::pin = cs_pin;
    max31856::CR0_config = CR0_config;
    max31856::CR1_config = CR1_config;
    gpio_pin_configure(gpio_dev,cs_pin, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW);
    write(MAX31856_CR0_REG, CR0_config);
    write(MAX31856_CR1_REG, CR1_config);
    
}

max31856::~max31856()
{
}

int max31856::write(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = {reg | MAX31856_RD_WR_BIT, val};

    struct spi_buf tx_buf = {
        .buf = &data,
        .len = sizeof(data)};
    struct spi_buf_set tx_buf_set = {
        .buffers = &tx_buf,
        .count = 1};
    gpio_pin_set(gpio_dev, pin, 1);

    int ret = spi_write(max31856::spi_dev, &config, &tx_buf_set);
    gpio_pin_set(gpio_dev, pin, 0);
    return ret;
}

int max31856::read(uint8_t reg, uint8_t *result)
{
    uint8_t data[1] = {reg};
    uint8_t datarx[1];

    struct spi_buf tx_buf = {
        .buf = data,
        .len = sizeof(data)};
    struct spi_buf rx_buf = {
        .buf = datarx,
        .len = sizeof(datarx)};

    struct spi_buf_set tx_buf_set = {
        .buffers = &tx_buf,
        .count = 1};
    struct spi_buf_set rx_buf_set = {
        .buffers = &rx_buf,
        .count = 1};
    gpio_pin_set(gpio_dev, 14, 1);
    int ret = spi_write(spi_dev, &config, &tx_buf_set);
    ret = spi_read(spi_dev, &config, &rx_buf_set);
    gpio_pin_set(gpio_dev, pin, 0);

    *result = datarx[0];
    return ret;
}

int max31856::readTemperature(float *result)
{
    uint8_t HB;
    uint8_t MB;
    uint8_t LB;
    int ret = read(MAX31856_LTCBH_REG, &HB);
    ret = read(MAX31856_LTCBM_REG, &MB);
    ret = read(MAX31856_LTCBL_REG, &LB);

    printf("%02x,%02x,%02x\n", HB,MB,LB);
    
    int32_t i = (HB << 16) | (MB<<8) | (LB);
    i >>= 5;
    if(HB & 0x80){
        i -= 0x80000;
    }
    *result = i/128.f;
    return 0;
}

int max31856::readColdJunction(float *result)
{
    uint8_t HB;
    uint8_t LB;
    int ret = read(MAX31856_CJTH_REG, &HB);
    ret = read(MAX31856_CJTL_REG, &LB);

    uint16_t i = (HB << 8) | (LB);
    *result = (i >> 2)/64.f;

    return 0;
}
