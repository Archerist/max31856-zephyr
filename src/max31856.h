#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define MAX31856_RD_WR_BIT         BIT(7)
#define MAX31856_CR0_AUTOCONVERT   BIT(7)
#define MAX31856_CR0_1SHOT         BIT(6)
#define MAX31856_CR0_OCFAULT       BIT(4)
#define MAX31856_CR0_OCFAULT_MASK  GENMASK(5, 4)
#define MAX31856_CR0_FILTER_50HZ   BIT(0)
#define MAX31856_AVERAGING_MASK    GENMASK(6, 4)
#define MAX31856_AVERAGING_SHIFT   4
#define MAX31856_TC_TYPE_MASK      GENMASK(3, 0)
#define MAX31856_FAULT_OVUV        BIT(1)
#define MAX31856_FAULT_OPEN        BIT(0)
/* The MAX31856 registers */
#define MAX31856_CR0_REG           0x00
#define MAX31856_CR1_REG           0x01
#define MAX31856_MASK_REG          0x02
#define MAX31856_CJHF_REG          0x03
#define MAX31856_CJLF_REG          0x04
#define MAX31856_LTHFTH_REG        0x05
#define MAX31856_LTHFTL_REG        0x06
#define MAX31856_LTLFTH_REG        0x07
#define MAX31856_LTLFTL_REG        0x08
#define MAX31856_CJTO_REG          0x09
#define MAX31856_CJTH_REG          0x0A
#define MAX31856_CJTL_REG          0x0B
#define MAX31856_LTCBH_REG         0x0C
#define MAX31856_LTCBM_REG         0x0D
#define MAX31856_LTCBL_REG         0x0E
#define MAX31856_SR_REG            0x0F


class max31856
{
private:
    const device* spi_dev;
    const device* gpio_dev;
    int pin;
    uint8_t CR0_config;
    uint8_t CR1_config;
    
public:
    max31856(const device *spi_dev, const device *gpio_dev, gpio_pin_t pin, uint8_t CR0_config, uint8_t CR1_config);
    ~max31856();
    int write(uint8_t reg, uint8_t val);
    int read(uint8_t reg, uint8_t *result);

    int readTemperature(float *result);
    int readColdJunction(float *result);


};
