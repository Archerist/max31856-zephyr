#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include "max31856.h"
#define SPI1_DEV    DT_NODELABEL(spi1)
#define GPIOD_DEV     DT_NODELABEL(gpiod)
#define BUTTON_NODE   DT_ALIAS(sw0)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);

const struct device *const spi1_dev = DEVICE_DT_GET(SPI1_DEV);

const struct device *const gpiod_dev = DEVICE_DT_GET(GPIOD_DEV);

struct gpio_callback cb_data;

int main(){

    if(device_is_ready(gpiod_dev)){
        printf("GPIOD Ready!\n");
    }

    if(device_is_ready(spi1_dev)){
        printf("SPI1 Ready!\n");
    }
    printf("Config Done!\n");
    max31856 sensor(spi1_dev, gpiod_dev,14,0b10110110,0b00010110) ;


    while(1){
        float result;
        sensor.readTemperature(&result);

        printf("Temperature: %3.2f\n", result);
        sensor.readColdJunction(&result);
    
        printf("Cold Junction: %3.2f\n", result);

        k_sleep(K_SECONDS(1));
    }

    return 0;
}

