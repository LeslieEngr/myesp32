#ifndef __PCA9555_H__
#define __PCA9555_H__

#include <stddef.h>
#include <esp_err.h>
#include <esp_log.h>
#include "../i2cdev/i2cdev.h"

typedef enum
{
    P00_LED1 = 0,     // 0 P00
    P01_LED3,         // 1
    P02_LED2,         // 2
    P03_REVERSED,     // 3
    P04_REVERSED,     // 4
    P05_REVERSED,     // 5
    P06_REVERSED,     // 6
    P07_REVERSED,     // 7 P07
    P10_REVERSED,     // 0 P10
    P11_REVERSED,     // 1
    P12_SD_CS,     // 2
    P13_GSM_EN,       // 3
    P14_WDT,          // 4
    P15_ETH_PHY_nRST, // 5
    P16_GSM_RST,      // 6
    P17_GSM_PWR,      // 7 P17

} PCA9555_BIT;

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))

#define BV(x) (1 << (x))
#define CHECK(x)                \
    do                          \
    {                           \
        esp_err_t __;           \
        if ((__ = x) != ESP_OK) \
            return __;          \
    } while (0)

#define I2C_MASTER_SDA (gpio_num_t)5
#define I2C_MASTER_SCL (gpio_num_t)2

#define I2C_FREQ_HZ 400000
#define PCA95X5_I2C_ADDR_BASE 0x20

#define REG_IN0 0x00
#define REG_OUT0 0x02
#define REG_CONF0 0x06

class PCA9555
{
public:
    PCA9555()
    {
        dev = new i2c_dev_t();
    }

    ~PCA9555()
    {
    }

    esp_err_t pca95x5_init_desc(uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);
    esp_err_t pca95x5_free_desc();
    esp_err_t pca95x5_port_get_mode(uint16_t *mode);
    esp_err_t pca95x5_port_set_mode(uint16_t mode);
    esp_err_t pca95x5_port_read(uint16_t *val);
    esp_err_t pca95x5_port_write(uint16_t val);
    esp_err_t pca95x5_get_level(uint8_t pin, bool *val);
    esp_err_t pca95x5_set_level(uint8_t pin, bool val);

    esp_err_t pca95x5_set_led(bool led1, bool led2, bool led3);

private:
    esp_err_t read_reg_16(uint8_t reg, uint16_t *val);
    esp_err_t write_reg_16(uint8_t reg, uint16_t val);

    i2c_dev_t *dev = nullptr;

    uint16_t segment = 0;
    uint16_t segment_last = 0;
};

#endif
