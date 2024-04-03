#include "pca9555.h"

esp_err_t PCA9555::read_reg_16(uint8_t reg, uint16_t *val)
{
    CHECK_ARG(dev && val);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, reg, val, 2));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t PCA9555::write_reg_16(uint8_t reg, uint16_t val)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, reg, &val, 2));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t PCA9555::pca95x5_init_desc(uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev && (addr & PCA95X5_I2C_ADDR_BASE));

    dev->port = port;
    dev->addr = addr;
    dev->cfg.mode = I2C_MODE_MASTER;
    dev->cfg.sda_io_num = 5;
    dev->cfg.scl_io_num = 2;
    dev->cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    dev->cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    dev->cfg.master.clk_speed = 100000;

    ESP_LOGI("PCA", "PORT: %d, ADDR: %X, SDA: %d, SCL: %d", dev->port, dev->addr, dev->cfg.sda_io_num, dev->cfg.scl_io_num);

    return i2c_dev_create_mutex(dev);
}

esp_err_t PCA9555::pca95x5_free_desc()
{
    CHECK_ARG(dev);
    return i2c_dev_delete_mutex(dev);
}

esp_err_t PCA9555::pca95x5_port_get_mode(uint16_t *mode)
{
    return read_reg_16(REG_CONF0, mode);
}

esp_err_t PCA9555::pca95x5_port_set_mode(uint16_t mode)
{
    return write_reg_16(REG_CONF0, mode);
}

esp_err_t PCA9555::pca95x5_port_read(uint16_t *val)
{
    return read_reg_16(REG_IN0, val);
}

esp_err_t PCA9555::pca95x5_port_write(uint16_t val)
{
    return write_reg_16(REG_OUT0, val);
}

esp_err_t PCA9555::pca95x5_get_level(uint8_t pin, bool *val)
{
    uint16_t v;
    CHECK(read_reg_16(REG_IN0, &v));
    *val = v & BV(pin) ? 1 : 0;

    return ESP_OK;
}

esp_err_t PCA9555::pca95x5_set_level(uint8_t pin, bool val)
{
    if (val)
        bitSet(segment, pin);
    else
        bitClear(segment, pin);

    if (segment != segment_last)
    {
        I2C_DEV_TAKE_MUTEX(dev);
        I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, REG_OUT0, &segment, 2));
        I2C_DEV_GIVE_MUTEX(dev);
        segment_last = segment;
    }

    return ESP_OK;
}

esp_err_t PCA9555::pca95x5_set_led(bool led1, bool led2, bool led3)
{
    if (led1)
        bitSet(segment, P00_LED1);
    else
        bitClear(segment, P00_LED1);
    if (led2)
        bitSet(segment, P02_LED2);
    else
        bitClear(segment, P02_LED2);
    if (led3)
        bitSet(segment, P01_LED3);
    else
        bitClear(segment, P01_LED3);


    if (segment != segment_last)
    {
        I2C_DEV_TAKE_MUTEX(dev);
        I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, REG_OUT0, &segment, 2));
        I2C_DEV_GIVE_MUTEX(dev);
        segment_last = segment;
    }

    return ESP_OK;
}