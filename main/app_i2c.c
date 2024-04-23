#include "app_include/app_i2c.h"

/**
 * @brief i2c master initialization
 */
esp_err_t app_i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Proper implementation of this API is yet to be done

// This function populates a data buffer for writing I2C data to the AD5272. All logical protection should be completed elsewhere (for instance some data should only be 6 bits, etc.)
static void ad5272_update_data_buff(uint8_t buff[AD5272_TRANSACTION_SIZE_BYTES], uint8_t command, uint16_t data) {
    buff[0] = (uint8_t)((command << 2) | (data >> 8));
    buff[AD5272_TRANSACTION_SIZE_BYTES - 1] = (uint8_t)(data);
}

esp_err_t ad5272_ctrl_reg_write(uint8_t code) {
    uint8_t buff[AD5272_TRANSACTION_SIZE_BYTES];

    if (!(code <= 0x7)) { /* Only bits 0:2 are writeable. Bit 3 is to monitor status of most recent 50tp write */
        return ESP_FAIL;
    }

    ad5272_update_data_buff(buff, AD5272_CTRL_WRITE, code);
    return i2c_master_write_to_device(I2C_MASTER_NUM, AD5272_ADDR, &buff, AD5272_TRANSACTION_SIZE_BYTES, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t ad5272_ctrl_reg_read(uint8_t read_buff[AD5272_TRANSACTION_SIZE_BYTES]) {
    uint8_t buff[AD5272_TRANSACTION_SIZE_BYTES];

    ad5272_update_data_buff(buff, AD5272_CTRL_READ, 0x0);
    return i2c_master_write_read_device(I2C_MASTER_NUM, AD5272_ADDR, &buff, AD5272_TRANSACTION_SIZE_BYTES, read_buff, AD5272_TRANSACTION_SIZE_BYTES, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/* This function will only work for updating the RDAC granted that the control register RDAC WEN bit has been set */
esp_err_t ad5272_rdac_reg_write(uint16_t value) {
    uint8_t buff[AD5272_TRANSACTION_SIZE_BYTES];

    if (!(value >= 0x0 && value <= 0x3FF)) {
        return ESP_FAIL;
    }

    ad5272_update_data_buff(buff, AD5272_RDAC_WRITE, value);
    return i2c_master_write_to_device(I2C_MASTER_NUM, AD5272_ADDR, &buff, AD5272_TRANSACTION_SIZE_BYTES, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t ad5272_rdac_reg_read(uint8_t read_buff[AD5272_TRANSACTION_SIZE_BYTES]) {
    uint8_t buff[AD5272_TRANSACTION_SIZE_BYTES];

    ad5272_update_data_buff(buff, AD5272_RDAC_READ, 0x0);
    return i2c_master_write_read_device(I2C_MASTER_NUM, AD5272_ADDR, &buff, AD5272_TRANSACTION_SIZE_BYTES, read_buff, AD5272_TRANSACTION_SIZE_BYTES, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t ad5272_50tp_mem_write(void) {
    uint8_t buff[AD5272_TRANSACTION_SIZE_BYTES];

    ad5272_update_data_buff(buff, AD5272_50TP_ADDR, 0x0);
    return i2c_master_write_to_device(I2C_MASTER_NUM, AD5272_ADDR, &buff, AD5272_TRANSACTION_SIZE_BYTES, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t ad5272_50tp_mem_read(uint8_t addr, uint8_t read_buff[AD5272_TRANSACTION_SIZE_BYTES]) {
    uint8_t buff[AD5272_TRANSACTION_SIZE_BYTES];
    
    if (!(addr >= 0x01 && addr <= 0x32)) {
        return ESP_FAIL;    // Simple protection against invalid addresses
    }

    ad5272_update_data_buff(buff, AD5272_50TP_READ, addr);
    return i2c_master_write_read_device(I2C_MASTER_NUM, AD5272_ADDR, &buff, AD5272_TRANSACTION_SIZE_BYTES, read_buff, AD5272_TRANSACTION_SIZE_BYTES, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// Reset RDAC register to most recently programmed 50tp value
esp_err_t ad5272_sw_reset(void) {
    uint8_t buff[AD5272_TRANSACTION_SIZE_BYTES];

    ad5272_update_data_buff(buff, AD5272_SW_RESET, 0x0);
    return i2c_master_write_to_device(I2C_MASTER_NUM, AD5272_ADDR, &buff, AD5272_TRANSACTION_SIZE_BYTES, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

// Not sure what this does; thanks AD!
esp_err_t ad5272_sw_shutdown(void) {
    uint8_t buff[AD5272_TRANSACTION_SIZE_BYTES];

    ad5272_update_data_buff(buff, AD5272_SW_RESET, 0x1);
    return i2c_master_write_to_device(I2C_MASTER_NUM, AD5272_ADDR, &buff, AD5272_TRANSACTION_SIZE_BYTES, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/*
esp_err_t ad5272_write(uint8_t buff[2], uint8_t code, uint16_t data) {

    static esp_err_t ret;


    switch(code) {

        case (AD5272_RDAC_WRITE):
            ad5272_update_data_buff(buff, code, data);
            return i2c_master_write_to_device(I2C_MASTER_NUM, AD5272_ADDR, &buff, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        case (AD5272_50TP_WRITE):
        case (AD5272_CTRL_WRITE):
        default:

    }
}*/