
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"




#define I2C_MASTER_FREQ_100KHZ 100000

void init_sensor_als(i2c_mode_t mode, gpio_num_t sda, gpio_num_t scl, uint32_t freq);
esp_err_t ControlRegisters(uint8_t addr);
esp_err_t ALS_Measurement_Rates(uint8_t addr);
esp_err_t start_get_data(uint8_t addr);
esp_err_t setup(uint8_t addr);
esp_err_t status(uint8_t addr,uint8_t *status);
esp_err_t get_data(uint8_t addr,float *LUX);






