#include "als303.h"

//Only for ALS ALS Integration Time 50ms/ALS_MEAS_RATE 500MS 
#define ALS_GAIN 1
#define ALS_INT 5



static const char *TAG = "ALS";

float Calculate_Lux(uint16_t Channel0Data, uint16_t Channel1Data)
{
    float ratio, ALS_LUX;

    ratio = Channel1Data/(Channel0Data + Channel1Data);

    if (ratio < 0.45)
    {
        ALS_LUX = ( (1.7743 * Channel0Data) + (1.1059 * Channel1Data) ) / ALS_GAIN / ALS_INT;
    }
    else if (ratio < 0.64 && ratio >= 0.45)
    {
        ALS_LUX = ( (4.2785 * Channel0Data) - (1.9548 * Channel1Data) ) / ALS_GAIN / ALS_INT;
    }
    else if (ratio < 0.85 && ratio >= 0.64)
    {
        ALS_LUX = ( (0.5926 * Channel0Data) + (0.1185 * Channel1Data) ) / ALS_GAIN / ALS_INT;
    }
    else
    {
        ALS_LUX = 0;
    }

    return ALS_LUX;
}


void init_sensor_als(i2c_mode_t mode, gpio_num_t sda, gpio_num_t scl, uint32_t freq)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num =  GPIO_NUM_33;
    conf.scl_io_num = GPIO_NUM_14;

    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;

    conf.master.clk_speed = freq;
     conf.clk_flags=0;

    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode,
                       0,
                       0,
                       0);
}

esp_err_t ControlRegisters(uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x80, true);
    i2c_master_write_byte(cmd, 0x01, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 2);
    i2c_cmd_link_delete(cmd);
    if(err)
        ESP_LOGI(TAG, "%s", esp_err_to_name(err));
    return err;
}


esp_err_t ALS_Measurement_Rates(uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x85, true);
    i2c_master_write_byte(cmd, 0x12, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 2);
    i2c_cmd_link_delete(cmd);
    if(err)
        ESP_LOGI(TAG, "%s", esp_err_to_name(err));
    return err;
}


esp_err_t start_get_data(uint8_t addr)
{
    vTaskDelay(10 / portTICK_RATE_MS);
    //Read T First
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x80, true);
    i2c_master_write_byte(cmd, 0x01, true);//active mode
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1);
    if (err)
        return err;

    i2c_cmd_link_delete(cmd);

    //Read
    uint8_t data[1];

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0x53, true);
    i2c_master_read(cmd, data,1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1);
    i2c_cmd_link_delete(cmd);
 uint8_t Interrupt_Status = data[0] & 0x08; // Interrupt_Status = 8(decimal)  ALS Interrupt
 uint8_t NewData_Status = data[0]& 0x04; // NewData_Status = 4(decimal)  ALS New Data

    printf("status:%d newdata:%d",Interrupt_Status,NewData_Status);


//////////////

  vTaskDelay(10 / portTICK_RATE_MS);
return err;
}
//setup

esp_err_t setup(uint8_t addr)
{
    vTaskDelay(10 / portTICK_RATE_MS);
    //Read T First
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x85, true);
    i2c_master_write_byte(cmd, 0x0B, true); ///ALS ALS Integration Time 50ms/ALS_MEAS_RATE 500MS
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1);
    if (err)
        return err;

    i2c_cmd_link_delete(cmd);

    //Read
    uint8_t data[1];

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0x53, true);
    i2c_master_read(cmd, data,1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1);
    i2c_cmd_link_delete(cmd);
 uint8_t Interrupt_Status = data[0] & 0x08; // Interrupt_Status = 8(decimal)  ALS Interrupt
 uint8_t NewData_Status = data[0]& 0x04; // NewData_Status = 4(decimal)  ALS New Data

    printf("status:%d newdata:%d",Interrupt_Status,NewData_Status);


//////////////

  vTaskDelay(10 / portTICK_RATE_MS);
return err;
}
//

//setup

esp_err_t status(uint8_t addr,uint8_t *status)
{
    vTaskDelay(10 / portTICK_RATE_MS);
    //Read T First
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x8c, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1);
    if (err)
        return err;

    i2c_cmd_link_delete(cmd);

    //Read
    uint8_t data[1];

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0x53, true);
    i2c_master_read(cmd, data,1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1);
    i2c_cmd_link_delete(cmd);
    *status=data[0];
 //uint8_t Interrupt_Status = data[0] & 0x08; // Interrupt_Status = 8(decimal)  ALS Interrupt
 //uint8_t NewData_Status = data[0]& 0x04; // NewData_Status = 4(decimal)  ALS New Data

   // printf("status:%d newdata:%d",Interrupt_Status,NewData_Status);


//////////////

  vTaskDelay(10 / portTICK_RATE_MS);
return err;
}
//

//setup

esp_err_t get_data(uint8_t addr,float *LUX)
{
    vTaskDelay(10 / portTICK_RATE_MS);
    //Read T First
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x88, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1);
    if (err)
        return err;

    i2c_cmd_link_delete(cmd);

    //Read
    uint8_t data[6];

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0x53, true);
    i2c_master_read(cmd, data,4, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1);
    i2c_cmd_link_delete(cmd);
 //uint8_t Interrupt_Status = data[0] & 0x08; // Interrupt_Status = 8(decimal)  ALS Interrupt
 //uint8_t NewData_Status = data[0]& 0x04; // NewData_Status = 4(decimal)  ALS New Data

   // printf("status:%d newdata:%d",Interrupt_Status,NewData_Status);
     printf("%x,%x,%x,%x\n",data[0],data[1],data[2],data[3]);
      uint16_t ALS_CH1_ADC_Data=(data[1] << 8) | data[0];
      uint16_t ALS_CH0_ADC_Data=(data[3] << 8) | data[2];

     *LUX= Calculate_Lux(ALS_CH0_ADC_Data,ALS_CH1_ADC_Data);
    // printf("CH1:%d,CH0:%d\n",ALS_CH1_ADC_Data,ALS_CH0_ADC_Data);
    // printf("%f",lux);




//////////////

  vTaskDelay(10 / portTICK_RATE_MS);
return err;
}
//




