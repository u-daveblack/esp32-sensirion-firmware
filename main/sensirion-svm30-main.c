
#include <stdio.h>

#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"


static const char *TAG = "SENSIRION-SVM30";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)


#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer (only needed in slave mode)*/
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

//Sensorion SVM30 Sesnsor Config
//#define SGB30_SENSOR_I2C_MASTER_NUM 0
//Address
#define SGB30_SENSOR_ADDR 0x58
//Commands 
#define SGB30_SENSOR_SERIAL_ID_COMMAND 0x3682
#define SGB30_SENSOR_INIT_AIR_QUALITY_COMMAND 0x2003 
#define SGB30_SENSOR_MEASURE_AIR_QUALITY_COMMAND 0x2008 //get 6 bytes
#define SGB30_SENSOR_GET_BASELINE_COMMAND 0x2015 //get 6 bytes
#define SGB30_SENSOR_SET_BASELINE_COMMAND 0x201e //send 6 bytes
#define SGB30_SENSOR_SET_HUMIDITY_COMMAND 0x2061 //send 3 bytes
#define SGB30_SENSOR_MEASURE_TEST_COMMAND 0x2032 //get 3 bytes 
#define SGB30_SENSOR_GET_FEATURE_SET_VERSION_COMMAND 0x202f //get 3 bytes 
#define SGB30_SENSOR_MEASURE_RAW_SIGNALS_COMMAND 0x2050 //get 6 bytes

//SHTC1
#define SHTC1_SENSOR_ADDR 0x70
//Commands
#define SHTC1_SENSOR_SERIAL_ID_COMMAND 0xEFC8
#define SHTC1_SENSOR_READ_TEMP_COMMAND 0x7866
#define SHTC1_SENSOR_READ_HUMID_COMMAND 0x58E0

SemaphoreHandle_t i2c_master_mux = NULL;
SemaphoreHandle_t globals_write_mux = NULL;

//GLOBALS

int MAX_SEQ_NUM = 10000;

long TH_SEQ_NUM = 0;
float TemperatureDegC;
float HumidityDegC;

long AQ_SEQ_NUM = 0;
uint16_t CO2PPM;
uint16_t TOVCPPB;


/**
 * @brief transforms temperature value from sensor into degC. 
 */
static float calcluate_temperature(uint16_t sensor_reading_in)
{
    float percent = sensor_reading_in / 65536.0;
    float t = -45.68 + 175.7 * percent;
    return t;
}

/**
 * @brief transforms humidity value from sensor into RH. 
 */
static float claculate_RH(uint16_t sensor_temp_in, uint16_t sensor_humidity_in)
{
    float tPercent = ((float)sensor_temp_in) / 65536.0;
    float rhPercent = ((float) sensor_humidity_in) / 65536.0;
    return (103.7 - 3.2 * tPercent) * rhPercent;
}

/*
Abstraction for reading and writing from Sensor that takes 1 16bit command and returns a specified number of bytes.
*/
static esp_err_t i2c_SH_write_read(i2c_port_t i2c_num, uint8_t sensor_addr, uint16_t command_in, int read_delay_ms, int numBytesToRead, uint8_t * data_out)
{   
    esp_err_t ret = ESP_OK;         
    if (i2c_master_mux != NULL)
    {
        if (xSemaphoreTake(i2c_master_mux, portMAX_DELAY) == pdTRUE)
        {
            do  //try - finally
            {              
                //printf("Ceeating Link\n");  
                i2c_cmd_handle_t cmd = i2c_cmd_link_create();
                ret = i2c_master_start(cmd);
                if (ret != ESP_OK)
                {
                    break;
                }
                i2c_master_write_byte(cmd, sensor_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
                i2c_master_write_byte(cmd, (uint8_t)((command_in & 0xFF00) >> 8), ACK_CHECK_EN);
                i2c_master_write_byte(cmd, (uint8_t)(command_in & 0x00FF), ACK_CHECK_EN);
                i2c_master_stop(cmd);
                ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
                if (ret != ESP_OK)
                {
                    break;
                }

                i2c_cmd_link_delete(cmd);
                if (ret != ESP_OK) {
                    break;
                }

                //wait 30ms
                vTaskDelay(read_delay_ms / portTICK_RATE_MS);

                cmd = i2c_cmd_link_create();

                ret = i2c_master_start(cmd);
                if (ret != ESP_OK){
                    break;
                }
                ret = i2c_master_write_byte(cmd, sensor_addr << 1 | READ_BIT, ACK_CHECK_EN); 

                if (ret != ESP_OK){
                    break;
                }

                uint8_t *ptr = data_out;

                //printf("\n");
                //printf("Starting to read from sensor:\n");
                //now read words
                for (int i =0; i < numBytesToRead; i++){
                    uint8_t ACK = ACK_VAL;
                    if (i == (numBytesToRead - 1)){
                        ACK = NACK_VAL;
                    }
                    i2c_master_read_byte(cmd, ptr, ACK);
                    //printf("Read byte %u from SHTC1 Sensor\n", *ptr);
                    ptr++;
                }

                ret = i2c_master_stop(cmd);
                if (ret != ESP_OK){
                    break;
                }

                ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
                if (ret != ESP_OK){
                    break;
                }
                i2c_cmd_link_delete(cmd);
            }
            while (false);
            
            xSemaphoreGive(i2c_master_mux);
        }
        else 
        {
            //failed to get semaphore
            ret = ESP_ERR_TIMEOUT;
        }
    } 
    else
    {
        //ESP_LOGE(TAG, "ERROR: (%s) : i2c_master_mux was NULL when function i2c_SH_write_read was called.", esp_err_to_name(ret));
        ret = ESP_ERR_INVALID_STATE;
    }
    return ret;
}

static esp_err_t i2c_SHTC1_read_temperature_and_humidity(i2c_port_t i2c_num, float *temperature_degC, float *humidity_percent)
{

    uint8_t data[6] = { 0, 0, 0, 0, 0, 0 };

    esp_err_t ret = i2c_SH_write_read(
        i2c_num, 
        SHTC1_SENSOR_ADDR,
        (uint16_t)SHTC1_SENSOR_READ_TEMP_COMMAND,
        30,
        6,
        data);
    
    if (ret != ESP_OK)
    {
        return ret;
    }

    uint16_t tWord = (data[0] << 8) | (data[1]);
    //printf("tWord is %x \n", tWord);
    float temp = calcluate_temperature(tWord);
    *temperature_degC = temp;
    
    //printf("The Temperature is %f ºC\n", temp);

    uint16_t hWord = (data[3] << 8) | data[4];
    //printf("hWord is %x \n", hWord);
    float humidity = claculate_RH(tWord, hWord);
    *humidity_percent = humidity;
    //printf("The Humidity is %f %%\n", humidity);

    return ret;
}

static esp_err_t i2c_SGB30_measure_air_quality(i2c_port_t i2c_num, uint16_t *co2_ppm, uint16_t *tvoc_ppb)
{

    uint8_t data[6] = { 0, 0, 0, 0, 0, 0 };

    esp_err_t ret = i2c_SH_write_read(
        i2c_num, 
        SGB30_SENSOR_ADDR,
        (uint16_t)SGB30_SENSOR_MEASURE_AIR_QUALITY_COMMAND,
        12,
        6,
        data);
    
    if (ret != ESP_OK)
    {
        return ret;
    }

    uint16_t co2Word = (data[0] << 8) | (data[1]);
    //printf("co2Word is %x \n", co2Word);
    *co2_ppm = co2Word;
    //printf("The CO2 is %d ppm\n", co2);

    uint16_t tvocWord = (data[3] << 8) | data[4];
    //printf("tvocWord is %x \n", tvocWord);
    *tvoc_ppb = tvocWord;
    //printf("The TOVC is %d ppb\n", tvoc);

    return ret;
}



/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Task for testing the SVM30 sensor, prints values to the serial port every second.
 */
static void i2c_SVM30_test_task(void *arg)
{
    esp_err_t ret = ESP_OK; 
    while(1){
        vTaskDelay(1000 / portTICK_RATE_MS);

        float temp;
        float humid;

        ret = i2c_SHTC1_read_temperature_and_humidity(I2C_MASTER_NUM, &temp, &humid);
        if (ret != ESP_OK){
            //ESP_LOGE(TAG, "ERROR: (%s) : returned from i2c_SHTC1_read_temperature_and_humidity", esp_err_to_name(ret));
        }
        else
        {
            //update current temp and humidity and timestamp
            printf("Temperature: %f degC, Humidity: %f %%\n", temp, humid);
            if (xSemaphoreTake(globals_write_mux, portMAX_DELAY) == pdTRUE)
            {   
                TemperatureDegC = temp;
                HumidityDegC = humid;
                TH_SEQ_NUM++;
                if (TH_SEQ_NUM > MAX_SEQ_NUM)
                {
                    TH_SEQ_NUM = 0;
                }
                xSemaphoreGive(globals_write_mux);
            }
            else
            {
                //ESP_LOGE(TAG, "ERROR: (%s) : FAILED TO GET globals_write_mux when updating temp and humid", esp_err_to_name(ret));
            }
        }
    }
}

/**
 * @brief Sends the startup signal to the SGB30 sensor.
 */
static esp_err_t startup_SGB30(i2c_port_t i2c_num)
{
    printf("Starting SGB30 Sensor...\n");
    uint8_t data[0] = {};
    return i2c_SH_write_read(i2c_num, SGB30_SENSOR_ADDR, SGB30_SENSOR_INIT_AIR_QUALITY_COMMAND, 10, 0, data);
}

/**
 * @brief Task for testing the SGB30 sensor, starts printing values after 15 second warmup
 * to the serial port every second.
 */
static void i2c_SGB30_test_task(void *arg)
{
    esp_err_t ret = ESP_OK;
    uint16_t co2_ppm;
    uint16_t tvoc_ppb;
    //startup
    ret = startup_SGB30(I2C_MASTER_NUM);
    if (ret != ESP_OK)
    {
        //ESP_LOGE(TAG, "ERROR (%s) STARTING UP SGB30.", esp_err_to_name(ret));
    }
    else
    {
    
        //read bad values for 15 seconds
        int iter = 0;

        while (1)
        {

            vTaskDelay(1000 / portTICK_RATE_MS);
            ret = i2c_SGB30_measure_air_quality(I2C_MASTER_NUM, &co2_ppm, &tvoc_ppb);
            if (ret != ESP_OK)
            {
                //ESP_LOGE(TAG, "ERROR (%s) retuned from i2c_SGB30_measure_air_quality (%s)", esp_err_to_name(ret)); 
            }
            else
            {
                //don't log warmup
                if (iter < 15)
                {
                    iter++;
                    //printf("IGNORE TVOC AND CO2 - SENSOR STILL STARTING!\n");
                }
                else
                {
                    if (xSemaphoreTake(globals_write_mux, portMAX_DELAY) == pdTRUE)
                    {   
                        printf("CO2: %u ppm, TVOC: %u ppb\n", co2_ppm, tvoc_ppb);   
                        CO2PPM = co2_ppm;
                        TOVCPPB = tvoc_ppb;
                        AQ_SEQ_NUM++;
                        if (AQ_SEQ_NUM > MAX_SEQ_NUM)
                        {
                            AQ_SEQ_NUM = 0;
                        }
                        xSemaphoreGive(globals_write_mux);
                    }
                    else
                    {
                        //ESP_LOGE(TAG, "ERROR: (%s) : FAILED TO GET globals_write_mux when updating temp and humid.", esp_err_to_name(ret));
                    }
                }
            }
        }
    }
}

void app_main(void)
{
    i2c_master_mux = xSemaphoreCreateMutex();
    globals_write_mux = xSemaphoreCreateMutex();
    
    printf("Main Function Running\n");
    printf("portTICK_RATE_MS is %d\n", portTICK_RATE_MS);

    ESP_ERROR_CHECK(i2c_master_init());

    xTaskCreate(i2c_SVM30_test_task, "i2c_SVM30_test_task", 1024 * 2, (void *)0, 10, NULL);
    
    xTaskCreate(i2c_SGB30_test_task, "i2c_SGB30_test_task", 1024 * 2, (void *)0, 20, NULL);
}
