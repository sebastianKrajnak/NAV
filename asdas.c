#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "ssd1306.h"
#include "font8x8_basic.h"
#include "../components/max30102/max30102.h"

//#define I2C_MASTER_SCL_IO 22
//#define I2C_MASTER_SDA_IO 21

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */

#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0

/* The following data structures are used to interact with MAX30102 */
SemaphoreHandle_t print_mux = NULL;

/* The following data structures are used to interact with MAX30102 */
static MAX30102_DEVICE device;
static MAX30102_DATA mess_data;
static int32_t samples[MAX30102_BPM_SAMPLES_SIZE];

//
// I2C master initialization
//
int i2c_master_port = 0;
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };

    esp_err_t err = i2c_param_config(i2c_master_port, &conf);

    if (err != ESP_OK)
    {
        return err;
    }

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


//
// Test task
//
static void i2c_test_task(void *arg)
{
    uint8_t ret = MAX30102_OK;
    uint32_t task_idx = (uint32_t)arg;
    uint8_t bpmBuffer[8];
    uint8_t bpmIdx = 0;
    uint32_t bpmAvg = 0;
    const uint8_t bpmAvgSize = 4;
    int cnt = 0, i;

    // Here is the main loop. Periodically reads and print the parameters
    // measured from MAX30102.
    while (1) 
    {
        printf("T: %lu test #: %d", task_idx, cnt++);

        // Setup the sensor operation mode for heart rate, wait for a 700 ms
        // and then, read the data buffer. The read is made by
        // passing the mess_data struct, which contains the
        // meas result.
        // After getting the values, get a semaphore in order to
        // print the information for monitoring

        ret = max30102_set_sensor_mode(MAX30102_HR_MODE, &device);
        device.delay_us(700000);
    
        for (i = 0; i < MAX30102_BPM_SAMPLES_SIZE; i++){
            ret = max30102_get_sensor_data(MAX30102_BPM, &mess_data, &device);
            samples[i] = mess_data.bpm32;
            device.delay_us(20000);
        }

        // Once the data buffer is full, we need to get the BPM measurement.
        // In order to improve performance, the BPM mess is filtered (mean)
        // 
        bpmAvg = 0;
        for (i = bpmAvgSize - 1; i; i--){
            bpmBuffer[i] = bpmBuffer[i-1];
        }

        bpmBuffer[0] = max30102_get_bpm(samples);
        for (i = 0; i < bpmAvgSize; i++){
            bpmAvg += bpmBuffer[i];
        }

        bpmAvg /= bpmAvgSize;

        xSemaphoreTake(print_mux, portMAX_DELAY);

        // Print information retrieved. If the connection was successful, print
        // the sensor ID and the BPM average value.
        // If the connection is NOK, print an error message.
        if (ret == MAX30102_OK)
        {
            printf("***********************************\n");
            printf("T: %lu -  READING SENSOR( MAX30102 )\n", task_idx);
            printf("***********************************\n\n");
            printf("Print direct values:\n");
            printf("Sensor ID: %d\n", device.chip_id);
            printf("BPM: %lu \n", bpmAvg);
        }
        else
        {
            printf("%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }

        // Give the semaphore taken and wait for the next read.
        xSemaphoreGive(print_mux);
        device.delay_us(1000000);
    }

    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}


void app_main(void)
{

	SSD1306_t dev;
	int center, top;

    // CONFIG_SPI_INTERFACE
	spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO);
    // CONFIG SSD1306 DISPLAY
	ssd1306_init(&dev, 128, 64);
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
    ssd1306_display_text_x3(&dev, 0, "Hello", 5, false);

    // DISPLAY DEMO ---------------------------------------------------------------------------------------------
    vTaskDelay(3000 / portTICK_PERIOD_MS);

	top = 2;
	center = 3;
	ssd1306_display_text(&dev, 0, "SSD1306 128x64", 14, false);
	ssd1306_display_text(&dev, 1, "ABCDEFGHIJKLMNOP", 16, false);
	ssd1306_display_text(&dev, 2, "abcdefghijklmnop",16, false);
	ssd1306_display_text(&dev, 3, "Hello World!!", 13, false);
	//ssd1306_clear_line(&dev, 4, true);
	//ssd1306_clear_line(&dev, 5, true);
	//ssd1306_clear_line(&dev, 6, true);
	//ssd1306_clear_line(&dev, 7, true);
	ssd1306_display_text(&dev, 4, "SSD1306 128x64", 14, true);
	ssd1306_display_text(&dev, 5, "ABCDEFGHIJKLMNOP", 16, true);
	ssd1306_display_text(&dev, 6, "abcdefghijklmnop",16, true);
	ssd1306_display_text(&dev, 7, "Hello World!!", 13, true);

	vTaskDelay(3000 / portTICK_PERIOD_MS);

	// Display Count Down
	uint8_t image[24];
	memset(image, 0, sizeof(image));
	ssd1306_display_image(&dev, top, (6*8-1), image, sizeof(image));
	ssd1306_display_image(&dev, top+1, (6*8-1), image, sizeof(image));
	ssd1306_display_image(&dev, top+2, (6*8-1), image, sizeof(image));
	for(int font=0x39;font>0x30;font--) {
		memset(image, 0, sizeof(image));
		ssd1306_display_image(&dev, top+1, (7*8-1), image, 8);
		memcpy(image, font8x8_basic_tr[font], 8);
		if (dev._flip) ssd1306_flip(image, 8);
		ssd1306_display_image(&dev, top+1, (7*8-1), image, 8);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	// Invert
	ssd1306_clear_screen(&dev, true);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text(&dev, center, "  Good Bye!!", 12, true);
	vTaskDelay(5000 / portTICK_PERIOD_MS);

    int8_t ret;
    print_mux = xSemaphoreCreateMutex();

    // Before access to the MAX30102, we need to setup the device handler
    // by assign the platform specific functions which brings access
    // to the communication port. These functions are implemented in
    // API/driver/MAX30102_ESP32C3.c file.
    device.read = esp32c3_read_max30102;
    device.write = esp32c3_write_max30102;
    device.delay_us = esp32c3_delay_us_max30102;

    // The first step is to initialize the I2C peripheral as usual
    ESP_ERROR_CHECK(i2c_master_init());

    // After I2C initialization, BME280 initialization could be done.
    if (MAX30102_E_DEV_NOT_FOUND == max30102_init(&device))
    {
        printf("MAX30102 sensor is not connected.");
        while(1);
    }

    // Setup some adquisition parameters
    // - ADC range for 16384 counts.
    // - 50 samples per second.
    // - 411 microseconds pulse width for LEDs.
    ret = max30102_set_spo2(MAX30102_SPO2_RANGE_16384 | MAX30102_SPO2_50_SPS | MAX30102_SPO2_LED_PW_411, &device);
    device.delay_us(40000);

    // Setup the FIFO
    // - No samples average.
    ret = max30102_set_fifo(MAX30102_SMP_AVE_NO, &device);
    device.delay_us(40000);

    // Setup the LEDs current amplitude
    // - Aprox. 3 mA
    ret = max30102_set_led_amplitude(0x0F, &device);
    device.delay_us(40000);

    // For this example there is only one task which setup the MAX30102, and then
    // reads the parameters periodically.
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
	// Fade Out
	// ssd1306_fadeout(&dev);

	// Restart module
	// esp_restart();
}