/* ESP32 Heartbeat sensor using ESP-IDF framework
    Author: Sebastian Krajnak - xkrajn05
    Date: April 2023
    Used libraries:
     - SSD1306 OLED Display: https://github.com/nopnop2002/esp-idf-ssd1306
     - MAX30102 Oximeter: https://github.com/Gustbel/max30102_esp-idf
     - KX-040 (KY-040) Rotary Encoder: https://github.com/nopnop2002/esp-idf-RotaryEncoder
    some parts of the code were reused from example codes of linked libraries. These librabries are ass and I would
    rather use arduino framework with proper libraries next time.
     BPM values that the MAX30102 library returns are hella off scale.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

// Import component libraries
#include "ssd1306.h"
#include "font8x8_basic.h"
#include "../components/max30102/max30102.h"
#include "../components/RotaryEncoder/RotaryEncoder.h"

static const char *TAG = "HeartRate sensor";

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */

#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define MENU_OPTIONS 3

SemaphoreHandle_t print_mux = NULL;

// The following data structures are used to interact with MAX30102 and SSD1306
static MAX30102_DEVICE oximeter_device;
static MAX30102_DATA mess_data;
static int32_t samples[MAX30102_BPM_SAMPLES_SIZE];
static SSD1306_t display_device;

int selected_option = 0;
bool is_selected = false;
char menu_options[3][10] = {"Text", "Image", "None"};

static uint8_t heart_icon[63] = {
    0x03, 0xC0, 0xF0, 0x06, 0x71, 0x8C, 0x0C, 0x1B, 0x06, 0x18, 0x0E, 0x02, 0x10, 0x0C, 0x03, 0x10,
    0x04, 0x01, 0x10, 0x04, 0x01, 0x10, 0x40, 0x01, 0x10, 0x40, 0x01, 0x10, 0xC0, 0x03, 0x08, 0x88,
    0x02, 0x08, 0xB8, 0x04, 0xFF, 0x37, 0x08, 0x01, 0x30, 0x18, 0x01, 0x90, 0x30, 0x00, 0xC0, 0x60,
    0x00, 0x60, 0xC0, 0x00, 0x31, 0x80, 0x00, 0x1B, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x04, 0x00
};


// Function to display menu options
void update_display() {
    // Clear display
    ssd1306_clear_screen(&display_device, false);

    ssd1306_display_text(&display_device, 0, "Select BPM UI",13, false );
    // Print currently selected option
    ssd1306_display_text(&display_device, selected_option+1, menu_options[selected_option], 10, true);

    // Print other options
    for (int i = 0; i < MENU_OPTIONS; i++) {
        if (i != selected_option) {
            ssd1306_display_text(&display_device, i+1, menu_options[i], 10, false);
        }
    }
}

void scroll_options(int val) {
    // Scroll menu options down
    if (val >= 0) selected_option++;
    else if ( val < 0) selected_option--;

    if (selected_option < 0) {
        selected_option = MENU_OPTIONS - 1;
    } else if (selected_option >= MENU_OPTIONS) {
        selected_option = 0;
    }
}

// I2C master initialization
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = 0;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(i2c_master_port, &conf);

    if (err != ESP_OK)
    {
        return err;
    }

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


// Oximeter task
static void oximeter_task(void *arg){
    uint8_t ret = MAX30102_OK;
    uint32_t task_idx = (uint32_t)arg;
    uint8_t bpmBuffer[8];
    uint32_t bpmAvg = 0;
    const uint8_t bpmAvgSize = 4;
    int cnt = 0, i;
    char bpm_buffer[20];

    // MENU SELECTION ------------------------------------------------------------------------------------------------
    // It would be better to have a second task for menu selection and I tried and kept failing
    //so I just gave up and stuffed it into one

    // Update display with initial menu options
    update_display();

    while (!is_selected) {
        // Read rotary encoder value
        int count; // Current count value
		int sw; // Current switch value
		int interrupt; // Interrupt occurrence count
		int event = readRotaryEncoder(&count, &sw, &interrupt);

        if (event == 0) {
            // Encoder value changed, scroll menu options
            scroll_options(count);
            update_display();
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        if (event == 1) {
            // Encoder button pressed, select current option
            printf("Selected option is %d\n", selected_option);
            is_selected = !is_selected;
        }

        // Delay before checking encoder again
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    ssd1306_clear_screen(&display_device, true);

    // BPM MEASUREMENT -----------------------------------------------------------------------------------------------
    // Here is the main loop. Periodically reads and print the parameters
    // measured from MAX30102.
    while (1){
        ESP_LOGI(TAG, "T: %lu test #: %d", task_idx, cnt++);

        // Setup the sensor operation mode for heart rate, wait for a 700 ms
        // and then, read the data buffer. The read is made by
        // passing the mess_data struct, which contains the
        // meas result.
        // After getting the values, get a semaphore in order to
        // print the information for monitoring

        ret = max30102_set_sensor_mode(MAX30102_HR_MODE, &oximeter_device);
        oximeter_device.delay_us(700000);

        for (i = 0; i < MAX30102_BPM_SAMPLES_SIZE; i++){
            ret = max30102_get_sensor_data(MAX30102_BPM, &mess_data, &oximeter_device);
            samples[i] = mess_data.bpm32;
            oximeter_device.delay_us(20000);
        }

        if(samples[0] < 7000 && samples[5] < 7000){
            ssd1306_clear_screen(&display_device, false);
            ssd1306_display_text(&display_device, 2, "Please place", 12, false);
            ssd1306_display_text(&display_device, 3, "your finger on", 14, false);
            ssd1306_display_text(&display_device, 4, "the sensor...", 13, false);

            xSemaphoreTake(print_mux, portMAX_DELAY);

            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        if(samples[0] >= 7000 && samples[5] >= 7000){
            // Once the data buffer is full, we need to get the BPM measurement.
            // In order to improve performance, the BPM mess is filtered (mean)
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

            // BPM DISPLAY -------------------------------------------------------------------------------------------------
            // If the connection was successful, print retrieved information .
            // If the connection is NOK, print an error message.
            if (ret == MAX30102_OK){
                printf("***********************************\n");
                printf("T: %lu -  READING SENSOR( MAX30102 )\n", task_idx);
                printf("Sensor ID: %d\n", oximeter_device.chip_id);
                printf("BPM: %lu \n", bpmAvg);
                sprintf(bpm_buffer, "%lu", bpmAvg);
                ssd1306_clear_screen(&display_device, false);
                switch (selected_option) {
                    case 0:
                        ssd1306_display_text_x3(&display_device, 0, "BPM", 3, false);
                        ssd1306_display_text_x3(&display_device, 4, bpm_buffer, 3, false);
                        break;
                    case 1:
                        ssd1306_bitmaps(&display_device, 0, 0, heart_icon, 24, 21, false);
                        ssd1306_display_text_x3(&display_device, 4, bpm_buffer, 3, false);
                        break;
                    case 2:
                        ssd1306_display_text_x3(&display_device, 3, bpm_buffer, 3, false);
                        break;
                    default:
                        break;
                }
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            else{
                ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
            }
        }
        // Give the semaphore taken and wait for the next read.
        xSemaphoreGive(print_mux);
        oximeter_device.delay_us(1000000);

    }
    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}


void app_main(void)
{
    // Initialize and conifg MAX30102 ---------------------------------------------------------------------------------
    int8_t ret;
    print_mux = xSemaphoreCreateMutex();

    // Setup MAX30102 device handler
    oximeter_device.read = esp32c3_read_max30102;
    oximeter_device.write = esp32c3_write_max30102;
    oximeter_device.delay_us = esp32c3_delay_us_max30102;

    // Initialize I2C peripheral
    ESP_ERROR_CHECK(i2c_master_init());

    if (MAX30102_E_DEV_NOT_FOUND == max30102_init(&oximeter_device)){
        ESP_LOGW(TAG, "MAX30102 sensor is not connected.");
        while(1);
    }

    // Setup some aquisition parameters
    // - ADC range for 16384 counts.
    // - 50 samples per second.
    // - 411 microseconds pulse width for LEDs.
    ret = max30102_set_spo2(MAX30102_SPO2_RANGE_16384 | MAX30102_SPO2_50_SPS | MAX30102_SPO2_LED_PW_411, &oximeter_device);
    oximeter_device.delay_us(40000);

    // Setup the FIFO
    // - No samples average.
    ret = max30102_set_fifo(MAX30102_SMP_AVE_NO, &oximeter_device);
    oximeter_device.delay_us(40000);

    // Setup the LEDs current amplitude
    // - Aprox. 3 mA
    ret = max30102_set_led_amplitude(0x0F, &oximeter_device);
    oximeter_device.delay_us(40000);

    // Config and initiate SSD1306 OLED display ------------------------------------------------------------------------------
    // Initiate SPI interface
	spi_master_init(&display_device, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO);

    // Initialize and config SSD1306 DISPLAY
	ssd1306_init(&display_device, 128, 64);
	ssd1306_clear_screen(&display_device, false);
	ssd1306_contrast(&display_device, 0xff);
    ssd1306_display_text_x3(&display_device, 0, "Hello", 5, false);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
	ssd1306_clear_screen(&display_device, false);

    // Initialize rotary encoder
    initRotaryEncoder(CONFIG_GPIO_OUT_A, CONFIG_GPIO_OUT_B, CONFIG_GPIO_SWITCH, NULL);

    // Read data from oximeter and display them on the OLED display via the selected UI
    xTaskCreate(oximeter_task, "i2c_oximeter_task_0", 1024 * 2, (void *)0, 10, NULL);
}