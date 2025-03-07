#include <stdio.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <driver/adc.h>
#include "esp_task_wdt.h"
#include <stdatomic.h>
#include "soc/gpio_reg.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "driver/ledc.h"
#include "esp_random.h"




#include "driver/i2c_master.h"


#define TAG "APP"


#define UART_BUF_SIZE 512
#define UART_BAUD_RATE 115200

#define I2C_SPEED 100000



// Define PWM parameters
#define PWM_FREQUENCY 5000 
#define PWM_RESOLUTION LEDC_TIMER_8_BIT
#define PWM_TIMER LEDC_TIMER_0


// Define analog read

#define SENSORS_WIDTH_BIT ADC_WIDTH_BIT_12
#define SENSOR_COUNT 2

static const adc2_channel_t SENSORS_PINS[SENSOR_COUNT] = {
    ADC2_CHANNEL_7,
    ADC2_CHANNEL_0
};




static void init_analog() {
    
    if (adc1_config_width(SENSORS_WIDTH_BIT) != ESP_OK) {
        ESP_LOGE(TAG, "Adc1_config_width failed.");
        return;
    }

    for (int i = 0; i < SENSOR_COUNT; ++i) {
        if (adc2_config_channel_atten(SENSORS_PINS[i], ADC_ATTEN_DB_0)  != ESP_OK) {
            ESP_LOGE(TAG, "adc2_config_channel_atten failed pin %d.", SENSORS_PINS[i]);
            return;
        }
    }
}


static void read_sensors() {
    
    for (int i = 0; i < SENSOR_COUNT; ++i) {
        
        int sensor_value;

        esp_err_t r = adc2_get_raw(SENSORS_PINS[i], SENSORS_WIDTH_BIT, &sensor_value);

        if ( r != ESP_OK ) ESP_LOGE(TAG, "ADC2 read error pin %d", i);
        if (i != 0) printf("Analog read index: %d value: %d \n", i, sensor_value);
        else printf("Light sensor: %d value: %d \n", i, sensor_value);

    }

}


static void uart_init(uart_port_t uart_num, int tx, int rx, int buffer_size) {

    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    if (uart_driver_install(uart_num, buffer_size, buffer_size, 0, NULL, ESP_INTR_FLAG_IRAM) != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed uart %d", uart_num);
    }

    if (uart_set_pin(uart_num, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed uart %d", uart_num);
    }
    
    if (uart_param_config(uart_num, &uart_config) != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed uart %d", uart_num);
    }

    if (uart_set_mode(uart_num, UART_MODE_UART) != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_mode failed uart %d", uart_num);
    }

    if (gpio_set_pull_mode(rx, GPIO_PULLUP_ONLY) != ESP_OK) {
        ESP_LOGE(TAG, "gpio_set_pull_mode failed uart %d", uart_num);
    }

}


static i2c_master_bus_handle_t bus_handle;

void init_i2c(i2c_port_t i2c_num) {

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT, 
        .i2c_port = i2c_num,
        .scl_io_num = 22,
        .sda_io_num = 21,
        .glitch_ignore_cnt = 8,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(err));
        return;
    }
    
}

void pwm_init() {

    ledc_channel_config_t pwm_channel_0 = {
        .gpio_num = 32,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&pwm_channel_0);

    ledc_channel_config_t pwm_channel_1 = {
        .gpio_num = 13,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&pwm_channel_1);

    ledc_channel_config_t pwm_channel_2 = {
        .gpio_num = 2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&pwm_channel_2);

    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_USE_APB_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

}

bool check_slave_presence(uint8_t address) {
    esp_err_t err = i2c_master_probe(bus_handle, address, 100 / portTICK_PERIOD_MS);
    return err == ESP_OK;
}




void app_main()
{

    printf("Starting...\n");

    // rs485
    uart_init(UART_NUM_1, 17, 16, UART_BUF_SIZE);

    // UART port
    //uart_init(UART_NUM_2, 25, 26, UART_BUF_SIZE);
    uart_init(UART_NUM_2, 14, 12, UART_BUF_SIZE);

    // I2C
    init_i2c(I2C_NUM_0);

    // PMW
    pwm_init();

    // Analog
    init_analog();

    int count = 0;

    while (1) {

        printf("\n\n\nLoop index: %d\n" , count++);

        // rs485
        char rs485Send[] = "sending from rs485\n";
        printf("Sending rs485: %s", rs485Send);
        uart_write_bytes(UART_NUM_1, rs485Send, sizeof(rs485Send));

        char rs485Rec[UART_BUF_SIZE] = "                                             ";
        uart_read_bytes(UART_NUM_1, rs485Rec, UART_BUF_SIZE, 0);
        printf("Reciving rs485: %s\n", rs485Rec);

        // UART
        char uartSend[] = "sending from UART\n";
        printf("Sending UART: %s", uartSend);
        uart_write_bytes(UART_NUM_2, uartSend, sizeof(uartSend));

        char uartRec[UART_BUF_SIZE] = "                                             ";
        uart_read_bytes(UART_NUM_2, uartRec, UART_BUF_SIZE, 0);
        printf("Reciving UART: %s\n", uartRec);

        // I2C
        for (size_t address = 0x70; address < 0x71; address++) {
            if (check_slave_presence(address)) printf("Slave at %d adress exists.\n", address);
        }
        

        // Pmw
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (esp_random() & 255) / 20);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (esp_random() & 255) / 20);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, (esp_random() & 255) / 20);

        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
        
        
        // Analog
        read_sensors();

        printf("\n\n\n");
        
        vTaskDelay(300 / portTICK_PERIOD_MS);


    }
}
