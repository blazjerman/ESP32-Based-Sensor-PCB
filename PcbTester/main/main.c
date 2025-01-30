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


#include "driver/i2c_master.h"


#define UART_BUF_SIZE 512
#define UART_BAUD_RATE 115200

#define I2C_SPEED 100000

#define TAG "APP"



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


static i2c_master_bus_handle_t bus_handle = NULL;

void init_i2c(i2c_port_t i2c_num) {

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT, 
        .i2c_port = i2c_num,
        .scl_io_num = 22,
        .sda_io_num = 21,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(err));
        return;
    }
    
}


bool check_slave_presence(uint8_t address) {
    esp_err_t err = i2c_master_probe(bus_handle, address, I2C_ADDR_BIT_LEN_7);
    return err == ESP_OK;
}


void app_main()
{

    printf("Starting...\n");

    // rs485
    uart_init(UART_NUM_1, 17, 16, UART_BUF_SIZE);

    // UART port
    uart_init(UART_NUM_2, 25, 26, UART_BUF_SIZE);

    // I2C
    init_i2c(0);

    int i = 0;

    while (1) {

        printf("\n\n\nLoop index: %d\n" , i++);

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
        for (size_t i = 0x08; i < 0x77; i++) {
            if (check_slave_presence(i)) printf("Slave at adress %d exists.", i);
        }
        

        // Pmw
        
        

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
