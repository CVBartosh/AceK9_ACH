#include "vim_controller.hpp"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hal/gpio_hal.h"
#include "soc/gpio_periph.h"

#define EX_UART_NUM UART_NUM_2
#define EX_UART_PIN_RX GPIO_NUM_47
#define EX_UART_PIN_TX GPIO_NUM_48
#define PATTERN_CHR_NUM (3) /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static vim_on_receive_cb vim_on_receive_callback=nullptr;
static void *vim_on_receive_callback_state =nullptr;
static QueueHandle_t vim_uart_queue = nullptr;
static SemaphoreHandle_t vim_sync = nullptr;
static vim_data vim_user_data;
static void vim_uart_event_task(void* pvParameters) {
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*)malloc(RD_BUF_SIZE+1);
    dtmp[RD_BUF_SIZE]=0;
    for (;;) {
        // Waiting for UART event.
        if (xQueueReceive(vim_uart_queue, (void*)&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            // Serial.printf("uart[%d] event:\n", EX_UART_NUM);
            switch (event.type) {
                // Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    //printf("[UART DATA]: %d\n", event.size);
                    //uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    // Serial.printf("[DATA EVT]:\n");
                   // uart_write_bytes(EX_UART_NUM, "ACK\n", 4);
                   uart_read_bytes(EX_UART_NUM,dtmp,event.size,portMAX_DELAY);
                   if(vim_on_receive_callback!=nullptr) {
                       dtmp[event.size] = 0;

                       vim_on_receive_callback((const char*)dtmp, vim_on_receive_callback_state);
                   } else {
                       uart_flush_input(EX_UART_NUM);
                   }
                    break;
                // Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    //printf("hw fifo overflow\n");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(vim_uart_queue);
                    break;
                // Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    //printf("ring buffer full\n");
                    // If buffer full happened, you should consider increasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(vim_uart_queue);
                    break;
                // Event of UART RX break detected
                // often you get these when the line is wrong inversion
                case UART_BREAK:
                    //printf("uart rx break\n");
                    break;
                // Event of UART parity check error
                case UART_PARITY_ERR:
                    //printf("uart parity error\n");
                    break;
                // Event of UART frame error
                case UART_FRAME_ERR:
                    //printf("uart frame error\n");
                    break;
                // UART_PATTERN_DET
                /*case UART_PATTERN_DET: {
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos(EX_UART_NUM);
                    Serial.printf("[UART PATTERN DETECTED] pos: %d, buffered size: %d\n", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(EX_UART_NUM);
                    } else {
                        uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                    }
                    break;
                }*/
                // Others
                default:
                    printf("uart event type: %d\n", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}
void vim_write_sz(const char* sz) {
    if(sz==nullptr) return;
    uart_write_bytes(EX_UART_NUM, sz, strlen(sz));
}
void vim_init(vim_on_receive_cb on_rx_callback, void* callback_state) {
    vim_on_receive_callback = on_rx_callback;
    vim_on_receive_callback_state = callback_state;
    uart_config_t uart_config = {
        .baud_rate = 4800,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .source_clk = UART_SCLK_DEFAULT
#else
        .source_clk = UART_SCLK_APB
#endif
    };
    // Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &vim_uart_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);
    // invert the tx line 
    uart_set_line_inverse(EX_UART_NUM, UART_SIGNAL_TXD_INV );
    // Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, (int)EX_UART_PIN_TX, (int)EX_UART_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    vim_sync = xSemaphoreCreateMutex();
    memset(&vim_user_data,0,sizeof(vim_user_data));
    // Create a task to handler UART event from ISR
    xTaskCreatePinnedToCore(vim_uart_event_task, "vim_uart_event_task", 2048, NULL, 12, NULL,1-xTaskGetAffinity(xTaskGetCurrentTaskHandle()));
}
void vim_store(const vim_data& newData) {
    if(vim_sync==nullptr) return;
    xSemaphoreTake(vim_sync,portMAX_DELAY);
    memcpy(&vim_user_data,&newData,sizeof(vim_user_data));
    xSemaphoreGive(vim_sync);
}
vim_data vim_load() {
    vim_data result;
    if (vim_sync == nullptr) { 
        memset(&result,0,sizeof(result));
        return result;
    }

    xSemaphoreTake(vim_sync, portMAX_DELAY);
    memcpy(&result, &vim_user_data, sizeof(vim_user_data));
    xSemaphoreGive(vim_sync);
    return result;
}
vim_data vim_exchange(const vim_data& newData) {
    vim_data result;
    if (vim_sync == nullptr) {
        memset(&result, 0, sizeof(result));
        return result;
    }
    xSemaphoreTake(vim_sync, portMAX_DELAY);
    memcpy(&result, &vim_user_data, sizeof(vim_user_data));
    memcpy(&vim_user_data,&newData,sizeof(vim_user_data));
    xSemaphoreGive(vim_sync);
    return result;
}