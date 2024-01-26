#include <Arduino.h>
#include <FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

extern SemaphoreHandle_t acedata_sync;
extern uint8_t* acedata_rx_buffer;
extern size_t acedata_rx_buffer_size;
extern HardwareSerial* acedata_serial;
void acedata_serial_init(HardwareSerial& serial);
//void acedata_serial_task(void* state);
size_t acedata_serial_available();
size_t acedata_serial_read(uint8_t* data,size_t size);
void acedata_serial_send(const char* data);
