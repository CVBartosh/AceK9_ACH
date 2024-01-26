#include <acedata_serial_task.hpp>
#define ACEDATA_RX 47
#define ACEDATA_TX 48
HardwareSerial* acedata_serial = nullptr;
size_t acedata_rx_buffer_size=0;
static uint8_t acedata_rx_buffer_array[8192];
uint8_t* acedata_rx_buffer = acedata_rx_buffer_array;
SemaphoreHandle_t acedata_sync=nullptr;
size_t acedata_serial_available() {
    xSemaphoreTake(acedata_sync,portMAX_DELAY);
    size_t result =  acedata_rx_buffer_size;
    xSemaphoreGive(acedata_sync);
    return result;
}
size_t acedata_serial_read(uint8_t* data,size_t size) {
    xSemaphoreTake(acedata_sync,portMAX_DELAY);
    size_t result =  acedata_rx_buffer_size;
    if(size>result) {
        size = result;
    }
    memcpy(data,acedata_rx_buffer,size);
    memmove(acedata_rx_buffer, acedata_rx_buffer+size,sizeof(acedata_rx_buffer)-size);
    acedata_rx_buffer_size-=size;
    xSemaphoreGive(acedata_sync);
    return result;
}


void acedata_serial_task(void*state) {
    
    while(true) {
        int i = 100;
        while(i--) {
            if(acedata_serial->available()) {
                xSemaphoreTake(acedata_sync,portMAX_DELAY);
                acedata_rx_buffer_size += acedata_serial->readBytes(acedata_rx_buffer+acedata_rx_buffer_size,sizeof(acedata_rx_buffer)-acedata_rx_buffer_size);
                xSemaphoreGive(acedata_sync);
            } else {
                delay(10);
            }
        }
    }
}
void acedata_serial_init(HardwareSerial& serial) {
    acedata_serial = &serial;
    acedata_sync = xSemaphoreCreateMutex();
    pinMode(ACEDATA_TX,OUTPUT_OPEN_DRAIN);
    acedata_serial->begin(4800,SERIAL_8N1,ACEDATA_RX,-1);
    TaskHandle_t t;
    xTaskCreate(acedata_serial_task,"acedata_serial",4092,nullptr,0,&t);
}
void acedata_serial_send(const char* data) {
    xSemaphoreTake(acedata_sync,portMAX_DELAY);
    acedata_serial->end(true);
    acedata_serial->begin(4800,SERIAL_8N1,-1,ACEDATA_TX,true);
    acedata_serial->print(data);
    acedata_serial->flush(true);
    acedata_serial->end(true);
    pinMode(ACEDATA_TX,OUTPUT_OPEN_DRAIN);
    acedata_serial->begin(4800,SERIAL_8N1,ACEDATA_RX,-1);
    xSemaphoreGive(acedata_sync);
}
