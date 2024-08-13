#include <Arduino.h>
#include <limits.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "xbee/serial.h"

#define XBEE_SER_CHECK(ptr) \
    do { if (xbee_ser_invalid(ptr)) return -EINVAL; } while (0)

void dump_hex (
    const void * addr,
    size_t len,
    int perLine = 16
) {
    // Silently ignore silly per-line values.

    if (perLine < 4 || perLine > 64) perLine = 16;

    int i;
    unsigned char buff[perLine+2];
    const unsigned char * pc = (const unsigned char *)addr;

    // Length checks.

    if (len == 0) {
        return;
    }
    // Process every byte in the data.

    for (i = 0; i < len; i++) {
        // Multiple of perLine means new or first line (with line offset).
         if ((i % perLine) == 0) {
            // Only print previous-line ASCII buffer for lines beyond first.

            if (i != 0) Serial.printf ("  %s\n", buff);

            // Output the offset of current line.

            Serial.printf ("  %04x ", i);
        }
        

        Serial.printf (" %02x", pc[i]);
       
        // And buffer a printable ASCII character for later.

        if ((pc[i] < 0x20) || (pc[i] > 0x7e)) // isprint() may be better.
            buff[i % perLine] = '.';
        else
            buff[i % perLine] = pc[i];
        buff[(i % perLine) + 1] = '\0';
    }

    // Pad out last line if not exactly perLine characters.

    while ((i % perLine) != 0) {
        Serial.printf ("   ");
        i++;
    }

    // And print the final ASCII buffer.

    Serial.printf ("  %s\n", buff);
}
int xbee_ser_invalid( xbee_serial_t *serial)
{

   
    if (serial)
    {
        return 0;
    }

    #define XBEE_SERIAL_VERBOSE
    #ifdef XBEE_SERIAL_VERBOSE
        
    #endif

    if (serial == NULL)
        {
            
            //printf( "%s: serial=NULL\n", __FUNCTION__);
        }
        else
        {
            //printf( "%s: serial=%p, serial->fd=%d (invalid)\n", __FUNCTION__,serial, serial->fd);
        }


    return 1;
}


const char *xbee_ser_portname( xbee_serial_t *serial)
{
    
    
    if (serial == NULL)
    {
        return "(invalid)";
    }

    return serial->portname;
}


int xbee_ser_write( xbee_serial_t *serial, const void FAR *buffer,int length)
{
    //Serial.println("ser_write");
    
    int result;
    const char* buf = (const char*)buffer;
#ifdef DUMP_PACKETS
    dump_hex(buffer,length,16);
#endif
    ((HardwareSerial*)serial->ser)->write((char*)buffer,length);
   // USBSerial.write((char*)buffer,length);


    return 0;
}


int xbee_ser_read( xbee_serial_t *serial, void FAR *buffer, int bufsize)
{
    // for(int i = 0;i<1000;++i) {
    //     if(((HardwareSerial*)serial->ser)->available()>=bufsize) {
    //         break;
    //     }
    //     delay(1);
    // }
    int avail = min(bufsize,((HardwareSerial*)serial->ser)->available());
    int result = (int)((HardwareSerial*)serial->ser)->read((char*)buffer,avail);
    return result;
}

int xbee_ser_putchar( xbee_serial_t *serial, uint8_t ch)
{
    if(0==((HardwareSerial*)serial->ser)->availableForWrite()) {
        return -ENOSPC;
    }
    int retval;

    retval = xbee_ser_write( serial, &ch, 1);
    if (retval == 1)
    {
        return 0;
    }
    else if (retval == 0)
    {
        return -ENOSPC;
    }
    else
    {
        return retval;
    }
}


int xbee_ser_getchar( xbee_serial_t *serial)
{
    int i;
    for(i = 0;i<1000;++i) {
        if(((HardwareSerial*)serial->ser)->available()) {
            break;
        }
        delay(1);
    }
    i= ((HardwareSerial*)serial->ser)->read();
    uint8_t ch = 0;
    if (0>i )
    {
        return -ENODATA;
    }

    return i;
}


int xbee_ser_tx_free( xbee_serial_t *serial)
{
    return INT_MAX;
}


int xbee_ser_tx_used( xbee_serial_t *serial)
{
     return 0;
}


int xbee_ser_tx_flush( xbee_serial_t *serial)
{
    ((HardwareSerial*)serial->ser)->flush();
     return 0;
}


int xbee_ser_rx_free( xbee_serial_t *serial)
{
    return INT_MAX;
}


int xbee_ser_rx_used( xbee_serial_t *serial)
{
    return ((HardwareSerial*)serial->ser)->available();
}


int xbee_ser_rx_flush( xbee_serial_t *serial)
{
    ((HardwareSerial*)serial->ser)->flush();
    return 0;
}


#define _BAUDCASE(b)        case b: baud = B ## b; break
int xbee_ser_baudrate( xbee_serial_t *serial, uint32_t baudrate)
{
    ((HardwareSerial*)serial->ser)->end();
    serial->baudrate = baudrate;
    ((HardwareSerial*)serial->ser)->begin(baudrate,SERIAL_8N1,serial->pin_rx,serial->pin_tx);
    return 0;
}


int xbee_ser_open( xbee_serial_t *serial, uint32_t baudrate)
{
    if(baudrate!=0) {
        serial->baudrate = baudrate;
    }
    ((HardwareSerial*)serial->ser)->begin(serial->baudrate,SERIAL_8N1,serial->pin_rx,serial->pin_tx);
    //serial->ser.begin() baudrate,serial->pin_tx);
    return 0;
}


int xbee_ser_close( xbee_serial_t *serial)
{
    ((HardwareSerial*)serial->ser)->end();
    return 0;
}


int xbee_ser_break( xbee_serial_t *serial, int enabled)
{
    return 0;
}


int xbee_ser_flowcontrol( xbee_serial_t *serial, int enabled)
{
    return 0;
}


int xbee_ser_set_rts( xbee_serial_t *serial, int asserted)
{
    return 0;
}


int xbee_ser_get_cts( xbee_serial_t *serial)
{
    return 1;
}

///@}
