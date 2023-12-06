#define XBEE Serial1
#define MONITOR Serial
#include <Arduino.h>
#include <lvgl.h>
#include <ui.h>
#include <interface.h>
//================================================== MY STUFF =========================================
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "display.hpp"
#include "input.hpp"

#include "xbee/platform.h"
#include "xbee/atcmd.h"
#include "xbee/tx_status.h"
#include "xbee/user_data.h"

//#include "parse_serial_args.h"

struct last_info {
    STATUS_CODE status; // = STATUS_CODE::SUCCESS;
    COMMAND_ID cmd; // = (COMMAND_ID)0;
};
static last_info last = {STATUS_CODE::SUCCESS,(COMMAND_ID)0};
static bool at_cmd_recv = false;
int on_xbee_at_cmd(const xbee_cmd_response_t FAR *response)
{
    at_cmd_recv = true;
    bool_t printable;
    uint_fast8_t length, i;
    uint8_t status;
    const uint8_t FAR *p;

    MONITOR.printf("\nResponse for: %s\n", response->command.str);

    if (response->flags & XBEE_CMD_RESP_FLAG_TIMEOUT)
    {
        MONITOR.println("(timed out)");
        return XBEE_ATCMD_DONE;
    }

    status = response->flags & XBEE_CMD_RESP_MASK_STATUS;
    if (status != XBEE_AT_RESP_SUCCESS)
    {
        MONITOR.printf("(error: %s)\n",
                      (status == XBEE_AT_RESP_ERROR) ? "general" : (status == XBEE_AT_RESP_BAD_COMMAND) ? "bad command"
                                                               : (status == XBEE_AT_RESP_BAD_PARAMETER) ? "bad parameter"
                                                               : (status == XBEE_AT_RESP_TX_FAIL)       ? "Tx failure"
                                                                                                        : "unknown error");
        return XBEE_ATCMD_DONE;
    }

    length = response->value_length;
    if (!length) // command sent successfully, no value to report
    {
        MONITOR.println("(success)");
        return XBEE_ATCMD_DONE;
    }

    // check to see if we can print the value out as a string
    printable = 1;
    p = response->value_bytes;
    for (i = length; printable && i; ++p, --i)
    {
        printable = isprint(*p);
    }

    if (printable)
    {
        MONITOR.printf("= \"%.*" PRIsFAR "\" ", length, response->value_bytes);
    }
    if (length <= 4)
    {
        // format hex string with (2 * number of bytes in value) leading zeros
        MONITOR.printf("= 0x%0*" PRIX32 " (%" PRIu32 ")\n", length * 2, response->value,
                      response->value);
    }
    else if (length <= 32)
    {
        // format hex string
        MONITOR.printf("= 0x");
        for (i = length, p = response->value_bytes; i; ++p, --i)
        {
            MONITOR.printf("%02X", *p);
        }
        MONITOR.println("");
    }
    else
    {
        MONITOR.printf("= %d bytes:\n", length);
        hex_dump(response->value_bytes, length, HEX_DUMP_FLAG_TAB);
    }

    return XBEE_ATCMD_DONE;
}


//================================================== MY STUFF =========================================



//================================================== XBEE STUFF =========================================*/
    xbee_serial_t XBEE_SERPORT;
    int status;
    xbee_dev_t my_xbee;

//================================================== ACECON GPIOs =========================================
    #define ACECON_PPS_IN 9
    #define ACECON_IGN_IN 10
    #define ACECON_POP_IN 11
    #define ACECON_PPS_OUT 12
    #define ACECON_PPT_OUT 13
    #define ACECON_HPS_OUT 14
    #define ACECON_ALM_OUT 21
    #define ACEDATA_RX 47
    #define ACEDATA_TX 48



//================================================== MY STUFF =========================================*/

// function that handles received User Data frames
int user_data_rx(xbee_dev_t *xbee, const void FAR *raw,uint16_t length, void FAR *context)
{
    XBEE_UNUSED_PARAMETER(xbee);
    XBEE_UNUSED_PARAMETER(context);
    
    const xbee_frame_user_data_rx_t FAR *data = (const xbee_frame_user_data_rx_t FAR *)raw;
    int payload_length = length - offsetof(xbee_frame_user_data_rx_t,
                                           payload);

    MONITOR.printf("Message from %s interface:\n",
        xbee_user_data_interface(data->source));

    if(payload_length<1) {
        MONITOR.println("No frame payload received");
        return 0;
    }
    const uint8_t* payload = data->payload;
    int cmd = payload[0];
    switch((COMMAND_ID)cmd) {
        case COMMAND_ID::ACKNOWLEDGE: {
            acknowledge_packet pck;
            memcpy(&pck,payload+1,payload_length-1);
            last.status = pck.status;
        }
        break;
            
    }

    // If all characters of message are printable, just print it as a string
    // with printf().  Otherwise use hex_dump() for non-printable messages.
    int printable = TRUE;
    for (size_t i = 0; printable && i < payload_length; ++i) {
        if (!isprint(data->payload[i])) {
            printable = FALSE;
        }
    }

    if (printable) {
        MONITOR.printf("%.*s\n\n", payload_length, data->payload);
    } else {
        hex_dump(data->payload, payload_length, HEX_DUMP_FLAG_OFFSET);
    }

    return 0;
}
int dump_tx_status(xbee_dev_t *xbee, const void FAR *frame, uint16_t length, void FAR *context)
{
    XBEE_UNUSED_PARAMETER(xbee);
    XBEE_UNUSED_PARAMETER(length);
    XBEE_UNUSED_PARAMETER(context);

    const xbee_frame_tx_status_t *tx_status = (const xbee_frame_tx_status_t*) frame;
    char buffer[40];
    const char *status = NULL;

    // Provide descriptive strings for the only two errors we expect
    // from sending User Data Relay frames.
    switch (tx_status->delivery) {
        case XBEE_TX_DELIVERY_INVALID_INTERFACE:
            status = "invalid interface";
            break;
        case XBEE_TX_DELIVERY_INTERFACE_BLOCKED:
            status = "interface blocked";
            break;
        default:
            sprintf(buffer, "unknown status 0x%X", tx_status->delivery);
            status = buffer;
    }

    printf("Error on message id 0x%02X: %s\n", tx_status->frame_id, status);

    return 0;
}

const xbee_dispatch_table_entry_t xbee_frame_handlers[] =
{
    XBEE_FRAME_HANDLE_LOCAL_AT,
    { XBEE_FRAME_USER_DATA_RX, 0, user_data_rx, NULL },
    { XBEE_FRAME_TX_STATUS, 0, dump_tx_status, NULL },
    XBEE_FRAME_TABLE_END
};

int sendUserDataRelayAPIFrame(xbee_dev_t *xbee, const char *tx, const int num_tx)
{
    // Note: It's safe to pass the pointer here (no need to create a deep copy) because xbee_user_data_relay_tx() will write out the bytes right away.
    int ret = xbee_user_data_relay_tx(xbee, XBEE_USER_DATA_IF_MICROPYTHON, tx, num_tx);
    if (ret < 0)
    {
        printf("%s: ERROR: Failed to send frame to the XBee via serial. Error code: %d\n", __func__, ret);
        return ret;  // The value is negative so it contains the error code.
    }
    return 0;
}
uint32_t crc32(uint32_t crc, unsigned char *buf, size_t len)
{
    int k;

    crc = ~crc;
    while (len--) {
        crc ^= *buf++;
        for (k = 0; k < 8; k++)
            crc = crc & 1 ? (crc >> 1) ^ 0xedb88320 : crc >> 1;
    }
    return ~crc;
}
//hex_dump2(&pkt, sizeof(pkt), HEX_DUMP_FLAG_OFFSET);
void hex_dump2( const void FAR *address, uint16_t length, uint16_t flags)
{
   char linebuf[80];
   char *p, *q, *hex, *chars;
   unsigned char ch;
   uint16_t i;
   const char FAR *data = (const char FAR *)address;

   hex = linebuf;
   if (flags & HEX_DUMP_FLAG_OFFSET)
   {
      hex += 6;         // 0000:<sp>
   }
   else if (flags & HEX_DUMP_FLAG_ADDRESS)
   {
      hex += 8;         // 000000:<sp>
   }
   else if (flags & HEX_DUMP_FLAG_TAB)
   {
      *hex++ = '\t';
   }
   // start printing ASCII characters at position <chars>
   chars = hex + (16 * 3 + 3);

   for(i = 0; i < length; )
   {
      if (flags & HEX_DUMP_FLAG_OFFSET)
      {
         sprintf( linebuf, "%04x: ", i);
      }
      else if (flags & HEX_DUMP_FLAG_ADDRESS)
      {
         sprintf( linebuf, "%" PRIpFAR ": ", data);
      }
      p = hex;
      q = chars;
      do {
         ch = *data++;
         if ((i & 15) == 8)
         {
            // insert space between two sets of 8 bytes
            *p++ = ' ';
            *q++ = ' ';
         }
         p[0] = "0123456789abcdef"[ch >> 4];
         p[1] = "0123456789abcdef"[ch & 0x0F];
         p[2] = ' ';
         p += 3;
         *q++ = isprint(ch) ? ch : '.';
      } while ((++i < length) && (i & 15));
      // add missing spaces between hex and printed chars
      memset( p, ' ', chars - p);
      #ifdef __DC__
         q[0] = '\n';
         q[1] = '\0';                     // null terminate ASCII characters
         fputs( linebuf, stdout);
         // only necessary to flush stdout on Rabbit platform
         fflush( stdout);
      #else
         *q = '\0';                       // null terminate ASCII characters
         MONITOR.println( linebuf);
      #endif
   }
}
void on_xbee_error(COMMAND_ID id, STATUS_CODE code) {
    MONITOR.printf("Error: Command (%d), Status (%d)\n",(int)id,(int)code);
    last.status = (STATUS_CODE)0;
}
void on_monitor_init(const char* data) {
            // Initialize the AT Command layer for this XBee device and have the
        // driver query it for basic information (hardware version, firmware version,
        // serial number, IEEE address, etc.)
        xbee_cmd_init_device(&my_xbee);
        do {
            xbee_dev_tick(&my_xbee);
            status = xbee_cmd_query_status(&my_xbee);
        } while (status == -EBUSY);
        if (status) {
            MONITOR.printf("Error: (%d) waiting for query to complete.\n",status);
        }

}
void on_monitor_at(const char* data) {
    char sz[3];
    memcpy(sz,data+2,2);
    sz[2]=0;
    int16_t request = xbee_cmd_create(&my_xbee, sz);
    if (request < 0)
    {
        // Note that strerror() expects the positive error value
        // (what would have been stored in errno) so we have to
        // negate the xbee_cmd_create() return value.
        MONITOR.printf("Error creating request: %d\n",
                        request, strerror(-request));
    }
    else
    {

        MONITOR.println("Sending command to xbee");
        // if (ieee)
        // {
        //     xbee_cmd_set_target( request, ieee, WPAN_NET_ADDR_UNDEFINED);
        // }
        at_cmd_recv = false;
        xbee_cmd_set_callback(request, on_xbee_at_cmd, NULL);
        
        xbee_cmd_send(request);
    }
}
void on_monitor_connect(const char* str) {
    last.cmd = COMMAND_ID::CONNECT;
    connect_packet data;
    memset(&data,0,sizeof(data));
    strcpy(data.host,"71.8.150.180");
    strcpy(data.lastWillMessage,"Disconnected");
    strcpy(data.lastWillTopic,"unit/vp01234/connection");
    strcpy(data.username,"");
    strcpy(data.password,"");
    strcpy(data.unitname,"vp01234");
    data.cleanSession = ACE_TRUE;
    data.lastWillQos = 1;
    data.port = 1883;
    data.lastWillRetain = ACE_TRUE;
    uint32_t crc = crc32(0,(unsigned char*)&data,sizeof(data));
    
    uint8_t* payload = (uint8_t*)malloc(sizeof(data)+5);
    if(payload==nullptr) {
        MONITOR.println("Out of memory");
        while(1);
    }
    payload[0]=(uint8_t)COMMAND_ID::CONNECT;
    memcpy(payload+1,&crc,sizeof(uint32_t));
    memcpy(payload+5,&data,sizeof(data));
    
    // one or the other of the following two code blocks
    //memcpy(tmp+1,&data,sizeof(data));
    // status = sendUserDataRelayAPIFrame(&my_xbee, tmp, sizeof(data)+1); // no crc
    //hex_dump2(payload, sizeof(data)+5, HEX_DUMP_FLAG_OFFSET);
    status = sendUserDataRelayAPIFrame(&my_xbee,(const char*)payload, sizeof(data)+5); 
    free(payload);
    //status = sendUserDataRelayAPIFrame(&my_xbee, PING_REQUEST, sizeof PING_REQUEST);
    //printf("sent message id 0x%s\n", PING_REQUEST);

    if (status < 0) 
    {
        MONITOR.printf("Error %d sending data\n", status);

        
    }
    else 
    {
        
    }

    
}
void monitor_dev_tick(HardwareSerial& s) {
    if(s.available()) {
        String str = s.readString();
        MONITOR.printf("ECHO: %s\n",str.c_str());
        String cmd = str;
        cmd.toLowerCase();
        cmd.trim();
        if(cmd.substring(0,2)=="at") {
            on_monitor_at(str.c_str());
        } else if(cmd=="init") {
            on_monitor_init(str.c_str());
        } else if(cmd=="connect") {
            on_monitor_connect(str.c_str());
        }
    }
}
void setup() {
     
    Wire.begin( 18,8,100*1000);
    lv_init();
    display_init();
    input_init();
    ui_init();
    dimmer.max_level(.0625);
    

    // your setup code here:
#ifndef CUSTOM
    XBEE.begin(115200, SERIAL_8N1, 18, 17);
#else
    XBEE.begin(115200, SERIAL_8N1, 4, 5);
#endif
    MONITOR.begin(115200);
    MONITOR.printf("Booted\n");
    XBEE_SERPORT.ser = &XBEE;
    XBEE_SERPORT.baudrate = 115200;
    strcpy(XBEE_SERPORT.portname,"XBEE");
#ifndef CUSTOM
    XBEE_SERPORT.pin_rx = 18;
    XBEE_SERPORT.pin_tx = 17;
#else
    XBEE_SERPORT.pin_rx = 4;
    XBEE_SERPORT.pin_tx = 5;
#endif
    if (xbee_dev_init(&my_xbee, &XBEE_SERPORT, NULL, NULL)) {
        MONITOR.printf("Failed to initialize device.\n");
        while(1);
    }

}


void loop() {
    display_update();
    lv_timer_handler();
    if(((int)last.status)<0) {
        on_xbee_error(last.cmd, last.status);
    }
    monitor_dev_tick(MONITOR);
    xbee_dev_tick(&my_xbee);
}

