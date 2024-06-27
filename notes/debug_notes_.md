## XBee Version 9_5 / Sending All but last 4 bytes (673 of 677 bytes) 
  
  
Results: XBee works as expected. It does not process the incomplete message, it reports a CRC Error and the CLI does not crash afterwards. The XBee Debug does not report the entire packet; it stops at hex-line# "000002a0"
  
*Additional Note: This was the same result when we sent all but the last byte (676 of 677 bytes)*
  
### XBee DEBUG
  
```
  
  
Welcome to Xbee CLI
  
  
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: connected      
DEBUG relay_cb: got message, 673 bytes:
  
DEBUG 00000000  01 7b 43 0b 12 76 70 30  31 32 33 34 00 00 00 00  .{C..vp01234....
DEBUG 00000010  00 00 00 00 00 74 65 73  74 2e 6d 6f 73 71 75 69  .....test.mosqui
DEBUG 00000020  74 74 6f 2e 6f 72 67 00  00 00 00 00 00 00 00 00  tto.org.........
DEBUG 00000030  00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  ................
DEBUG *
DEBUG 00000190  00 00 00 00 00 00 00 00  00 75 6e 69 74 2f 76 70  .........unit/vp
DEBUG 000001a0  30 31 32 33 34 2f 63 6f  6e 6e 65 63 74 69 6f 6e  01234/connection
DEBUG 000001b0  00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  ................
DEBUG *
DEBUG 00000210  00 00 00 00 00 00 00 00  00 01 00 00 00 44 69 73  .............Dis
DEBUG 00000220  63 6f 6e 6e 65 63 74 65  64 00 00 00 00 00 00 00  connected.......
DEBUG 00000230  00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  ................
DEBUG *
DEBUG 00000290  00 00 00 00 00 00 00 00  00 00 00 00 00 01 00 00  ................
DEBUG 000002a0  00                                                .               
DEBUG 000002a1
  
INFO relay_cb: got COMMAND ID: CONNECT ( 1 )
  
  
DIAG CRC mismatch, actual: 0x8839d3fa
  
INFO relay_send_ack: status: False
  
DEBUG relay_send: sending 9 bytes
  
DEBUG 00000000  02 1c df 44 21 00 00 00  00                       ...D!....       
DEBUG 00000009
  
  
```
  
### ESP DEBUG
  
```
  
  
Connect Command Received
CRC-32: 0x120b437b
'sendUserDataRelayAPIFrame' called
'xbee_ser_get_cts' called
'xbee_ser_tx_free' called
'xbee_ser_tx_used' called
xbee_frame_write: frame type 0x2d, id 0x0d (676-byte payload)
ser_write
7e 02 a4    ~..
ser_write
2d 0d 02    -..
ser_write
01 7b 43 0b 12 76 70 30 31 32 33 34 00 00 00 00 00 00 00 00 00 74 65 73 74 2e 6d 6f 73 71 75 69 74 74 6f 2e 6f 72 67 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 75 6e 69 74 2f 76 70 30 31 32 33 34 2f 63 6f 6e 6e 65 63 74 69 6f 6e 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 00 00 00 44 69 73 63 6f 6e 6e 65 63 74 65 64 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 00 00 00    .{C..vp01234.........test.mosquitto.org..................................................................................................................................................................................................................................................................................................................................................................................unit/vp01234/connection.............................................................................................................Disconnected........................................................................................................................
ser_write
ab    .
available count: 15
'xbee_ser_read' returned: ~
_xbee_frame_load: got start-of-frame
available count: 14
'xbee_ser_read' returned: [00]
available count: 13
'xbee_ser_read' returned: [0B]
_xbee_frame_load: got length 11
available count: 12
'xbee_ser_read' returned: ­[02][02][1C]ßD![00][00][00][00]î
_xbee_frame_load: dispatch frame #1
_xbee_frame_dispatch: dispatch frame type 0xad, id 0x02
ad 02 02 1c df 44 21 00  00 00 00                  .....D!. ...
_xbee_frame_dispatch: calling frame handler @0x42003764, w/context 0x0
'user_data_rx' called
available count: 0
'xbee_ser_read' returned: 
available count: 0
'xbee_ser_read' returned: 
available count: 0
'xbee_ser_read' returned: 
Read Attempts Timed Out
  
  
```
  
## XBee Version 9_5 / Sending complete payload (677 of 677 bytes) 
  
  
Results: XBee CLI does not report anything. When attempting to send a "help" command it does not return anything. Appears to be locked up and requires a power reset to get back to operating. 
  
The ESP Debug shows the packet being sent but the dispatch frame shows an incorrect return payload (ad 02 02 ff ff ff ff ff  ff ff ff).
  
### XBee DEBUG
  
```
  
Welcome to Xbee CLI
  
  
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: need to connect
DEBUG cellular_obj: connected      
  
  
  
```
  
### ESP DEBUG
  
```
Connect Command Received
CRC-32: 0x120b437b
'sendUserDataRelayAPIFrame' called
'xbee_ser_get_cts' called
'xbee_ser_tx_free' called
'xbee_ser_tx_used' called
xbee_frame_write: frame type 0x2d, id 0x0d (680-byte payload)
ser_write
7e 02 a8    ~..
ser_write
2d 0d 02    -..
ser_write
01 7b 43 0b 12 76 70 30 31 32 33 34 00 00 00 00 00 00 00 00 00 74 65 73 74 2e 6d 6f 73 71 75 69 74 74 6f 2e 6f 72 67 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 75 6e 69 74 2f 76 70 30 31 32 33 34 2f 63 6f 6e 6e 65 63 74 69 6f 6e 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 00 00 00 44 69 73 63 6f 6e 6e 65 63 74 65 64 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 00 00 00 01 00 00 00    .{C..vp01234.........test.mosquitto.org..................................................................................................................................................................................................................................................................................................................................................................................unit/vp01234/connection.............................................................................................................Disconnected............................................................................................................................
ser_write
aa    .
available count: 15
'xbee_ser_read' returned: ~
_xbee_frame_load: got start-of-frame
available count: 14
'xbee_ser_read' returned: [00]
available count: 13
'xbee_ser_read' returned: [0B]
_xbee_frame_load: got length 11
available count: 12
'xbee_ser_read' returned: ­[02][02]ÿÿÿÿÿÿÿÿV
_xbee_frame_load: dispatch frame #1
_xbee_frame_dispatch: dispatch frame type 0xad, id 0x02
ad 02 02 ff ff ff ff ff  ff ff ff                  ........ ...
_xbee_frame_dispatch: calling frame handler @0x42003764, w/context 0x0
'user_data_rx' called
available count: 0
'xbee_ser_read' returned: 
available count: 0
'xbee_ser_read' returned: 
available count: 0
'xbee_ser_read' returned: 
Read Attempts Timed Out
  
  
```
  
  