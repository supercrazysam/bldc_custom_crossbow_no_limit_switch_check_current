#include "mbed.h"

#include <stdint.h>
//#include <mcp_can.h>
//#include <SPI.h>

bool sendPacket(uint32_t id, uint8_t packet[], int32_t len);
void vesc_can_set_duty(uint8_t controller_id, float duty);
void comm_can_set_rpm(uint8_t controller_id, float rpm);

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

typedef enum {
   CAN_PACKET_SET_DUTY = 0,
   CAN_PACKET_SET_CURRENT,
   CAN_PACKET_SET_CURRENT_BRAKE,
   CAN_PACKET_SET_RPM,
   CAN_PACKET_SET_POS,
   CAN_PACKET_FILL_RX_BUFFER,
   CAN_PACKET_FILL_RX_BUFFER_LONG,
   CAN_PACKET_PROCESS_RX_BUFFER,
   CAN_PACKET_PROCESS_SHORT_BUFFER,
   CAN_PACKET_STATUS
} CAN_PACKET_ID;


#define receive true;                            // Set INT to pin 2
//MCP_CAN CAN0(10);     // Set CS to pin 10
CAN CAN0(PA_11, PA_12, 500000);
Serial pc(USBTX, USBRX);
char display_buffer[8];


void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
   buffer[(*index)++] = number >> 24;
   buffer[(*index)++] = number >> 16;
   buffer[(*index)++] = number >> 8;
   buffer[(*index)++] = number;
}

void buffer_append_int32ex(char* buffer, int32_t number, int32_t *index) {
   buffer[(*index)++] = number >> 24;
   buffer[(*index)++] = number >> 16;
   buffer[(*index)++] = number >> 8;
   buffer[(*index)++] = number;
}
   //Serial.begin(115200);

   // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
   //if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
   //else                                         Serial.println("Error Initializing MCP2515...");

   //CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted


void vesc_can_set_duty(uint8_t controller_id, float duty)
{
   int32_t send_index = 0;
   uint8_t buffer[4];
   buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
   sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}
void comm_can_set_rpm(uint8_t controller_id, float rpm) {
   int32_t send_index = 0;
   uint8_t buffer[4];
   buffer_append_int32(buffer, (int32_t)rpm, &send_index);
   sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

//https://github.com/skipper762/teensy_VESC_CANBUS
void comm_can_set_pos(uint8_t controller_id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 1e6), &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}


void crossbow(uint8_t controller_id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 1e6), &send_index);
  sendPacket(controller_id | ((uint32_t)14 << 8), buffer, send_index);   //crossbow=14
}



bool sendPacket(uint32_t id, uint8_t packet[], int32_t len)
{
   // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
   //byte data[4] = { 0x00, 0x00, 0x0F, 0xFF };
   //byte sndStat = CAN0.sendMsgBuf(id, 0, 4, data);\
   //CAN0.sendMsgBuf(id, 1, 4, packet);\ v   //id   , extended frame=1 
   
   /*CAN.sendMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf);

This is a function to send data onto the bus. In which:

id represents where the data come from.
ext represents the status of the frame. '0' means standard frame. '1' means extended frame.
len represents the length of this frame.
buf is the content of this message.*/

   //*******
   //int32_t send_index = 0;
   //uint8_t buffer[4];
   //buffer_append_int32(buffer, (int32_t)10000, &send_index);
   //
   //*******if (CAN0.write(CANMessage(id, (const char*)buffer, sizeof(buffer),CANData,CANExtended)))
   if (CAN0.write(CANMessage(id, (const char*)packet, sizeof(packet),CANData,CANExtended)))
   {
   
      return true;
   }
   else {
      //Serial.println("Error Sending Message...");
      return false;
   }
}


CANMessage receive_msg;
CANMessage send_msg;
int32_t erpm = 0;
int16_t current = 0;
int16_t duty = 0;

void show_status()
{
 if (CAN0.read(receive_msg)) 
{
        
    if ((receive_msg.id & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
         sprintf(display_buffer ,"Extended ID: 0x%.8lX  DLC: %1d  Data:", (receive_msg.id & 0x1FFFFFFF), sizeof(receive_msg.id));
      else
         sprintf(display_buffer, "Standard ID: 0x%.3lX       DLC: %1d  Data:", receive_msg.id, sizeof(receive_msg.id));
    pc.printf(display_buffer);
    pc.printf("\n\r");
    
    
    if ((receive_msg.id & 0x40000000) == 0x40000000) 
    {    // Determine if message is a remote request frame.
         sprintf(display_buffer, " REMOTE REQUEST FRAME");
         pc.printf(display_buffer);
         pc.printf("\n\r");
    }
    else 
    {
        erpm = (receive_msg.data[0] << 24) | (receive_msg.data[1] << 16) | (receive_msg.data[2] << 8) | receive_msg.data[3];
        current = (receive_msg.data[4] << 8) | receive_msg.data[5];
        duty = (receive_msg.data[6] << 8) | receive_msg.data[7];
        
//         for (int i = 0; i<  sizeof(receive_msg.data); i++)
//         {
//            
//            if (i >= 0 && i <= 3)
//            {
//              erpm = erpm << 8 | receive_msg.data[i];
//            }
//            
//            if (i >= 4 && i <= 5)
//            {
//              current = current << 8 | receive_msg.data[i];
//            }
//            
//            if (i >= 6 && i <= 7)
//            {
//              duty = duty << 8 | receive_msg.data[i];
//            }
            //sprintf(display_buffer, " 0x%.2X", receive_msg.data[i]);
            //pc.printf(display_buffer);
            //pc.printf("\n\r"); 
//        }
    }
      
      sprintf(display_buffer, "RPM %d", erpm );
      pc.printf(display_buffer);
      pc.printf("\n\r");
      float curr = (float)current;
      curr = curr/10;
      float dut = (float)duty;
      dut = dut/1000;
      sprintf(display_buffer, "Current %f", curr );
      pc.printf(display_buffer);
      pc.printf("\n\r");
      
      sprintf(display_buffer, "DUTY %f", dut );
      pc.printf(display_buffer);
      pc.printf("\n\r");
      
      
      for (int i = 0; i< sizeof(receive_msg.data);i++){
                      sprintf(display_buffer, " 0x%.2X", receive_msg.data[i]);
            pc.printf(display_buffer);
            pc.printf("\n\r"); 
          
          }
      sprintf(display_buffer, "SIZE %d", sizeof(receive_msg.data) );
      pc.printf(display_buffer);
      pc.printf("\n\r");
          
}   
}


int main() 
{
/*
while(1)
 {
   crossbow(9,0);
   wait(10);
   
 }
 */   
       wait(2);
       crossbow(9,0);
       wait(2);
}


