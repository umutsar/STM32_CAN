#include <SPI.h>
#include <mcp2515.h>

struct can_frame Received_Data;
struct can_frame Transmitted_Data;

MCP2515 mcp2515(10);

uint8_t counter = 0;


void setup() {
  Serial.begin(9600);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  Serial.println("------- CAN Read ----------");
}

void loop() {
  //if (mcp2515.readMessage(&Received_Data) != MCP2515::ERROR_OK) {
    mcp2515.readMessage(&Received_Data);
    Serial.print("ID: ");
    Serial.println(Received_Data.can_id, HEX);
    Serial.print("DLC: ");
    Serial.println(Received_Data.can_dlc, DEC);
    
    for (int i = 0; i<8; i++)  {  // print the data
      Serial.println(Received_Data.data[i], DEC);
    }

    if(1) {
      counter = counter + 1;
      delay(100);
      if(counter == 255) {
        counter = 0;
      }
    }

    Serial.print("İlk 7 cell ");
    Transmitted_Data.can_id = 0x370;
    Transmitted_Data.can_dlc = 8;
    
    Transmitted_Data.data[0] = 1;
    Transmitted_Data.data[1] = 1;
    Transmitted_Data.data[2] = 2;
    Transmitted_Data.data[3] = 3;

    Transmitted_Data.data[4] = 4;
    Transmitted_Data.data[5] = 5;
    Transmitted_Data.data[6] = 6;
    Transmitted_Data.data[7] = 7;

    mcp2515.sendMessage(&Transmitted_Data);
    delay(100);


    Serial.print("İlk 7 cell ");
    Transmitted_Data.can_id = 0x370;
    Transmitted_Data.can_dlc = 8;
    
    Transmitted_Data.data[0] = 2;
    Transmitted_Data.data[1] = 8;
    Transmitted_Data.data[2] = 9;
    Transmitted_Data.data[3] = 10;

    Transmitted_Data.data[4] = 11;
    Transmitted_Data.data[5] = 12;
    Transmitted_Data.data[6] = 13;
    Transmitted_Data.data[7] = counter;

    mcp2515.sendMessage(&Transmitted_Data);
    
    Serial.println();      
  // }
}
