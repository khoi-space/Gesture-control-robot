#include <SPI.h>
#include "RF24.h"

const int x_out = A0;
const int y_out = A1;
const int z_out = A2;  


RF24 radio(8, 10); 
const uint8_t address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

struct data {
  int xAxis;
  int yAxis;
  int zAxis;  
};
data send_data;



void setup() {
  Serial.begin(9600);
  
  Serial.println("=== GESTURE TRANSMITTER ===");
  Serial.println("Checking NRF24L01+...");
  

  
  // Check initialize module NRF24L01
  if (!radio.begin()) {
    Serial.println("ERROR: NRF24L01+ not respond!");
    while(1); // Go to infinite loop to halt program
  }
  
  Serial.println("OK: NRF24L01+ done init!");
  
  // Configuration
  radio.setChannel(76);             
  radio.setDataRate(RF24_250KBPS);  
  radio.setPALevel(RF24_PA_HIGH);    
  radio.setAutoAck(false);          
  radio.disableDynamicPayloads();
  radio.setPayloadSize(6);          
  radio.setCRCLength(RF24_CRC_16);  
  
  // Put module NRF24L01 into TX mode (only sent data)
  radio.openWritingPipe(address);
  radio.stopListening();
  
  Serial.println("Payload size: " + String(sizeof(data)) + " bytes");
  delay(2000);
}

void loop() {
  // Read sensor data
  send_data.xAxis = analogRead(x_out);
  send_data.yAxis = analogRead(y_out);
  send_data.zAxis = analogRead(z_out);
  
  // Send data
  bool result = radio.write(&send_data, sizeof(data));
  
  // Debug
  Serial.print("X: ");
  Serial.print(send_data.xAxis);
  Serial.print(" | Y: ");
  Serial.print(send_data.yAxis);
  Serial.print(" | Z: ");
  Serial.print(send_data.zAxis);
  
  delay(500); // 2Hz
}