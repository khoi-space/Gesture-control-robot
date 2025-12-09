#include <SPI.h>
#include "RF24.h"

// Fixed data values instead of sensor inputs
const int x_out = 100;
const int y_out = 120;
const int z_out = 140;  
const int irq_pin = 2; // IRQ pin - use pin 2 or 3 for interrupt

RF24 radio(8, 10); 
const uint8_t address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

struct data {
  int xAxis;
  int yAxis;
  int zAxis;  
};
data send_data;

// IRQ flag
volatile bool irq_flag = false;

// Status variables for whatHappened()
bool tx_ok, tx_fail, rx_ready;

// IRQ interrupt handler
void irq_handler() {
  irq_flag = true; // Set flag when IRQ triggered
}

void setup() {
  Serial.begin(9600);
  
  Serial.println("=== GESTURE TRANSMITTER WITH FIXED DATA ===");
  Serial.println("Kiem tra NRF24L01+...");
  
  // Setup IRQ pin
  pinMode(irq_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(irq_pin), irq_handler, FALLING); // IRQ is active low
  
  // Kiểm tra khởi tạo NRF24L01+
  if (!radio.begin()) {
    Serial.println("ERROR: NRF24L01+ khong phan hoi!");
    Serial.println("Kiem tra ket noi day:");
    while(1); // Dừng chương trình
  }
  
  Serial.println("OK: NRF24L01+ khoi tao thanh cong!");
  
  // Configuration
  radio.setChannel(76);             
  radio.setDataRate(RF24_250KBPS);  
  radio.setPALevel(RF24_PA_HIGH);    
  radio.setAutoAck(false);          
  radio.disableDynamicPayloads();
  radio.setPayloadSize(6);          
  radio.setCRCLength(RF24_CRC_16);  
  
  // Enable IRQ for TX_DS (transmission complete)
  radio.maskIRQ(1, 1, 0); // Mask RX_DR, MAX_RT, enable TX_DS
  radio.maskIRQ(0, 1, 1);
  
  radio.openWritingPipe(address);
  radio.stopListening();
  
  Serial.println("Setup OK with IRQ. Dang gui du lieu co dinh...");
  Serial.println("Payload size: " + String(sizeof(data)) + " bytes");
  Serial.println("Fixed values - X: 100, Y: 120, Z: 140");
  delay(2000);
}

void loop() {
  // Use fixed data values instead of reading from sensor
  send_data.xAxis = x_out;
  send_data.yAxis = y_out;
  send_data.zAxis = z_out;
  
  // Send data
  bool result = radio.write(&send_data, sizeof(data));
  
  // Check IRQ flag
  if (irq_flag) {
    // Serial.println("[IRQ] Transmission complete interrupt received!");
    irq_flag = false; // Clear flag
    
    // Clear IRQ flags using public functions
    radio.whatHappened(tx_ok, tx_fail, rx_ready); // This clears IRQ flags automatically
  }
  
  // Debug info
  Serial.print("X: ");
  Serial.print(send_data.xAxis);
  Serial.print(" | Y: ");
  Serial.print(send_data.yAxis);
  Serial.print(" | Z: ");
  Serial.print(send_data.zAxis);
  // Serial.print(" | Gui: ");
  // Serial.print(result ? "OK" : "FAIL");
  // Serial.print(" | TX_OK: ");
  // Serial.print(tx_ok ? "Y" : "N");
  Serial.print("| IRQ pin: ");
  Serial.println(digitalRead(irq_pin));
  
  delay(500); // 10Hz
}
