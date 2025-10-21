#include <SPI.h>

// Arduino SPI Pins:
// Pin 10 -> SS (Slave Select) - Connect to STM32 PB12
// Pin 11 -> MOSI - Connect to STM32 PB15
// Pin 12 -> MISO - Connect to STM32 PB14
// Pin 13 -> SCK - Connect to STM32 PB13

const int slaveSelectPin = 10;
char txBuffer[32] = "Hello from Arduino!";
char rxBuffer[32];

void setup() {
  Serial.begin(9600);

  // Set slave select pin as output
  pinMode(slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, HIGH);  // Deselect slave initially

  // Initialize SPI
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);  // 1MHz (16MHz/16)
  SPI.setDataMode(SPI_MODE0);            // CPOL=0, CPHA=0
  SPI.setBitOrder(MSBFIRST);

  Serial.println("Arduino Master Ready");
}

void loop() {
  char txData[16] = "Arduino Master!";
  char rxData[16];

  delay(100);

  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(100);  // Increase delay to 100us

  for (int i = 0; i < 16; i++) {
    rxData[i] = SPI.transfer(txData[i]);
    delayMicroseconds(10);  // Add delay between bytes
  }

  digitalWrite(slaveSelectPin, HIGH);

  Serial.print("Sent: ");
  Serial.println(txData);
  Serial.print("Received: ");
  Serial.println(rxData);

  delay(1000);
}