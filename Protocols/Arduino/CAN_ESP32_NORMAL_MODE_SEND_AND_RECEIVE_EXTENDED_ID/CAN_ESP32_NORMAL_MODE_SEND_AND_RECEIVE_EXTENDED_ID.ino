#include <ESP32-TWAI-CAN.hpp>

#define CAN_TX 6
#define CAN_RX 7

// Callback when a CAN frame is received
void CAN_RECEIVE(const CanFrame &rxFrame) {
  Serial.println("CAN Message Received!");
  Serial.print("  ID: 0x");
  Serial.println(rxFrame.identifier, HEX);
  Serial.print("  DLC: ");
  Serial.println(rxFrame.data_length_code);
  Serial.print("  Data: ");

  for (uint8_t i = 0; i < rxFrame.data_length_code; i++) {
    Serial.printf("0x%02X ", rxFrame.data[i]);
  }
  Serial.println();
  Serial.println("==========================\r\n");
}

void setup() {
  Serial.begin(115200);
  Serial.println("\r\n==========================");
  Serial.println("ESP32 CAN Send + Receive");
  Serial.println("==========================\r\n");

  if (ESP32Can.begin(TWAI_SPEED_1000KBPS, CAN_TX, CAN_RX)) {
    Serial.println("✓ CAN Init OK");
    ESP32Can.onReceive(&CAN_RECEIVE);
  } else {
    Serial.println("✗ CAN Init ERROR");
    while (1)
      ;
  }
}

void loop() {
  // Create a CAN frame

  CanFrame txFrame;
  txFrame.identifier = 0x1234567;  // CAN ID
  txFrame.extd = 1;                // 0 = Standard ID, 1 = Extended ID
  txFrame.rtr = 0;                 // 0 = Data frame, 1 = Remote frame
  txFrame.data_length_code = 8;    // Number of bytes (0–8)

  // Example data payload
  for (int i = 0; i < 8; i++) {
    txFrame.data[i] = i;  // Fill with simple counter values
  }

  // Send the frame
  if (ESP32Can.writeFrame(txFrame)) {
    Serial.println("CAN Message Sent!");
  } else {
    Serial.println("Failed to send CAN message.");
  }

  delay(1000);  // send every 1 second
}
