#include <FlexCAN_T4.h>

// Create a CAN bus object with a different name to avoid conflict
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// Define a CAN message object
CAN_message_t msg;
CAN_message_t helloMsg;

// Timer variables
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 15000; // 15 seconds

void setup() {
  // Start serial communication at 115200 baud rate
  Serial.begin(115200);

  // Wait for serial port to connect (useful for debugging)
  while (!Serial && millis() < 5000);

  // Initialize the CAN bus at 500 kbps
  Can1.begin();
  Can1.setBaudRate(500000);

  // Prepare the hello message
  helloMsg.id = 0x123; // Standard ID
  helloMsg.len = 8;    // Data length code
  const char helloData[] = "HelloWorld";
  memcpy(helloMsg.buf, helloData, helloMsg.len);

  // Print a message to the serial monitor
  Serial.println("CAN Bus Receiver and Sender Initialized");
}

void loop() {
  // Check if a new CAN message is available
  if (Can1.read(msg)) {
    // Print the received CAN message
    if (msg.flags.extended) {
      Serial.print("Extended ID: 0x");
    } else {
      Serial.print("Standard ID: 0x");
    }
    Serial.print(msg.id, HEX);
    Serial.print(" DLC: ");
    Serial.print(msg.len);
    Serial.print(" Data: ");
    for (uint8_t i = 0; i < msg.len; i++) {
      Serial.print(msg.buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Check if the received message is "Hello World"
    if (memcmp(msg.buf, "HelloWorld", msg.len) == 0) {
      // Reset the timer
      lastSendTime = millis();
    }
  }

  // Check if it's time to send the hello message
  if (millis() - lastSendTime >= sendInterval) {
    // Send the hello message
    Can1.write(helloMsg);
    Serial.println("Sent: HelloWorld");

    // Update the timer
    lastSendTime = millis();
  }
}
