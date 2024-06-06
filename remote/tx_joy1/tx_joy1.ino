#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(4, 5); // (CE, CSN)

// Define joystick pins
#define JOYSTICK1_X_PIN 14 // example pin, change as needed
#define JOYSTICK1_Y_PIN 27 // example pin, change as needed

byte joystickData[2];

void setup() {
  Serial.begin(9600); // begin Serial communication
  radio.begin();      // start radio
  radio.setChannel(5);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openWritingPipe(0x1234567890LL); // set address at which nrf24 will communicate
  radio.stopListening(); // set transmitter mode

  pinMode(JOYSTICK1_X_PIN, INPUT);
  pinMode(JOYSTICK1_Y_PIN, INPUT);
}

void loop() {
  // Read joystick values
  int rawX = analogRead(JOYSTICK1_X_PIN);
  int rawY = analogRead(JOYSTICK1_Y_PIN);

  // Map values from 0-4095 to 0-255
  joystickData[0] = map(rawX, 0, 4095, 0, 255);
  joystickData[1] = map(rawY, 0, 4095, 0, 255);

  // Send joystick data
  radio.write(&joystickData, sizeof(joystickData));

  delay(10);
  Serial.print("Joystick X: ");
  Serial.print(joystickData[0]);
  Serial.print(" | Joystick Y: ");
  Serial.println(joystickData[1]);
}
