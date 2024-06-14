#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// map booleans into bits for smaller transmission, set package size
// or try with joystick (it's only two bytes)

RF24 radio(4, 5); // (CE, CSN)

// Timeout variables
unsigned long previousMillis = 0; // Stores the last time a packet was sent
static int txFailCount = 0; // counts how many times transmission failed
const unsigned long timeoutInterval = 50; // Timeout interval in milliseconds

// Define joystick pins
#define JOYSTICK1_X_PIN 14 // example pin, change as needed
#define JOYSTICK1_Y_PIN 27 // example pin, change as needed

byte joystickData[2];


void setup() {
  Serial.begin(9600); // begin Serial communication
  radio.begin();      // start radio
  radio.setChannel(5);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH); // should be RF24_PA_HIGH. We want the radio to be as lossy as possible to test timeout
  
  radio.setPayloadSize(2);  // this sets the package size to 2 bytes (default, max: 32 bytes) - 2 bytes for joystick values. might increase range
  radio.enableAckPayload(); // enable automatic acknowledge signals
  radio.setAutoAck(1);
  radio.setRetries(5,5); // delay, max no.of retries
  //radio.setAutoAck(true);

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
  send();

  // Check for timeout
  checkTimeout();
}

void send() {
    bool rslt;
    rslt = radio.write(&joystickData, sizeof(joystickData));
    delay(10);
    Serial.print("Data Sent ");
    Serial.print("Joystick X: ");
    Serial.print(joystickData[0]);
    Serial.print(" | Joystick Y: ");
    Serial.print(joystickData[1]);
    Serial.print("   ");


    if (rslt) {
        Serial.println("RECEIVED");
        previousMillis = millis(); // Reset the timeout timer on successful transmission
    }
    else {
        Serial.println("Tx failed");
        if(txFailCount < 5){
          txFailCount++;
        }
        else{
          resetTransmission(); // Reset transmission based on number of transmissions failes (5)
          txFailCount = 0;  // reset transmission fail count to 0 
        }
    }
}

void checkTimeout() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= timeoutInterval) {
        // Timeout action here
        Serial.println("Transmission timeout");
        previousMillis = currentMillis; // Reset the timer
        // resetTransmission(); // Reset transmission based on timeout conditions (timeoutInterval)
    }
}

void resetTransmission() { // transmission can be reset due to timeout (checkTimeout) or number of failed transmissions (send)
    // Reinitialize the radio settings
    radio.begin();
    radio.setChannel(5);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_HIGH);

    radio.enableAckPayload(); // enable automatic acknowledge signals
    radio.setRetries(5, 5); // delay, max number of retries

    radio.openWritingPipe(0x1234567890LL); // set address at which nrf24 will communicate
    radio.stopListening(); // set transmitter mode

    Serial.println("Transmission reset");
}




