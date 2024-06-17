#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(4, 5); // (CE, CSN)

// Timeout variables
unsigned long previousMillis = 0; // Stores the last time a packet was sent
static int txFailCount = 0; // counts how many times transmission failed
const unsigned long timeoutInterval = 50; // Timeout interval in milliseconds

// Define joystick pins
// For car direction
#define JOYSTICK1_X_PIN 14 // example pin, change as needed
#define JOYSTICK1_Y_PIN 27 // example pin, change as needed
#define JOYSTICK1_SWITCH_PIN 12 // active low

// For turret
#define JOYSTICK2_X_PIN 25
#define JOYSTICK2_Y_PIN 33
#define JOYSTICK2_SWITCH_PIN 26 // active low

// package to transmit joystick data
struct {
  byte joystick1Data[3];
  byte joystick2Data[3];
  
} joysticks;

void setup() {
  Serial.begin(9600); // begin Serial communication
  radio.begin();      // start radio
  radio.setChannel(5);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH); // should be RF24_PA_HIGH. We want the radio to be as lossy as possible to test timeout
  
  radio.setPayloadSize(4);  // this sets the package size to 4 bytes (default, max: 32 bytes) - 4 bytes for joystick values. might increase range
  radio.enableAckPayload(); // enable automatic acknowledge signals
  radio.setAutoAck(1);
  radio.setRetries(5,5); // delay, max no.of retries
  radio.setAutoAck(true);

  radio.openWritingPipe(0x1234567890LL); // set address at which nrf24 will communicate
  radio.stopListening(); // set transmitter mode

  pinMode(JOYSTICK1_X_PIN, INPUT);
  pinMode(JOYSTICK1_Y_PIN, INPUT);
  pinMode(JOYSTICK1_SWITCH_PIN, INPUT_PULLUP);
  pinMode(JOYSTICK2_X_PIN, INPUT);
  pinMode(JOYSTICK2_Y_PIN, INPUT);
  pinMode(JOYSTICK2_SWITCH_PIN, INPUT_PULLUP);
}

void loop() {
  // Read joystick 1 values
  int rawX1 = analogRead(JOYSTICK1_X_PIN);
  int rawY1 = analogRead(JOYSTICK1_Y_PIN);
  byte rawSW1 = digitalRead(JOYSTICK1_SWITCH_PIN);

  // Read joystick 2 values
  int rawX2 = analogRead(JOYSTICK2_X_PIN);
  int rawY2 = analogRead(JOYSTICK2_Y_PIN);
  byte rawSW2 = digitalRead(JOYSTICK2_SWITCH_PIN);

  // Map values from 0-4095 to 0-255 joystick 1
  joysticks.joystick1Data[0] = map(rawX1, 0, 4095, 0, 255);
  joysticks.joystick1Data[1] = map(rawY1, 0, 4095, 0, 255);
  joysticks.joystick1Data[2] = rawSW1;

  // mapping values for joystick 2
  joysticks.joystick2Data[0] = map(rawX2, 0, 4095, 0, 255);
  joysticks.joystick2Data[1] = map(rawY2, 0, 4095, 0, 255);
  joysticks.joystick2Data[2] = rawSW2;

  // Send joystick data
  send();

  // Check for timeout
  checkTimeout();
}

void send() {
    bool rslt;
    rslt = radio.write(&joysticks, sizeof(joysticks));
    delay(10);
    Serial.print("Data Sent ");

    if (rslt) {
        Serial.println("RECEIVED");
        previousMillis = millis(); // Reset the timeout timer on successful transmission
        printJoysticksData();
    }
    else {
        Serial.println("Tx failed");
        if(txFailCount < 5){
          txFailCount++;
        }
        else{
          resetTransmission(); // Reset transmission based on number of transmissions fails (5)
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

void printJoysticksData(){
  Serial.print("Joystick 1: ");
  Serial.print(joysticks.joystick1Data[0]);
  Serial.print(" , ");
  Serial.println(joysticks.joystick1Data[1]);  
  Serial.print("Joystick 2: ");
  Serial.print(joysticks.joystick2Data[0]);
  Serial.print(" , ");
  Serial.println(joysticks.joystick2Data[1]);
}





