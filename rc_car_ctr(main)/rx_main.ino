#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(4, 5); // (CE, CSN)

// Timeout variables
unsigned long lastReceiveTime = 0;
const long timeoutInterval = 50; // 50 ms timeout interval

// package to receive joystick data
struct {
  byte joystick1Data[3];
  byte joystick2Data[3];
} joysticks;

#define MOTOR_DIR_L 13
#define MOTOR_PWN_L 12
#define MOTOR_DIR_R 14
#define MOTOR_PWN_R 27

const int freq = 10000;       // 10 kHz frequency
const int pwmChannelL = 0;    // PWM channel for left motor
const int pwmChannelR = 1;    // PWM channel for right motor
const int resolution = 8;     // 8-bit resolution (0-255)

void setup() {
  // Initialize motor direction pins as outputs
  pinMode(MOTOR_DIR_L, OUTPUT);
  pinMode(MOTOR_DIR_R, OUTPUT);

  // Configure PWM channels
  ledcSetup(pwmChannelL, freq, resolution);
  ledcSetup(pwmChannelR, freq, resolution);

  // Attach the PWM channels to the motor PWM pins
  ledcAttachPin(MOTOR_PWN_L, pwmChannelL);
  ledcAttachPin(MOTOR_PWN_R, pwmChannelR);

  delay(10);
  Serial.begin(9600); // begin Serial communication
  radio.begin();      // start radio
  radio.setChannel(5);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);

  radio.setPayloadSize(4);  // this sets the package size to 4 bytes (default, max: 32 bytes) - 4 bytes for joystick values. might increase range
  radio.enableAckPayload(); // enable automatic acknowledge signals
  radio.setAutoAck(1);
  radio.setRetries(5,5); // (delay, max no.of retries)
  radio.setAutoAck(true);

  radio.openReadingPipe(1, 0x1234567890LL); // set address at which nrf24 will communicate
  radio.startListening(); // set receiver mode

}

void loop() {
  if (radio.available()) {
    radio.read(&joysticks, sizeof(joysticks));
    // data from joystick 1 (car movement)
    byte x1 = joysticks.joystick1Data[0];
    byte y1 = joysticks.joystick1Data[1];
    byte sw1 = joysticks.joystick1Data[2]; // switch joystick 1
    
    // data from joystick 2 (turret direction)
    byte x2 = joysticks.joystick2Data[0];
    byte y2 = joysticks.joystick2Data[1];
    byte sw2 = joysticks.joystick2Data[2]; // switch joystick 2

    printJoysticksData(x1, y1, sw1, x2, y2, sw2);

    // handle joystick input for car movement
    handleJoystick1Input(x1, y1);

    // add here functions to handle switch of joystick 1
    // handle joystick 2 input (turret direction)
    // handle switch of joystick 2

    // Update the last received time
    lastReceiveTime = millis();
  }

  // Check for timeout
  checkTimeout();

  Serial.println("X");

}

// Handle joystick input for car movement
void handleJoystick1Input(byte x, byte y) {
  const byte deadZoneMin = 118 - 5;
  const byte deadZoneMax = 118 + 5;

  if (x >= deadZoneMin && x <= deadZoneMax && y >= deadZoneMin && y <= deadZoneMax) {
    stop();
  } else if (x > deadZoneMax) {
    right(x);
  } else if (x < deadZoneMin) {
    left(x);
  } else if (y > deadZoneMax) {
    forward(y);
  } else if (y < deadZoneMin) {
    backward(y);
  }
}

// Placeholder functions for joystick actions
void stop() {
  Serial.println("Stop");
  ledcWrite(pwmChannelL, 0);
  ledcWrite(pwmChannelR, 0);

}

void forward(int speed) {
  Serial.println("Forward");
  speed = map(speed,123,255,0,255);
  digitalWrite(MOTOR_DIR_L, HIGH);
  digitalWrite(MOTOR_DIR_R, HIGH);
  ledcWrite(pwmChannelL, speed);
  ledcWrite(pwmChannelR, speed);
}

void backward(int speed) {
  Serial.println("Backward");
  speed = map(speed,113,0,0,255);
  digitalWrite(MOTOR_DIR_L, LOW);
  digitalWrite(MOTOR_DIR_R, LOW);
  ledcWrite(pwmChannelL, speed);
  ledcWrite(pwmChannelR, speed);
}

void left(int speed) {
  Serial.println("Left");
  speed = map(speed,113,0,0,255);
  digitalWrite(MOTOR_DIR_L, LOW);
  digitalWrite(MOTOR_DIR_R, HIGH);
  ledcWrite(pwmChannelL, speed);
  ledcWrite(pwmChannelR, speed);
}

void right(int speed) {
  Serial.println("Right");
  speed = map(speed,123,255,0,255);
  digitalWrite(MOTOR_DIR_L, HIGH);
  digitalWrite(MOTOR_DIR_R, LOW);
  ledcWrite(pwmChannelL, speed);
  ledcWrite(pwmChannelR, speed);
}

// Check for transmission timeout
void checkTimeout() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastReceiveTime >= timeoutInterval) {
        // Timeout action here
        Serial.println("Transmission timeout");
        lastReceiveTime = currentMillis; // Reset the timer
        // stop();
        resetReception(); // Reset reception
    }
}

// Reset reception settings
void resetReception() {
    // Reinitialize the radio settings
    radio.begin();
    radio.setChannel(5);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_HIGH);

    radio.enableAckPayload(); // enable automatic acknowledge signals
    radio.setRetries(5, 5); // delay, max number of retries
    radio.setAutoAck(true);

    radio.openReadingPipe(1, 0x1234567890LL); // set address at which nrf24 will communicate
    radio.startListening(); // set receiver mode

    Serial.println("Reception reset");
}

void printJoysticksData(byte x1, byte y1, byte sw1, byte x2, byte y2, byte sw2){
  Serial.print("Joystick 1: ");
  Serial.print(x1);
  Serial.print(" , ");
  Serial.print(y1);
  Serial.print(" , ");
  Serial.println(sw1);  
  Serial.print("Joystick 2: ");
  Serial.print(x2);
  Serial.print(" , ");
  Serial.print(y2);
  Serial.print(" , ");
  Serial.println(sw2);
}
