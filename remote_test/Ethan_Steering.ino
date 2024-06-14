#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


RF24 radio(4, 5); // (CE, CSN)

byte joystickData[2]; // Array to hold X and Y values


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
  radio.openReadingPipe(1, 0x1234567890LL); // set address at which nrf24 will communicate
  radio.startListening(); // set receiver mode


}

void loop() {
  if (radio.available()) {
    radio.read(&joystickData, sizeof(joystickData));
    byte x = joystickData[0];
    byte y = joystickData[1];

    Serial.print("Joystick X: ");
    Serial.print(x);
    Serial.print(" | Joystick Y: ");
    Serial.println(y);

    handleJoystickInput(x, y);
  }
  Serial.println("X");
}

// Handle joystick input
void handleJoystickInput(byte x, byte y) {
  const byte deadZoneMin = 118 - 40;
  const byte deadZoneMax = 118 + 40;

  int joyValueMid = 118; // Midpoint for joystick values
  int motorSpeedMin = 0; // Minimum motor speed (adjust as needed)
  int motorSpeedMax = 255; // Maximum motor speed

  int speedFwd = 0;
  int speedTurn = 0;
  int speedLeft = 0;
  int speedRight = 0;

  if (y > deadZoneMax) { // Forward
    speedFwd = map(y, deadZoneMax, 255, motorSpeedMin, motorSpeedMax);
  } else if (y < deadZoneMin) { // Backward
    speedFwd = map(y, deadZoneMin, 0, -motorSpeedMin, -motorSpeedMax);
  } else {
    speedFwd = 0;
  }

  if (x > deadZoneMax) { // Right
    speedTurn = map(x, deadZoneMax, 255, motorSpeedMin, motorSpeedMax);
  } else if (x < deadZoneMin) { // Left
    speedTurn = map(x, deadZoneMin, 0, -motorSpeedMin, -motorSpeedMax);
  } else {
    speedTurn = 0;
  }

  // Convert Forward speed and Turn Speed to Motor left and Motor Right speed
  speedLeft = speedFwd + speedTurn;
  speedRight = speedFwd - speedTurn;

  speedLeft = constrain(speedLeft, -255, 255);
  speedRight = constrain(speedRight, -255, 255);

  // Send the Motor speed value to Motor Driver
  moveRobot(speedLeft, speedRight);
}

// Send the Motor speed value to Motor Driver
void moveRobot(int spdL, int spdR) {
  // Set Motor Direction
  if (spdL > 0) {
      digitalWrite(MOTOR_DIR_L, HIGH);
  } else {
      digitalWrite(MOTOR_DIR_L, LOW);
  }
  if (spdR > 0) {
      digitalWrite(MOTOR_DIR_R, HIGH);
  } else {
      digitalWrite(MOTOR_DIR_R, LOW);
  }

  // Set Motor Speed
  ledcWrite(pwmChannelL, abs(spdL));
  ledcWrite(pwmChannelR, abs(spdR));
}

// Placeholder functions for joystick actions (no longer needed)
void stop() {
  Serial.println("Stop");
  ledcWrite(pwmChannelL, 0);
  ledcWrite(pwmChannelR, 0);
}
