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
