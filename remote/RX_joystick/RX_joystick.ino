#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>

RF24 radio(4, 5); // (CE, CSN)

byte joystickData[2]; // Array to hold X and Y values

#define MOTOR_DIR_L 13
#define MOTOR_PWN_L 12
#define MOTOR_DIR_R 14
#define MOTOR_PWN_R 27

#define MAX_SPEED 255/2

const byte deadZoneMin = 108; // 118 - 10
const byte deadZoneMax = 128; // 118 + 10

void setup() {
    Serial.begin(9600); // Begin Serial communication

    // Initialize motor direction pins as outputs
    pinMode(MOTOR_DIR_L, OUTPUT);
    pinMode(MOTOR_DIR_R, OUTPUT);

    // Initialize radio
    if (!radio.begin()) {
        Serial.println("Radio initialization failed");
        while (1);
    }
    radio.setChannel(5);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_HIGH);
    radio.openReadingPipe(1, 0x1234567890LL); // Set address at which nrf24 will communicate
    radio.startListening(); // Set receiver mode

    delay(10);
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

    delay(10); // Adding a small delay for smoother control
}

// Handle joystick input and control motors
void handleJoystickInput(byte x, byte y) {
  //TODO Add exponential decay instead of absrubt stop

    if (x >= deadZoneMin && x <= deadZoneMax && y >= deadZoneMin && y <= deadZoneMax) {
        stop();
        return;
    }

    int mappedX = map(x, 0, MAX_SPEED, -MAX_SPEED, MAX_SPEED);
    int mappedY = map(y, 0, MAX_SPEED, -MAX_SPEED, MAX_SPEED);

    float speedL, speedR;
    calculateMotorSpeeds(mappedX, mappedY, speedL, speedR);

    // Ensure speeds are within bounds
    int motorSpeedL = constrain(speedL, -MAX_SPEED, MAX_SPEED);
    int motorSpeedR = constrain(speedR, -MAX_SPEED, MAX_SPEED);

    // This sets the motor speed and direction
    setMotorSpeed(motorSpeedL, MOTOR_DIR_L, MOTOR_PWN_L);
    setMotorSpeed(motorSpeedR, MOTOR_DIR_R, MOTOR_PWN_R);

    Serial.print("Motor Speed L: ");
    Serial.print(motorSpeedL);
    Serial.print(" | Motor Speed R: ");
    Serial.println(motorSpeedR);
}

// Function to calculate motor speeds based on joystick inputs
void calculateMotorSpeeds(int mappedX, int mappedY, float &speedL, float &speedR) {
    // Dynamic turn factor based on joystick's position
    float turnFactor = 1.0 - (abs(mappedY) / MAX_SPEED); // Higher Y input results in lower turn factor

    // Calculate the motor speeds directly
    speedL = mappedY + (mappedX * turnFactor);
    speedR = mappedY - (mappedX * turnFactor);
}


// Set motor speed and direction
void setMotorSpeed(int speed, int dirPin, int pwmPin) {
    if (speed > 0) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
        speed = -speed;
    }
    analogWrite(pwmPin, speed);
}

// Stop the motors
void stop() {
    Serial.println("Stop");
    analogWrite(MOTOR_PWN_L, 0);
    analogWrite(MOTOR_PWN_R, 0);
}
