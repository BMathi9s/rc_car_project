#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <ESP32Servo.h> 
//esp32servo by kevin 
// only work by using version 1.2.1!!
#define BASESERVO_PIN 26      // GPIO pin used to connect the servo control (digital out)
#define HEADSERVO_PIN 33

RF24 radio(4, 5); // (CE, CSN)

byte engineJoystickData[2]; // Array to hold X and Y values for the engines
byte servoJoystickData[2]; // Array to hold X and Y values for servo

#define MOTOR_DIR_L 13
#define MOTOR_PWM_L 12
#define MOTOR_DIR_R 14
#define MOTOR_PWM_R 27

// Timeout variables
unsigned long lastReceiveTime = 0;
const long timeoutInterval = 50; // 50 ms timeout interval

const int freq = 10000;       // 10 kHz frequency
const int pwmChannelL = 0;    // PWM channel for left motor
const int pwmChannelR = 1;    // PWM channel for right motor
const int resolution = 8;     // 8-bit resolution (0-255)

#define MAX_SPEED 255

const byte deadZoneMin = 98; // 118 - 20
const byte deadZoneMax = 138; // 118 + 20

// servo motors
Servo head; // (x)
Servo base; // (y)

// initialize 
int xShift = 90;
int yShift = 90;

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

    radio.setPayloadSize(4);  // this sets the package size to 4 bytes (default, max: 32 bytes) - 4 bytes for joystick values. might increase range
    radio.enableAckPayload(); // enable automatic acknowledge signals
    radio.setAutoAck(1);
    radio.setRetries(5, 5); // (delay, max no.of retries)
    radio.setAutoAck(true);

    radio.openReadingPipe(1, 0x1234567890LL); // set address at which nrf24 will communicate
    radio.startListening(); // set receiver mode

    servo_init();
}

void loop() {
    if (radio.available()) {
        radio.read(&engineJoystickData, sizeof(engineJoystickData));
        radio.read(&servoJoystickData, sizeof(servoJoystickData));

        byte x1 = engineJoystickData[0];
        byte y1 = engineJoystickData[1];

        byte x2 = servoJoystickData[0]; 
        byte y2 = servoJoystickData[1]; 

        Serial.print("Joystick1 X: ");
        Serial.print(x1);
        Serial.print(" | Joystick1 Y: ");
        Serial.println(y1);

        handleJoystickInput(x1, y1);
        handleServoJoystick(x2, y2);

        lastReceiveTime = millis();
    }

    checkTimeout(); // Check for timeout condition

    delay(10); // Adding a small delay for smoother control
}

// Handle joystick input and control motors
void handleJoystickInput(byte x, byte y) {
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

    // Set the motor speed and direction
    setMotorSpeed(motorSpeedL, MOTOR_DIR_L, MOTOR_PWM_L);
    setMotorSpeed(motorSpeedR, MOTOR_DIR_R, MOTOR_PWM_R);

    Serial.print("Motor Speed L: ");
    Serial.print(motorSpeedL);
    Serial.print(" | Motor Speed R: ");
    Serial.println(motorSpeedR);
}

// Function to calculate motor speeds based on joystick inputs
void calculateMotorSpeeds(float mappedX, float mappedY, float &speedL, float &speedR) {
    // Diagonal Movement
    float turnFactor = ((abs(mappedY) / (float)MAX_SPEED)) * (abs(mappedX) / (float)MAX_SPEED);

    speedL = mappedY + (turnFactor * mappedX);
    speedR = mappedY - (turnFactor * mappedX);
}

float exponentialDecay(float input, float decayRate) {
    return input * exp(-decayRate * abs(input));
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

void servo_init(){
    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    base.setPeriodHertz(50); // Standard 50hz servo
    base.attach(BASESERVO_PIN, 500, 2400);
    head.setPeriodHertz(50);
    head.attach(HEADSERVO_PIN, 500, 2400); 
}

void handleServoJoystick(byte x, byte y) {
    const byte deadZoneMin = 128 - 20;
    const byte deadZoneMax = 128 + 20;

    if (x >= deadZoneMin && x <= deadZoneMax && y >= deadZoneMin && y <= deadZoneMax) {
        // do nothing
    } else {
        if (x > deadZoneMax) {
            if (xShift <= 180)
                xShift++;
        }
        if (x < deadZoneMin) {
            if (xShift >= 0)
                xShift--;
        }
        if (y > deadZoneMax) {
            if (yShift <= 180)
                yShift++;
        }
        if (y < deadZoneMin) {
            if (yShift >= 0)
                yShift--;
        }
    }

    head.write(xShift);
    base.write(yShift);
}

// Stop the motors
void stop() {
    Serial.println("Stop");
    analogWrite(MOTOR_PWM_L, 0);
    analogWrite(MOTOR_PWM_R, 0);
}

void checkTimeout() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastReceiveTime >= timeoutInterval) {
        // Timeout action here
        Serial.println("Transmission timeout");
        lastReceiveTime = currentMillis; // Reset the timer
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
