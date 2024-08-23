#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>



#define BASESERVO_CHANNEL 0      // GPIO pin used to connect the servo control (digital out)
#define HEADSERVO_CHANNEL 1
#define turretinc 2 
#define BRUSHLESS_CHANNEL 2
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


#define STEPPER_IN1 15
#define STEPPER_IN2 2
#define STEPPER_IN3 33
#define STEPPER_IN4 32
AccelStepper non_blocking_stepper(AccelStepper::FULL4WIRE, STEPPER_IN1, STEPPER_IN2, STEPPER_IN3, STEPPER_IN4);

#define CE_PIN 4
#define CSN_PIN 5
RF24 radio(CE_PIN, CSN_PIN); // (CE, CSN)

struct { //received packet
byte engineJoystickData[3]; 
byte servoJoystickData[3]; 
byte canon_data[3]; 
} joysticks;

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

const byte deadZoneMin = 127-10; // 118 - 20
const byte deadZoneMax = 127+10; // 118 + 20


// initialize  fpv turret
int xShift = 127;
int yShift = 127;
int fpv_inipos_y = 80;
int fpv_inipos_x = 45;

// Declare speedL and speedR as global variables
float speedL, speedR;

void setup() {
    Serial.begin(115200); // Begin Serial communication

    stepper_init();
    servo_init();
    radio_init();

    // Initialize motor direction pins as outputs
    pinMode(MOTOR_DIR_L, OUTPUT);
    pinMode(MOTOR_DIR_R, OUTPUT);

}


void loop() {
    if (radio.available()) {
        radio.read(&joysticks, sizeof(joysticks));

        byte x1 = joysticks.engineJoystickData[0];
        byte y1 = joysticks.engineJoystickData[1];
        byte sw1 = joysticks.engineJoystickData[2]; // switch joystick 1

        byte x2 = joysticks.servoJoystickData[0]; 
        byte y2 = joysticks.servoJoystickData[1]; 
        byte sw2 = joysticks.servoJoystickData[2]; // switch joystick 2
        //printJoysticksData(x1, y1, sw1, x2, y2, sw2);

        byte pot = joysticks.canon_data[0]; 
        byte left = joysticks.canon_data[1]; 
        byte right = joysticks.canon_data[2]; 

        handleJoystickInput(x1, y1);
        handleServoJoystick(x2, y2, sw2, sw1);
        handlecanon_data(pot,left,right);

        lastReceiveTime = millis();
    }
    non_blocking_stepper.runSpeed();

    checkTimeout(); // Check for timeout condition

    delay(10); // Adding a small delay for smoother control
}


void servo_init(){
pwm.begin();
pwm.setPWMFreq(50);  // Standard servo frequency is 50Hz

}
void moveServo(uint8_t channel, int angle) {
  int pulse = map(angle, 0, 180, 102, 550);  // Map 0-180 degrees to PCA9685 pulse range
  //PWM signals with a resolution of 12 bits (4096 steps), so 500->2500us to 102->512
  pwm.setPWM(channel, 0, pulse);
}


void handlecanon_data(byte speed, byte left, byte right){
    speed = map(speed,0,255,0,180);
    moveServo(BRUSHLESS_CHANNEL, speed);
    Serial.print(" angle 3th ");
    Serial.println(speed);

    if(right){
        //Serial.println(" reload right");
        non_blocking_stepper.setSpeed(100);//100
      }
    if(left){
       
        non_blocking_stepper.setSpeed(1000); //problem with the library, it is not turn ccw
        //Serial.println(" reload left");
      }
    if(!left  && !right){
    //Serial.println("reload not moving");
    non_blocking_stepper.setSpeed(0); // Stop movement
    }
}


void stop_cannon(){
  //moveServo(BRUSHLESS_CHANNEL, 0);
  //non_blocking_stepper.setSpeed(0); // Stop movement
}
// Handle joystick for servo (camera) movement
void handleServoJoystick(byte x, byte y, byte sw, byte state){
  
  const int deadZoneMin = 110 - 10;
  const int deadZoneMax = 110 + 10; //idk why, the joytick midpoint is 111x, 110y, so doing its own point

  if(!state){
    if(sw == HIGH){ // if joystick is pressed (switch)
      xShift = fpv_inipos_x;  // reset angles
      yShift = fpv_inipos_y;
    } else if(x >= deadZoneMin && x <= deadZoneMax && y >= deadZoneMin && y <= deadZoneMax){
      // do nothing
    } 
    if (x > deadZoneMax){
        if(xShift <= 180){xShift += turretinc;}
          
    } 
    if (x < deadZoneMin){
        if(xShift >= 0){xShift -= turretinc;}
    } 
    if (y > deadZoneMax){
            if(yShift <= 180){yShift += turretinc;}
          
    } 
    if (y < deadZoneMin){
        if(yShift >= 0){yShift -= turretinc;}
          
    }
    //  Serial.print(" x : :");Serial.print(x);Serial.print(":  y : :");Serial.println(y);
  }
  if(state){
    if(sw){
      //reset mpu somehow
    }
    xShift = x;
    yShift = y;
  }
  moveServo(BASESERVO_CHANNEL, yShift);
  moveServo(HEADSERVO_CHANNEL, xShift);
}

// Stop the motors
void stop_motors() {
    //Serial.println("Stop");
    analogWrite(MOTOR_PWM_L, 0);
    analogWrite(MOTOR_PWM_R, 0);
}

void checkTimeout() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastReceiveTime >= timeoutInterval) {
        // Timeout action here
        Serial.print("Transmission timeout  :::: Stopping activities ");
        stop_motors();
        stop_cannon();
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


// Handle joystick input and control motors
void handleJoystickInput(byte x, byte y) {
    if (x >= deadZoneMin && x <= deadZoneMax && y >= deadZoneMin && y <= deadZoneMax) {
        stop_motors();
        return;
    }

    int mappedX = map(x, 0, MAX_SPEED, -MAX_SPEED, MAX_SPEED); // 0-255 -> -255 to 255
    int mappedY = map(y, 0, MAX_SPEED, -MAX_SPEED, MAX_SPEED);

    calculateMotorSpeeds(mappedX, mappedY);

    // Ensure speeds are within bounds
    int motorSpeedL = constrain(speedL, -MAX_SPEED, MAX_SPEED);
    int motorSpeedR = constrain(speedR, -MAX_SPEED, MAX_SPEED);

    // Set the motor speed and direction
    setMotorSpeed(motorSpeedL, MOTOR_DIR_L, MOTOR_PWM_L);
    setMotorSpeed(motorSpeedR, MOTOR_DIR_R, MOTOR_PWM_R);

     //printDebugInfo(mappedX, mappedY, speedL, speedR);
}
// Calculate motor speeds based on input
void calculateMotorSpeeds(float mappedX, float mappedY) {
    
    float normalizedX = mappedX / MAX_SPEED; 
    float normalizedY = mappedY / MAX_SPEED;
    
    // Calculate magnitude and ensure it does not exceed 1.0
    float magnitude = fmin(sqrt(normalizedX * normalizedX + normalizedY * normalizedY), 1.0f);
    float turnFactor = pow(fabs(normalizedX), 1.5) * (1 - fabs(normalizedY));
    
    // Calculate speed while taking into acount the turn
    speedR = normalizedY - turnFactor * sgn(normalizedX);  
    speedL = normalizedY + turnFactor * sgn(normalizedX);  
    
    // Scale by the magnitude of the vector
    speedL *= MAX_SPEED * magnitude;  
    speedR *= MAX_SPEED * magnitude; 
}

// Set motor speed and direction
void setMotorSpeed(int speed, int dirPin, int pwmPin) {
    if (speed > 0) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
        speed = -speed;
    }
    if(speed < 50){speed = 0;} //avoid annoying noise
    analogWrite(pwmPin, speed);
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
void printDebugInfo(int mappedX, int mappedY, int speedL, int speedR) {
    Serial.print("MappedX: ");
    Serial.print(mappedX);

    Serial.print(" MappedY: ");
    Serial.println(mappedY);

    Serial.print("SpeedL: ");
    Serial.print(speedL);
    Serial.print(" DirectionL: ");
    Serial.println(speedL > 0 ? "Forward" : "Backward");

    Serial.print("SpeedR: ");
    Serial.print(speedR);
    Serial.print(" DirectionR: ");
    Serial.println(speedR > 0 ? "Forward" : "Backward");
}

// To determine the sign of some variable
int sgn(float val) {return (0 < val) - (val < 0);}

void radio_init(){
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
}

void stepper_init(){
  non_blocking_stepper.setMaxSpeed(1000);
  non_blocking_stepper.setSpeed(0);
}
