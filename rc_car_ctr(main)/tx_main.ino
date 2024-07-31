#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "I2Cdev.h" // for mpu
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

RF24 radio(4, 5); // (CE, CSN)
MPU6050 mpu;      // gyro in headset

// Timeout variables
unsigned long previousMillis = 0; // Stores the last time a packet was sent
static int txFailCount = 0; // counts how many times transmission failed
const unsigned long timeoutInterval = 50; // Timeout interval in milliseconds

// Define joystick pins
// For car direction (left)
#define JOYSTICK1_X_PIN 14 // example pin, change as needed
#define JOYSTICK1_Y_PIN 27 // example pin, change as needed
#define JOYSTICK1_SWITCH_PIN 12 // active low
bool switch1State = false;
bool oldSW1 = 0;

// For turret (joystick right)
#define JOYSTICK2_X_PIN 25
#define JOYSTICK2_Y_PIN 33
#define JOYSTICK2_SWITCH_PIN 26 // active low


// package to transmit joystick data
struct {
  byte joystick1Data[3];
  byte servoData[3]; // [x, y, switch] OR [yaw, pitch, roll]
  
} package;

//  MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  Serial.begin(9600); // begin Serial communication

  // radio initialization
  radio.begin();      // start radio
  radio.setChannel(5);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH); // should be RF24_PA_HIGH
  
  radio.setPayloadSize(6);  // this sets the package size to 6 bytes (default, max: 32 bytes) - 6 bytes for joystick values
  radio.enableAckPayload(); // enable automatic acknowledge signals
  radio.setAutoAck(1);
  radio.setRetries(5,5); // delay, max no.of retries
  radio.setAutoAck(true);

  radio.openWritingPipe(0x1234567890LL); // set address at which nrf24 will communicate
  radio.stopListening(); // set transmitter mode

  // mpu initialization
  mpu.initialize(ACCEL_FS::A2G, GYRO_FS::G250DPS);
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize(); 

  // supply your own gyro offsets here, scaled for min sensitivity RUN IMU_ZERO W/ MPU AND ESP TO GET THESE VALUES - THEN REPLACE THEM
  // XAccel : -1747
  // YAccel : -332
  mpu.setZAccelOffset(3456); 
  mpu.setXGyroOffset(45);
  mpu.setYGyroOffset(-11);
  mpu.setZGyroOffset(9);

  if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6); // LOOK INTO THIS
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

  pinMode(JOYSTICK1_X_PIN, INPUT);
  pinMode(JOYSTICK1_Y_PIN, INPUT);
  pinMode(JOYSTICK1_SWITCH_PIN, INPUT_PULLUP);
  pinMode(JOYSTICK2_X_PIN, INPUT);
  pinMode(JOYSTICK2_Y_PIN, INPUT);
  pinMode(JOYSTICK2_SWITCH_PIN, INPUT_PULLUP);

}

void loop() {
  // Read joystick 1 values (direction)
  int rawX1 = analogRead(JOYSTICK1_X_PIN);
  int rawY1 = analogRead(JOYSTICK1_Y_PIN);
  byte rawSW1 = digitalRead(JOYSTICK1_SWITCH_PIN);
 
 
  if(rawSW1 == 1 && rawSW1 != oldSW1){
    switch1State = !switch1State;
    Serial.println(switch1State);
  }

  oldSW1 = rawSW1;

  // Map values from 0-4095 to 0-255 joystick 1
  package.joystick1Data[0] = map(rawX1, 0, 4095, 0, 255);
  package.joystick1Data[1] = map(rawY1, 0, 4095, 0, 255);
  package.joystick1Data[2] = switch1State; // 1 is gyro, 0 is joystick 

  // Read joystick 2 values
  int rawX2 = analogRead(JOYSTICK2_X_PIN);
  int rawY2 = analogRead(JOYSTICK2_Y_PIN);
  byte rawSW2 = not digitalRead(JOYSTICK2_SWITCH_PIN);

  // read MPU yaw, pitch, roll
  // only use yaw, pitch for X and Y respectively, in the transmission package
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // Serial.print("ypr\t");
    // Serial.print(ypr[0] * 180/M_PI); // YAW
    // Serial.print("\t");
    // Serial.print(ypr[1] * 180/M_PI); // PITCH
    // Serial.print("\t");
    // Serial.println(ypr[2] * 180/M_PI); // ROLL
  }


  if(!switch1State){
    // fill servo data with joystick 2 values (map)
    Serial.println("JOYSTICK 2");
    package.servoData[0] = map(rawX2, 0, 4095, 0, 255);
    package.servoData[1] = map(rawY2, 0, 4095, 0, 255);
    package.servoData[2] = rawSW2;
  }
  else if(switch1State){
    Serial.println("GYRO");
    // yaw, pitch and roll in degrees (-180 to 180)
    float yaw = ypr[0] * 180/M_PI;
    float pitch = ypr[1] * 180/M_PI;
    float roll = ypr[2] * 180/M_PI;

    yaw += 90;
    pitch += 90;
    roll += 90;
    // Map yaw, pitch, roll to 0-255 for sending byte
    
    // fill servo data with mpu yaw, pitch, roll
    // we're only really using yaw (left-right) and pitch (up-down)
    yaw = constrain(yaw, 0 ,180);
    pitch = constrain(pitch, 0 ,180);
    roll = constrain(roll, 0 ,180);
    package.servoData[0] = pitch;   // (X)
    package.servoData[1] = yaw; // (Y)
    package.servoData[2] = rawSW2; // we won't send the roll because it won't control the servo direction
  }

  // Send package
  send();

  // Check for timeout
  checkTimeout();
}

void send() {
    bool rslt;
    rslt = radio.write(&package, sizeof(package));
    delay(10);
    //Serial.print("Data Sent ");

    if (rslt) {
        // Serial.println("RECEIVED");
        previousMillis = millis(); // Reset the timeout timer on successful transmission
        printPackageData();
    }
    else {
        //Serial.println("Tx failed");
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
        // Timeout action 
        // Serial.println("Transmission timeout");
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

    // Serial.println("Transmission reset");
}

void printPackageData(){
  Serial.print("Joystick 1: ");
  Serial.print(package.joystick1Data[0]);
  Serial.print(" , ");
  Serial.println(package.joystick1Data[1]);  
  Serial.print("Servo Data: ");
  Serial.print(package.servoData[0]);
  Serial.print(" , ");
  Serial.println(package.servoData[1]);
}





