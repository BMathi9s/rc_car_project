//remote control reading


//inputs
//9 buttons
//2 potentiometer
//1 slider (potentiometer)
//2 3way switch
//2 joystick
//mpu6050 thought i2c (D22,D21)

//output 
//nrf2401 spi ((mosi)D23,(miso)D19,(SDK)D18,(CSN)D5,(CE) D4 )
#include "I2Cdev.h"
#include "MPU6050.h"  //mpu6050 from electric
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu6050;




// NRF24L01 pins
#define NRF_MOSI_PIN 23
#define NRF_MISO_PIN 19
#define NRF_SCK_PIN 18
#define NRF_CSN_PIN 5
#define NRF_CE_PIN 4

// I2C pins for MPU6050
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// Potentiometer pins
#define POT1_PIN 34  // Analog pin for potentiometer 1
#define POT2_PIN 35  // Analog pin for potentiometer 2

// Joystick pins
#define JOYSTICK1_X_PIN 32  // Analog pin for joystick 1 X-axis
#define JOYSTICK1_Y_PIN 33  // Analog pin for joystick 1 Y-axis
#define JOYSTICK2_X_PIN 36  // Analog pin for joystick 2 X-axis
#define JOYSTICK2_Y_PIN 39  // Analog pin for joystick 2 Y-axis

// Switch pins
#define SWITCH1_PIN1 14
#define SWITCH2_PIN1 27

// Unused pins allocated for buttons
#define BUTTON1_PIN 12
#define BUTTON2_PIN 13
#define BUTTON3_PIN 15
#define BUTTON4_PIN 16
#define BUTTON5_PIN 17
#define BUTTON6_PIN 25
#define BUTTON7_PIN 26
#define BUTTON8_PIN 32
#define BUTTON9_PIN 33
#define BUTTON10_PIN 35


//Adafruit_MPU6050 mpu;
 struct Inputs  {
        bool button1;
        bool button2;
        bool button3;
        bool button4;
        bool button5;
        bool button6;
        bool button7;
        bool button8;
        bool button9;
        bool button10;
        bool switch1Pin1;
        bool switch2Pin1;
        //instead of using bool, we can encode all in a byte
        byte pot1;
        byte pot2;
        byte joystick1X;
        byte joystick1Y;
        byte joystick2X;
        byte joystick2Y;
        byte yaw_fpv;
        byte pitch_fpv; 
    } inputs;

// Volatile variables for button states
//volatile -> sudden change expected
volatile bool button1State = false;
volatile bool button2State = false;
volatile bool button3State = false;
volatile bool button4State = false;
volatile bool button5State = false;
volatile bool button6State = false;
volatile bool button7State = false;
volatile bool button8State = false;
volatile bool button9State = false;
volatile bool button10State = false;
int16_t yaw_fpv;
int16_t pitch_fpv;

// Volatile variables for switch states
volatile bool switch1State1 = false;
volatile bool switch2State1 = false;


// Interrupt service routines (ISRs) for buttons
void IRAM_ATTR handleButton1Interrupt() { button1State = !button1State; }
void IRAM_ATTR handleButton2Interrupt() { button2State = !button2State; }
void IRAM_ATTR handleButton3Interrupt() { button3State = !button3State; }
void IRAM_ATTR handleButton4Interrupt() { button4State = !button4State; }
void IRAM_ATTR handleButton5Interrupt() { button5State = !button5State; }
void IRAM_ATTR handleButton6Interrupt() { button6State = !button6State; }
void IRAM_ATTR handleButton7Interrupt() { button7State = !button7State; }
void IRAM_ATTR handleButton8Interrupt() { button8State = !button8State; }
void IRAM_ATTR handleButton9Interrupt() { button9State = !button9State; }
void IRAM_ATTR handleButton10Interrupt() { button10State = !button10State; }

// Interrupt service routines (ISRs) for switches
void IRAM_ATTR handleSwitch1Pin1Interrupt() { switch1State1 = digitalRead(SWITCH1_PIN1); }
void IRAM_ATTR handleSwitch2Pin1Interrupt() { switch2State1 = digitalRead(SWITCH2_PIN1); }


void mpu6050_begin(){
   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


    Serial.println("Initializing I2C devices...");
    mpu6050.initialize();
     Serial.println("Testing device connections...");
    Serial.println(mpu6050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

}


void setup() {
    Serial.begin(115200);
    // Initialize MPU6050 and I2C
    

    // Initialize button pins with pull-up resistors and attach interrupts
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON1_PIN), handleButton1Interrupt, RISING);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON2_PIN), handleButton2Interrupt, RISING);
    pinMode(BUTTON3_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON3_PIN), handleButton3Interrupt, RISING);
    pinMode(BUTTON4_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON4_PIN), handleButton4Interrupt, RISING);
    pinMode(BUTTON5_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON5_PIN), handleButton5Interrupt, RISING);
    pinMode(BUTTON6_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON6_PIN), handleButton6Interrupt, RISING);
    pinMode(BUTTON7_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON7_PIN), handleButton7Interrupt, RISING);
    pinMode(BUTTON8_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON8_PIN), handleButton8Interrupt, RISING);
    pinMode(BUTTON9_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON9_PIN), handleButton9Interrupt, RISING);
    pinMode(BUTTON10_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON10_PIN), handleButton10Interrupt, RISING);
    pinMode(SWITCH1_PIN1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SWITCH1_PIN1), handleSwitch1Pin1Interrupt, CHANGE);
    pinMode(SWITCH2_PIN1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SWITCH2_PIN1), handleSwitch2Pin1Interrupt, CHANGE);

    mpu6050_begin();
   


}



void loop() {

  
   

   
    //   Byte, "char"	255	2 2 hex characted   -> but char is -128 to 127 while byte is 0->255
    // "short" 	65535	4 hex characted
    // "int" (Windows DWORD)	>4 billion	8 hex characted
    // Read potentiometers
    byte pot1Value = map(analogRead(POT1_PIN), 0, 4095, 0, 255);
    byte pot2Value = map(analogRead(POT2_PIN), 0, 4095, 0, 255);
    // Read joysticks
    byte joystick1X = map(analogRead(JOYSTICK1_X_PIN), 0, 4095, 0, 255); //
    byte joystick1Y = map(analogRead(JOYSTICK1_Y_PIN), 0, 4095, 0, 255);
    byte joystick2X = map(analogRead(JOYSTICK2_X_PIN), 0, 4095, 0, 255);
    byte joystick2Y = map(analogRead(JOYSTICK2_Y_PIN), 0, 4095, 0, 255);
    
    
    yaw_fpv =  mpu6050.getRotationZ();
    pitch_fpv = mpu6050.getRotationY();

    Serial.println(yaw_fpv);
    delay(300);
    



    inputs.button1 = button1State;
    inputs.button2 = button2State;
    inputs.button3 = button3State;
    inputs.button4 = button4State;
    inputs.button5 = button5State;
    inputs.button6 = button6State;
    inputs.button7 = button7State;
    inputs.button8 = button8State;
    inputs.button9 = button9State;
    inputs.button10 = button10State;
    inputs.switch1Pin1 = switch1State1;
    inputs.switch2Pin1 = switch2State1;
    inputs.pot1 = pot1Value;
    inputs.pot2 = pot2Value;
    inputs.joystick1X = joystick1X;
    inputs.joystick1Y = joystick1Y;
    inputs.joystick2X = joystick2X;
    inputs.joystick2Y = joystick2Y; 
    inputs.yaw_fpv = yaw_fpv;
    inputs.pitch_fpv = pitch_fpv;

    //send packet

    
}

