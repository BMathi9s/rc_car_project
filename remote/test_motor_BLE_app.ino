// --------------------------------------------------
//
// Code for control of ESP32 through MIT inventor app (Bluetooth). 
// device used for tests: ESP32-WROOM-32D
// 
// App on phone has three buttons:
// Button 1: 11 for ON and 10 for OFF
// Button 2: 21 for ON and 20 for OFF
// Button 3: 31 for ON and 30 for OFF
//
// Written by mo thunderz (last update: 20.4.2021)
//
// --------------------------------------------------

// this header is needed for Bluetooth Serial -> works ONLY on ESP32
#include "BluetoothSerial.h" 


#define MOTOR_DIR_L 13
#define MOTOR_PWN_L 12
#define MOTOR_DIR_R 14
#define MOTOR_PWN_R 27
const int freq = 10000;       // 10 kHz frequency
const int pwmChannelL = 0;    // PWM channel for left motor
const int pwmChannelR = 1;    // PWM channel for right motor
const int resolution = 8;     // 8-bit resolution (0-255)

// init Class:
BluetoothSerial ESP_BT; 

// init PINs: assign any pin on ESP32
int led_pin_1 = 4;
int led_pin_2 = 0;
int led_pin_3 = 2;     // On some ESP32 pin 2 is an internal LED, mine did not have one

// Parameters for Bluetooth interface
int incoming;

void setup() {
  Serial.begin(19200);
  ESP_BT.begin("ESP32_Control"); //Name of your Bluetooth interface -> will show up on your phone


    pinMode(MOTOR_DIR_L, OUTPUT);
    pinMode(MOTOR_DIR_R, OUTPUT);

    // Configure PWM channels
    ledcSetup(pwmChannelL, freq, resolution);
    ledcSetup(pwmChannelR, freq, resolution);

    // Attach the PWM channels to the motor PWM pins
    ledcAttachPin(MOTOR_PWN_L, pwmChannelL);
    ledcAttachPin(MOTOR_PWN_R, pwmChannelR);
}

void loop() {
  
  // -------------------- Receive Bluetooth signal ----------------------
  if (ESP_BT.available()) 
  {
    incoming = ESP_BT.read(); //Read what we receive 

    // separate button ID from button value -> button ID is 10, 20, 30, etc, value is 1 or 0
    int button = floor(incoming / 10);
    int value = incoming % 10;
    
    switch (button) {
      case 1:  
        Serial.print("Button 1:"); Serial.println(value);
        forward();

        break;
      case 2:  
        Serial.print("Button 2:"); Serial.println(value);
       stop();
        break;
      case 3:  
        Serial.print("Button 3:"); Serial.println(value);
        back();
        break;
    }
  }
}


void forward(){
   digitalWrite(MOTOR_DIR_L, HIGH);
  digitalWrite(MOTOR_DIR_R, HIGH);

    // Increase speed from 0 to max in 1 second
    
        ledcWrite(pwmChannelL, 255);
        ledcWrite(pwmChannelR, 255);
        delay(10); // 1000 ms / 255 steps ≈ 4 ms per step
    
}

void stop(){
   ledcWrite(pwmChannelL, 0);
        ledcWrite(pwmChannelR, 0);
}

void back(){
  digitalWrite(MOTOR_DIR_L, LOW);
    digitalWrite(MOTOR_DIR_R, LOW);

        ledcWrite(pwmChannelL, 255);
        ledcWrite(pwmChannelR, 255);
        delay(10); // 1000 ms / 255 steps ≈ 4 ms per step

}
