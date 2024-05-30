// remote control receiver code

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// rf24 object to receive data
RF24 radio(4, 5); // (CE, CSN)

// define structure to receive data from transmitter
struct{
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

// joystick data
byte joystick1Data[2]; // Array holds X and Y values for joystick1
byte joystick2Data[2]; // X and Y values for joystick2

// pot data
byte pot1Data;
byte pot2Data;



void rf24_begin(){
  radio.begin();      // start radio
  radio.setChannel(5);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openReadingPipe(1, 0x1234567890LL); // set address at which nrf24 will communicate
  radio.startListening(); // set receiver mode
}

void setup() {
  // set up rf24 receiver mode
  Serial.begin(115200);
  rf24_begin();
  
  }


void loop() {
  if (radio.available()) {
    // receive package from transmitter
    radio.read(&inputs, sizeof(inputs));

    // get button states from input package
    button1State = inputs.button1;
    button2State = inputs.button2;
    button3State = inputs.button3;
    button4State = inputs.button4;
    button5State = inputs.button5;
    button6State = inputs.button6;
    button7State = inputs.button7;
    button8State = inputs.button8;
    button9State = inputs.button9;
    button10State = inputs.button10;

    // get switch states from input package
    switch1State1 = inputs.switch1Pin1;
    switch2State1 = inputs.switch2Pin1;

    // get pot states from input package
    pot1Data = inputs.pot1;
    pot2Data = inputs.pot1;

    // get joystick data from input package
    joystick1Data[0] = inputs.joystick1X;
    joystick1Data[1] = inputs.joystick1Y;
    joystick2Data[0] = inputs.joystick2X;
    joystick2Data[1] = inputs.joystick2Y;

    // get gyro data from input package
    yaw_fpv = inputs.yaw_fpv;
    pitch_fpv = inputs.pitch_fpv;

      // checking inputs by printing them
      printInputs();

    // here call any fuctions to map/process recevied data
  }


}

void printInputs(){
    Serial.println("");
    Serial.println("");
    Serial.print("Button 1: ");
    Serial.print(button1State);    
    Serial.print(" | Button 2: ");
    Serial.print(button2State);
    Serial.print(" | Button 3: ");
    Serial.print(button3State);
    Serial.print(" | Button 4: ");
    Serial.print(button4State);
    Serial.print(" | Button 5: ");
    Serial.print(button5State);
    Serial.print(" | Button 6: ");
    Serial.print(button6State);
    Serial.print(" | Button 7: ");
    Serial.print(button7State);
    Serial.print(" | Button 8: ");
    Serial.print(button8State);
    Serial.print(" | Button 9: ");
    Serial.print(button9State);
    Serial.print(" | Button 10: ");
    Serial.println(button10State);
    Serial.print("Switch 1: ");
    Serial.print(switch1State1);  
    Serial.print(" | Switch 2: ");
    Serial.println(switch2State1);  
    Serial.print("Pot 1: ");
    Serial.print(pot1Data);  
    Serial.print(" | Pot 2: ");
    Serial.println(pot2Data);  
    Serial.print("Joystick 1 X: ");
    Serial.print(joystick1Data[0]);
    Serial.print(" | Joystick 1 Y: ");
    Serial.println(joystick1Data[1]);
    Serial.print("Joystick 2 X: ");
    Serial.print(joystick2Data[0]);
    Serial.print(" | Joystick 2 Y: ");
    Serial.println(joystick2Data[1]);
    Serial.print("Gyro yaw: ");
    Serial.print(yaw_fpv);  
    Serial.print(" | Gyro pitch: ");
    Serial.println(pitch_fpv);  
}
