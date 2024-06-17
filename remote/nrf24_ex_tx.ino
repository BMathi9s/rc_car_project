#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(4,5); // (CE, CSN)

long weight;

void setup() {
  Serial.begin(9600); // begin
  radio.begin();      // start radio
  radio.setChannel(5);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openWritingPipe(0x1234567890LL); // set address at which nrf24 will communicate
  radio.stopListening(); // set transmitter mode
}

void loop() {
  weight = 13381; // establish weight
  radio.write(&weight, sizeof(weight)); // send address of 'weight' and size of 'weight'
  delay(1000);
  Serial.println("running...");
}
