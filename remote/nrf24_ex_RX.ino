#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(4,5); // (CE, CSN)

float weight;

void setup() {
  delay(10);
  Serial.begin(9600); // begin
  radio.begin();      // start radio
  radio.setChannel(5);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openReadingPipe(1, 0x1234567890LL); // set address at which nrf24 will communicate
  // radio.stopListening(); // set transmitter mode
  radio.startListening ();
}

void loop() {
  if (radio.available()){
    radio.read(&weight, sizeof(weight));
    Serial.println(weight);
  }

}