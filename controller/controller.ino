/*
 * From https://nRF24.github.io/RF24
 * Based on https://nrf24.github.io/RF24/examples_2AcknowledgementPayloads_2AcknowledgementPayloads_8ino-example.html
 * 
 */

/**
 * Light control controller
 * Receives radio updates from remotes
 * Responds to remotes polling for relay status'
 */
#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN 9
#define CSN_PIN 10

// Controls serial monitor printing, set to false if not debugging
#define DEBUG_MODE false

int local_status[8] = {0,0,0,0,0,0,0,0};
// Physical relays on relay board 1,2,3,4,5,6,7,8
int relays[8] = {2,3,4,5,14,15,16,17};

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);
uint8_t address[][6] = { "1Node", "2Node" };
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// For this example, we'll be using a payload containing
// a string & an integer number that will be incremented
// on every successful transmission.
// Make a data structure to store the entire payload of different datatypes
struct PayloadStruct {
  char message[7];  // only using 6 characters for TX & ACK payloads
  int action;
  int status[8];
  uint8_t counter;
};
PayloadStruct payload;

void setup() {

  // Initialize relays
  for (int i; i < 8; i++) {
    pinMode(relays[i], OUTPUT);
  }

  if (DEBUG_MODE) {
    Serial.begin(115200);
    while (!Serial) {
      // some boards need to wait to ensure access to serial over USB
    }
  }

  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }

  // RX radio number
  radioNumber = 1;
  if (DEBUG_MODE) {
    Serial.print(F("radioNumber = "));
    Serial.println((int)radioNumber);
  }

  // This level seems to be enough for the distance we need
  radio.setPALevel(RF24_PA_MIN);

  // Make sure the receiver talks back to transmitter
  radio.enableDynamicPayloads();
  radio.enableAckPayload();

  // TX mode uses radio 0 and RX mode uses radio 1
  radio.openWritingPipe(address[radioNumber]);
  radio.openReadingPipe(1, address[!radioNumber]);

  // setup the ACK payload & load the first response into the FIFO
  memcpy(payload.message, "World ", 6);  // set the payload message
  for (int i = 0; i < 8; i++) {
    payload.status[i] = local_status[i];
  }
  // load the payload for the first received transmission on pipe 0
  radio.writeAckPayload(1, &payload, sizeof(payload));
  radio.startListening();  // put radio in RX mode
}

void loop() {
  // This device is a RX node
  uint8_t pipe;
  if (radio.available(&pipe)) {                     // is there a payload? get the pipe number that recieved it
    uint8_t bytes = radio.getDynamicPayloadSize();  // get the size of the payload
    PayloadStruct received;
    radio.read(&received, sizeof(received));  // get incoming payload
    if (DEBUG_MODE) {
      Serial.print(F("Received "));
      Serial.print(bytes);  // print the size of the payload
      Serial.print(F(" bytes on pipe "));
      Serial.print(pipe);  // print the pipe number
      Serial.print(" [Action ");
      Serial.print(received.action);  // print incoming action
      Serial.print("]");
      Serial.println("");
    }
    // This is a request to update controller status
    if (received.action == 1) {
      for (int i = 0; i < 8; i++) {
        local_status[i] = received.status[i];
      }
    }

    if (DEBUG_MODE) {
      Serial.print("Local status ");
      for (int i = 0; i < 8; i++) {
        Serial.print(String(i) + ":" + String(local_status[i]) + " ");
      }
      Serial.println("");
      Serial.print("Relay status ");
      for (int i = 0; i < 8; i++) {
        int state = digitalRead(relays[i]);
        Serial.print(String(i) + ":" + String(state) + " ");
      }
      Serial.println("");
    }

    // save incoming counter & increment for next outgoing
    payload.counter = received.counter + 1;
    for (int i = 0; i < 8; i++) {
      payload.status[i] = local_status[i];
    }
    // load the payload for the first received transmission on pipe 0
    radio.writeAckPayload(1, &payload, sizeof(payload));
    if (DEBUG_MODE) {
      Serial.println("---");
    }
  }

  // Adjust relays state
  for (int i = 0; i < 8; i++) {
    if (local_status[i] == 1) {
      digitalWrite(relays[i], HIGH);
    }

    if (local_status[i] == 0) {
      digitalWrite(relays[i], LOW);
    }
  }
}  // loop