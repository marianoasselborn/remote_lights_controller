/*
 * From https://nRF24.github.io/RF24
 * Based on https://nrf24.github.io/RF24/examples_2AcknowledgementPayloads_2AcknowledgementPayloads_8ino-example.html
 * 
 */

/**
 * Light control remote
 * Reads local button activity and queues updates for the controller, sent on every TRANSMISSION_INTERVAL nth loop
 * Polls controller for lights' status, and updates local leds to reflect remote state
 */
#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define CE_PIN 9
#define CSN_PIN 10

// Controls serial monitor printing, set to false if not debugging
#define DEBUG_MODE false

#define TRANSMISSION_INTERVAL 2000
int loop_counter = 0;

// Physical buttons on control: 1,2,3,4,5,6,7,8
int buttons[8] = {A4,A5,A6,A7,A0,A1,A2,A3};
// Physical leds on control: 1,2,3,4,5,6,7
// There is no led number 8 as I do not have a pin for it
// Leds are controlled by the controller, not by this remote, this is why there's a delay in
// local leds reflecting controller state, as they require at least one radio trip to update
int leds[7] = {6,7,8,5,2,3,4};

// This is the source of truth for this control, it gets updated from the main controller
// and it is used to determine which leds to toggle on/off
int local_status[8] = {0,0,0,0,0,0,0,0};

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);
uint8_t address[][6] = { "1Node", "2Node" };
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

struct PayloadStruct {
  char message[7];  // only using 6 characters for TX & ACK payloads
  int action = 0; // 1 for controller update, 0 for no-op
  int status[8];
  uint8_t counter;
};

PayloadStruct payload;

void setup() {

  // Initialize buttons
  for (int i; i < 8; i++) {
    // A6 and A7 can't be set to digital input on Arduino Nano
    // Added pullup resistors and using 0 to represent 'pressed' and anything above to represent 'not pressed'
    if (buttons[i] == A6 || buttons[i] == A7) {
      pinMode(buttons[i], INPUT);
    } else {
      // All other buttons using internal pullup resistors, however, they behaved weirdly anyway
      // so they all have a 10k ohm resistor to 5v
      pinMode(buttons[i], INPUT_PULLUP);
    }
  }

  // Initialize leds
  for (int i; i < 7; i++) {
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], HIGH);
    delay(50);
    digitalWrite(leds[i], LOW);
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
    while (1) {
      // Make leds go crazy to be able to tell that the radio is not working
      for (int i; i < 7; i++) {
        digitalWrite(leds[i], HIGH);
        delay(100);
        digitalWrite(leds[i], LOW);
      }
    }  // hold in infinite loop
  }

  // TX radio number
  radioNumber = 0;
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

  // setup the TX payload
  payload.action = 0; // controller no-op
  for (int i = 0; i < 8; i++) {
    payload.status[i] = local_status[i];
  }
  radio.stopListening();                 // put radio in TX mode
}

void loop() {
  // Go over buttons to check if they are pressed
  for (int i = 0; i < 8; i++) {
    bool pressed = false;
    // Buttons A6 and A7 on the Nano are special
    // Simulating a digital read
    if (buttons[i] == A6 || buttons[i] == A7) {
      pressed = analogRead(buttons[i]) == 0;
    } else {
      pressed = digitalRead(buttons[i]) == LOW;
    }

    if (pressed) {
      if (DEBUG_MODE) {
        Serial.println("Button " + String(i) + " was pressed");
        Serial.println(digitalRead(buttons[i]));
      }
      // Build request on a toggled status
      if (local_status[i] == 0) {
        payload.status[i] = 1;
        digitalWrite(leds[i], HIGH);
        delay(40);
        digitalWrite(leds[i], LOW);
        delay(40);
        digitalWrite(leds[i], HIGH);
        delay(40);
        digitalWrite(leds[i], LOW);
      } else {
        payload.status[i] = 0;
        digitalWrite(leds[i], LOW);
        delay(40);
        digitalWrite(leds[i], HIGH);
        delay(40);
        digitalWrite(leds[i], LOW);
        delay(40);
        digitalWrite(leds[i], HIGH);
      }

      // Handle panic|all-on button
      if (i == 7) {
        for (int j = 0; j < 7; j++) {
          payload.status[j] = payload.status[i];
        }
      }

      if (DEBUG_MODE) {
        Serial.println("Queueing payload: " + String(i) + ":" + String(payload.status[i]));
      }

      payload.action = 1;
      delay(500);
    }
  }

  // The radio does not work on every loop, only on the TRANSMISSION_INTERVAL nth loop
  if (loop_counter >= TRANSMISSION_INTERVAL) {
    // Reset counter
    loop_counter = 0;
    // This device is a TX node
    unsigned long start_timer = micros();                  // start the timer
    bool report = radio.write(&payload, sizeof(payload));  // transmit & save the report
    payload.action = 0; // reset payload action to no-op
    unsigned long end_timer = micros();                    // end the timer

    if (report) {
      if (DEBUG_MODE) {
        Serial.println(F("Polling light controller"));  // payload was delivered
        Serial.print(F("Time to send transmission = "));
        Serial.print(end_timer - start_timer);  // print the timer result
        Serial.println("");

      }
      uint8_t pipe;
      if (radio.available(&pipe)) {  // is there an ACK payload? grab the pipe number that received it
        PayloadStruct received;
        radio.read(&received, sizeof(received));  // get incoming ACK payload
        if (DEBUG_MODE) {
          Serial.println(F("ACK Recieved:"));
        }

        for (int i = 0; i < 8; i++) {
          // Use ACK payload to update local status from remote status
          local_status[i] = received.status[i];
        }

        if (DEBUG_MODE) {
          Serial.print("Relay status ");
          for (int i = 0; i < 8; i++) {
            Serial.print(String(i) + ":" + String(received.status[i]) + " ");
          }
          Serial.println("");
          Serial.print("Local status ");
          for (int i = 0; i < 8; i++) {
            Serial.print(String(i) + ":" + String(local_status[i]) + " ");
          }
          Serial.println("");
        }

        // save incoming counter & increment for next outgoing
        payload.counter = received.counter + 1;

      } else {
        if (DEBUG_MODE) {
          Serial.println(F(" Recieved: an empty ACK packet"));  // empty ACK packet received
        }
        // Signal an empty ACK packet with the first led
        // 1 long light + 3 short lights
        digitalWrite(leds[0], HIGH);
        delay(1000);
        digitalWrite(leds[0], LOW);
        delay(50);
        digitalWrite(leds[0], HIGH);
        delay(50);
        digitalWrite(leds[0], LOW);
        delay(50);
        digitalWrite(leds[0], HIGH);
        delay(50);
        digitalWrite(leds[0], LOW);
        delay(50);
        digitalWrite(leds[0], HIGH);
        delay(50);
        digitalWrite(leds[0], LOW);
        delay(50);
      }


    } else {
      if (DEBUG_MODE) {
        Serial.println(F("Transmission failed or timed out"));
      }
      // Signal a failed transmission with led 2
      // 1 long light + 3 short lights
      digitalWrite(leds[1], HIGH);
      delay(1000);
      digitalWrite(leds[1], LOW);
      delay(50);
      digitalWrite(leds[1], HIGH);
      delay(50);
      digitalWrite(leds[1], LOW);
      delay(50);
      digitalWrite(leds[1], HIGH);
      delay(50);
      digitalWrite(leds[1], LOW);
      delay(50);
      digitalWrite(leds[1], HIGH);
      delay(50);
      digitalWrite(leds[1], LOW);
      delay(50);
      radio.startListening();
      radio.stopListening();
    }

    if (DEBUG_MODE) {
      Serial.println(F("---"));
    }
  }
  loop_counter = loop_counter + 1;
  // to make this example readable in the serial monitor

  for (int i = 0; i < 7; i++) {
    if (local_status[i] == 0) {
      digitalWrite(leds[i], LOW);
    }
    if (local_status[i] == 1) {
      digitalWrite(leds[i], HIGH);
    }
  }
}  // loop