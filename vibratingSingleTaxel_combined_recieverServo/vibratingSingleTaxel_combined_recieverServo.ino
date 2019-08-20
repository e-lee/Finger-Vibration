/*
  Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  version 2 as published by the Free Software Foundation.
*/

/**
   Example for Getting Started with nRF24L01+ radios.

   This is an example of how to use the RF24 class.  Write this sketch to two
   different nodes.  Put one of the nodes into 'transmit' mode by connecting
   with the serial monitor and sending a 'T'.  The ping node sends the current
   time to the pong node, which responds by sending the value back.  The ping
   node can then see how long the whole cycle took.
*/

/* add something so if the transmitter
  doesn't hear back from the receiver, the pwm is set to zero (or should it
  be maintained?*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include <Servo.h>

//
// Hardware configuration
//

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10

RF24 radio(5, 6);

//
// Topology
//

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

//
// Role management
//
// Set up role.  This sketch uses the same software for all the nodes
// in this system.  Doing so greatly simplifies testing.
//

// The various roles supported by this sketch
typedef enum { role_ping_out = 1, role_pong_back } role_e;

// The debug-friendly names of those roles
const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};

// The role of the current running sketch
role_e role = role_pong_back;

// declare servo object
Servo armServo;

/* state pin names */
const int servoSignal = 9;
const int vibpin1 = 11; 

void setup(void)
{

  /* configure pins */
  pinMode(vibpin1, OUTPUT);

//  armServo.attach(servoSignal);// attach servo to pin 9
//  armServo.write(90); // set initial servo value to not moving

  //
  // Print preamble
  //

  Serial.begin(9600);
  printf_begin();
  printf("\n\rRF24/examples/GettingStarted/\n\r");
  printf("ROLE: %s\n\r", role_friendly_name[role]);
  printf("*** PRESS 'T' to begin transmitting to the other node\n\r");

  //
  // Setup and configure rf radio
  //

  radio.begin();
  radio.setAutoAck(false);

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15, 15);


  //
  // Start listening
  //

  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1, pipes[0]);

  radio.startListening();

  //
  // Dump the configuration of the rf unit for debugging
  //

  radio.printDetails();
}

void loop(void)
{

  // if there is data ready
  if ( radio.available() )
  {
    // Dump the payloads until we've gotten everything
    float output;
    bool done = false;
    while (!done)
    {
      // Fetch the payload, and see if this was the last one.
      done = radio.read( &output, sizeof(float) );

      // Spew it
//      printf("Got payload %lu...", output);
      Serial.print("Got payload ");
      Serial.println(output);
      /* control servo (or output pwm) */
      // analogWrite(5, (int)output);

      armServo.attach(servoSignal);
      if (output < 0) {
        armServo.write(170);
        Serial.println("arm is moving down");
        delay(30 * abs(output));
      }
      else if (output > 0) {
        armServo.write(0);
        Serial.println("arm is moving up");
        delay(30 * output);
      }
      else {// otherwise do nothing
        Serial.println("don't move arm");
      }

      armServo.write(90); // stop moving the arm
      armServo.detach();


      // Delay just a little bit to let the other unit
      // make the transition to receiver
      delay(20);
    }

    // First, stop listening so we can talk
    radio.stopListening();

    // Send the final one back.
    radio.write( &output, sizeof(unsigned long) );
    printf("Sent response.\n\r");


    // Now, resume listening so we catch the next packets.
    radio.startListening();
  }
}
// vim:cin:ai:sts=2 sw=2 ft=cpp
