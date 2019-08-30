/* Author: Emily Lee
   Date: August 2019
   Purpose: Reads incoming data from transmitting microcontroller and outputs this data (either pwm or servo position) onto the vibration motor
   or a positional servo. Optionally, this pwm can be transmitted to another microcontroller using nrF24l01+ modules. These features, as well as touch sensitivity, can be controlled with user-defined variables.
   Email: emilylee.email@gmail.com
*/
/* add something so if the transmitter
  doesn't hear back from the receiver, the pwm is set to zero (or should it
  be maintained?*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include <Servo.h>

#define OUTPUT_VIB 8888
#define OUTPUT_SERVO 9999

#define POS_HIGH 24

bool isOutputVib = true; // output is servo by default
int output;
//
// Hardware configuration
//

// Set up nRF24L01 radio on SPI bus plus pins 5 & 6

RF24 radio(6, 7); //CE is 5, CSN is 6

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
//typedef enum { role_ping_out = 1, role_pong_back } role_e;

// The debug-friendly names of those roles
const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};

// The role of the current running sketch
//role_e role = role_pong_back;

// declare servo object
Servo armServo;

/* state pin names */
const int servoSignal = 9;
const int vibpin1 = 3;

void setup(void)
{

  /* configure pins */
  pinMode(vibpin1, OUTPUT);

  //
  // Print preamble
  //

  Serial.begin(9600);
  printf_begin();
  printf("\n\rRF24/examples/GettingStarted/\n\r");
  printf("*** PRESS 'T' to begin transmitting to the other node\n\r");

  //
  // Setup and configure rf radio
  //

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
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

  // attach servo, initialize position
  armServo.attach(servoSignal);
  armServo.write(0);
}

void loop(void)
{
  receiveOutput();
}

/* Function: receiveOutput
   "Inputs": Data coming from transmitting microcontroller
   "Outputs": servo position or vibration.
   Purpose: Receives data to be outputted from transmitting microcontroller and outputs either
   servo position or vibration depending on specification spent from transmitting microcontroller.
   Constantly checks to see if output should be changed from vibration to servo or vice versa.
*/

void receiveOutput() {
  // if there is data ready
  if ( radio.available() )
  {
    // Dump the payloads until we've gotten everything
    bool done = false;
    while (!done)
    {
      // Fetch the payload, and see if this was the last one.
      
      // type of variable read must match type of variable sent (i.e. if "output" is type int on the transmitter, it should also be int on the receiver)
      done = radio.read( &output, sizeof(int) ); 
      // Spew it
      Serial.print("Got payload ");
      Serial.println(output);
      // Delay just a little bit to let the other unit
      // make the transition to receiver

      if (!checkOutput()) {
        sendOutput();
      }
    }

    // First, stop listening so we can talk
    radio.stopListening();

    // Send the final one back.
    radio.write( &output, sizeof(int) );
    printf("Sent response.\n\r");


    // Now, resume listening so we catch the next packets.
    radio.startListening();
  }
}

/* Function: sendOutput
    "Inputs": output (either pwm or arm position)
    Purpose: Outputs pwm onto vibration motor or arm position onto servo depending on output type
*/

void sendOutput() {
  if (isOutputVib == true) {
    analogWrite(vibpin1, output);
  }
  else {
    armServo.write(output);
    delay(25); // a delay is needed here, otherwise the servo will not respond to commands
    Serial.print("arm position is: ");
    Serial.println(output);

     if (output > POS_HIGH) {
        analogWrite(vibpin1, 255);
      }
      else 
        analogWrite(vibpin1, 0);
  }
}

/* Function: checkOutput
   "Inputs": output
   Output: returns 1 if output type has been changed, 0 otherwise
   Purpose: determines if output contains OUTPUT_VIB (signals output should be set to vibration) or OUTPUT_SERVO (output set to servo)
*/

int checkOutput() {

  if (output == OUTPUT_VIB || output == OUTPUT_SERVO) {
    if (output == OUTPUT_VIB) {
      isOutputVib = true;
      Serial.println("Output is vibration");
      return 1;
    }
    else {
      isOutputVib = false;
      Serial.println("Output is servo");
      return 1;
    }
  }
  else
    return 0;
}
// vim:cin:ai:sts=2 sw=2 ft=cpp
