/*******************************
   The following MUST be true for this code to work:

   1. in "TimerOne.h", TCNT1 = 1 (not TCNT1 = 0) in the TimerOne library files
      otherwise, interrupt will trigger immediately after it is enabled/started

   2. if using Piksey Pico board (uses mcu called atmega328pb), add "|| defined (__AVR_ATmega328PB__)" to line 94 in "known_16bit_timers.h" in the TimerOne library files
      otherwise, the TimerOne library will not be able to recognize the board

   3. this code uses the ManiacBug version of the RF24 library -- don't use the Tmrh20 version without changing the radio code!
  `
  Other Important Notes:


   ** if using the Piksey Pico board and there is some error in the debugging window when you try to upload saying "...Permission Denied",
   try switching "Arduino AVR Boards" (under Tools -> Board -> Boards Manager) to version 1.6.22.

   ** the TimerOne and Servo Libraries both use the built-in harware timer called timer1. This means that in this version of the code, trying to activate
   vibrational output (the fadeaway function uses timer1) and the servo at the same time may yield unexpected results

   ** The inclusion of the <Servo.h> library (used to control the servo) or the usage of the <TimerOne.h> library (both are used in this code)
   means that the pins 9, 10 are unavailable for use.

   ** Sometimes the rectangular shaped micro USB cord doesn't let you upload code to the piksey microcontroller. I'm not sure why this is,
   but using a different cord can fix the problem.

   ** All the radio code is based off of an rf24 example called "GettingStarted". if you have the rf24 library installed, you can access this file  
   * under File -> Examples -> RF24 -> GettingStarted.
   

 *******************************/

/* Author: Emily Lee
  Date: August 2019
  Purpose: Reads capacitance from AD7746 CDC and calculates a corresponding pwm (to be outputted to a vibration motor) or servo position. Optionally, this pwm can be
  transmitted to another microcontroller using nrF24l01+ modules. These features, as well as touch sensitivity, can be controlled with user-defined variables.
  Email: emilylee.email@gmail.com
*/

#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "printf.h"
#include <TimerOne.h>
#include <Servo.h>

/********** USER DEFINED CONSTANTS ************/
const bool isTransmitter = false; // set as true if want to act as transmitter, false if want to output vibration on vibpin (pin 11)
const bool isOutputLinear = true; // set as true if want output to change linearly with changes in capacitance, false if want output to remain within 4 discrete values
const bool isOutputVib = false; // set as true if want output to be pwm into vibration motor, false if want output to be servo angle

/* linear pwm constants */
#define PWM_HIGH 255 // the highest pwm outputted
#define PWM_LOW 90 // the lowest pwm outputted (if a pwm less than PWM_LOW is calculated, pwm outputted will be 0)
#define CAP_RANGE 0.8 // the range of capacitance (pF) between pwm = PWM_LOW and pwm = PWM_HIGH

/* linear servo constants*/
#define POS_HIGH 24
#define POS_LOW 0

/* segmented cap constants */
#define MED_TOUCH_CAP 0.3 // the minimum amount of capacitance above baseline + cap_touch for a touch to be considered a "medium touch"
#define HARD_TOUCH_CAP 0.6 // the minimum amount of capacitance above baseline + cap_touch for a touch to be considered a "hard touch"

#define CAP_STATIONARY 0.2 // range of values cap can oscillate between to be considered "constant"
#define CONST_TIME 4000000 // the minimum time (in microseconds) a constant touch has to be held to trigger pwm fade 


/**********************************************/
//changing arm position values might accidentally change the vibration output too
//just make sure you test that again

/* constants for kinds of touches */
typedef enum {no_touch = 0, light_touch = 1, med_touch = 2, hard_touch = 3} armPos;

/* define timer flags */
typedef enum {not_started, started, finished} timer_flags;

/* constants for CDC */
#define I2C_ADDRESS  0x48           // 0x90 shift one to the right
#define REGISTER_STATUS 0x00
#define REGISTER_CAP_DATA 0x01
#define REGISTER_CAP_SETUP 0x07
#define REGISTER_EXC_SETUP 0x09
#define REGISTER_CONFIGURATION 0x0A
#define RESET_ADDRESS 0xBF

/* constants defining output change*/
#define OUTPUT_VIB 8888
#define OUTPUT_SERVO 9999

/* constant for calculating baseline */
#define NUM_READINGS 100 // the number of readings averaged when calculating the baseline (increasing this number also increases calibration time)

/* pin names */
const int vibpin1 = 11; // the pin at which the calculated pwm is outputted
const int cdcPower = 3;
const int servoSignal = 3;

/* global variables */
float baseline = 0;
float cap_touch = 0;
float capacitance = 0;
float old_capacitance = 0;
float first_touch = 0;
int servoArmPos = 0;
int pwm = 0;
boolean faded = false;

// declare enums (flags)
volatile timer_flags timerStatus = not_started;
armPos armPosition = no_touch;

// declare global variables that may be changed through interrupts
volatile boolean needRecalibrate = false;

// declare global variables for NEW rf transmission
RF24 radio(5, 6); // 5 is CE, 6 is CSN
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

Servo armServo; // declare servo object



/* ______FUNCTION TO INITIALIZE ARDUINO______ */
void setup()
{
  // ** Set pin modes
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(vibpin1, OUTPUT);
  pinMode(cdcPower, OUTPUT);

  digitalWrite(cdcPower, HIGH); // power the servo

  digitalWrite(LED_BUILTIN, HIGH); //signal the start of calibration

  Wire.begin();             // Set up I2C for operation

  Serial.begin(9600);           // Set up baud rate for serial communication to com port
  printf_begin();
  Wire.beginTransmission(I2C_ADDRESS);  // Start I2C cycle
  Wire.write(RESET_ADDRESS);        // Reset the device
  Wire.endTransmission();         // End I2C cycle
  delay(1);               // Wait a tad for reboot

  // **Cap Channel Excitation Setup**
  //    _BV(3) = enables EXCA pin as excitation output
  //    _BV(1), BV(0) = set excitation voltage level
  writeRegister(REGISTER_EXC_SETUP, _BV(3) | _BV(1) | _BV(0));

  // **Cap Channel Setup**
  //    _BV(7) = enables channel for single conversion | continuous conversion | calibration
  writeRegister(REGISTER_CAP_SETUP, _BV(7)); //Cap setup reg - cap enabled

  delay(10);

  // **Converter Update Rate and Mode of Operation Setup**
  //    _BV(0) = sets operation to continuous convertion (set at highest speed)
  writeRegister(REGISTER_CONFIGURATION, _BV(0));

  findStartVals(); //  Find the baseline (avg), and cap_touch (min/max) based off the first NUM_READINGS readings

  //** Enable interrupt on pin 2
  //  attachInterrupt(digitalPinToInterrupt(2), Button_ISR, FALLING); // pin will go high to low when interrupt

  // ** Enable timer interrupt (ONLY if using vibration output)
  if ( isOutputVib == true) {
    Timer1.initialize(CONST_TIME); // sets a timer of length CONST_TIME microseconds
    Timer1.attachInterrupt(timerIsr); // attaches the timer service routine
    Timer1.stop();
  }

  //** Set up RF transmission
  if (isTransmitter == true) {
    radio.begin();
    radio.setPALevel(RF24_PA_LOW);
    radio.setAutoAck(false);
    radio.setRetries(15, 15);
    radio.openWritingPipe(pipes[0]); // open pipes for transmission
    radio.openReadingPipe(1, pipes[1]);
    radio.startListening();

    // ** constantly tell receiving microcontroller if output is vibration or servo until acknowledged
    if (isOutputVib == true)
      while (!sendOutput(OUTPUT_VIB)); // send unique number to signal output is vibration
    else
      while (!sendOutput(OUTPUT_SERVO)); // otherwise send other unique number is servo
  }

  capacitance = baseline;   // initialize capacitance (so old_capacitance has a proper value in the first run of loop())
  timerStatus = not_started; // make sure timer is not started

  if (isOutputVib == false) { // initialize servo ONLY if using servo
    armServo.attach(servoSignal);// attach servo to pin 9
    armServo.write(0); // initialize servo position
    delay(30);
  }

  digitalWrite(LED_BUILTIN, LOW);   //signal the end of calibration
}


/* ______MAIN PROGRAM______ */
void loop()
{
  old_capacitance = capacitance; // save the old capacitance
  readCapacitance(); // new capacitance is stored in global variable capacitance
    findpwm(); // find new pwm according to new capacitance
  if (isOutputVib == true) {
    fadeawayTimer(); // check if need to fade (if using vibration output)
    sendOutput(pwm); // check if supposed to act as transmitter, then send pwm accordingly
  }
  else {
    setArmPosition(); // otherwise, set the servo armPosition according to the new capacitance
    sendOutput(servoArmPos); // check if supposed to act as transmitter, then send servo position accordingly
  }
}


/* Function: sendOutput
   Input: outputSignal - holds either pwm or servoArmPos depending on isTransmitter
   Output: returns 1 if send and receive successful, returns 0 otherwise
   Purpose: checks the state of isTransmitter and either writes the calculated pwm to vibpin1 or transmits it to another microcontroller using RF transmission
*/

int sendOutput(int outputSignal)
{
  if (isTransmitter == false) {
    if (isOutputVib == true) {
      analogWrite(vibpin1, outputSignal); //  if we don't need to transmit pwm, write pwm to vibpin
      Serial.println(pwm);
    }
    else {
      /* write servo position output*/
      armServo.write(outputSignal);
      delay(25); // a delay is needed here, otherwise the servo will not respond to commands
      Serial.print("arm position is: ");
      Serial.println(outputSignal);

      /* vibrate if servo at max rotation */
      if (outputSignal == POS_HIGH) {
        analogWrite(vibpin1, 255);
      }
      else 
        analogWrite(vibpin1, 0);
        
    }
    return 1; // assume non transmitter codes submit successfully
  }
  else {
    bool ok;
    // transmit pwm to other microcontroller
    radio.stopListening(); // Stop listening for a response so we can send a message
    // type of variable sent must match type of variable read (i.e. if "output" is type int on the transmitter, it should also be int on the receiver)

    ok = radio.write( &outputSignal, sizeof(int) );
    if (ok) {
      printf("ok...Sent ");
      Serial.print(outputSignal);
    }
    else {
      printf(" failed.\n\r");
      return 0; // don't listen back for message if send fails
    }
    radio.startListening();  // switch to listening role

    // Wait here until we get a response, or timeout (250ms)
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while ( ! radio.available() && ! timeout )
      if (millis() - started_waiting_at > 200 )
        timeout = true;

    // Describe the results
    if ( timeout )
    {
      printf(" Failed, response timed out.\n\r");
      return 0; // if timeout happens before hear response, assume transmission failed
    }
    else
    {
      // Grab the response, compare, and send to debugging spew
      int response;
      radio.read( &response, sizeof(int) );

      // Spew it
      Serial.print(" Got response ");
      Serial.println(response);

      if (response != outputSignal)
        return 0;
      else
        return 1;
    }

    // wait a tad before trying again
    delay(50);
  }
}

/* Function: setArmPosition
   "Inputs": capacitance
   "outputs": new arm position
   Purpose: checks isOutputLinear and either linearly maps capacitance to an arm position in the range [POS_LOW, POS_HIGH] or maps capacitance to one of three discrete values
*/

void setArmPosition()
{
  float current_touch = checkTouch(capacitance);

  if (isOutputLinear == true) {
    /* linear servo pos mapping */
    //  map capacitance to pwm (make the lowest possible vibration pwm 80):
    servoArmPos = myMap(capacitance, baseline + cap_touch , baseline + cap_touch + CAP_RANGE, POS_LOW, POS_HIGH);

    if (capacitance < baseline + cap_touch) { //if capacitance is smaller than what we consider a touch...
      servoArmPos = 0; //make the pwm zero percent
    }
    else if (capacitance > baseline + cap_touch + CAP_RANGE) { //if capacitance is over the "highest pressure"...
      servoArmPos = POS_HIGH; //make the pwm 100 percent
    }
  }
  else {
    /* segmented servo pos mapping */
    if (current_touch == no_touch) {
      servoArmPos = 0;
    }
    else if (current_touch == light_touch) {
      servoArmPos = 8;
    }
    else if (current_touch == med_touch) {
      servoArmPos = 16;
    }
    else
      servoArmPos = 24;
  }
}

/* Function: findpwm
   "Inputs": isOutputLinear, capacitance, baseline, cap_touch
   "Outputs": pwm
   Purpose: checks isOutputLinear and either linearly maps capacitance to a pwm in the range [PWM_LOW, PWM_HIGH] or maps capacitance to one of three discrete values
*/
void findpwm()
{
  float current_touch = checkTouch(capacitance);

  if (isOutputLinear == true) {
    /* linear pwm mapping */
    //  map capacitance to pwm (make the lowest possible vibration pwm 80):
    pwm = myMap(capacitance, baseline + cap_touch , baseline + cap_touch + CAP_RANGE, PWM_LOW, PWM_HIGH);

    if (capacitance < baseline + cap_touch) { //if capacitance is smaller than what we consider a touch...
      pwm = 0; //make the pwm zero percent
    }
    else if (capacitance > baseline + cap_touch + CAP_RANGE) { //if capacitance is over the "highest pressure"...
      pwm = 255; //make the pwm 100 percent
    }
  }
  else {
    /* segmented vibration mapping */
    if (current_touch == no_touch) {
      pwm = 0;
    }
    else if (current_touch == light_touch) {
      pwm = 90;
    }
    else if (current_touch == med_touch) {
      pwm = 130;
    }
    else
      pwm = 255;
  }
}

/* Function: readCapacitance
   "Outputs": capacitance
   Purpose: reads in capacitance from CDC and saves it into capacitance.
*/

void readCapacitance()
{
  capacitance = (float)readValue();         // Read in capacitance value

  // **Converts CDC value to capacitance (pF)**
  // CDC full-scale input range is +- 4.096 pF (|8.192| pF)
  //    -4.096 pF --> 0x000000 --> 0
  //    0 pF      --> 0x800000 --> 8388608
  //    +4.096 pF --> 0xFFFFFF --> 16777215
  //
  //    {y = mx + b} -->  {y = (|C range in pF|/|equivalent CDC range|)x + (C val in pF when CDC val = 0)}
  //    {y= (|8.192pF|/|16777215|)x -4.096pF}

  capacitance = ((capacitance / 16777215) * 8.192) - 4.096; // converted val is capacitance value
  delay(15);                //Need a delay here or data will be transmitted out of order (or not at all)

  // **don't need to return capacitance, it is a global variable
}

/* Function: fadeawayTimer
   Purpose: checks whether pwm should be linearly mapped or segmented by checking isOutputLinear and calls the appropriate fadeaway timer function.
*/

void fadeawayTimer()
{
  if (isOutputLinear == true) {
    linearFadeTimer();
  }
  else {
    segFadeTimer();
  }
}

/* Function: timerIsr
   "Outputs": timerStatus
   Purpose: sets timerStatus to finished to let the main program know that timer has counted up to CONST_TIME
*/
void timerIsr() // if timerISR gets triggered mistakenly, can add a variable count
{
  Serial.println("timer finished");
  timerStatus = finished; //let the main program know the timer has gone off
  Timer1.stop(); // stop the timer from counting any further
}

/*  Function: checkTouch
    Input: capVal
    Output: one of three constant values representing different touch intensities
    Purpose: Takes in a capacitance value capVal and maps it to an intensity segment called either LIGHT_TOUCH_CAP, MED_TOUCH_CAP, or HARD_TOUCH_CAP
*/

float checkTouch(float capVal) {
  if (capVal <= baseline + cap_touch) {
    return no_touch;
  }
  else if (capVal <= baseline + cap_touch + MED_TOUCH_CAP) {
    return light_touch;
  }
  else if (capVal <= baseline + cap_touch + HARD_TOUCH_CAP) {
    return med_touch;
  }
  else {
    return hard_touch;
  }
}

/* Function: linearFadeTimer
   "Inputs": old_capacitance, timerStatus, faded, pwm
   "Outputs": timerStatus
   Purpose: Detects constant touch (assuming capacitance is linearly mapped to a pwm). This is achieved by recording the first capacitance
   that maps to a pwm > 0 and starting a timer.  The timer is stopped and reset if a capacitance reading maps to a pwm < 0 or the difference
   between the current capcitance reading and first_touch is greater than CAP_STATIONARY. If the timer counts up to CONST_TIME,
   linearFade is called.
*/
void linearFadeTimer()
{
  /* only check iff a touch has been detected, the timer is not finished counting, and we have no   */
  if ((faded == false) && (timerStatus != finished) && (pwm > 0)) { // if a real touch has been detected AND timer is not finished counting
    if (timerStatus == not_started) { // if time not started:
      if (abs(capacitance - old_capacitance) < CAP_STATIONARY) { // start time
        first_touch = capacitance; // save the first val for comparison with later cap values
        Timer1.restart(); //start time at the beginning of new period
        timerStatus = started; //signal timer has been started but not finished counting
        Serial.println("timer started");
      }
    }
    else if (timerStatus == started) { // if the timer has been started and we are out of stationary range
      if (abs(capacitance - first_touch) > CAP_STATIONARY) {
        Serial.println("timer stopped, out of constant range");
        Timer1.stop(); // stop counting
        timerStatus = not_started; // reset timer flag - internal timer count is reset when time is started again
      }
    }
  }
  else { // either there is no touch, we have faded away, or we have finished counting and still need to fade
    if (faded == true) { // we have already faded pwm away, so the pwm is zero
      Serial.print("faded, first_touch = ");
      Serial.print(first_touch);
      if ((abs(capacitance - first_touch) > CAP_STATIONARY) || (capacitance < baseline + cap_touch )) { // if out of stationary range or no touch detected:
        timerStatus = not_started; // reset timer flag
        faded = false; // reset faded flag, revert to using regular pwm
        Serial.println("const touch broken, reverting to regular pwm");
      }
      pwm = 0; // otherwise, we need to stay faded
    }
    else if (timerStatus == started) { // if the timer has been started and we are out of stationary range
      Serial.print("timer stopped, no touch detected");
      Timer1.stop(); // stop timer
      timerStatus = not_started; // reset timer flag, internal timer count is reset when time is started again
    }
    else if (timerStatus == finished) { // timer has finished counting 4s (and we have not faded yet
      Serial.println("timer up, beginning to fade");
      linearFade(); // call function to slowly decrease pwm
    }
    /*else there is no touch*/
  }
}

/* Function: segFadeTimer
   "Inputs": old_capacitance, timerStatus, faded, pwm
   "Outputs": timerStatus
   Purpose: Detects constant touch (assuming capacitance is linearly mapped to a pwm). This is achieved by recording the first capacitance
   that maps to a pwm > 0 and starting a timer.  The timer is stopped and reset if a capacitance reading maps to a pwm < 0 or the difference
   between the current capcitance reading and first_touch is greater than CAP_STATIONARY. If the timer counts up to CONST_TIME,
   linearFade is called.
*/

void segFadeTimer()
{
  if ((faded == false) && (timerStatus != finished) && (pwm > 0)) { // only enter this part of the funct if a real touch has been detected AND timer is not finished counting    if (timerStatus == not_started) { // if time not started
    if (timerStatus == not_started) { // if time not started:
      if (abs(capacitance - old_capacitance) < CAP_STATIONARY) { // if in range:
        /* save the initial segment we are in: */
        first_touch = checkTouch(capacitance);
        Serial.print("first_touch = ");
        Serial.println(first_touch);

        Timer1.restart(); //start time at the beginning of new period
        timerStatus = started; //signal timer has been started but not finished counting
        Serial.println("timer started");
      }
    }
    else if (timerStatus == started) { /*the timer has been started and we are out of stationary range*/
      if (checkTouch(capacitance) != first_touch) {
        Serial.print("timer stopped, out of constant range");
        Timer1.stop(); // stop counting
        timerStatus = not_started; // internal timer count is reset when time is started again
      }
    }
  }
  else { /* either there is no touch, we have faded away, or we have finished counting and still need to fade */
    if (faded == true) { /*we have already faded pwm away, so the pwm is zero*/
      Serial.print(first_touch);
      if (checkTouch(capacitance) != first_touch || (capacitance < baseline + cap_touch)) {
        //revert to using regular pwm:
        timerStatus = not_started; // reset timer flag
        /*revert to using regular pwm*/
        faded = false;
        Serial.println("breaking out of fade");
      }
      pwm = 0; // otherwise, keep the pwm "faded" away
    }
    else if (timerStatus == started) { /*the timer has been started and we are out of stationary range*/
      Serial.print("timer stopped, no touch detected");
      Timer1.stop(); // stop the timer
      timerStatus = not_started; // reset timer flag, internal timer count is reset when time is started again
    }
    else if (timerStatus == finished) { /*timer has finished counting 4s (and we have not faded yet)*/
      Serial.println("timer up, beginning to fade");
      segFade(); // begin decreasing the pwm
    }
    /*else there is no touch*/
  }
}

/* Function: linearFade
   "Inputs": capacitance, first_touch
   "Outputs": pwm, timer_count
   Purpose: Slowly decreases the pwm to zero, and continuously calls sendOutput() to output the new pwm without revisiting the main loop.
   Breaks if constant touch is no longer detected, or when pwm equals zero. Function is called when the timer set in linearFadeTimer
   successfully counts up to CONST_TIME.
*/
void linearFade() {
  while (1) {
    readCapacitance();

    if ((abs(capacitance - first_touch) > CAP_STATIONARY)) {
      faded = false;
      timerStatus = not_started;
      break; // if we are out of stationary range or we need to recalibrate
    }
    else if (pwm <= 0) {
      faded = true;
      pwm = 0;
      timerStatus = not_started;
      break;
    }
    else {// else we must be in range and outputtedPwm is positive (pwm will never be negative)
      pwm -= 2;
      Serial.println(pwm); // print the new pwm
      sendOutput(pwm); // transmit or send pwm to pin
    }
  }
}

void segFade() {
  while (1) {
    readCapacitance();

    if ((checkTouch(capacitance) != first_touch )) {
      timerStatus = not_started;
      Serial.println("timer stopped, touch out of range");
      break; // if we are out of stationary range or we need to recalibrate
    }
    else if (pwm <= 0) {
      faded = true;
      pwm = 0;
      timerStatus = not_started;
      break;
    }
    else {// else we must be in range and outputtedPwm is positive (pwm will never be negative)
      pwm -= 2;
      Serial.println(pwm); // print the new pwm
      sendOutput(pwm); // transmit or send pwm to pin
    }
  }
}


/* Function: findStartVals
   "Outputs": baseline, cap_touch
   Purpose: Reads NUM_READINGS capacitnace values and calculates the baseline capacitance (the average of the readings) and cap_touch (the minimum
   difference in capacitance from the baseline needed to output a nonzero pwm)
*/

void findStartVals()
{

  float baselineVals[NUM_READINGS];
  float total = 0;
  float minVal;
  float maxVal;

  // find baseline capacitance value by reading first 100 values (but omit first 10 values, are sometimes negative values):
  for (int i = 0; i < 1; i++) {
    //    Serial.println((((float)readValue() / 16777215) * 8.192) - 4.096); //make sure this line actually executes
  }

  Serial.println("START BASELINE");
  for (int i = 0; i < NUM_READINGS; i++) {
    delay(15); // need delay here or readings will be of a bigger range than usual
    baselineVals[i] = (((float)readValue() / 16777215) * 8.192) - 4.096;  // Read in capacitance value, cast as float (from long)
    Serial.println(baselineVals[i]);
  }

  // initialize minVal and maxVal
  minVal = baselineVals[0];
  maxVal = baselineVals[0];

  // average values in array to find "baseline" cap value
  for (int i = 0; i < NUM_READINGS; i++) {
    total = total + baselineVals[i];

    // check whether current value bigger than previous maxVal
    if ((baselineVals[i] > maxVal) && (baselineVals[i] > 0)) {
      maxVal = baselineVals[i];
    }

    // check whether current value bigger than previous minVal
    if ((baselineVals[i] < minVal) && (baselineVals[i] > 0)) {
      minVal = baselineVals[i];
    }

  }

  baseline = total / NUM_READINGS; // put baseline value into array
  cap_touch = (maxVal - minVal) + 0.05; // calculate full range of values read in baseline
  Serial.println("total, baseline, Min, Max:");
  Serial.println(total);
  Serial.println(baseline);
  Serial.println(minVal);
  Serial.println(maxVal);
}

/*
   Function: myMap
   Inputs: x, in_min, in_max, out_min, out_max
   Outputs: a scaled version of x (int)
   Purpose: Maps a number x from the range [in_min to in_max] and maps it into the range [out_min, out_max]
*/
int myMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}


/* ______READ VALUE FUNCTION______
   This function checks the sensor's status reg until the data is ready and then reads it in
*/
long readValue()
{
  char status = 0;

  // **Waits until conversion is complete**
  //    _BV(0), _BV(2) = when either register is zero, indicates conversion is finished/new data is available
  while (!(status & (_BV(2) | _BV(0))))
  {
    status = readRegister(REGISTER_STATUS);   // Wait for the next conversion
  }

  unsigned long value =  readLong(REGISTER_CAP_DATA);     //Size of unsigned long : 4 bytes (32bits)
  value >>= 8;    //We have read one byte too many - now we have to get rid of it - sensor is 24-bit

  return value;

}
