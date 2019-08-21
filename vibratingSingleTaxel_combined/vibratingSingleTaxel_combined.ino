/*******************************
   The following MUST be true for this code to work:

   1. if using Piksey Pico board, you are using Arduino 1.8.7 (NOT the most recent version!)

   2. in "TimerOne.h" that TCNT1 = 1 (not TCNT1 = 0) in the TimerOne library files
      otherwise, interrupt will trigger immediately after it is enabled/started

   3. if using Piksey Pico board, add "|| defined (__AVR_ATmega328PB__)" to line 94 in "known_16bit_timers.h" in the TimerOne library files
      otherwise, the TimerOne library will not be able to recognize the board

 *******************************/
/*
   - add something that recalibrates after high pressures ?
   - arduino pins can only sink max 40mA
   - digital write high is 5V

   - if a pin is set to an input (default), then internal pullup resistor is activated (desirable for reading inputs, as
   - do some kind of hard pressure test to see how much the displacement (capacitance) changes at rest to help with recalibration
   - smoothing funct so timer doesn't start/stop constantly on the borderline of a new pwm segment

*/

/* Author: Emily Lee
   Date: August 2019
   Purpose: Reads capacitance from AD7746 CDC and calculates a corresponding pwm (to be outputted to a vibration motor). Optionally, this pwm can be
   transmitted to another microcontroller using nrF24l01+ modules. These features, as well as touch sensitivity, can be controlled with user-defined variables.
   Email: emilylee.email@gmail.com
*/

#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <TimerOne.h>

/********** USER DEFINED CONSTANTS ************/
const int isTransmitter = false; // set as true if want to act as transmitter, false if want to output vibration on vibpin (pin 11)
const int isPwmLinear = false; // set as true if want pwm to change linearly with changes in capacitance, false if want pwm to remain within 4 discrete values

/* linear pwm constants */
#define PWM_HIGH 255 // the highest pwm outputted
#define PWM_LOW 90 // the lowest pwm outputted (if a pwm less than PWM_LOW is calculated, pwm outputted will be 0)
#define CAP_RANGE 0.8 // the range of capacitance (pF) between pwm = PWM_LOW and pwm = PWM_HIGH

/* segmented pwm constants */
#define NO_TOUCH -0.3 // placeholder
#define LIGHT_TOUCH 0 // placeholder for timerFade (a capacitance reading of baseline + cap_touch is considered a "light touch")
#define MED_TOUCH 0.3 // the amount of capacitance above baseline + cap_touch for a touch to be considered a "medium touch"
#define HARD_TOUCH 0.6 // the amount of capacitance above baseline + cap_touch for a touch to be considered a "hard touch"

#define CAP_STATIONARY 0.2 // range of values cap can oscillate between to be considered "constant"
#define CONST_TIME 4000000 // the minimum time (in microseconds) a constant touch has to be held to trigger pwm fade 
/**********************************************/

/* constants for CDC */
#define I2C_ADDRESS  0x48           // 0x90 shift one to the right
#define REGISTER_STATUS 0x00
#define REGISTER_CAP_DATA 0x01
#define REGISTER_CAP_SETUP 0x07
#define REGISTER_EXC_SETUP 0x09
#define REGISTER_CONFIGURATION 0x0A
#define RESET_ADDRESS 0xBF

/* constants for timer state in fadeawayTimer */
#define NOT_STARTED -1
#define STARTED 0
#define FINISHED 1

/* constant for calculating baseline */
#define NUM_READINGS 100 // the number of readings averaged when calculating the baseline (increasing this number also increases calibration time)

/* pin names */
const int vibpin1 = 11; // the pin at which the calculated pwm is outputted
//const int rfPower = 5;

/* global variables */
float baseline = 0;
float cap_touch = 0;
float capacitance = 0;
float old_capacitance = 0;
float first_touch = 0;
int pwm = 0;
boolean faded = false;

// declare global variables that may be changed through interrupts
volatile boolean needRecalibrate = false;
volatile int timerCount = NOT_STARTED;

// declare global variables for NEW rf transmission
RF24 radio(9, 10);
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };


/* ______FUNCTION TO INITIALIZE ARDUINO______ */
void setup()
{
  // ** Set pin modes
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(vibpin1, OUTPUT);

  //  digitalWrite(LED_BUILTIN, HIGH); //signal the start of calibration

  //    analogWrite(rfPower, 153); // power the rf module

  Wire.begin();             // Set up I2C for operation

  Serial.begin(9600);           // Set up baud rate for serial communication to com port
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

  // ** Enable timer interrupt
  Timer1.initialize(CONST_TIME); // sets a timer of length CONST_TIME microseconds
  Timer1.attachInterrupt(timerIsr); // attaches the timer service routine
  Timer1.stop();

  //** Set up RF transmission
  if (isTransmitter == true) {
    radio.begin();
    radio.setAutoAck(false);
    radio.setRetries(15, 15);
    radio.openWritingPipe(pipes[0]); // open pipes for transmission
    radio.openReadingPipe(1, pipes[1]);
    radio.startListening();
  }

  capacitance = baseline;   // initialize capacitance (so old_capacitance has a proper value in the first run of loop())
  timerCount = NOT_STARTED; // make sure timer is not started
  digitalWrite(LED_BUILTIN, LOW);   //signal the end of calibration
}


/* ______MAIN PROGRAM______ */
void loop()
{
  old_capacitance = capacitance; // save the old capacitance
  readCapacitance(); // new capacitance is stored in global variable capacitance
  findpwm(); // find new pwm according to new capacitance
  Serial.println(pwm); // print pwm to the screen
  fadeawayTimer(); // check if need to fade
  sendpwm(); // check if supposed to act as transmitter, then send pwm accordingly
}

/* Function: sendpwm
   "Inputs": pwm, isTransmitter
   Purpose: checks the state of isTransmitter and either writes the calculated pwm to vibpin1 or transmits it to another microcontroller using RF transmission
*/

void sendpwm()
{
  if (isTransmitter == false) {
    analogWrite(vibpin1, pwm); //  if we don't need to transmit pwm, write pwm to vibpin
  }
  else {
    // transmit pwm to other microcontroller
    radio.stopListening(); // Stop listening for a response so we can send a message

    bool ok = radio.write( &pwm, sizeof(int) );

    if (ok)
      printf("ok...");
    else
      printf("failed.\n\r");

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
      printf("Failed, response timed out.\n\r");
    }
    else
    {
      // Grab the response, compare, and send to debugging spew
      int response;
      radio.read( &response, sizeof(int) );

      // Spew it
      printf("Got response %d \n\r", response);
    }

    // wait a tad before trying again
    delay(50);
  }
}

/* Function: findpwm
   "Inputs": isPwmLinear, capacitance, baseline, cap_touch
   "Outputs": pwm
   Purpose: checks isPwmLinear and either linearly maps capacitance to a pwm in the range [PWM_LOW, PWM_HIGH] or maps capacitance to one of three discrete values
*/
void findpwm()
{

  if (isPwmLinear == true) {
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
    if (capacitance < baseline + cap_touch) {
      pwm = 0;
      //No Touch
    }
    else if (capacitance <= baseline + cap_touch + MED_TOUCH) {
      pwm = 90;
      //Touch
    }
    else if (capacitance <= baseline + cap_touch + HARD_TOUCH) {
      pwm = 130;
      //Pressure
    }
    else {
      pwm = 255;
      //Excessive Pressure
    }
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
   Purpose: checks whether pwm is linearly scaled or segmented by checking isPwmLinear and calls the appropriate fadeaway timer function.
*/

void fadeawayTimer()
{
  if (isPwmLinear == true) {
    linearFadeTimer();
  }
  else {
    segFadeTimer();
  }
}

/* Function: linearFadeTimer
   "Inputs": old_capacitance, timerCount, faded, pwm
   "Outputs": timerCount
   Purpose: Detects constant touch (assuming capacitance is linearly mapped to a pwm). This is achieved by recording the first capacitance
   that maps to a pwm > 0 and starting a timer.  The timer is stopped and reset if a capacitance reading maps to a pwm < 0 or the difference
   between the current capcitance reading and first_touch is greater than CAP_STATIONARY. If the timer counts up to CONST_TIME,
   linearFade is called.
*/
void linearFadeTimer()
{
  /* only check iff a touch has been detected, the timer is not finished counting, and we have no   */
  if ((faded == false) && (timerCount != FINISHED) && (pwm > 0)) { // if a real touch has been detected AND timer is not finished counting
    if (timerCount == NOT_STARTED) { // if time not started:
      if (abs(capacitance - old_capacitance) < CAP_STATIONARY) { // start time
        first_touch = capacitance; // save the first val for comparison with later cap values
        Timer1.restart(); //start time at the beginning of new period
        timerCount = STARTED; //signal timer has been started but not finished counting
        Serial.println("timer started");
      }
    }
    else if (timerCount == STARTED) { // if the timer has been started and we are out of stationary range
      if (abs(capacitance - first_touch) > CAP_STATIONARY) {
        Serial.print("timer stopped, out of constant range");
        Timer1.stop(); // stop counting
        timerCount = NOT_STARTED; // reset timer flag - internal timer count is reset when time is started again
      }
    }
  }
  else { // either there is no touch, we have faded away, or we have finished counting and still need to fade
    if (faded == true) { // we have already faded pwm away, so the pwm is zero
      Serial.print("faded, first_touch = ");
      Serial.print(first_touch);
      if ((abs(capacitance - first_touch) > CAP_STATIONARY) || (capacitance < baseline + cap_touch )) { // if out of stationary range or no touch detected:
        timerCount = NOT_STARTED; // reset timer flag
        faded = false; // reset faded flag, revert to using regular pwm
        Serial.println("const touch broken, reverting to regular pwm");
      }
      pwm = 0; // otherwise, we need to stay faded
    }
    else if (timerCount == STARTED) { // if the timer has been started and we are out of stationary range
      Serial.print("timer stopped, no touch detected");
      Timer1.stop(); // stop timer
      timerCount = NOT_STARTED; // reset timer flag, internal timer count is reset when time is started again
    }
    else if (timerCount == FINISHED) { // timer has finished counting 4s (and we have not faded yet
      Serial.println("timer up, beginning to fade");
      linearFade(); // call function to slowly decrease pwm
    }
    /*else there is no touch*/
  }
}

/* Function: segFadeTimer
   "Inputs": old_capacitance, timerCount, faded, pwm
   "Outputs": timerCount
   Purpose: Detects constant touch (assuming capacitance is linearly mapped to a pwm). This is achieved by recording the first capacitance
   that maps to a pwm > 0 and starting a timer.  The timer is stopped and reset if a capacitance reading maps to a pwm < 0 or the difference
   between the current capcitance reading and first_touch is greater than CAP_STATIONARY. If the timer counts up to CONST_TIME,
   linearFade is called.
*/

void segFadeTimer()
{
  if ((faded == false) && (timerCount != FINISHED) && (pwm > 0)) { // only enter this part of the funct if a real touch has been detected AND timer is not finished counting    if (timerCount == NOT_STARTED) { // if time not started
    if (timerCount == NOT_STARTED) { // if time not started:
      if (abs(capacitance - old_capacitance) < CAP_STATIONARY) { // if in range:
        /* save the initial segment we are in: */
        first_touch = checkTouch(capacitance);
        Serial.print("first_touch = ");
        Serial.println(first_touch);

        Timer1.restart(); //start time at the beginning of new period
        timerCount = STARTED; //signal timer has been started but not finished counting
        Serial.println("timer started");
      }
    }
    else if (timerCount == STARTED) { /*the timer has been started and we are out of stationary range*/
      if (checkTouch(capacitance) != first_touch) {
        Serial.print("timer stopped, out of constant range");
        Timer1.stop(); // stop counting
        timerCount = NOT_STARTED; // internal timer count is reset when time is started again
      }
    }
  }
  else { /* either there is no touch, we have faded away, or we have finished counting and still need to fade */
    if (faded == true) { /*we have already faded pwm away, so the pwm is zero*/
      Serial.print(first_touch);
      if (checkTouch(capacitance) != first_touch || (capacitance < baseline + cap_touch)) {
        //revert to using regular pwm:
        timerCount = NOT_STARTED; // reset timer flag
        /*revert to using regular pwm*/
        faded = false;
        Serial.println("breaking out of fade");
      }
      pwm = 0; // otherwise, keep the pwm "faded" away
    }
    else if (timerCount == STARTED) { /*the timer has been started and we are out of stationary range*/
      Serial.print("timer stopped, no touch detected");
      Timer1.stop(); // stop the timer
      timerCount = NOT_STARTED; // reset timer flag, internal timer count is reset when time is started again
    }
    else if (timerCount == FINISHED) { /*timer has finished counting 4s (and we have not faded yet)*/
      Serial.println("timer up, beginning to fade");
      segFade(); // begin decreasing the pwm
    }
    /*else there is no touch*/
  }
}

/* Function: timerIsr
   "Outputs": timerCount
   Purpose: sets timerCount to FINISHED to let the main program know that timer has counted up to CONST_TIME
*/
void timerIsr() // if timerISR gets triggered mistakenly, can add a variable count
{
  Serial.println("timer finished");
  timerCount = FINISHED; //let the main program know the timer has gone off
  Timer1.stop(); // stop the timer from counting any further
}

/*  Function: checkTouch
    Input: capVal
    Output: one of three constant values representing different touch intensities
    Purpose: Takes in a capacitance value capVal and maps it to an intensity segment called either LIGHT_TOUCH, MED_TOUCH, or HARD_TOUCH
*/

float checkTouch(float capVal) {
  if(capVal <= baseline + cap_touch) {
    return NO_TOUCH; 
  }
  else if (capVal <= baseline + cap_touch + MED_TOUCH) {
    return LIGHT_TOUCH;
  }
  else if (capVal <= baseline + cap_touch + HARD_TOUCH) {
    return MED_TOUCH;
  }
  else {
    return HARD_TOUCH;
  }
}

/* Function: linearFade
   "Inputs": capacitance, first_touch
   "Outputs": pwm, timer_count
   Purpose: Slowly decreases the pwm to zero, and continuously calls sendpwm() to output the new pwm without revisiting the main loop.
   Breaks if constant touch is no longer detected, or when pwm equals zero. Function is called when the timer set in linearFadeTimer
   successfully counts up to CONST_TIME.
*/
void linearFade() {
  while (1) {
    readCapacitance();

    if ((abs(capacitance - first_touch) > CAP_STATIONARY)) {
      faded = false;
      timerCount = NOT_STARTED;
      break; // if we are out of stationary range or we need to recalibrate
    }
    else if (pwm <= 0) {
      faded = true;
      pwm = 0;
      timerCount = NOT_STARTED;
      break;
    }
    else {// else we must be in range and outputtedPwm is positive (pwm will never be negative)
      pwm -= 2;
      Serial.println(pwm); // print the new pwm
      sendpwm(); // transmit or send pwm to pin
    }
  }
}
void segFade() { /**************************************** newest version of this function as of 9:15am*/
  while (1) {
    readCapacitance();
    if ((checkTouch(capacitance) != first_touch )) {
      timerCount = NOT_STARTED;
      Serial.println("timer stopped, touch out of range");
      break; // if we are out of stationary range or we need to recalibrate
    }
    else if (pwm <= 0) {
      faded = true;
      pwm = 0;
      timerCount = NOT_STARTED;
      break;
    }
    else {// else we must be in range and outputtedPwm is positive (pwm will never be negative)
      pwm -= 2;
      Serial.println(pwm); // print the new pwm
      sendpwm(); // transmit or send pwm to pin
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
