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

*/

#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <TimerOne.h>

#define I2C_ADDRESS  0x48           // 0x90 shift one to the right
#define REGISTER_STATUS 0x00
#define REGISTER_CAP_DATA 0x01
#define REGISTER_CAP_SETUP 0x07
#define REGISTER_EXC_SETUP 0x09
#define REGISTER_CONFIGURATION 0x0A

#define RESET_ADDRESS 0xBF

#define PWM_HIGH 255
#define PWM_LOW 0
#define CAP_RANGE 0.8
#define NUM_READINGS 100
#define CAP_STATIONARY 0.2 // range of values cap can oscillate between to be "const"
#define NOT_STARTED -1
#define STARTED 0
#define FINISHED 1

#define LIGHT_TOUCH 0
#define MED_TOUCH 1
#define HARD_TOUCH 2

float CDC_val = 0;              // initial CDC value

/* USER DEFINED CONSTANTS */
const int isTransmitter = true;
const int isPwmLinear = false;

// declare pin names
//const int LEDPower = 7;
//const int Gnd = 4;
//const int ButtonInterrupt = 2;
//const int ButtonPower = 5;
//const int ButtonGnd = 6;
const int vibpin1 = 11;

// declare global variables for finger vibration
float baseline = 0;
float CAP_TOUCH = 0;
float converted_val = 0;
float old_converted_val = 0;
float first_touch = 0;

int pwm = 0;
//int lastPwm = 0;

boolean faded = false;

// declare global variables that may be changed through interrupts
volatile boolean needRecalibrate = false;
volatile int timerCount = NOT_STARTED;

// declare global variables for rf transmission
//RF24 radio(10, 9); // CE, CSN (CSN is 9, CE is 10)
//const byte address[6] = "00001";

// declare global variables for NEW rf transmission
RF24 radio(9, 10);
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };


/* ______FUNCTION TO INITIALIZE ARDUINO______ */
void setup()
{
  // ** Set pin modes
  //  pinMode(LEDPower, OUTPUT);
  //  pinMode(Gnd, OUTPUT);
  //  pinMode(ButtonInterrupt, INPUT);
  //  pinMode(ButtonPower, OUTPUT);
  //  pinMode(ButtonGnd, OUTPUT);

  // ** Initialize pin voltages:
  //  digitalWrite(Gnd, LOW);
  //  digitalWrite(ButtonPower, HIGH);
  //  digitalWrite(ButtonGnd, LOW);

  //  digitalWrite(LEDPower, HIGH); //signal the start of calibration

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

  // ** Find the baseline (avg), and CAP_TOUCH (min/max) of the first NUM_READINGS readings
  findStartVals();

  //** Enable interrupt on pin 2
  //  attachInterrupt(digitalPinToInterrupt(2), Button_ISR, FALLING); // pin will go high to low when interrupt

  // ** Enable timer interrupt
  Timer1.initialize(4000000); // set a timer of length 4000000 microseconds (or 4 sec - or 0.25Hz)
  Timer1.attachInterrupt(timerIsr); // attach the service routine here
  Timer1.stop();

  //**** set up rf transmission
  if (isTransmitter == true) {
    //    radio.begin();
    //    radio.openWritingPipe(address);
    //    radio.setPALevel(RF24_PA_MIN);
    //    radio.stopListening();

    // ** set up NEW rf transmission
    radio.begin();
    radio.setAutoAck(false);
    radio.setRetries(15,15);
    radio.openWritingPipe(pipes[0]); // open pipes for transmission
    radio.openReadingPipe(1,pipes[1]);
    radio.startListening();

  }

  // initialize converted_val (so old_converted_val has a proper value in the first run of loop())
  converted_val = baseline;

  //signal the end of calibration
  //  digitalWrite(LEDPower, LOW);

  // why is my timer interrupt triggering on its own??
  timerCount = NOT_STARTED;
}


/* ______MAIN PROGRAM______ */
void loop()
{
  
  old_converted_val = converted_val; // save the old converted_val
  readCapacitance(); // result is stored in global variable converted_val

  //  Serial.print(timerCount);
  //  Serial.print(" ");
//  Serial.println(converted_val, 4); // print new capacitance value to serial

  findpwm(); // find new pwm according to new converted_val
  //  checkRecalibrate(); // check if need to recalibrate baseline (if button has been pressed)
  Serial.println(pwm);
  fadeaway(); // check if need to fade
  sendpwm(); // check if supposed to act as transmitter, then send pwm accordingly 
}

void sendpwm() 
{
  //  check whether we need to transmit or not
  if (isTransmitter == false) {
    //  write pwm to microcontroller
    analogWrite(vibpin1, pwm);
  }
  else {
    //    transmit pwm to other microcontroller
     // First, stop listening so we can talk.
    radio.stopListening();
   
    bool ok = radio.write( &pwm, sizeof(int) );
    
    if (ok)
      printf("ok...");
    else
      printf("failed.\n\r");

    // Now, continue listening
    radio.startListening();

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

//void checkRecalibrate()
//{
//  if (needRecalibrate == true) {
//    Serial.println("needRecalibrate is true");
//    findStartVals(); //call calibrating function
//    digitalWrite(LEDPower, LOW); // turn LED off to signal recalibration was finished
//    needRecalibrate = false;
//  }
//}

void findpwm()
{

  if (isPwmLinear == true) {
    /* linear pwm mapping */
    //  map capacitance to pwm (make the lowest possible vibration pwm 80):
    pwm = myMap(converted_val, baseline + CAP_TOUCH , baseline + CAP_TOUCH + CAP_RANGE, 90, 255);

    if (converted_val < baseline + CAP_TOUCH) { //if converted_val is smaller than what we consider a touch...
      pwm = 0; //make the pwm zero percent on
    }
    else if (converted_val > baseline + CAP_TOUCH + CAP_RANGE) { //if converted_val is over the "highest pressure"...
      pwm = 255; //make the pwm 100 percent on
    }
  }
  else {
    /* segmented vibration mapping */
    if (converted_val < baseline + CAP_TOUCH) {
      pwm = 0;
      //No Touch
    }
    else if (converted_val <= baseline + CAP_TOUCH + 0.3) {
      pwm = 90;
      //Touch
    }
    else if (converted_val <= baseline + CAP_TOUCH + 0.6) {
      pwm = 130;
      //Pressure
    }
    else {
      pwm = 255;
      //Excessive Pressure
    }
  }
}

void readCapacitance()
{
  converted_val = (float)readValue();         // Read in capacitance value

  // **Converts CDC value to capacitance (pF)**
  // CDC full-scale input range is +- 4.096 pF (|8.192| pF)
  //    -4.096 pF --> 0x000000 --> 0
  //    0 pF      --> 0x800000 --> 8388608
  //    +4.096 pF --> 0xFFFFFF --> 16777215
  //
  //    {y = mx + b} -->  {y = (|C range in pF|/|equivalent CDC range|)x + (C val in pF when CDC val = 0)}
  //    {y= (|8.192pF|/|16777215|)x -4.096pF}

  converted_val = ((converted_val / 16777215) * 8.192) - 4.096; // converted val is capacitance value
  delay(15);                //Need a delay here or data will be transmitted out of order (or not at all)

  // **don't need to return converted_val, it is a global variable
}

void fadeaway()
{
  if (isPwmLinear == true) {
    /* only check iff a touch has been detected, the timer is not finished counting, and we have no   */
    if ((faded == false) && (timerCount != FINISHED) && (pwm > 0)) { /* only enter this part of the funct iff a real touch has been detected AND timer is not finished counting */
//      Serial.print("const touch detected ");
      //    Serial.println(converted_val);
      //    Serial.println(first_touch);
      //    Serial.println(abs(converted_val - first_touch));
      if (timerCount == NOT_STARTED) { /*time not started*/
        /* start time */
        if (abs(converted_val - old_converted_val) < CAP_STATIONARY) {
          first_touch = converted_val; // save the first val for comparison with later cap values
          Timer1.restart(); //start time at the beginning of new period
          timerCount = STARTED; //signal timer has been started but not finished counting
          Serial.println("timer started!!!!!!!!!!");
//          Serial.println(first_touch);
        }
      }
      else if (timerCount == STARTED) { /*the timer has been started and we are out of stationary range*/
        if (abs(converted_val - first_touch) > CAP_STATIONARY) {
          Serial.print("timer stopped???");
//          Serial.println(converted_val);
//          Serial.println(old_converted_val);

          /*stop counting*/
          Timer1.stop();
          /*reset the timer*/
          timerCount = NOT_STARTED; // internal timer count is reset when time is started again
        }
      }
    }
    else { /* either there is no touch, we have faded away, or we have finished counting and still need to fade */
      if (faded == true) { /*we have already faded pwm away, so the pwm is zero*/
        //      if(abs(converted_val - old_converted_val) > CAP_STATIONARY) { // if we are outside of what we consider a "constant touch"
        Serial.print("faded, first_touch = ");
        Serial.print(first_touch);
        if ((abs(converted_val - first_touch) > CAP_STATIONARY) || (converted_val < baseline + CAP_TOUCH )) {
          //revert to using regular pwm:
          /*reset timer count*/
          timerCount = NOT_STARTED;
          /*revert to using regular pwm*/
          faded = false;
          Serial.println("breaking out of fade");
        }
        /*else keep fadeaway*/
        pwm = 0;
      }
      else if (timerCount == STARTED) { /*the timer has been started and we are out of stationary range*/
        Serial.print("timer stopped?????????????????? ");
        /*stop counting*/
        Timer1.stop();
        /*reset the timer*/
        timerCount = NOT_STARTED; // internal timer count is reset when time is started again
      }
      else if (timerCount == FINISHED) { /*timer has finished counting 4s (and we have not faded yet)*/
        /* start fadeaway */
        Serial.println("timer up, beginning to fade");
        touchfade();
      }
      /*else there is no touch*/
    }
  }
  else {
    if ((faded == false) && (timerCount != FINISHED) && (pwm > 0)) { /* only enter this part of the funct iff a real touch has been detected AND timer is not finished counting */
//      Serial.print("const touch detected ");
      if (timerCount == NOT_STARTED) { /*time not started*/
        /* start time */
        if (abs(converted_val - old_converted_val) < CAP_STATIONARY) {
          /* save the initial segment we are in: */
          // first_touch = converted_val; // save the first val for comparison with later cap values
          first_touch = checkTouch(converted_val);

          Timer1.restart(); //start time at the beginning of new period
          timerCount = STARTED; //signal timer has been started but not finished counting
          Serial.println("timer started!!!!!!!!!!");
//          Serial.println(first_touch);
        }
      }
      else if (timerCount == STARTED) { /*the timer has been started and we are out of stationary range*/
        // if (abs(converted_val - first_touch) > CAP_STATIONARY) {
        if (checkTouch(converted_val) != first_touch) {
          Serial.print("timer stopped???");
//          Serial.println(old_converted_val);
//          Serial.println(converted_val);

          /*stop counting*/
          Timer1.stop();
          /*reset the timer*/
          timerCount = NOT_STARTED; // internal timer count is reset when time is started again
        }
      }
    }
    else { /* either there is no touch, we have faded away, or we have finished counting and still need to fade */
      if (faded == true) { /*we have already faded pwm away, so the pwm is zero*/
        //      if(abs(converted_val - old_converted_val) > CAP_STATIONARY) { // if we are outside of what we consider a "constant touch"
//        Serial.print("faded, first_touch = ");
        Serial.print(first_touch);
        if (checkTouch(converted_val) != first_touch || (converted_val < baseline + CAP_TOUCH)) {
          //revert to using regular pwm:
          /*reset timer count*/
          timerCount = NOT_STARTED;
          /*revert to using regular pwm*/
          faded = false;
          Serial.println("breaking out of fade");
        }
        /*else keep fadeaway*/
        pwm = 0;
      }
      else if (timerCount == STARTED) { /*the timer has been started and we are out of stationary range*/
        Serial.print("timer stopped?????????????????? ");
        /*stop counting*/
        Timer1.stop();
        /*reset the timer*/
        timerCount = NOT_STARTED; // internal timer count is reset when time is started again
      }
      else if (timerCount == FINISHED) { /*timer has finished counting 4s (and we have not faded yet)*/
        /* start fadeaway */
        Serial.println("timer up, beginning to fade");
        touchfade();
      }
      /*else there is no touch*/
    }
  }

}

/*
   timerISR - goes off each time the timer overflows
*/
void timerIsr() // if timerISR gets triggered mistakenly, can add a variable count
{
  Serial.println("timer finished");
  timerCount = FINISHED; //let the main program know the timer has gone off
  Timer1.stop(); // stop the timer from counting any further
}

int checkTouch(float capVal) {
  if (capVal <= baseline + CAP_TOUCH + 0.3) {
    return LIGHT_TOUCH;
  }
  else if (capVal <= baseline + CAP_TOUCH + 0.6) {
    return MED_TOUCH;
  }
  else {
    return HARD_TOUCH;
  }
}

/*
   fadeaway funct - controls pwm output until it is zero
*/
int touchfade() // find a way to trigger recalibrate while in this function
{
  if (isPwmLinear == true) {
    while (1) {
      readCapacitance();

      if ((abs(converted_val - first_touch) > CAP_STATIONARY) || needRecalibrate == true) {
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

        // ** either transmit the pwm here or make it the new pwm
        if (isTransmitter == false) {
          analogWrite(vibpin1, pwm);
          Serial.println(pwm);
        }
        else {
          Serial.println(pwm);
          sendpwm();
        }
      }
    }
  }
  else {
    while (1) {
      readCapacitance();

      if ((checkTouch(converted_val) != first_touch ) || needRecalibrate == true) {
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

        // ** either transmit the pwm here or make it the new pwm
        if (isTransmitter == false) {
          analogWrite(vibpin1, pwm);
          Serial.println(pwm);
        }
        else {
          Serial.println(pwm);
          sendpwm();
        }
      }
    }
  }
}


/* if either pwm changes or recalibrate needed, break*/



/*
    min/max/baseline capacitance finding function
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
  CAP_TOUCH = (maxVal - minVal) + 0.05; // calculate full range of values read in baseline
  Serial.println("total, baseline, Min, Max:");
  Serial.println(total);
  Serial.println(baseline);
  Serial.println(minVal);
  Serial.println(maxVal);
}

/*
   maps a number x from the range [in_min to in_max] and maps it into the range [out_min, out_max]
*/
int myMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

/*
   Function to handle the button interrupt
*/

//void Button_ISR()
//{
//  digitalWrite(LEDPower, HIGH); // turn LED on to signal recalibration
//  Serial.println("interrupt is triggered");
//  needRecalibrate = true;
//
//}

/* ______READ VALUE FUNCTION______
   This function checks the sensor's status reg until the data is ready and then reads it in
*/
long readValue()
{
  char status = 0;

  // **Waits until conversionr is complete**
  //    _BV(0), _BV(2) = when either register is zero, indicates conversion is finished/new data is available
  while (!(status & (_BV(2) | _BV(0))))
  {
    status = readRegister(REGISTER_STATUS);   // Wait for the next conversion
  }

  unsigned long value =  readLong(REGISTER_CAP_DATA);     //Size of unsigned long : 4 bytes (32bits)
  value >>= 8;    //We have read one byte too many - now we have to get rid of it - sensor is 24-bit

  return value;

}
