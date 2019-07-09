/*
   - add something that recalibrates after high pressures ?
   - arduino pins can only sink max 40mA
   - digital write high is 5V

   - if a pin is set to an input (default), then internal pullup resistor is activated (desirable for reading inputs, as
   - do some kind of hard pressure test to see how much the displacement (capacitance) changes at rest to help with recalibration

*/

#include <Wire.h>

#define I2C_ADDRESS  0x48 					// 0x90 shift one to the right
#define REGISTER_STATUS 0x00
#define REGISTER_CAP_DATA 0x01
#define REGISTER_CAP_SETUP 0x07
#define REGISTER_EXC_SETUP 0x09
#define REGISTER_CONFIGURATION 0x0A

#define RESET_ADDRESS 0xBF

#define PWM_HIGH 255
#define PWM_LOW 0
#define CAP_RANGE 1.5
#define NUM_READINGS 100


float CDC_val = 0;							// CDC value

// declare pin names
const int vibPin1 = 9; // ******** changed from transmitter code
const int LEDPower = 5;
const int LEDGnd = 4;
const int ButtonInterrupt = 2;
const int ButtonPower = 7;
const int ButtonGnd = 6;

// declare global variables shared by setup/main
float baseline = 0;
float CAP_TOUCH = 0;
volatile boolean needRecalibrate = false;
float converted_val = 0;

int pwm = 0;


/* ______FUNCTION TO INITIALIZE ARDUINO______ */
void setup()
{
  // ** Set pin modes
  pinMode(vibPin1, OUTPUT);
  pinMode(LEDPower, OUTPUT);
  pinMode(LEDGnd, OUTPUT);
  pinMode(ButtonInterrupt, INPUT);
  pinMode(ButtonPower, OUTPUT);
  pinMode(ButtonGnd, OUTPUT);

  // ** Initialize pin voltages:
  digitalWrite(LEDGnd, LOW);
  digitalWrite(ButtonPower, HIGH);
  digitalWrite(ButtonGnd, LOW);

  digitalWrite(LEDPower, HIGH); //signal the start of calibration

  Wire.begin();							// Set up I2C for operation

  Serial.begin(9600);						// Set up baud rate for serial communication to com port
  Wire.beginTransmission(I2C_ADDRESS);	// Start I2C cycle
  Wire.write(RESET_ADDRESS);				// Reset the device
  Wire.endTransmission();					// End I2C cycle

  delay(1);								// Wait a tad for reboot

  // **Cap Channel Excitation Setup**
  //		_BV(3) = enables EXCA pin as excitation output
  // 		_BV(1), BV(0) = set excitation voltage level
  writeRegister(REGISTER_EXC_SETUP, _BV(3) | _BV(1) | _BV(0));

  // **Cap Channel Setup**
  //		_BV(7) = enables channel for single conversion | continuous conversion | calibration
  writeRegister(REGISTER_CAP_SETUP, _BV(7)); //Cap setup reg - cap enabled

  delay(10);

  // **Converter Update Rate and Mode of Operation Setup**
  //		_BV(0) = sets operation to continuous convertion (set at highest speed)
  writeRegister(REGISTER_CONFIGURATION, _BV(0));

  // ** Find the baseline (avg), and CAP_TOUCH (min/max) of the first NUM_READINGS readings
  findStartVals();

  //** Enable interrupt on pin 3
  //  attachInterrupt(digitalPinToInterrupt(2), Button_ISR, FALLING); // pin will go high to low when interrupt

  digitalWrite(LEDPower, LOW); //signal the end of calibration
}


/* ______MAIN PROGRAM______ */
void loop()
{

  converted_val = (float)readValue();					// Read in capacitance value

  // **Converts CDC value to capacitance (pF)**
  // CDC full-scale input range is +- 4.096 pF (|8.192| pF)
  // 		-4.096 pF --> 0x000000 --> 0
  //		0 pF      --> 0x800000 --> 8388608
  // 		+4.096 pF --> 0xFFFFFF --> 16777215
  //
  // 		{y = mx + b} -->  {y = (|C range in pF|/|equivalent CDC range|)x + (C val in pF when CDC val = 0)}
  // 		{y= (|8.192pF|/|16777215|)x -4.096pF}

  converted_val = ((converted_val / 16777215) * 8.192) - 4.096; // converted val is capacitance value

  delay(15);								//Need a delay here or data will be transmitted out of order (or not at all)


  /* linear pwm mapping */
  //  //  map capacitance to pwm (make the lowest possible vibration pwm 80):
  //  pwm = myMap(converted_val, baseline + CAP_TOUCH , baseline + CAP_TOUCH + CAP_RANGE, 80, 255);
  //
  //  if (converted_val < baseline + CAP_TOUCH) { //if converted_val is smaller than what we consider a touch...
  //    pwm = 0; //make the pwm zero percent on
  //  }
  //  else if (converted_val > baseline + CAP_TOUCH + CAP_RANGE) { //if converted_val is over the "highest pressure"...
  //    pwm = 255; //make the pwm 100 percent on
  //  }

  /* segmented vibration mapping */
  if (converted_val < baseline + CAP_TOUCH)
  {
    pwm = 0;
    //No Touch
  }

  else if (converted_val <= baseline + CAP_TOUCH + 0.3)
  {
    pwm = 80;
    //Touch
  }
  else if (converted_val <= baseline + CAP_TOUCH + 0.6)
  {
    pwm = 130;
    //Pressure
  }
  else
  {
    pwm = 255;
    //Excessive Pressure
  }
  /* */

  // write the new pwm to the pin
  analogWrite(vibPin1, pwm); // write the pwm to the pin
  Serial.println(pwm);
  // Serial.print(converted_val, 4);

  //check if need to recalibrate:
  if (needRecalibrate == true) {
    Serial.println("needRecalibrate is true");
    findStartVals(); //call calibrating function
    digitalWrite(LEDPower, LOW); // turn LED off to signal recalibration was finished
    needRecalibrate = false;
  }

}

/*
    min/max/baseline capacitance finding function
*/

void findStartVals()
{

  float baselineVals[NUM_READINGS];
  float average = 0;
  float minVal;
  float maxVal;

  // find baseline capacitance value by reading first 100 values (but omit first 10 values, are sometimes negative values):
  for (int i = 0; i < 10; i++) {
    readValue(); //make sure this line actually executes
  }


  for (int i = 0; i < NUM_READINGS; i++) {
    delay(15); // need delay here or readings will be of a bigger range than usual
    baselineVals[i] = (((float)readValue() / 16777215) * 8.192) - 4.096;  // Read in capacitance value, cast as float (from long)
    //    Serial.println(baselineVals[i]);
  }

  // initialize minVal and maxVal
  minVal = baselineVals[0];
  maxVal = baselineVals[0];

  // average values in array to find "baseline" cap value
  for (int i = 0; i < NUM_READINGS; i++) {
    average = average + baselineVals[i];

    // check whether current value bigger than previous maxVal
    if ((baselineVals[i] > maxVal) && (baselineVals[i] > 0)) {
      maxVal = baselineVals[i];
    }

    // check whether current value bigger than previous minVal
    if ((baselineVals[i] < minVal) && (baselineVals[i] > 0)) {
      minVal = baselineVals[i];
    }

  }

  baseline = average / NUM_READINGS; // put baseline value into array
  CAP_TOUCH = (maxVal - minVal) / 2 ; // calculate "average" error

  Serial.println("average, baseline, Min, Max:");
  Serial.println(average);
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

void Button_ISR()
{
  digitalWrite(LEDPower, HIGH); // turn LED on to signal recalibration
  Serial.println("interrupt is triggered");
  needRecalibrate = true;

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

/*
   possible patient tests
   - figure out where to put vibration
   - watch line of sight (how often looking at hand?)
   - blindfold test/egg test ?
   - squish foam block
   - holding cups (styrofoam, juice boxes...)
   - picking something out of bucket
   - talk to them !
   - ask most common objects picked up
   - cognitive load
   - handshake test
   - timed tests w/ 2 trials with or without
   - once w/ and w/o sensor, another w/ and w/o sensor without looking
   - vibration + mechanical ? !
   - on/off
   - stick with validated questions (questions that give reliable answers -- do not ask pre/post q's)
   - desired sensor position -- pick from a group
   - 1. pilot technology to fine-tune test ?
   - research am-ula
   - box and blocks test (brittany is familiar with this one.)
   - clothespin placement test
   - 9 hole peg test ????

   - get survey out
   - make everythign smaller
   - actually put it on a hand w/o the grey thing
   - correct vibration
   -
*/
