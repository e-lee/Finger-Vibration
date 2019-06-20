/*
   - add something that recalibrates after high pressures

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
#define CAP_RANGE 0.8
//#define CAP_TOUCH 0.05
#define NUM_READINGS 100


long CDC_val = 0;							// Unconverted value from CDC
float converted_val = 0; 					// Converted pF value

const int vibPin1 = 9; // make a #define?
const int vibPin2 = 4;

float baseline = 0;
int pwm = 0;
float baselineMinMax[3];
float CAP_TOUCH = 0; 

/* ______FUNCTION TO INITIALIZE ARDUINO______ */
void setup()
{
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

  pinMode(vibPin1, OUTPUT);
  pinMode(vibPin2, OUTPUT);

  findStartVals();
  Serial.println(baseline);

  CAP_TOUCH = baselineMinMax[2] - baselineMinMax[1];
}



/* ______MAIN PROGRAM______ */
void loop()
{

  CDC_val = readValue();					// Read in capacitance value
  converted_val = CDC_val;				// Cast CDC_val as float

  // **Converts CDC value to capacitance (pF)**
  // CDC full-scale input range is +- 4.096 pF (|8.192| pF)
  // 		-4.096 pF --> 0x000000 --> 0
  //		0 pF      --> 0x800000 --> 8388608
  // 		+4.096 pF --> 0xFFFFFF --> 16777215
  //
  // 		{y = mx + b} -->  {y = (|C range in pF|/|equivalent CDC range|)x + (C val in pF when CDC val = 0)}
  // 		{y= (|8.192pF|/|16777215|)x -4.096pF}

  converted_val = ((converted_val / 16777215) * 8.192) - 4.096; // converted val is capacitance value

//  Serial.print(converted_val, 4);
//  Serial.print("\n");

  delay(15);								//Need a delay here or data will be transmitted out of order (or not at all)

  //  map capacitance to pwm (make the lowest possible vibration pwm 80):
  pwm = myMap(converted_val, baseline + CAP_TOUCH , baseline + CAP_TOUCH + CAP_RANGE, 80, 255);

  if (converted_val < baseline + CAP_TOUCH) { //if converted_val is smaller than what we consider a touch...
    pwm = 0; //make the pwm zero
  }
  else if (converted_val > baseline + CAP_TOUCH + CAP_RANGE) { //if converted_val is over the "highest pressure"...
    pwm = 255; //make the pwm 100
  }

  analogWrite(vibPin1, pwm); // write the new pwm to the pin
  Serial.println(pwm);

}

/* ______READ VALUE FUNCTION______
   This function checks the sensor's status reg until the data is ready and then reads it in
*/
long readValue()
{
  char status = 0;

  // **Waits until conversion is complete**
  //		_BV(0), _BV(2) = when either register is zero, indicates conversion is finished/new data is available
  while (!(status & (_BV(2) | _BV(0))))
  {
    status = readRegister(REGISTER_STATUS); 	// Wait for the next conversion
  }

  unsigned long value =  readLong(REGISTER_CAP_DATA);			//Size of unsigned long : 4 bytes (32bits)
  value >>= 8; 		//We have read one byte too many - now we have to get rid of it - sensor is 24-bit

  return value;

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
    baselineVals[i] = (((float)readValue() / 16777215) * 8.192) - 4.096;  // Read in capacitance value, cast as float (from long)
//    Serial.println(baselineVals[i]);
  }

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

  baselineMinMax[0] = average/NUM_READINGS; // put baseline value into array
  baselineMinMax[1] = minVal;  // put minVal into array
  baselineMinMax[2] = maxVal; // put maxVal into array

  baseline = baselineMinMax[0]; // can assign baseline directly because it's a global variable... 

//    Serial.println("average, baseline, Min, Max:");
//    Serial.println(average);
//    Serial.println(baseline);
//    Serial.println(minVal);
//    Serial.println(maxVal);
//    
    
    
  //find max value in the first 100 readings:

}

/*
   maps a number x from the range [in_min to in_max] and maps it into the range [out_min, out_max]
*/
int myMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}
