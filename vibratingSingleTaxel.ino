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
#define CAP_RANGE 0.7
#define CAP_TOUCH 0.05


long CDC_val = 0;							// Unconverted value from CDC
float converted_val = 0; 					// Converted pF value

const int vibPin1=9; // make a #define?
const int vibPin2=4;

float baseline = 0;
int pwm = 0;

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

  pinMode(vibPin1,OUTPUT);
  pinMode(vibPin2,OUTPUT);
  
  baseline = findBaseline(); //find baseline cap value 
  Serial.println(baseline);
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

	Serial.print(converted_val, 4);
	Serial.print("\n");

	delay(15);								//Need a delay here or data will be transmitted out of order (or not at all)

  //map capacitance to pwm (make the lowest possible vibration pwm 80): 
//  pwm = map(converted_val, baseline + CAP_TOUCH , baseline + CAP_TOUCH + CAP_RANGE, 0, 255);
//
//  if(converted_val < baseline + CAP_TOUCH) { //if converted_val is smaller than what we consider a touch...
//      pwm = 0; //make the pwm zero
//    }
//    else if(converted_val > baseline + CAP_TOUCH + CAP_RANGE, 0, 255) { //if converted_val is over the "highest pressure"... 
//      pwm = 255; //make the pwm 100
//    }

// I tried making my own "parabolic map" function but it required me to solve a set of non-linear equations that 
// don't stay constant as the baseline cap value is always changing (so the a/b values in y = a(x-b)^2 were always changing too,
// and arduino can't solve nonlinear equations easily)

// so I'll try to square the analog values first, then use the map function (map the square of the read values): 

//map capacitance to pwm (make the lowest possible vibration pwm 80): 
  pwm = map(pow(converted_val,0.2), pow(baseline + CAP_TOUCH,0.2), pow(baseline + CAP_TOUCH + CAP_RANGE,0.2), 0, 255);

  if(converted_val < baseline + CAP_TOUCH) { //if converted_val is smaller than what we consider a touch...
      pwm = 0; //make the pwm zero
    }
    else if(converted_val > baseline + CAP_TOUCH + CAP_RANGE, 0, 255) { //if converted_val is over the "highest pressure"... 
      pwm = 255; //make the pwm 100
    }


   
  analogWrite(vibPin1, pwm);

  //if (converted_val>=0.9 && converted_val<=0.95) 
  //{
  //    analogWrite(vibPin1,80);
  //    //Touch
  //}
  //else if (converted_val>0.95 && converted_val<=1.5)
  //    {
  //    analogWrite(vibPin1,110);
  //    //Pressure
  //}
  //else if (converted_val>1.5)
  //{
  //    analogWrite(vibPin1,255);
  //    //Excessive Pressure
  //}
  //    else 
  // {
  //    analogWrite(vibPin1,0);
  //}
  //  

}


/* ______READ VALUE FUNCTION______
 * This function checks the sensor's status reg until the data is ready and then reads it in
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
 * baseline capacitance finding function 
 */

float findBaseline() 
{
  float baselineVals[100];
  float baseline = 0;
    
  // ** find baseline capacitance value by reading first 100 values (but omit first 10 values, are sometimes negative values): 
    for(int i = 0; i < 10; i++) {
      readValue(); //make sure this line actually executes
    }
    
    for(int i = 0; i < 100; i++) { 
        baselineVals[i] = (((float)readValue() / 16777215) * 8.192) - 4.096;  // Read in capacitance value, cast as float (from long)
    }
  
  // **average values in array to find "baseline" cap value  
    for(int i = 0; i < 100; i++) {
      baseline = baseline + baselineVals[i];
    }
    
    return baseline/100; //return 
}
