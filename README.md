# Finger-Vibration

This is a quick guide on the different folders of code in this repo.

## Most Recent Codes

**vibratingSingleTaxel_combinedServo:** the most recent version of the main (transmitter) code. Optionally has servo/vibration output, and
also optionally acts as a transmitter/receiver. 

**vibratingSingleTaxel_combined_receiverServo:** Wirelessly receives vibration/servo data and outputs it on an analog pin.  Pairs with
vibratingSingleTaxel_combinedServo.

## Previous iterations of vibratingSingleTaxel_transmitterServo

the following codes listed represent previous iterations of vibratingSingleTaxel_combinedServo:

**vibtest:** Linearly maps capacitance to a pwm output and transmits it to another microcontroller. Uses the tmrh20 RF24 library.

**vibratingSingleTaxel:** Capacitance is mapped linearly/segmented into a pwm (but you must comment out the appropriate parts 
of the code to select the desired mapping).  Has fadeaway funciton, but it is not the latest version

**vibratingSingleTaxel_combined:** Capacitance is mapped linearly/segmented into a pwm (but you must comment out the appropriate parts 
of the code to select the desired mapping), and has the option to transmit this code to another microcontroller (i.e. vibratingSingleTaxel *combined* with the transmitter code). Has fadeaway funciton, but it is not the latest version

**vibratingSingleTaxel_combined_receiver:** Wirelessly receives vibration data and outputs it on an analog pin.  Pairs with 
vibratingSingleTaxel_combined.  

**vibratingSingleTaxel_transmitter:** outputs pwm mapped in segments from capacitance, and either outputs it on a pin (must comment/
uncomment code) or transmits it to another microcontroller (uses the tmrh20 library).  Has fadeaway function, but it is not the latest version.  

**vibratingSingleTaxel_reciever:** Wirelessly receives vibration data and outputs it on an analog pin.  Pairs with 
vibratingSingleTaxel_combined.  

## Test Codes

**blinky:** an unfinished, initial attempt to figure out how to use the TimerOne library in anticipation of using TimerOne in the 
main transmitter code

**newWireless:** almost a copy/paste of the ManiacBug GettingStarted example for the RF24 library.  Simply sends/receives data between 
two microcontrollers using the nrf24l01+ radio modules. *This is the radio code that is used inside the main codes. 

**newrflibrary:** an rf library I tried to use when the tmrh20 versions and the ManiacBug versions weren't working.  Never got it working. (This code is not included anywhere in the main codes)

**pintest:** Simply writes HIGH to a specified pin. Previously used to check if the pins on the microcontroller could fluctuate between 
HIGH/LOW properly (to see if its 5V logic was still working after soldering).  

**pwmtest:** Simply writes a specified pwm to a pin. Previously used to debug an old issue where analogWrites to a few analogpins only
outputted LOW or HIGH instead of a pwm. 

**wireless:** The first RF24 library I tried using for the nrf24l01+ modules.  This is the tmrh20 version of the library (this is not
the radio code that is currently used in the main code, however, this library is supposed to be an improved version of the 
ManiacBug version).

**segmented_fadeaway:** a file used to test the segmented fadeaway in isolation from the main code.
