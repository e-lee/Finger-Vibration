#include <TimerOne.h>

// declare pin names
const int LEDPower = 5;
const int LEDGnd = 4;
const int ButtonPower = 7; 
const int ButtonGnd = 6; 
const int ButtonInput = 2;

volatile boolean isTimerStarted = true;

void setup() {
  // enable serial
  Serial.begin(9600);
    
  // declare pin modes
  pinMode(LEDPower, OUTPUT);
  pinMode(LEDGnd, OUTPUT);
  pinMode(ButtonPower, OUTPUT);
  pinMode(ButtonGnd, OUTPUT);
  pinMode(ButtonInput, INPUT);

  // initialize timer
  Timer1.initialize(100000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt(timerIsr); // attach the service routine here

  // initialize button interrupt 
  attachInterrupt(digitalPinToInterrupt(3), buttonIsr, FALLING);

  // initialize pin values
  digitalWrite(LEDGnd, LOW);
  digitalWrite(LEDPower, LOW);
  digitalWrite(ButtonPower, HIGH);
  digitalWrite(ButtonGnd, LOW);
  
  
}

void loop() {
  while(1) {
//  Serial.println(isTimerStarted);
  } 
}

void timerIsr() {
   if(digitalRead(LEDPower) == LOW) 
    digitalWrite(LEDPower, HIGH);
  else 
    digitalWrite(LEDPower, LOW);
}

void buttonIsr(){
  Serial.println("interrupt triggered");
  if(isTimerStarted == true) {
   Timer1.detachInterrupt();
    isTimerStarted = false; 
  }
  else {
    Timer1.attachInterrupt(timerIsr);
    isTimerStarted = true;
  }
}
