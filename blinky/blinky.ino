#include <TimerOne.h>

// declare pin names
const int LEDPower = 5;
const int LEDGnd = 4;
const int ButtonPower = 7;
const int ButtonGnd = 6;
const int ButtonInput = 2;

volatile boolean isTimerStarted = false;

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
  Timer1.initialize(4000000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt(timerIsr); // attach the service routine here
//  Timer1.stop(); // keep the timer off at first

  // initialize button interrupt
  attachInterrupt(digitalPinToInterrupt(3), buttonIsr, FALLING);

  // initialize pin values
  digitalWrite(LEDGnd, LOW);
  digitalWrite(LEDPower, LOW);
  digitalWrite(ButtonPower, HIGH);
  digitalWrite(ButtonGnd, LOW);


}

void loop() {
  while (1) {
    //  Serial.println(isTimerStarted);
  }
}

void timerIsr() {
  Serial.println("timer interrupt triggered");
  digitalWrite(LEDPower, LOW);

}

void buttonIsr() {
  Serial.println("interrupt triggered");
  if (isTimerStarted == false) { // if timer has already started or finished counting, then we are pressing the button to reset it
//    Timer1.initialize(4000000);
//    Timer1.restart(); // start a new period
    isTimerStarted = true;
    digitalWrite(LEDPower, HIGH);
    for(int i = 0; i < 10; i++) {
    Serial.println("isTimerStarted is TRUE");
    }
  }
  else {
//    Timer1.stop(); // else, we are pressing it
    digitalWrite(LEDPower, LOW);
    isTimerStarted = false;
    Serial.println("isTimerStarted is FALSE");
  }
}
