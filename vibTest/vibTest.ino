
int pwm; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(9, OUTPUT);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  pwm = 90;
  analogWrite(9, pwm); 
  delay(1000);
  pwm = 0;
  analogWrite(9, pwm);
  delay(1000);
}
