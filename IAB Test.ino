// Encoder output to Arduino Interrupt pin. Tracks the pulse count, is a static value
#define ENC_IN_RIGHT_A 2
#define motor1pin1 9
#define motor1pin2 10
#define motor2pin1 11
#define motor2pin2 12
// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;
int count = 0;
void setup() {
// Open the serial port at 9600 bps, you use this to output info from arduino
  Serial.begin(9600); 
// Set pin states of the encoder and h bridge/actuator
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);\
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
// Every time the pin goes high, this is a pulse, this allows the pulses to be counted, gets reset after time period is over
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
//Releases break
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  delay(5000);
}

void loop() {
// After 2 Rotations brakes will be applied
  if(right_wheel_pulse_count>36&&count==0)
  {
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
    delay(5000);
    count++;
  }

}
// Increment the number of pulses by 1
void right_wheel_pulse(){
  right_wheel_pulse_count++;
}