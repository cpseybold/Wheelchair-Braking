#include<Wire.h>

//tiltometer stuff
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double x;
double y;
double z;

// Encoder output to Arduino Interrupt pin. Tracks the pulse count, is a static value
//Right encoder pin is A
#define ENC_IN_RIGHT_A 2
#define motor1pin1 9
#define motor1pin2 10
#define motor2pin1 11
#define motor2pin2 12
//testing swtiches

// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;
int count = 0;
void setup() {
//more tiltometer stuff
Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(9600);



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

  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
delay(5000);

  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  delay(2000);
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor2pin1, LOW);
}

void loop() {
Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
AcX=Wire.read()<<8|Wire.read();
AcY=Wire.read()<<8|Wire.read();
AcZ=Wire.read()<<8|Wire.read();
int xAng = map(AcX,minVal,maxVal,-90,90);
int yAng = map(AcY,minVal,maxVal,-90,90);
int zAng = map(AcZ,minVal,maxVal,-90,90);
 
x= (RAD_TO_DEG * (atan2(-yAng, -zAng)+PI)-357.0112) * -1;
y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI)-6.5028;
z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI)-335.394;
Serial.println(x);
delay(1000);

  if(x>=3 && x<=60)
  {
// After 2 Rotations brakes will be applied
    if(right_wheel_pulse_count>36&&count==0)
    {
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor1pin2, HIGH);
      digitalWrite(motor2pin1, LOW);
      digitalWrite(motor2pin2, HIGH);
      delay(2000);
      digitalWrite(motor1pin2, LOW);
      digitalWrite(motor2pin2, LOW);
      count++;
    

    }
  }

}
// Increment the number of pulses by 1
void right_wheel_pulse(){

if(x>=3 && x<=60){  
  right_wheel_pulse_count++;
  //Serial.println(right_wheel_pulse_count);
}
}