#include <PID_v1.h>
// Motor encoder output pulses per 360 degree revolution, is a static value
#define ENC_COUNT_REV 18

// Encoder output to Arduino Interrupt pin. Tracks the pulse count, is a static value
#define ENC_IN_RIGHT_A 2
#define ENC_IN_LEFT_A 3

// Other encoder output to Arduino to keep track of wheel direction, is a static value
// Tracks the direction of rotation.
#define ENC_IN_RIGHT_B 4
#define ENC_IN_LEFT_B 8

//Actuator Pins
#define motor1pin1 9
#define motor1pin2 10
#define motor2pin1 11
#define motor2pin2 12

//All the time settings in milliseconds
int setting1 = 2000; 
int setting2 = 1800;
int setting3 = 1600;
int setting4 = 1400;
int setting5 = 1200;
int setting6 = 1000;
int setting7 = 800;
int setting8 = 600;
int setting9 = 400;
int setting10 = 0;
int setting [] = {setting1,setting2,setting3,setting4,setting5,setting6,setting7,setting8,setting9,setting10};
//PID Variables
double Setpoint, Input, Output;

//Min or max for time of actuating
double min = 1, max = 10;

//time check for PID (need to find out if it is in seconds or milliseconds)
int time_check = 500;
//Current Setting for actuator
int current_setting = 0;
//Time actuated
int actuator_time = 0;
//Previous PID output
int previousOutput = 10;
//Current PID output as an integer
int cOutput = 10;
//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
//PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// True = Forward; False = Reverse
boolean Direction_right = true;
boolean Direction_left = true;

// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;
volatile long left_wheel_pulse_count = 0;

// half second interval for measurements in milliseconds
int interval = 500;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
long previousMillisPID = 0;
long currentMillisPID = 0;

// Variable for RPM measuerment
float rpm_right = 0;
float rpm_left = 0;

//Constants for conversions
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;


void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600);

  //PID Target: Change to 3.5 mph in rpm
  Setpoint = 49.02;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(min, max);
  myPID.SetSampleTime(time_check);
  
  // Set pin states of the encoder
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);

  //Input Pullup measures the change in input a high vs low signal
  //This helps reduce the noise in the signal
  pinMode(ENC_IN_RIGHT_B, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT_PULLUP);

  // Set pin states of the encoder and h bridge/actuator
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  // Every time the pin goes high, this is a pulse, this allows the pulses to be counted, gets reset after time period is over
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_pulse, RISING);
  //Pull in  
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  delay(10000);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin2, LOW);
  //Push Out
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  delay(2000);
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor2pin1, LOW);

}

void loop() 
{
  // put your main code here, to run repeatedly:
  currentMillis = millis();
  currentMillisPID = millis();

  // If one second has passed it prints the number of pulses
  if (currentMillis - previousMillis > interval) 
  {

    previousMillis = currentMillis;

    // Calculate rpm
    rpm_right = (float)(right_wheel_pulse_count * 120 / ENC_COUNT_REV);
    rpm_left = (float)(left_wheel_pulse_count * 120 / ENC_COUNT_REV);
    right_wheel_pulse_count = 0;
    left_wheel_pulse_count = 0;
  }

  // Sets Input to average of two rpms
  Input = rpm_right;
  previousMillisPID = currentMillisPID;
  //PID computes
  myPID.Compute();
  cOutput = (int)Output;
  if(cOutput<previousOutput)
  {
    
  //Brake    
    actuator_time = setting[cOutput-1] - current_setting;
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
    //Delay
    delay(actuator_time);
    //Stops signal to actuator
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin2, LOW);
  }
  if(cOutput>previousOutput)
  {
    //Ease
    actuator_time = current_setting-setting[cOutput-1];
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
    //Delay
    delay(actuator_time);
    //Stops signal to actuator
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor2pin1, LOW);      
  }
  current_setting=setting[cOutput-1];
  previousOutput=cOutput;
  Serial.println(actuator_time);
}
void right_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
    right_wheel_pulse_count++;
  }

}
void left_wheel_pulse() {

      // Read the value for the encoder for the right wheel
      int val = digitalRead(ENC_IN_LEFT_B);

      if (val == LOW) {
        Direction_left = true;
        // Reverse
      } else {
        Direction_left = false;
        // Forward
      }

      if (Direction_left) {
        left_wheel_pulse_count++;
      }
}
void brake()
{
  actuator_time = setting[cOutput-1] - current_setting;
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
    //Delay
    delay(actuator_time);
    //Stops signal to actuator
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin2, LOW);

}
void ease()
{
  actuator_time = current_setting-setting[cOutput-1];
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
    //Delay
    delay(actuator_time);
    //Stops signal to actuator
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor2pin1, LOW);  
}
