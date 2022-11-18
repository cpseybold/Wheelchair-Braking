// Motor encoder output pulses per 360 degree revolution, is a static value
#define ENC_COUNT_REV 18
 
// Encoder output to Arduino Interrupt pin. Tracks the pulse count, is a static value
#define ENC_IN_RIGHT_A 2
#define ENC_IN_LEFT_A 3
// Other encoder output to Arduino to keep track of wheel direction, is a static value
// Tracks the direction of rotation.
#define ENC_IN_RIGHT_B 4
#define ENC_IN_LEFT_B 8


 
// True = Forward; False = Reverse
boolean Direction_right = true;
boolean Direction_left = true;
// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;
volatile long left_wheel_pulse_count = 0; 
// half second interval for measurements in milliseconds
int interval = 500;
const int divider = 60/(interval/1000);
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
 
// Variable for RPM measuerment
float rpm_right = 0;
float rpm_left = 0;
 //Constants for conversions
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;
 
void setup() {
 
  // Open the serial port at 9600 bps, you use this to output info from arduino
  Serial.begin(9600); 
 
  // Set pin states of the encoder
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  //Input Pullup measures the change in input a high vs low signal
  //This helps reduce the noise in the signal
  pinMode(ENC_IN_RIGHT_B , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT_PULLUP);
  // Every time the pin goes high, this is a pulse, this allows the pulses to be counted, gets reset after time period is over
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
  
}
 
void loop() {
 
  // time variable
  currentMillis = millis();
 
  // If one second has passed it prints the number of pulses
  if (currentMillis - previousMillis > interval) {
 
    previousMillis = currentMillis;
 
    // Calculate rpm
    rpm_right = (float)(right_wheel_pulse_count * 120 / ENC_COUNT_REV);
    rpm_left = (float)(left_wheel_pulse_count * 120 / ENC_COUNT_REV);
    //ang_velocity_right = rpm_right * rpm_to_radians;   
    //ang_velocity_right_deg = ang_velocity_right * rad_to_deg;
    Serial.print(" Right Pulses: ");
    Serial.println(right_wheel_pulse_count);
    Serial.print(" Right Speed: ");
    Serial.print(rpm_right);
    Serial.println(" RPM");
    
    Serial.print(" Left Pulses: ");
    Serial.println(left_wheel_pulse_count);
    Serial.print(" Left Speed: ");
    Serial.print(rpm_left);
    Serial.println(" Left RPM");
    
    right_wheel_pulse_count = 0;
    left_wheel_pulse_count = 0;
   
  }
}
 
// Increment the number of pulses by 1
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
 /* else {
    right_wheel_pulse_count--;
  }*/
}
void left_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel

  if(val == LOW) {
    Direction_left = true;
    // Reverse
  }
 else {
     Direction_left = false;
     // Forward
  }
   
  if (Direction_left) {
    left_wheel_pulse_count++;
  }
  /*else {
    right_wheel_pulse_count--;
  }
  */
}