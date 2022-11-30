//Declares variables used to declare the pinmode
#define motor1pin1 9
#define motor1pin2 10
#define motor2pin1 11
#define motor2pin2 12

void setup() {
  // Declares the pin mode of each pin
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
//Pulls the actuator in for 5 seconds
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  delay(5000);

}

void loop() {
  // Pushes Actuator out for 2 seconds  
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  delay(2000);

//Pulls the actuator in for 2 seconds
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  delay(2000);
}