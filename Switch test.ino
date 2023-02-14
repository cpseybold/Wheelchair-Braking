// this constant won't change:


const int buttonPin1 = 5;
const int buttonPin2 = 6;

// Variables will change:
int buttonState1 = 0;        // current state of the button
int lastButtonState1 = 0; 
int buttonState2 = 0;        // current state of the button
int lastButtonState2 = 0; 

#define motor1pin1 9
#define motor1pin2 10
#define motor2pin1 11
#define motor2pin2 12


int i = 0;
int k = 0;   // previous state of the button
int l = 0;
int m = 0;   // previous state of the button

void setup() {
  // initialize the button pin as a input:
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  Serial.begin(9600);
  lastButtonState1 = digitalRead(buttonPin1);
  lastButtonState2 = digitalRead(buttonPin2);

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  }


void loop() {
  // read the pushbutton input pin:
  buttonState1 = digitalRead(buttonPin1);

  // compare the buttonState to its previous state
  if (buttonState1 != lastButtonState1) {
    i++;
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState1 = buttonState1;

  if(i != k)
    {
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor1pin2, HIGH);
      digitalWrite(motor2pin1, LOW);
      digitalWrite(motor2pin2, HIGH);
      delay(2000);
      digitalWrite(motor1pin2, LOW);
      digitalWrite(motor2pin2, LOW);
      k++;
    }
    
  buttonState2 = digitalRead(buttonPin2);

  // compare the buttonState to its previous state
  if (buttonState2 != lastButtonState2) {
    l++;
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState2 = buttonState2;

  if(l != m)
    {
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor1pin2, HIGH);
      digitalWrite(motor2pin1, LOW);
      digitalWrite(motor2pin2, HIGH);
      delay(5000);
      m++;
    }


  
}
