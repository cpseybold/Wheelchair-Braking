// this constant won't change:


const int buttonPin1 = 5;
const int buttonPin2 = 6;
const int LEDPin = 13;

// Variables will change:
int buttonState1 = 0;        // current state of the button
int lastButtonState1 = 0; 
int buttonState2 = 0;        // current state of the button
int lastButtonState2 = 0; 


int i = 0;
int k = 0;   // previous state of the button
int l = 0;
int m = 0;   // previous state of the button

void setup() {
  // initialize the button pin as a input:
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(LEDPin, OUTPUT);
  Serial.begin(9600);
  lastButtonState1 = digitalRead(buttonPin1);
  lastButtonState2 = digitalRead(buttonPin2);
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
      digitalWrite(LEDPin, HIGH);
      delay(1000);
      digitalWrite(LEDPin, LOW);
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
      digitalWrite(LEDPin, HIGH);
      delay(1000);
      digitalWrite(LEDPin, LOW);
      m++;
    }


  
}
