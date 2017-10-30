#include <Servo.h>

Servo botServo;
Servo topServo;
int pot1 = A0;
int pot2 = A1;
int statusPin = 3;
int buttonPin = 13;
bool prevButtonState;
bool accelOn = true;
int serialByte = 0;
int pos = 0;
int topServoAngle = 90;
int botServoAngle = 80;


void setup() {
  Serial.begin(9600);
  botServo.attach(9);
  topServo.attach(6);
  pinMode(buttonPin, INPUT);
  pinMode(statusPin, OUTPUT);
}

void loop() {
  // 1. Button to toggle between accelerometer and potentiometers
  int buttonState = digitalRead(buttonPin);
  if (buttonState != prevButtonState && buttonState == HIGH) {
    accelOn = !accelOn;
  }
  prevButtonState = buttonState;

  if (accelOn) {
    digitalWrite(statusPin, LOW);
    
    topServo.write(topServoAngle);
    botServo.write(botServoAngle);

    delay(15);

    if (Serial.available() > 0) {
      serialByte = Serial.read();
      pos += 10;
      Serial.println(pos);
      topServo.write(pos);
    }

    
    
  } else {
    digitalWrite(statusPin, HIGH);

    int val1 = analogRead(pot1);
    int val2 = analogRead(pot2);
    
    Serial.print("val1 \t");
    Serial.print(val1);
    Serial.print("\tval2 \t");
    Serial.print(val2);
    
    int mappedVal1 = map(val1, 0, 1023, 0, 180);
    int mappedVal2 = map(val2, 0, 1023, 0, 180);
  
    Serial.print("\tmappedVal1 \t");
    Serial.print(mappedVal1);
    Serial.print("\tmappedVal2 \t");
    Serial.print(mappedVal2);
    
    botServo.write(mappedVal1);
    topServo.write(mappedVal2);
    delay(15); // waits for the servo to get there
  }
  
  Serial.println();
}

