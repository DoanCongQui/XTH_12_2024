#include <Servo.h>

const int trigPin = A1;
const int echoPin = A0;
const int servoPin = 3;

long duration;
int distance;

Servo servo;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  servo.attach(servoPin);
  Serial.begin(9600);
}

void loop() {

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');

    // Turn Right
    if (command[0] == 'O') 
    {
      int angle = command.substring(1).toInt();
      servo.write(angle);
    } 

    else if (command[0] == 'S') 
    {
      servo.detach();
    }
  }

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  
  distance = duration * 0.034 / 2;
  Serial.println(distance);
  delay(100);
}
