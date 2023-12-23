#include <Servo.h>

const int enPin = 8;
const int stepXPin = 2; // X.STEP
const int dirXPin = 5;  // X.DIR

int stepPin = stepXPin;
int dirPin = dirXPin;

const int stepsPerRev = 200;
int pulseWidthMicros = 100; // microseconds

// Speed control
int initialSpeed = 2000;   // Initial speed in microseconds
int minSpeed = 500;        // Minimum speed in microseconds
int speedIncrement = 100;  // Speed increment in microseconds

int millisBtwnSteps = 10000; // Initial value

// Servo setup
Servo myservo;  // create servo object to control a servo
const int servoPin = A0;  // choose the pin for the servo

// Initial position (in steps)
int initialPosition = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  myservo.attach(servoPin); // attaches the servo on pin 9 to the servo object
}

void rotateMotor(int steps, int speed)
{
  int targetPosition = initialPosition + steps;

  for (int i = initialPosition; i < targetPosition; i++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speed);
  }

  initialPosition = targetPosition % stepsPerRev; // Update the initial position
}

void loop()
{
  unsigned long startTime = millis();
  int choice = 0;

  while (millis() - startTime < 5000 && Serial.available() == 0)
  {
    // Wait for user input or 5 seconds timeout
  }

  if (Serial.available() > 0)
  {
      String receivedMessage = Serial.readStringUntil('\n');

    // Process the received message accordingly


      if(receivedMessage == "metal"){
        //Serial.println(F("No rotation"));
        rotateMotor(0, 0);
        delay(5000); // Wait for 5 seconds

        // Servo rotation
        myservo.write(0);  // rotate the servo to 90 degrees
        delay(5000);  // wait for 5 seconds
        myservo.write(90);

        //Serial.println(F("No rotation"));
        rotateMotor(0, 0);
        //Serial.write("ResumeDetection");
        Serial.println("ResumeDetection");
      }
      else if(receivedMessage == "plastic"  || receivedMessage == "glass" ){
        //Serial.println(F("Rotating 120 degrees clockwise"));
        rotateMotor((stepsPerRev * 4) / 9, millisBtwnSteps); // 120 degrees
        delay(5000); // Wait for 5 seconds

        // Servo rotation
        myservo.write(0);  // rotate the servo to 90 degrees
        delay(5000);  // wait for 5 seconds
        myservo.write(90);

        //Serial.println(F("Rotating 120 degrees counterclockwise"));
        rotateMotor((stepsPerRev * 7) / 9, millisBtwnSteps); // Reverse 120 degrees
        //Serial.write("ResumeDetection");
        Serial.println("ResumeDetection");
      }
      else if(receivedMessage == "paper"){
        //Serial.println(F("Rotating 240 degrees clockwise"));
        rotateMotor((stepsPerRev * 7) / 9, millisBtwnSteps); // 240 degrees
        delay(5000); // Wait for 5 seconds

        // Servo rotation
        myservo.write(0);  // rotate the servo to 90 degrees
        delay(5000);  // wait for 5 seconds
        myservo.write(90);

        //Serial.println(F("Rotating 240 degrees counterclockwise"));
        rotateMotor((stepsPerRev * 4) / 9, millisBtwnSteps); // Reverse 240 degrees
        //Serial.write("ResumeDetection");
        Serial.println("ResumeDetection");
      }
      else{
        //Serial.println("Uauthorized Object");
        //Serial.write("ResumeDetection");
        Serial.println("ResumeDetection");
      }
  }
}
