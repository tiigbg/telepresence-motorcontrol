#include <ScanPacket.h>
#include <Sweep.h>

#include <Servo.h>

// this tracks how far away an obstacle is detected
// in 8 directions.
// directions are (in order): forward, forward right, right, backward right,
// backward, backward left, left, forward left
#define MAX_DISTANSE 999
int obstacleDirections[] = {MAX_DISTANSE, MAX_DISTANSE, MAX_DISTANSE, MAX_DISTANSE, MAX_DISTANSE, MAX_DISTANSE, MAX_DISTANSE, MAX_DISTANSE};
#define DIR_FRONT 0
#define DIR_FRONT_RIGHT 1
#define DIR_RIGHT 2
#define DIR_BACK_RIGHT 3
#define DIR_BACK 4
#define DIR_BACK_LEFT 5
#define DIR_LEFT 6
#define DIR_FRONT_LEFT 7

int ledPin = 13;
int bumperPinFR = 24;
int bumperPinRF = 25;
int bumperPinRB = 26;
int bumperPinBR = 27;
int bumperPinBL = 28;
int bumperPinLB = 29;
int bumperPinLF = 30;
int bumperPinFL = 31;
// pin 32 is unused but available

int bumperValueFR, bumperValueRF, bumperValueRB, bumperValueBR, bumperValueBL, bumperValueLB, bumperValueLF, bumperValueFL;

// pitch is looking up/down
int servoPitchPin = 2;
// yaw is looking left/right
int servoYawPin = 3;
// height is lifting the stick up/down
int servoHeightPin = 4;

Servo servoPitch, servoYaw, servoHeight;

void setup() {                
  pinMode(ledPin, OUTPUT);

  pinMode(bumperPinFR, INPUT_PULLUP);
  pinMode(bumperPinRF, INPUT_PULLUP);
  pinMode(bumperPinRB, INPUT_PULLUP);
  pinMode(bumperPinBR, INPUT_PULLUP);
  pinMode(bumperPinBL, INPUT_PULLUP);
  pinMode(bumperPinLB, INPUT_PULLUP);
  pinMode(bumperPinLF, INPUT_PULLUP);
  pinMode(bumperPinFL, INPUT_PULLUP);

  servoPitch.attach(servoPitchPin);
  servoYaw.attach(servoYawPin);
  servoHeight.attach(servoHeightPin);
  
  Serial.begin(115200);
  Serial1.begin(115200);

  
  
  Serial.println("Teensy started ok");
  blink(200);
  blink(200);
  blink(200);
}

void loop() {
  /*
  while(Serial1.available() > 0)
  {
    Serial.print(char(Serial1.read()));
  }
  */

  // reset readings
  for(int i = 0; i < 8; i++) {
    obstacleDirections[i] = MAX_DISTANSE;
  }

  readBumpers();
  readPingSensors();

  Serial.print("{");
  Serial1.print("{");
  for(int i = 0; i < 8; i++) {
    Serial.print(obstacleDirections[i]);
    Serial1.print(obstacleDirections[i]);
  
    if(i<7)
    {
      Serial.print(", ");
      Serial1.print(", ");
    }
  }
  Serial.println("}");
  Serial1.println("}");

  blink(50);
}

void readPingSensors() {
  // TODO implement
}

void readBumpers() {
  bumperValueFR = digitalRead(bumperPinFR);
  bumperValueRF = digitalRead(bumperPinRF);
  bumperValueRB = digitalRead(bumperPinRB);
  bumperValueBR = digitalRead(bumperPinBR);
  bumperValueBL = digitalRead(bumperPinBL);
  bumperValueLB = digitalRead(bumperPinLB);
  bumperValueLF = digitalRead(bumperPinLF);
  bumperValueFL = digitalRead(bumperPinFL);

  if(bumperValueFL || bumperValueFR)
  {
    obstacleDirections[DIR_FRONT] = 0;
  }
  if(bumperValueFR || bumperValueRF)
  {
    obstacleDirections[DIR_FRONT_RIGHT] = 0;
  }
  if(bumperValueRF || bumperValueRB)
  {
    obstacleDirections[DIR_RIGHT] = 0;
  }
  if(bumperValueRB || bumperValueBR)
  {
    obstacleDirections[DIR_BACK_RIGHT] = 0;
  }
  if(bumperValueBR || bumperValueBL)
  {
    obstacleDirections[DIR_BACK] = 0;
  }
  if(bumperValueBL || bumperValueLB)
  {
    obstacleDirections[DIR_BACK_LEFT] = 0;
  }
  if(bumperValueLB || bumperValueLF)
  {
    obstacleDirections[DIR_LEFT] = 0;
  }
  if(bumperValueLF || bumperValueFL)
  {
    obstacleDirections[DIR_FRONT_LEFT] = 0;
  }
}

void blink(int delayTime) {
  digitalWrite(ledPin, HIGH);
  delay(delayTime);
  digitalWrite(ledPin, LOW);
  delay(delayTime);
}

