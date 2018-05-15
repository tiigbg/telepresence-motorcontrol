#include "MeOrion.h"
#include <Wire.h>
#include <SoftwareSerial.h>

MeEncoderNew motor1(0x09, SLOT1); // front right
MeEncoderNew motor2(0x09, SLOT2); // front left
MeEncoderNew motor3(0x0a, SLOT1); // back right
MeEncoderNew motor4(0x0a, SLOT2); // back left

#define MAXSPEED 150
#define DEFAULTSPEED 100
#define COMMAND_TIMEOUT 1000

int moveSpeed = DEFAULTSPEED,motor1speed=0,motor2speed=0,motor3speed=0,motor4speed=0;
String currentDirection = "stop";
boolean motorsEnabled = false;
unsigned long lastCommandTime;

// This array is to set motor's direction.
// Change the symbol to change the motor's direction
signed char directionAdjustment[4]={1,-1,1,-1};

void setup()
{
  motor1.begin();
  //  motor2.begin(); // not sure why this is commented out,
                      // maybe because it is only about starting the motor board?
  motor3.begin();
  //  motor4.begin();
  delay(10);
  adjustDirections();
  motor1.runSpeed(0,1);
  motor2.runSpeed(0,1);
  motor3.runSpeed(0,1);
  motor4.runSpeed(0,1);
  Serial.begin(115200);
  Serial.println("Makeblock started ok");


}



void loop()
{
  
  while(Serial.available() > 0)
  {
    //String serialInput = Serial.readString();
    int incomingByte = Serial.read();
    Serial.println("---");
    Serial.print("Received:");
    Serial.println(incomingByte, DEC);
    lastCommandTime = millis();
    if(incomingByte == 112) // p
    {
      /*int bufferLength = 20;
      byte buff[bufferLength];
      int nrOfBytes = Serial.readBytesUntil('p', buff, bufferLength);
      Serial.println("Received buffer:");
      for(int i = 0; i<bufferLength; i++)
      {
        Serial.print(" ");
        Serial.print(buff[i]);
      }
      Serial.println();
      */
      String incomingString = "";
      while(Serial.available() > 0)
      {
        incomingByte = Serial.read();
        incomingString += (char)incomingByte;
        delay(1);
      }
      Serial.println(incomingString);
      
      //incomingString.substring(0, incomingString.indexOf(';'));
      
      calculateSpeeds(0, 1, 0);
      
    }
    else if(incomingByte == 119) // w
    {
      calculateSpeeds(0, 1, 0);
      //forward();
    }
    else if(incomingByte == 115) // s
    {
      calculateSpeeds(0, -1, 0);
      //backward();
    }
    else if(incomingByte == 100) // d
    {
      calculateSpeeds(0, 0, 1);
      //right();
    }
    else if(incomingByte == 97) // a
    {
      calculateSpeeds(0, 0, -1);
      //left();
    }
    else if(incomingByte == 101) // e
    {
      rightUp();
    }
    else if(incomingByte == 113) // q
    {
      leftUp();
    }
    else if(incomingByte == 99) // c
    {
      rightDown();
    }
    else if(incomingByte == 122) // z
    {
      leftDown();
    }
    else if(incomingByte == 116) // t
    {
      calculateSpeeds(PI*1.75, 1, 0);
      //strafeRightUp();
    }
    else if(incomingByte == 114) // r
    {
      calculateSpeeds(PI*0.25, 1, 0);
      //strafeLeftUp();
    }
    else if(incomingByte == 103) // g
    {
      calculateSpeeds(PI*1.5, 1, 0);
      //strafeRight();
    }
    else if(incomingByte == 102) // f
    {
      calculateSpeeds(PI*0.5, 1, 0);
      //strafeLeft();
    }
    else if(incomingByte == 98) // b
    {
      calculateSpeeds(PI*1.25, 1, 0);
      //strafeRightDown();
    }
    else if(incomingByte == 118) // v
    {
      calculateSpeeds(PI*0.75, 1, 0);
      //strafeLeftDown();
    }
    else if(incomingByte == 49) // 1
    {
      motorsEnabled = false;
      Serial.print("motorsEnabled:");
      Serial.println(motorsEnabled);
    }
    else if(incomingByte == 50) // 2
    {
      motorsEnabled = true;
      Serial.print("motorsEnabled:");
      Serial.println(motorsEnabled);
    }
    else
    {
      stopAll();
    }
    Serial.println(currentDirection);
  }

  if(millis() > lastCommandTime + COMMAND_TIMEOUT)
  {
    stopAll();
  }
}

void adjustDirections()
{
  motor1speed = moveSpeed*directionAdjustment[0];
  motor2speed = moveSpeed*directionAdjustment[1];
  motor3speed = moveSpeed*directionAdjustment[2];
  motor4speed = moveSpeed*directionAdjustment[3];
}


void forward()
{
  currentDirection = "forward";
  motor1.runSpeed(motor1speed,0);
  motor2.runSpeed(motor2speed,0);
  motor3.runSpeed(motor3speed,0);
  motor4.runSpeed(motor4speed,0);
 }

 void backward()
{
  currentDirection = "backward";
  motor1.runSpeed(-motor1speed,0);
  motor2.runSpeed(-motor2speed,0);
  motor3.runSpeed(-motor3speed,0);
  motor4.runSpeed(-motor4speed,0);
 }

 void right()
{
  currentDirection = "right";
  motor1.runSpeed(-motor1speed,0);
  motor2.runSpeed(motor2speed,0);
  motor3.runSpeed(-motor3speed,0);
  motor4.runSpeed(motor4speed,0);
 }

 void left()
{
  motor1.runSpeed(motor1speed,0);
  motor2.runSpeed(-motor2speed,0);
  motor3.runSpeed(motor3speed,0);
  motor4.runSpeed(-motor4speed,0);
 }
 
void leftUp()
{
  currentDirection = "leftUp";
  motor1.runSpeed(motor1speed,0);
  motor2.runSpeed(0,0);
  motor3.runSpeed(motor3speed,0);
  motor4.runSpeed(0,0);
}

void leftDown()
{
  currentDirection = "leftDown";
  motor1.runSpeed(0,0);
  motor2.runSpeed(-motor2speed,0);
  motor3.runSpeed(0,0);
  motor4.runSpeed(-motor4speed,0);
 
}

void rightUp()
{
  currentDirection = "rightUp";
  motor1.runSpeed(0,0);
  motor2.runSpeed(motor2speed,0);
  motor3.runSpeed(0,0);
  motor4.runSpeed(motor4speed,0);
}

void rightDown()
{
  currentDirection = "rightDown";
  motor1.runSpeed(-motor1speed,0);
  motor2.runSpeed(0,0);
  motor3.runSpeed(-motor3speed,0);
  motor4.runSpeed(0,0);
}

void turnRight()
{
  currentDirection = "turnRight";
  motor1.runSpeed(-motor1speed,0);
  motor2.runSpeed(motor2speed,0);
  motor3.runSpeed(motor3speed,0);
  motor4.runSpeed(-motor4speed,0);
}

void turnLeft()
{
  currentDirection = "turnLeft";
  motor1.runSpeed(motor1speed,0);
  motor2.runSpeed(-motor2speed,0);
  motor3.runSpeed(-motor3speed,0);
  motor4.runSpeed(motor4speed,0);
}

void strafeRight()
{
  currentDirection = "strafeRight";
  motor1.runSpeed(-motor1speed,0);
  motor2.runSpeed(motor2speed,0);
  motor3.runSpeed(motor3speed,0);
  motor4.runSpeed(-motor4speed,0);
}

void strafeLeft()
{
  currentDirection = "strafeLeft";
  motor1.runSpeed(motor1speed,0);
  motor2.runSpeed(-motor2speed,0);
  motor3.runSpeed(-motor3speed,0);
  motor4.runSpeed(motor4speed,0);
}

void strafeRightUp()
{
  currentDirection = "strafeRightUp";
  //motor1.runSpeed(-motor1speed/2,0);
  motor2.runSpeed(motor2speed,0);
  motor3.runSpeed(motor3speed,0);
  //motor4.runSpeed(-motor4speed/2,0);
}

void strafeRightDown()
{
  currentDirection = "strafeRightDown";
  motor1.runSpeed(-motor1speed,0);
  //motor2.runSpeed(motor2speed/2,0);
  //motor3.runSpeed(motor3speed/2,0);
  motor4.runSpeed(-motor4speed,0);
}

void strafeLeftUp()
{
  currentDirection = "strafeLeft";
  motor1.runSpeed(motor1speed,0);
  //motor2.runSpeed(-motor2speed/2,0);
  //motor3.runSpeed(-motor3speed/2,0);
  motor4.runSpeed(motor4speed,0);
}

void strafeLeftDown()
{
  //currentDirection = "strafeLeft";
  motor1.runSpeed(motor1speed/2,0);
  motor2.runSpeed(-motor2speed,0);
  motor3.runSpeed(-motor3speed,0);
  //motor4.runSpeed(motor4speed/2,0);
}



void speedUp()
{
  moveSpeed +=2;
  if(moveSpeed>MAXSPEED)  moveSpeed=MAXSPEED;

  adjustDirections();
}
void speedDown()
{
   moveSpeed -= 2;
  if(moveSpeed<0)  moveSpeed=0;

  adjustDirections();
}
void stopAll()
{
  currentDirection = "stop";
  motor1.runSpeed(0,1);
  motor2.runSpeed(0,1);
  motor3.runSpeed(0,1);
  motor4.runSpeed(0,1);
}

// angle to translate at: [0, 2pi]
// speed: [-1,1]
// rotation: [-1,1]
void calculateSpeeds(float myAngle, float mySpeed, float myRotation)
{
  // flipping direction on the rotation, because we want right turn to be 1 and left turn -1
  myRotation = myRotation * -1;
  float motor1multiplier = mySpeed * sin(myAngle + PI / 4.0) + myRotation;
  float motor2multiplier = mySpeed * cos(myAngle + PI / 4.0) - myRotation;
  float motor3multiplier = mySpeed * cos(myAngle + PI / 4.0) + myRotation;
  float motor4multiplier = mySpeed * sin(myAngle + PI / 4.0) - myRotation;

  // scale all multipliers based on the biggest, to keep them inside the [1,-1] range
  float maxMultiplier = max(fabs(motor1multiplier),
                         max(fabs(motor2multiplier),
                          max(fabs(motor3multiplier),
                           fabs(motor4multiplier))));
  if(maxMultiplier != 0)
  {
    Serial.print("Scaling multipliers with:");
    Serial.println(maxMultiplier);
    motor1multiplier = motor1multiplier / maxMultiplier;
    motor2multiplier = motor2multiplier / maxMultiplier;
    motor3multiplier = motor3multiplier / maxMultiplier;
    motor4multiplier = motor4multiplier / maxMultiplier;
  }
  
  motor1speed = directionAdjustment[0] * moveSpeed * motor1multiplier;
  motor2speed = directionAdjustment[1] * moveSpeed * motor2multiplier;
  motor3speed = directionAdjustment[2] * moveSpeed * motor3multiplier;
  motor4speed = directionAdjustment[3] * moveSpeed * motor4multiplier;

  Serial.print("m1m:");
  Serial.print(motor1multiplier);
  Serial.print(" m2m:");
  Serial.print(motor2multiplier);
  Serial.print(" m3m:");
  Serial.print(motor3multiplier);
  Serial.print(" m4m:");
  Serial.println(motor4multiplier);
  
  Serial.print("m1s:");
  Serial.print(motor1speed);
  Serial.print(" m2s:");
  Serial.print(motor2speed);
  Serial.print(" m3s:");
  Serial.print(motor3speed);
  Serial.print(" m4s:");
  Serial.println(motor4speed);
  if(motorsEnabled)
  {
    motor1.runSpeed(motor1speed,0);
    motor2.runSpeed(motor2speed,0);
    motor3.runSpeed(motor3speed,0);
    motor4.runSpeed(motor4speed,0);
  }
}

