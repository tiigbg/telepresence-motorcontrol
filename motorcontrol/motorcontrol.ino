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
  Serial.begin(9600);
  Serial.println("Makeblock started ok");


}



void loop()
{
  while(Serial.available() > 0)
  {
    //String serialInput = Serial.readString();
    int incomingByte = Serial.read();
    Serial.println("Received:");
    Serial.println(incomingByte, DEC);
    lastCommandTime = millis();
    if(incomingByte == 119) // w
    {
      forward();
    }
    else if(incomingByte == 115) // s
    {
      backward();
    }
    else if(incomingByte == 100) // d
    {
      right();
    }
    else if(incomingByte == 97) // a
    {
      left();
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
      strafeRightUp();
    }
    else if(incomingByte == 114) // r
    {
      strafeLeftUp();
    }
    else if(incomingByte == 103) // g
    {
      strafeRight();
    }
    else if(incomingByte == 102) // f
    {
      strafeLeft();
    }
    else if(incomingByte == 98) // b
    {
      strafeRightDown();
    }
    else if(incomingByte == 118) // v
    {
      strafeLeftDown();
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

