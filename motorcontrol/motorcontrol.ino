#include "MeOrion.h"
#include <Wire.h>
#include <SoftwareSerial.h>

MeEncoderNew motor1(0x09, SLOT1); // front right
MeEncoderNew motor2(0x09, SLOT2); // front left
MeEncoderNew motor3(0x0a, SLOT1); // back right
MeEncoderNew motor4(0x0a, SLOT2); // back left

#define MAXSPEED 150
#define DEFAULTSPEED 50
#define COMMAND_TIMEOUT 300

int moveSpeed = DEFAULTSPEED,motor1speed=0,motor2speed=0,motor3speed=0,motor4speed=0;
boolean motorsEnabled = true;
unsigned long lastCommandTime;

// This array is to set motor's direction.
// Change the symbol to change the motor's direction
signed char directionAdjustment[4]={1,-1,1,-1};

// Servo's

MePort port(PORT_3);
Servo servo1;  // create servo object to control a servo 
Servo servo2;  // create servo object to control another servo
int16_t servo1pin =  port.pin1();//attaches the servo on PORT_3 SLOT1 to the servo object
int16_t servo2pin =  port.pin2();//attaches the servo on PORT_3 SLOT2 to the servo object

int servo1Center = 90;
int servo1Min = 20;
int servo1Max = 160;
int servo2Center = 90;
int servo2Min = 20;
int servo2Max = 160;

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

  servo1.attach(servo1pin);  // attaches the servo on servopin1
  servo2.attach(servo2pin);  // attaches the servo on servopin2

  servo1.write(servo1Center);
  servo2.write(servo2Center);
  
  Serial.begin(115200);
  Serial.println("Makeblock started ok");
}



void loop()
{
  
  if(Serial.available() > 0)
  {
    String incomingString = "";
    while(Serial.available() > 0)
    {
      incomingString += (char)Serial.read();
      delay(1);
    }
    Serial.print("Received string:");
    Serial.println(incomingString);

    int beginCharIndex = incomingString.indexOf('<');
    int endCharIndex = incomingString.indexOf('>');
    if(beginCharIndex == -1 || endCharIndex == -1 || beginCharIndex >= endCharIndex)
    {
      Serial.print("<!> Ignored incoming serial due to wrong formatting of start and end characters. beginCharIndex=");
      Serial.print(beginCharIndex);
      Serial.print(" endCharIndex=");
      Serial.println(endCharIndex);
      return;
    }
    // trim off begin and end characters
    incomingString = incomingString.substring(beginCharIndex+1, endCharIndex);

    if(incomingString.substring(0,1) == "s")
    {
      Serial.println("This is a servo only command.");
      incomingString = incomingString.substring(1);
      int firstBreakCharIndex = incomingString.indexOf(';');
      if(firstBreakCharIndex == -1)
      {
        Serial.println("<!> Did not find first break character (;).");
        return;
      }
      int servo1Value = incomingString.substring(0, firstBreakCharIndex).toInt();
      int servo2Value = incomingString.substring(firstBreakCharIndex+1).toInt();

      servo1Value = min(max(servo1Value, servo1Min), servo1Max);
      servo2Value = min(max(servo2Value, servo2Min), servo2Max);
      Serial.print(servo1Value);
      Serial.print(" ");
      Serial.println(servo2Value);
      
      servo1.write(servo1Value);
      servo2.write(servo2Value);
      
      
      return;
    }
    
    int firstBreakCharIndex = incomingString.indexOf(';');
    if(firstBreakCharIndex == -1)
    {
      Serial.println("<!> Did not find first break character (;).");
      return;
    }
    int secondBreakCharIndex = incomingString.indexOf(';', firstBreakCharIndex+1);
    if(secondBreakCharIndex == -1)
    {
      Serial.println("<!> Did not find second break character (;).");
      return;
    }
    String firstValue = incomingString.substring(0, firstBreakCharIndex);
    String secondValue = incomingString.substring(firstBreakCharIndex+1, secondBreakCharIndex);
    String thirdValue = incomingString.substring(secondBreakCharIndex+1);

    Serial.print("Parsed: ");
    Serial.print(firstValue);
    Serial.print(" // ");
    Serial.print(secondValue);
    Serial.print(" // ");
    Serial.println(thirdValue);

    float newDirection = firstValue.toFloat();
    float newSpeed = secondValue.toFloat();
    float newRotation = thirdValue.toFloat();

    Serial.print("Floats: ");
    Serial.print(newDirection);
    Serial.print(" // ");
    Serial.print(newSpeed);
    Serial.print(" // ");
    Serial.println(newRotation);

    // make sure arguments are within the boundaries
    newDirection = fmod((fmod(newDirection,float(2.0*PI)) + 2.0*PI),float(2.0*PI));
    newSpeed = max(min(newSpeed, 1), -1);
    newRotation = max(min(newRotation, 1), -1);

    Serial.print("Floats: ");
    Serial.print(newDirection);
    Serial.print(" // ");
    Serial.print(newSpeed);
    Serial.print(" // ");
    Serial.println(newRotation);

    lastCommandTime = millis();
    
    calculateSpeeds(newDirection, newSpeed, newRotation);
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

/*
void forward()
{
  motor1.runSpeed(motor1speed,0);
  motor2.runSpeed(motor2speed,0);
  motor3.runSpeed(motor3speed,0);
  motor4.runSpeed(motor4speed,0);
 }

 void backward()
{
  motor1.runSpeed(-motor1speed,0);
  motor2.runSpeed(-motor2speed,0);
  motor3.runSpeed(-motor3speed,0);
  motor4.runSpeed(-motor4speed,0);
 }

 void right()
{
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
  motor1.runSpeed(motor1speed,0);
  motor2.runSpeed(0,0);
  motor3.runSpeed(motor3speed,0);
  motor4.runSpeed(0,0);
}

void leftDown()
{
  motor1.runSpeed(0,0);
  motor2.runSpeed(-motor2speed,0);
  motor3.runSpeed(0,0);
  motor4.runSpeed(-motor4speed,0);
 
}

void rightUp()
{
  motor1.runSpeed(0,0);
  motor2.runSpeed(motor2speed,0);
  motor3.runSpeed(0,0);
  motor4.runSpeed(motor4speed,0);
}

void rightDown()
{
  motor1.runSpeed(-motor1speed,0);
  motor2.runSpeed(0,0);
  motor3.runSpeed(-motor3speed,0);
  motor4.runSpeed(0,0);
}

void turnRight()
{
  motor1.runSpeed(-motor1speed,0);
  motor2.runSpeed(motor2speed,0);
  motor3.runSpeed(motor3speed,0);
  motor4.runSpeed(-motor4speed,0);
}

void turnLeft()
{
  motor1.runSpeed(motor1speed,0);
  motor2.runSpeed(-motor2speed,0);
  motor3.runSpeed(-motor3speed,0);
  motor4.runSpeed(motor4speed,0);
}

void strafeRight()
{
  motor1.runSpeed(-motor1speed,0);
  motor2.runSpeed(motor2speed,0);
  motor3.runSpeed(motor3speed,0);
  motor4.runSpeed(-motor4speed,0);
}

void strafeLeft()
{
  motor1.runSpeed(motor1speed,0);
  motor2.runSpeed(-motor2speed,0);
  motor3.runSpeed(-motor3speed,0);
  motor4.runSpeed(motor4speed,0);
}

void strafeRightUp()
{
  //motor1.runSpeed(-motor1speed/2,0);
  motor2.runSpeed(motor2speed,0);
  motor3.runSpeed(motor3speed,0);
  //motor4.runSpeed(-motor4speed/2,0);
}

void strafeRightDown()
{
  motor1.runSpeed(-motor1speed,0);
  //motor2.runSpeed(motor2speed/2,0);
  //motor3.runSpeed(motor3speed/2,0);
  motor4.runSpeed(-motor4speed,0);
}

void strafeLeftUp()
{
  motor1.runSpeed(motor1speed,0);
  //motor2.runSpeed(-motor2speed/2,0);
  //motor3.runSpeed(-motor3speed/2,0);
  motor4.runSpeed(motor4speed,0);
}

void strafeLeftDown()
{
  motor1.runSpeed(motor1speed/2,0);
  motor2.runSpeed(-motor2speed,0);
  motor3.runSpeed(-motor3speed,0);
  //motor4.runSpeed(motor4speed/2,0);
}
*/


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
  motor1.runSpeed(0,1);
  motor2.runSpeed(0,1);
  motor3.runSpeed(0,1);
  motor4.runSpeed(0,1);
}

// angle to translate at: [0, 2pi]
//    PI*1.5 = right direction
//    0      = forward direction
//    PI*0.5 = left direction
//    PI     = backwards direction
// speed: [-1,1]
//    1 = forward (in the chosen direction)
//    -1 = backward (in the chosen direction)
// rotation: [-1,1]
//    -1 = turn maximum left
//    1 = turn maximum right
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

  Serial.print("multipliers 1:");
  Serial.print(motor1multiplier);
  Serial.print(" 2:");
  Serial.print(motor2multiplier);
  Serial.print(" 3:");
  Serial.print(motor3multiplier);
  Serial.print(" 4:");
  Serial.println(motor4multiplier);
  
  Serial.print("speeds 1:");
  Serial.print(motor1speed);
  Serial.print(" 2:");
  Serial.print(motor2speed);
  Serial.print(" 3:");
  Serial.print(motor3speed);
  Serial.print(" 4:");
  Serial.println(motor4speed);
  if(motorsEnabled)
  {
    motor1.runSpeed(motor1speed,0);
    motor2.runSpeed(motor2speed,0);
    motor3.runSpeed(motor3speed,0);
    motor4.runSpeed(motor4speed,0);
  }
  else
  {
    Serial.println("Motors are disabled.");
  }
}

