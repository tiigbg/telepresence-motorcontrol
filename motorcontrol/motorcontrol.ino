#include "MeOrion.h"
#include <Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial teensySerial(A3, A2); // RX, TX

MeEncoderNew motor1(0x09, SLOT1); // front right
MeEncoderNew motor2(0x09, SLOT2); // front left
MeEncoderNew motor3(0x0a, SLOT1); // back right
MeEncoderNew motor4(0x0a, SLOT2); // back left

#define MAXSPEED 150
#define DEFAULTSPEED 10
#define COMMAND_TIMEOUT 300

int moveSpeed = DEFAULTSPEED;
float motor1Speed = 0, motor2Speed = 0, motor3Speed = 0, motor4Speed = 0;
float motor1SpeedTarget = 0, motor2SpeedTarget = 0, motor3SpeedTarget = 0, motor4SpeedTarget = 0; 

float motorFilterQ = 0.04; // adjusts how fast the motor will get to its actual target speed
boolean motorsEnabled = true;
unsigned long lastCommandTime;

// This array is to set motor's direction.
// Change the symbol to change the motor's direction
signed char directionAdjustment[4]={-1,1,-1,1};

// === Serial ===

bool serialSynced = false;

#define CONFIRM_CORRECT 123
#define CONFIRM_WRONG 456
struct teensyOrionServoMsgType {
  uint16_t confirm = CONFIRM_CORRECT;
  uint16_t pitch = 90;
  uint16_t yaw = 90;
  uint16_t height = 90;
} servoMsg;

// this tracks how far away an obstacle is detected
// in 8 directions.
// directions are (in order): forward, forward right, right, backward right,
// backward, backward left, left, forward left
#define MAX_DISTANCE 999
#define DIR_FRONT 0
#define DIR_FRONT_RIGHT 1
#define DIR_RIGHT 2
#define DIR_BACK_RIGHT 3
#define DIR_BACK 4
#define DIR_BACK_LEFT 5
#define DIR_LEFT 6
#define DIR_FRONT_LEFT 7

struct teensyOrionDistanceMsgType {
  uint16_t confirm = CONFIRM_CORRECT;
  uint16_t obstacleDirections[8] = {MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE};
} distanceMsg;

// === ===

void setup()
{
  Serial.begin(115200);
  Serial.println("Makeblock starting..");
  motor1.begin();
  //  motor2.begin(); // not sure why this is commented out,
                      // maybe because it is only about starting the motor board?
  motor3.begin();
  //  motor4.begin();
  delay(10);
  stopAll();
  
  
  
  teensySerial.begin(9600);
  syncWithTeensy();

  Serial.println("Makeblock started ok");
  Serial.println("Example command:");
  Serial.println("<0;0;0;90;90;90>");
}



void loop()
{
  readTeensySerial();


  if(Serial.available() > 0)  // example: <0;0;0;90;90;90>
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
    int thirdBreakCharIndex = incomingString.indexOf(';', secondBreakCharIndex+1);
    if(thirdBreakCharIndex == -1)
    {
      Serial.println("<!> Did not find third break character (;).");
      //return;
    }
    String firstValue = incomingString.substring(0, firstBreakCharIndex);
    String secondValue = incomingString.substring(firstBreakCharIndex+1, secondBreakCharIndex);
    String thirdValue = "";
    if(thirdBreakCharIndex == -1)
    {
      thirdValue = incomingString.substring(secondBreakCharIndex+1);
    }else {
      thirdValue = incomingString.substring(secondBreakCharIndex+1, thirdBreakCharIndex);
    }
    
    float newDirection = firstValue.toFloat();
    float newSpeed = secondValue.toFloat();
    float newRotation = thirdValue.toFloat();

    // Serial.print("Floats: ");
    // Serial.print(newDirection);
    // Serial.print(" // ");
    // Serial.print(newSpeed);
    // Serial.print(" // ");
    // Serial.println(newRotation);

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

    if(thirdBreakCharIndex != -1)
    {
      /*
      String fourthValue = incomingString.substring(thirdBreakCharIndex+1);
      Serial.println("To Teensy:<"+fourthValue+">");
      teensySerial.println("<"+fourthValue+">");
      */
      int fourthBreakCharIndex = incomingString.indexOf(';', thirdBreakCharIndex+1);
      int fifthBreakCharIndex = incomingString.indexOf(';', fourthBreakCharIndex+1);

      if(fourthBreakCharIndex == -1 || fifthBreakCharIndex == -1) {
        return;
      }
      String fourthValue = incomingString.substring(thirdBreakCharIndex+1, fourthBreakCharIndex);
      String fifthValue = incomingString.substring(fourthBreakCharIndex+1, fifthBreakCharIndex);
      String sixthValue = incomingString.substring(fifthBreakCharIndex+1);

      servoMsg.pitch = fourthValue.toInt();
      servoMsg.yaw = fifthValue.toInt();
      servoMsg.height = sixthValue.toInt();

      teensySerial.write((const char *) &servoMsg, sizeof(teensyOrionServoMsgType));
      Serial.print("writing to teensy:");
      Serial.print(servoMsg.pitch);
      Serial.print(" // ");
      Serial.print(servoMsg.yaw);
      Serial.print(" // ");
      Serial.println(servoMsg.height);
    }
  }

  if(motorsEnabled && millis() > lastCommandTime + COMMAND_TIMEOUT)
  {
    stopAll();
  }
  else{
    // lowpass filter
    // current = (1-q)*current+q*target
    motor1Speed = (1-motorFilterQ)*motor1Speed + motorFilterQ * motor1SpeedTarget;
    motor2Speed = (1-motorFilterQ)*motor2Speed + motorFilterQ * motor2SpeedTarget;
    motor3Speed = (1-motorFilterQ)*motor3Speed + motorFilterQ * motor3SpeedTarget;
    motor4Speed = (1-motorFilterQ)*motor4Speed + motorFilterQ * motor4SpeedTarget;

    if(motorsEnabled)
    {
      //Serial.println(motor1Speed);
      motor1.runSpeed(motor1Speed,0);
      motor2.runSpeed(motor2Speed,0);
      motor3.runSpeed(motor3Speed,0);
      motor4.runSpeed(motor4Speed,0);
    }
  }

} // end of loop()

void adjustDirections()
{
  motor1SpeedTarget = moveSpeed*directionAdjustment[0];
  motor2SpeedTarget = moveSpeed*directionAdjustment[1];
  motor3SpeedTarget = moveSpeed*directionAdjustment[2];
  motor4SpeedTarget = moveSpeed*directionAdjustment[3];
}

void speedUp()
{
  moveSpeed +=1;
  if(moveSpeed>MAXSPEED)  moveSpeed=MAXSPEED;

  adjustDirections();
}
void speedDown()
{
  moveSpeed -= 1;
  if(moveSpeed<0)  moveSpeed=0;

  adjustDirections();
}
void stopAll()
{
  motor1Speed = 0;
  motor2Speed = 0;
  motor3Speed = 0;
  motor4Speed = 0;
  
  motor1SpeedTarget = 0;
  motor2SpeedTarget = 0;
  motor3SpeedTarget = 0;
  motor4SpeedTarget = 0;
  

  motor1.runSpeed(0,0);
  motor2.runSpeed(0,0);
  motor3.runSpeed(0,0);
  motor4.runSpeed(0,0);
}

void disableMotors()
{
  motorsEnabled = false;
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
  // TODO FIXME There seems to be some kind of rounding issue where sin and cos don't return the same value 
  // when they should (for example when driving straight forward, all motors should get equal, now they
  // differ with 1
  float motor1multiplier = mySpeed * sin(myAngle + PI / 4.0) + myRotation;
  float motor2multiplier = mySpeed * cos(myAngle + PI / 4.0) - myRotation;
  float motor3multiplier = mySpeed * cos(myAngle + PI / 4.0) + myRotation;
  float motor4multiplier = mySpeed * sin(myAngle + PI / 4.0) - myRotation;

  // scale all multipliers based on the biggest, to keep them inside the [1,-1] range
  float maxMultiplier = max(fabs(motor1multiplier),
                         max(fabs(motor2multiplier),
                          max(fabs(motor3multiplier),
                           fabs(motor4multiplier))));
  if(maxMultiplier != 0 && maxMultiplier > 1)
  {
    // Serial.print("Scaling multipliers with:");
    // Serial.println(maxMultiplier);
    motor1multiplier = motor1multiplier / maxMultiplier;
    motor2multiplier = motor2multiplier / maxMultiplier;
    motor3multiplier = motor3multiplier / maxMultiplier;
    motor4multiplier = motor4multiplier / maxMultiplier;
  }
  
  motor1SpeedTarget = directionAdjustment[0] * moveSpeed * motor1multiplier;
  motor2SpeedTarget = directionAdjustment[1] * moveSpeed * motor2multiplier;
  motor3SpeedTarget = directionAdjustment[2] * moveSpeed * motor3multiplier;
  motor4SpeedTarget = directionAdjustment[3] * moveSpeed * motor4multiplier;

  // Serial.print("multipliers   1:");
  // Serial.print(motor1multiplier);
  // Serial.print(" 2:");
  // Serial.print(motor2multiplier);
  // Serial.print(" 3:");
  // Serial.print(motor3multiplier);
  // Serial.print(" 4:");
  // Serial.println(motor4multiplier);
  
  // Serial.print("speeds        1:");
  // Serial.print(motor1SpeedTarget);
  // Serial.print(" 2:");
  // Serial.print(motor2SpeedTarget);
  // Serial.print(" 3:");
  // Serial.print(motor3SpeedTarget);
  // Serial.print(" 4:");
  // Serial.println(motor4SpeedTarget);

}

void readTeensySerial() {
  if(teensySerial.available() < (int) sizeof(teensyOrionDistanceMsgType)) {
    return;
  }
  teensySerial.readBytes((char *) &distanceMsg, sizeof(teensyOrionDistanceMsgType));

  if(distanceMsg.confirm != CONFIRM_CORRECT) {
    Serial.print("<!> Out of sync! (Confirm=");
    Serial.print(distanceMsg.confirm);
    Serial.println(")");
    // We are out of sync!
    serialSynced = false;
    // Send a reply with a wrong confirm value, to make sure orion also goes into re-sync mode
    servoMsg.confirm = CONFIRM_WRONG;
    delay(1000);
    Serial.println("Sending a wrong response");
    teensySerial.write((const char *) &servoMsg, sizeof(teensyOrionServoMsgType));
    delay(1000);
    // empty the incoming serial buffer
    while(teensySerial.available()> 0) {
      teensySerial.read();
    }
    syncWithTeensy();
    return;
  }
  //Serial.println("Received correct");
}

void syncWithTeensy() {
  Serial.println("Preparing to sync. Emptying serial input buffer.");
  while(teensySerial.available()> 0) {
    teensySerial.read();
  }
  
  Serial.print("Waiting for sync with Teensy");
  while(!serialSynced) {
    teensySerial.write('s');
    Serial.print(".");
    delay(100);
    while(teensySerial.available()) {
      uint8_t c = teensySerial.read();
      if(c == 's') {
        serialSynced = true;
      }
    }
  }
  // empty the buffer
  delay(100);
  Serial.println("Emptying input buffer.");
  while(teensySerial.available()> 0) {
    teensySerial.read();
  }
  Serial.println("Synced.");
  servoMsg.confirm = CONFIRM_CORRECT;
  Serial.println(teensySerial.available());
}