#include "MeOrion.h"
#include <Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial teensySerial(A3, A2); // RX, TX

MeEncoderNew motor1(0x09, SLOT1); // front right
MeEncoderNew motor2(0x09, SLOT2); // front left
MeEncoderNew motor3(0x0a, SLOT1); // back right
MeEncoderNew motor4(0x0a, SLOT2); // back left

#define MAXSPEED 150
#define DEFAULTSPEED 1
#define COMMAND_TIMEOUT 300 // if no command is received within this amount of ms, stop motors
#define MOTOR_UPDATE_TIME 20  // minimal ms between motor updates
#define TEENSY_COMMAND_TIME 250 // minimal ms between commands to teensy

float moveSpeed = DEFAULTSPEED;
float motor1Speed = 0, motor2Speed = 0, motor3Speed = 0, motor4Speed = 0;
float motor1SpeedTarget = 0, motor2SpeedTarget = 0, motor3SpeedTarget = 0, motor4SpeedTarget = 0; 

float motorFilterQ = 0.05; // adjusts how fast the motor will get to its actual target speed
boolean motorsEnabled = true;
unsigned long lastMotorUpdate;
unsigned long lastCommandTime;
unsigned long lastTeensyCommand;
boolean skippedLastTeensyCommand = false;

// This array is to set motor's direction.
// Change the symbol to change the motor's direction
signed char directionAdjustment[4]={-1,1,-1,1};

// === Serial ===

#define CONFIRM_CORRECT 123
#define CONFIRM_WRONG 456

struct webSerialMsgType {
  float driveAngle = 0.0;
  float driveSpeed = 0.0;
  float rotationSpeed = 0.0;
  uint16_t pitch = 90;
  uint16_t yaw = 90;
  uint16_t height = 90;
  uint16_t confirm = CONFIRM_CORRECT;
} webMsg;

bool serialTeensySynced = false;

struct teensyOrionServoMsgType {
  uint16_t pitch = 90;
  uint16_t yaw = 90;
  uint16_t height = 90;
  uint16_t confirm = CONFIRM_CORRECT;
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
  uint16_t obstacleDirections[8] = {MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE};
  uint16_t confirm = CONFIRM_CORRECT;
} distanceMsg;

// === ===

void setup()
{
  Serial.begin(9600);
  Serial.println("Makeblock starting..");
  motor1.begin();
  //  motor2.begin(); // not sure why this is commented out,
                      // maybe because it is only about starting the motor board?
  motor3.begin();
  //  motor4.begin();
  delay(10);
  stopAll();
  lastMotorUpdate = millis();
  
  
  
  teensySerial.begin(9600);
  syncWithTeensy();

  Serial.println("Makeblock started ok");
}



void loop()
{
  readTeensySerial();
  readWebSerial();
  updateMotorSpeeds();
  // send a delayed teensy command
  if(skippedLastTeensyCommand && millis() > lastTeensyCommand + TEENSY_COMMAND_TIME) {
    lastTeensyCommand = millis();
    skippedLastTeensyCommand = false;
    teensySerial.write((const char *) &servoMsg, sizeof(teensyOrionServoMsgType));
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
    stopAll();
    Serial.print("<!> Out of sync with teensy! (Confirm=");
    Serial.print(distanceMsg.confirm);
    Serial.println(")");
    // We are out of sync!
    serialTeensySynced = false;
    // Send a reply with a wrong confirm value, to make sure orion also goes into re-sync mode
    servoMsg.confirm = CONFIRM_WRONG;
    delay(100);
    Serial.println("Sending a wrong response");
    teensySerial.write((const char *) &servoMsg, sizeof(teensyOrionServoMsgType));
    delay(100);
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
  while(!serialTeensySynced) {
    teensySerial.write('s');
    Serial.print(".");
    delay(100);
    while(teensySerial.available()) {
      uint8_t c = teensySerial.read();
      if(c == 's') {
        serialTeensySynced = true;
      }
    }
  }
  // empty the buffer
  delay(10);
  Serial.println("Emptying input buffer.");
  while(teensySerial.available()> 0) {
    teensySerial.read();
  }
  Serial.println("Synced.");
  servoMsg.confirm = CONFIRM_CORRECT;
}

void readWebSerial() {
  if(Serial.available() < (int) sizeof(webSerialMsgType)) {
    return;
  }
  Serial.readBytes((char *) &webMsg, sizeof(webSerialMsgType));

  
  if(webMsg.confirm != CONFIRM_CORRECT) {
    stopAll();
    Serial.print("<!> Out of sync with webSerial! (Confirm=");
    Serial.print(webMsg.confirm);
    Serial.println(")");
    delay(500);
    // empty the input buffer from web serial
    // this is a ugly hack which might help to get it synced again?
    // TODO something better, or be sure about if this is ok.
    while(Serial.available() > 0) {
      Serial.read();
    }
    return;
  }

  Serial.print("Received:");
  Serial.print(webMsg.driveAngle);
  Serial.print("  //  ");
  Serial.print(webMsg.driveSpeed);
  Serial.print("  //  ");
  Serial.print(webMsg.rotationSpeed);
  Serial.print("  \\  ");
  Serial.print(webMsg.pitch);
  Serial.print("  //  ");
  Serial.print(webMsg.yaw);
  Serial.print("  //  ");
  Serial.println(webMsg.height);

  // make sure received values are in correct range
  webMsg.driveAngle = fmod((fmod(webMsg.driveAngle,float(2.0*PI)) + 2.0*PI),float(2.0*PI));
  webMsg.driveSpeed = max(min(webMsg.driveSpeed, 1), -1);
  webMsg.rotationSpeed = max(min(webMsg.rotationSpeed, 1), -1);

  lastCommandTime = millis();
  
  calculateSpeeds(webMsg.driveAngle, webMsg.driveSpeed, webMsg.rotationSpeed);

  servoMsg.pitch = webMsg.pitch;
  servoMsg.yaw = webMsg.yaw;
  servoMsg.height = webMsg.height;
  if(millis() > lastTeensyCommand + TEENSY_COMMAND_TIME) {
    lastTeensyCommand = millis();
    skippedLastTeensyCommand = false;
    teensySerial.write((const char *) &servoMsg, sizeof(teensyOrionServoMsgType));
  }
  else {
    skippedLastTeensyCommand = true;
  }
}

void updateMotorSpeeds() {
  if(millis() > lastCommandTime + COMMAND_TIMEOUT) {
    stopAll();
  }
  else if(millis() > lastMotorUpdate + MOTOR_UPDATE_TIME) {
    lastMotorUpdate += MOTOR_UPDATE_TIME;
    // lowpass filter
    // current = (1-q)*current+q*target
    motor1Speed = (1-motorFilterQ)*motor1Speed + motorFilterQ * motor1SpeedTarget;
    motor2Speed = (1-motorFilterQ)*motor2Speed + motorFilterQ * motor2SpeedTarget;
    motor3Speed = (1-motorFilterQ)*motor3Speed + motorFilterQ * motor3SpeedTarget;
    motor4Speed = (1-motorFilterQ)*motor4Speed + motorFilterQ * motor4SpeedTarget;

    motor1.runSpeed(motor1Speed,0);
    motor2.runSpeed(motor2Speed,0);
    motor3.runSpeed(motor3Speed,0);
    motor4.runSpeed(motor4Speed,0);
  }
}