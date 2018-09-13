#include "MeOrion.h"
#include <Wire.h>

MeEncoderNew motor1(0x09, SLOT1); // front right
MeEncoderNew motor2(0x09, SLOT2); // front left
MeEncoderNew motor3(0x0a, SLOT1); // back right
MeEncoderNew motor4(0x0a, SLOT2); // back left

#define MAXSPEED 150
#define DEFAULTSPEED 1
#define MAX_ROTATIONSPEED 0.5
#define COMMAND_TIMEOUT 300 // if no command is received within this amount of ms, stop motors
#define MOTOR_UPDATE_TIME 20  // minimal ms between motor updates

float moveSpeed = DEFAULTSPEED;
float motor1Speed = 0, motor2Speed = 0, motor3Speed = 0, motor4Speed = 0;
float driveAngleTarget = 0.0, driveSpeedTarget = 0.0, rotationSpeedTarget = 0.0;
float driveAngle = 0.0, driveSpeed = 0.0, rotationSpeed = 0.0;

float driveFilterQ = 0.05; // adjusts how fast the robot will get to its actual target drive speed
float rotationFilterQ = 0.05; // adjusts how fast the robot will get to its actual target rotation speed
boolean motorsEnabled = true;
unsigned long lastMotorUpdate;
unsigned long lastCommandTime;

// This array is to set motor's direction.
// Change the symbol to change the motor's direction
signed char directionAdjustment[4]={-1,1,-1,1};

// === Serial ===

#define CONFIRM_CORRECT 123

struct teensyOrionMsgType {
  uint16_t confirm = CONFIRM_CORRECT;
  // float driveAngle = 0.0;
  // float driveSpeed = 0.0;
  // float rotationSpeed = 0.0;
} motorMsg;

// === ===

void setup()
{
  Serial.begin(9600);
  motor1.begin();
  //  motor2.begin(); // not sure why this is commented out,
                      // maybe because it is only about starting the motor board?
  motor3.begin();
  //  motor4.begin();
  delay(10);
  stopAll();
  lastMotorUpdate = millis();
  Serial.println("Hej jag är Orion");
}



void loop()
{
  readTeensySerial();
  // updateMotorSpeeds();
} // end of loop()

void adjustDirections()
{
  motor1Speed = moveSpeed*directionAdjustment[0];
  motor2Speed = moveSpeed*directionAdjustment[1];
  motor3Speed = moveSpeed*directionAdjustment[2];
  motor4Speed = moveSpeed*directionAdjustment[3];
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
  driveAngle = 0.0;
  driveSpeed = 0.0;
  rotationSpeed = 0.0;
  driveAngleTarget = 0.0;
  driveSpeedTarget = 0.0;
  rotationSpeedTarget = 0.0;

  motor1Speed = 0;
  motor2Speed = 0;
  motor3Speed = 0;
  motor4Speed = 0;
  
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
void calculateMotorSpeeds()
{
  // TODO FIXME There seems to be some kind of rounding issue where sin and cos don't return the same value 
  // when they should (for example when driving straight forward, all motors should get equal, now they
  // differ with 1
  float motor1multiplier = driveSpeed * sin(driveAngle + PI / 4.0) - rotationSpeed;
  float motor2multiplier = driveSpeed * cos(driveAngle + PI / 4.0) + rotationSpeed;
  float motor3multiplier = driveSpeed * cos(driveAngle + PI / 4.0) - rotationSpeed;
  float motor4multiplier = driveSpeed * sin(driveAngle + PI / 4.0) + rotationSpeed;

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
  
  motor1Speed = directionAdjustment[0] * moveSpeed * motor1multiplier;
  motor2Speed = directionAdjustment[1] * moveSpeed * motor2multiplier;
  motor3Speed = directionAdjustment[2] * moveSpeed * motor3multiplier;
  motor4Speed = directionAdjustment[3] * moveSpeed * motor4multiplier;

  // Serial.print("multipliers   1:");
  // Serial.print(motor1multiplier);
  // Serial.print(" 2:");
  // Serial.print(motor2multiplier);
  // Serial.print(" 3:");
  // Serial.print(motor3multiplier);
  // Serial.print(" 4:");
  // Serial.println(motor4multiplier);
}

void readTeensySerial() {
  if(Serial.available() < (int) sizeof(teensyOrionMsgType)) {
    return;
  }
  Serial.readBytes((char *) &motorMsg, sizeof(teensyOrionMsgType));

  if(motorMsg.confirm != CONFIRM_CORRECT) {
    stopAll();
    // We are out of sync with teensy!
    // Send a reply with a wrong confirm value, to make sure teensy also goes into re-sync mode
    //Serial.write('f');
    Serial.print("Confirm=(");
    Serial.print(motorMsg.confirm);
    Serial.println(")");
    //Serial.flush();
    delay(20);
    // empty the incoming serial buffer
    while(Serial.available()> 0) {
      Serial.read();
    }
    return;
  }
  Serial.print("Crrct! vlbl=");
  Serial.println(Serial.available());

  // // make sure received values are in correct range
  // motorMsg.driveAngle = fmod((fmod(motorMsg.driveAngle,float(2.0*PI)) + 2.0*PI),float(2.0*PI));
  // motorMsg.driveSpeed = max(min(motorMsg.driveSpeed, 1), -1);
  // motorMsg.rotationSpeed = max(min(motorMsg.rotationSpeed, MAX_ROTATIONSPEED), -1.0*MAX_ROTATIONSPEED);

  // lastCommandTime = millis();
  
  // driveAngleTarget = motorMsg.driveAngle;
  // driveSpeedTarget = motorMsg.driveSpeed;
  // rotationSpeedTarget = motorMsg.rotationSpeed;
}

void updateMotorSpeeds() {
  if(millis() > lastCommandTime + COMMAND_TIMEOUT) {
    stopAll();
  }
  else if(millis() > lastMotorUpdate + MOTOR_UPDATE_TIME) {
    lastMotorUpdate += MOTOR_UPDATE_TIME;
    // lowpass filters
    // current = (1-q)*current+q*target
    // driveAngle = (1-driveFilterQ)*driveAngle + driveFilterQ * driveAngleTarget;
    driveAngle = driveAngleTarget;
    driveSpeed = (1-driveFilterQ)*driveSpeed + driveFilterQ * driveSpeedTarget;
    rotationSpeed = (1-rotationFilterQ)*rotationSpeed + rotationFilterQ * rotationSpeedTarget;

    // Serial.print("    ");
    // Serial.print(driveAngle);
    // Serial.print("    ");
    // Serial.print(driveSpeed);
    // Serial.print("    ");
    // Serial.print(rotationSpeed);
    // Serial.print("    (");
    // Serial.print(rotationSpeedTarget);
    // Serial.println(")");

    calculateMotorSpeeds();

    motor1.runSpeed(motor1Speed,0);
    motor2.runSpeed(motor2Speed,0);
    motor3.runSpeed(motor3Speed,0);
    motor4.runSpeed(motor4Speed,0);
  }
}