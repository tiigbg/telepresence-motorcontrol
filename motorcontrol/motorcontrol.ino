#include "MeOrion.h"
#include <Wire.h>

MeEncoderNew motor1(0x09, SLOT2); // front right
MeEncoderNew motor2(0x09, SLOT1); // front left
MeEncoderNew motor3(0x0a, SLOT1); // back right
MeEncoderNew motor4(0x0a, SLOT2); // back left

#define MAXSPEED 150
#define DEFAULTSPEED 0.8
#define MAX_ROTATIONSPEED 0.5
#define COMMAND_TIMEOUT 300  // if no command is received within this amount of ms, stop motors
#define MOTOR_UPDATE_TIME 20 // minimal ms between motor updates

float moveSpeed = DEFAULTSPEED;
float motor1Speed = 0, motor2Speed = 0, motor3Speed = 0, motor4Speed = 0;
float motor1Position = 0, motor2Position = 0, motor3Position = 0, motor4Position = 0;
float driveAngleTarget = 0.0, driveSpeedTarget = 0.0, rotationSpeedTarget = 0.0;
float driveAngle = 0.0, driveSpeed = 0.0, rotationSpeed = 0.0;

float driveFilterQ = 0.05;   // adjusts how fast the robot will get to its actual target drive speed
float rotationFilterQ = 1.0; // adjusts how fast the robot will get to its actual target rotation speed
boolean motorsEnabled = true;
unsigned long lastMotorUpdate;
unsigned long lastCommandTime;

// This array is to set motor's direction.
// Change the symbol to change the motor's direction
signed char directionAdjustment[4] = {-1, 1, -1, 1};

// === Serial ===

#define CONFIRM_CORRECT 123
#define CONFIRM_SYNC_ERROR 789

struct teensyOrionMsgType
{
  uint32_t confirm = CONFIRM_CORRECT;
  uint32_t loopTime = 0;
  float driveAngle = 0.0;
  float driveSpeed = 0.0;
  float rotationSpeed = 0.0;
  float motorSpeed[4];
  float motorPosition[4];
} incomingMotorMsg, outgoingMotorMsg;

// === ===
unsigned long sendMotorStatsInterval = 0;
unsigned long sendMotorStatsStamp = 0;

void setup()
{
  Serial.begin(115200);
  motor1.begin();
  //  motor2.begin(); // not sure why this is commented out,
  // maybe because it is only about starting the motor board?
  motor3.begin();
  //  motor4.begin();
  delay(50);
  stopAll();
  lastMotorUpdate = millis();
  sendMotorStatsStamp = millis();

  pinMode(LED_BUILTIN, OUTPUT);
  // Serial.print("expecting messages of size: ");
  // Serial.println(sizeof(teensyOrionMsgType));
}

unsigned long loopStamp = 0;
unsigned long loopTime = 0;
void loop()
{
  loopTime = millis() - loopStamp;
  loopStamp = millis();
  outgoingMotorMsg.loopTime = loopTime;
  // if (Serial.available())
  // {
  //   Serial.write(Serial.read());
  // }
  // return;

  readTeensySerial();
  updateMotorSpeeds();
  // if (millis() - sendMotorStatsStamp > sendMotorStatsInterval)
  // {
  // sendMotorStatsStamp = millis();
  getMotorStats();
  sendMotorStats();
  // printMotorStats();
  // }
} // end of loop()

void getMotorStats()
{

  outgoingMotorMsg.motorSpeed[0] = motor1.getCurrentSpeed();
  outgoingMotorMsg.motorSpeed[1] = motor2.getCurrentSpeed();
  outgoingMotorMsg.motorSpeed[2] = motor3.getCurrentSpeed();
  outgoingMotorMsg.motorSpeed[3] = motor4.getCurrentSpeed();

  outgoingMotorMsg.motorPosition[0] = motor1.getCurrentPosition();
  outgoingMotorMsg.motorPosition[1] = motor2.getCurrentPosition();
  outgoingMotorMsg.motorPosition[2] = motor3.getCurrentPosition();
  outgoingMotorMsg.motorPosition[3] = motor4.getCurrentPosition();
}

void sendMotorStats()
{
  // Serial.print(".");
  Serial.write((const char *)&outgoingMotorMsg, sizeof(teensyOrionMsgType));
}

void printMotorStats()
{
  Serial.println("MotorStats --------");

  for (int i = 0; i < 4; i++)
  {
    Serial.print("motor ");
    Serial.print(i);
    Serial.print(" speed: ");
    Serial.print(outgoingMotorMsg.motorSpeed[i]);
    Serial.print(", \t");
    Serial.print(" position: ");
    Serial.print(outgoingMotorMsg.motorPosition[i]);
    Serial.println();
  }
}

void adjustDirections()
{
  motor1Speed = moveSpeed * directionAdjustment[0];
  motor2Speed = moveSpeed * directionAdjustment[1];
  motor3Speed = moveSpeed * directionAdjustment[2];
  motor4Speed = moveSpeed * directionAdjustment[3];
}

void speedUp()
{
  moveSpeed += 1;
  if (moveSpeed > MAXSPEED)
    moveSpeed = MAXSPEED;

  adjustDirections();
}
void speedDown()
{
  moveSpeed -= 1;
  if (moveSpeed < 0)
    moveSpeed = 0;

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

  motor1.runSpeed(0, 0);
  motor2.runSpeed(0, 0);
  motor3.runSpeed(0, 0);
  motor4.runSpeed(0, 0);
}

// https://drive.google.com/drive/u/1/folders/1qwxI1x4Pp_o9gxtaNcFb_DFvaulr_N56
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
  if (maxMultiplier != 0 && maxMultiplier > 1)
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

void readTeensySerial()
{
  if (Serial.available() < (int)sizeof(teensyOrionMsgType))
  {
    return;
  }
  Serial.readBytes((char *)&incomingMotorMsg, sizeof(teensyOrionMsgType));

  if (incomingMotorMsg.confirm != CONFIRM_CORRECT)
  {
    stopAll();
    // We are out of sync with teensy!
    // Send a reply with a wrong confirm value, to make sure teensy also goes into re-sync mode
    //Serial.write('f');
    // Serial.print("<!Orion> Out of sync Confirm=(");
    // Serial.print(incomingMotorMsg.confirm);
    // Serial.println(")");
    //Serial.flush();

    // outgoingMotorMsg.confirm = CONFIRM_SYNC_ERROR;
    // Serial.write((const char *)&outgoingMotorMsg, sizeof(teensyOrionMsgType));
    // outgoingMotorMsg.confirm = CONFIRM_CORRECT;
    Serial.println("serial reaction");

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(20);
    // empty the incoming serial buffer
    while (Serial.available() > 0)
    {
      Serial.read();
    }
    return;
  }
  // Serial.print("Crrct! vlbl=");
  // Serial.println(Serial.available());
  // Serial.println(sizeof(teensyOrionMsgType));

  // make sure received values are in correct range
  incomingMotorMsg.driveAngle = fmod((fmod(incomingMotorMsg.driveAngle, float(2.0 * PI)) + 2.0 * PI), float(2.0 * PI));
  incomingMotorMsg.driveSpeed = max(min(incomingMotorMsg.driveSpeed, 1), -1);
  incomingMotorMsg.rotationSpeed = max(min(incomingMotorMsg.rotationSpeed, MAX_ROTATIONSPEED), -1.0 * MAX_ROTATIONSPEED);

  // Serial.print("Setting motorvalues:");
  // Serial.print(incomingMotorMsg.driveAngle);
  // Serial.print("  //  ");
  // Serial.print(incomingMotorMsg.driveSpeed);
  // Serial.print("  //  ");
  // Serial.print(incomingMotorMsg.rotationSpeed);
  // Serial.println();

  lastCommandTime = millis();

  driveAngleTarget = incomingMotorMsg.driveAngle;
  driveSpeedTarget = incomingMotorMsg.driveSpeed;
  rotationSpeedTarget = incomingMotorMsg.rotationSpeed;
}

void updateMotorSpeeds()
{
  if (millis() > lastCommandTime + COMMAND_TIMEOUT)
  {
    // Serial.println("not received command for a while. Stopping all motors");
    stopAll();
  }
  else if (millis() > lastMotorUpdate + MOTOR_UPDATE_TIME)
  {
    lastMotorUpdate += MOTOR_UPDATE_TIME;
    // lowpass filters
    // current = (1-q)*current+q*target
    // driveAngle = (1-driveFilterQ)*driveAngle + driveFilterQ * driveAngleTarget;
    driveAngle = driveAngleTarget;
    // driveSpeed = (1.0f-driveFilterQ)*driveSpeed + driveFilterQ * driveSpeedTarget;
    // rotationSpeed = (1.0f-rotationFilterQ)*rotationSpeed + rotationFilterQ * rotationSpeedTarget;

    driveSpeed = driveSpeedTarget;
    rotationSpeed = rotationSpeedTarget;

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

    motor1.runSpeed(motor1Speed, 0);
    motor2.runSpeed(motor2Speed, 0);
    motor3.runSpeed(motor3Speed, 0);
    motor4.runSpeed(motor4Speed, 0);
  }
}