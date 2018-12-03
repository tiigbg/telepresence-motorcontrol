#include <ScanPacket.h>
#include <Sweep.h>

#include <Servo.h>

// this tracks how far away an obstacle is detected
// in 8 directions.
// directions are (in order): forward, forward right, right, backward right,
// backward, backward left, left, forward left
#define MEDIUM_DISTANCE 120
#define CRITICAL_DISTANCE 30
#define MAX_DISTANCE 999
int obstacleDirections[] = {MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE};
#define DIR_FRONT 0
#define DIR_FRONT_RIGHT 1
#define DIR_RIGHT 2
#define DIR_BACK_RIGHT 3
#define DIR_BACK 4
#define DIR_BACK_LEFT 5
#define DIR_LEFT 6
#define DIR_FRONT_LEFT 7

#define MAX_ROTATIONSPEED 0.5

int ledPin = 13;

// === Serial ===

unsigned long lastSerialSend = millis();
unsigned long maxSerialSendTime = 500;
bool serialSynced = false;

#define CONFIRM_CORRECT 123
#define CONFIRM_WRONG 456
#define CONFIRM_SYNC_ERROR 789

struct webSerialMsgType
{
  float driveAngle = 0.0;
  float driveSpeed = 0.0;
  float rotationSpeed = 0.0;
  uint32_t pitch = 90;
  uint32_t yaw = 90;
  uint32_t height = 90;
  int32_t rotationTarget = 0;
  float distanceTarget = 0;
  uint32_t confirm = CONFIRM_CORRECT;
} webMsg;

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

// === Bumpers ===

int bumperPinFR = 24;
int bumperPinRF = 25;
int bumperPinRB = 26;
int bumperPinBR = 27;
int bumperPinBL = 28;
int bumperPinLB = 29;
int bumperPinLF = 30;
int bumperPinFL = 31;

int bumperValueFR, bumperValueRF, bumperValueRB, bumperValueBR, bumperValueBL, bumperValueLB, bumperValueLF, bumperValueFL;

// === Servos ===

// pitch is looking up/down
int servoPitchPin = 2;
// yaw is looking left/right
int servoYawPin = 3;
// height is lifting the stick up/down
int servoHeightPin = 4;

int servoPitchMin = 0;
int servoPitchMax = 180;
int servoYawMin = 0;
int servoYawMax = 180;
int servoHeightMin = 90;
int servoHeightMax = 180;

Servo servoPitch, servoYaw, servoHeight;

// === Lidar ===

int lidarEnabledPin = 32;
bool lidarEnabled = false;
Sweep device(Serial3);
bool lidarAvailable = false;
// keeps track of how many scans have been collected
uint8_t scanCount = 0;
uint8_t nrOfScansToMake = 2;
// keeps track of how many samples have been collected
uint16_t sampleCount = 0;
// Arrays to store attributes of collected scans
bool syncValues[500];         // 1 -> first reading of new scan, 0 otherwise
float angles[500];            // in degrees (accurate to the millidegree)
uint16_t distances[500];      // in cm
uint8_t signalStrengths[500]; // 0:255, higher is better

// === ===

// === Click to drive ===
unsigned long clickToDriveStamp = 0;
// unsigned long rotationDuration = 0;
// unsigned long driveDuration = 0;

unsigned long clickToDriveUpdateStamp = 0;
unsigned long clickToDriveUpdateInterval = 45;

// int currentRotation = 0;
float rotationTarget = 0;
// float currentDistance = 0;
float distanceTarget = 0;

void setup()
{
  pinMode(ledPin, OUTPUT);
  blink(50);
  blink(300);
  blink(50);
  blink(300);

  Serial.begin(115200);  // USB
  Serial1.begin(115200); // Orion board
  Serial3.begin(115200); // sweep lidar device

  Serial.println("Teensy starting up..");

  pinMode(bumperPinFR, INPUT_PULLUP);
  pinMode(bumperPinRF, INPUT_PULLUP);
  pinMode(bumperPinRB, INPUT_PULLUP);
  pinMode(bumperPinBR, INPUT_PULLUP);
  pinMode(bumperPinBL, INPUT_PULLUP);
  pinMode(bumperPinLB, INPUT_PULLUP);
  pinMode(bumperPinLF, INPUT_PULLUP);
  pinMode(bumperPinFL, INPUT_PULLUP);
  pinMode(lidarEnabledPin, INPUT_PULLUP);

  servoPitch.attach(servoPitchPin);
  servoYaw.attach(servoYawPin);
  servoHeight.attach(servoHeightPin);

  servoPitch.write(webMsg.pitch);
  servoYaw.write(webMsg.yaw);
  servoHeight.write(webMsg.height);

  // connected to HIGH means enabled
  lidarEnabled = !digitalRead(lidarEnabledPin);
  if (lidarEnabled)
  {
    Serial.println("Attempting to start the lidar.");
    ledOn();
    setupLidar();
    ledOff();
    resetLidarValues();
  }
  else
  {
    Serial.println("Lidar disabled.");
  }

  Serial.println("Teensy started ok");
  blink(200);
  blink(200);
  blink(200);
}

void randomMotorChange()
{
  float randomChange = random(0, 800);
  randomChange = (randomChange - 400.0f) / 1000.0f;

  outgoingMotorMsg.driveSpeed += randomChange;

  // make sure received values are in correct range
  outgoingMotorMsg.driveAngle = fmod((fmod(outgoingMotorMsg.driveAngle, float(2.0 * PI)) + 2.0 * PI), float(2.0 * PI));
  outgoingMotorMsg.driveSpeed = max(min(outgoingMotorMsg.driveSpeed, 1), -1);
  outgoingMotorMsg.rotationSpeed = max(min(outgoingMotorMsg.rotationSpeed, MAX_ROTATIONSPEED), -1.0 * MAX_ROTATIONSPEED);
}

// int i = 0;
unsigned long loopStamp = 0;
unsigned long loopTime = 0;
unsigned long orionReceiveInterval = 0;
unsigned long orionReceiveSerialStamp = 0;
void loop()
{
  loopTime = millis() - loopStamp;
  loopStamp = millis();
  outgoingMotorMsg.loopTime = loopTime;
  // Serial1.println(i++);
  // while (Serial1.available())
  // {
  //   Serial.write(Serial1.read());
  // }
  // delay(20);
  // return;

  outgoingMotorMsg.confirm = CONFIRM_CORRECT;
  readOrionSerial();

  // randomMotorChange();
  // delay(200);
  // Serial1.write((const char *) &outgoingMotorMsg, sizeof(teensyOrionMsgType));

  unsigned long calculatedInterval = min(100, max(clickToDriveUpdateInterval, orionReceiveInterval + 5));
  if (millis() - clickToDriveUpdateStamp > calculatedInterval)
  {
    clickToDriveUpdateStamp = millis();
    // Serial.printf("calculatedInterval: %i \n", calculatedInterval);
    updateClickToDrive();
  }

  readWebSerial();

  // // reset readings
  // for (int i = 0; i < 8; i++)
  // {
  //   obstacleDirections[i] = MAX_DISTANCE;
  // }

  // readPingSensors();
  // readLidar();
  // readBumpers();

  // blink(5);
}

//This
float defaultSpeed = 0.8f;
float millisPerDegree = 19.0f;
float startingRotationDuration = -250.0f;
float directionChangeDuration = 300.0f;
float latestRotationDirection = 0;

float motorPositionRotationStart[4] = {0.f, 0.f, 0.f, 0.f};
const float motorPositionRotationmultiplier = 0.674f;// 0.70710678118f;
const float rotationCircumference = 180.f;
const float motorPositionRotationAngleOffset = -34.f;
float rotationAngle = 0.f;

float distance = 0.f;

bool CTDIsActive = false;
bool CTDIsRotating = false;

void calculateRotationAngle()
{
  float rotationSum = 0;
  for (int i = 0; i < 4; i++)
  {
    float motorRotationPosition = motorPositionRotationStart[i] - incomingMotorMsg.motorPosition[i];
    Serial.printf("rotationDistance (cm) %i: %f \n", i, motorRotationPosition);
    // motorRotationPosition *= motorPositionRotationmultiplier;
    rotationSum += motorRotationPosition;
  }
  rotationSum /= 4.0f;
  rotationSum *= motorPositionRotationmultiplier;

  rotationAngle = -rotationSum / rotationCircumference * 360.0f;
}

void calculateDistance()
{
  float distanceSum = 0.f;

  for (int i = 0; i < 4; i++)
  {
    float deltaPos = motorPositionRotationStart[i] - incomingMotorMsg.motorPosition[i];
    if (i % 2)
    {
      deltaPos *= -1.f;
    }
    distanceSum += deltaPos;
  }

  distance = distanceSum / 4.f;
}

void resetMotorPosition()
{
  for (int i = 0; i < 4; i++)
  {
    motorPositionRotationStart[i] = incomingMotorMsg.motorPosition[i];
  }

  calculateRotationAngle();
  calculateDistance();
  Serial.printf("resetting motorPositions, rotationAngle: %f, \t distance: %f \n", rotationAngle, distance);
}

void initializeClickToDrive()
{
  Serial.printf("initializing CTD\n");

  resetMotorPosition();

  CTDIsActive = true;
  CTDIsRotating = true;
  calculateRotationAngle();
  calculateDistance();

  Serial.printf("rotationTarget: %f \t rotationAngle: %f \n", rotationTarget, rotationAngle);
  Serial.printf("distanceTarget: %f \t distance: %f \n", distanceTarget, distance);
}

// void calculateClickToDrive()
// {
//   clickToDriveStamp = millis();
//   float newRotationDirection = rotationTarget / abs(rotationTarget);
//   // Serial.printf("latestRotationDirection: %f \n", latestRotationDirection);
//   // Serial.printf("newRotationDirection: %f \n", newRotationDirection);
//   float rotationTimeAdjustment = abs(newRotationDirection - latestRotationDirection);
//   // Serial.printf("rotationTimeAdjustment: %f \n", rotationTimeAdjustment);

//   rotationDuration = rotationTimeAdjustment * directionChangeDuration + startingRotationDuration + abs(rotationTarget) * millisPerDegree;
//   rotationDuration = max(0, rotationDuration);
//   driveDuration = int(distanceTarget / defaultSpeed * 1000.0f);

//   Serial.print("calculate ctd durations: ");
//   Serial.print(rotationDuration);
//   Serial.print("  //  ");
//   Serial.println(driveDuration);
// }

// TODO: Find out whether we need to throttle this function to not overload the orion boards serialport.
void updateClickToDrive()
{
  if (!CTDIsActive)
  {
    return;
  }
  // Serial.printf("rotationAngle: %f, \t distance: %f \n", rotationAngle, distance);

  // unsigned long now = millis();
  // if (now - clickToDriveStamp < rotationDuration)
  // {
  //   outgoingMotorMsg.driveAngle = 0.0f;
  //   outgoingMotorMsg.driveSpeed = 0.0f;
  //   outgoingMotorMsg.rotationSpeed = rotationTarget != 0 ? rotationTarget / abs(rotationTarget) : 0;

  //   Serial1.write((const char *)&outgoingMotorMsg, sizeof(teensyOrionMsgType));
  // }
  // else if (now - clickToDriveStamp < (rotationDuration + driveDuration))
  // {
  //   outgoingMotorMsg.driveAngle = 0.0f;
  //   outgoingMotorMsg.driveSpeed = 1.0f;
  //   outgoingMotorMsg.rotationSpeed = 0.0f;

  //   Serial1.write((const char *)&outgoingMotorMsg, sizeof(teensyOrionMsgType));
  // }

  // Serial.printf("updating CTD \n");
  // Serial.printf("abs(%f) - abs(%f) = %f \n", rotationTarget, rotationAngle, (abs(rotationTarget) - abs(rotationAngle)));
  float angleUntilDone = abs(rotationTarget) - abs(rotationAngle) + motorPositionRotationAngleOffset;
  if(CTDIsRotating && angleUntilDone > 1.5f)
  {
    // Serial.printf("sending rotate command, target: %f, currentAngle: %f \n", rotationTarget, rotationAngle);
    float dynamicRotSpeed = rotationTarget / abs(rotationTarget);
    if (angleUntilDone < 20.0f)
    {
      Serial.printf("slower in the end \n");
      dynamicRotSpeed *= 0.6;
    }
    outgoingMotorMsg.driveAngle = 0.0f;
    outgoingMotorMsg.driveSpeed = 0.0f;
    outgoingMotorMsg.rotationSpeed = rotationTarget != 0.f ? dynamicRotSpeed : 0;

    Serial1.write((const char *)&outgoingMotorMsg, sizeof(teensyOrionMsgType));
  }
  else
  {
    if (CTDIsRotating)
    {
      CTDIsRotating = false;
      resetMotorPosition();
    }

    if (distance < distanceTarget)
    {
      // Serial.printf("sending forward command, target: %f, currentDistance: %f \n", distanceTarget, distance);

      outgoingMotorMsg.driveAngle = 0.0f;
      outgoingMotorMsg.driveSpeed = 1.0f;
      outgoingMotorMsg.rotationSpeed = 0.0f;

      Serial1.write((const char *)&outgoingMotorMsg, sizeof(teensyOrionMsgType));
    }
    else
    {
      CTDIsActive = false;
      // Serial.printf("CTD reached target\n");
    }
  }
}

void readPingSensors()
{
  // TODO implement
}

void readBumpers()
{
  bumperValueFR = digitalRead(bumperPinFR);
  bumperValueRF = digitalRead(bumperPinRF);
  bumperValueRB = digitalRead(bumperPinRB);
  bumperValueBR = digitalRead(bumperPinBR);
  bumperValueBL = digitalRead(bumperPinBL);
  bumperValueLB = digitalRead(bumperPinLB);
  bumperValueLF = digitalRead(bumperPinLF);
  bumperValueFL = digitalRead(bumperPinFL);

  if (bumperValueFL || bumperValueFR)
  {
    obstacleDirections[DIR_FRONT] = 0;
  }
  if (bumperValueFR || bumperValueRF)
  {
    obstacleDirections[DIR_FRONT_RIGHT] = 0;
  }
  if (bumperValueRF || bumperValueRB)
  {
    obstacleDirections[DIR_RIGHT] = 0;
  }
  if (bumperValueRB || bumperValueBR)
  {
    obstacleDirections[DIR_BACK_RIGHT] = 0;
  }
  if (bumperValueBR || bumperValueBL)
  {
    obstacleDirections[DIR_BACK] = 0;
  }
  if (bumperValueBL || bumperValueLB)
  {
    obstacleDirections[DIR_BACK_LEFT] = 0;
  }
  if (bumperValueLB || bumperValueLF)
  {
    obstacleDirections[DIR_LEFT] = 0;
  }
  if (bumperValueLF || bumperValueFL)
  {
    obstacleDirections[DIR_FRONT_LEFT] = 0;
  }
}

void blink(int delayTime)
{
  ledOn();
  delay(delayTime);
  ledOff();
  delay(delayTime);
}

void ledOn()
{
  digitalWrite(ledPin, HIGH);
}

void ledOff()
{
  digitalWrite(ledPin, LOW);
}

void readLidar()
{
  if (!lidarAvailable || !lidarEnabled)
  {
    return;
  }
  //Serial.println("Reading lidar.");
  bool success = device.startScanning();
  if (!success)
  {
    return;
  }
  //Serial.println(success ? "\nSuccessfully initiated scanning..." : "\nFailed to start scanning.");
  while (scanCount < nrOfScansToMake)
  {
    success = false;
    ScanPacket reading = device.getReading(success);
    if (success)
    {
      // check if this reading was the very first reading of a new 360 degree scan
      if (reading.isSync())
        scanCount++;

      // don't collect more than nrOfScansToMake scans
      if (scanCount > nrOfScansToMake)
        break;

      // store the info for this sample
      syncValues[sampleCount] = reading.isSync();
      angles[sampleCount] = reading.getAngleDegrees();
      distances[sampleCount] = reading.getDistanceCentimeters();
      signalStrengths[sampleCount] = reading.getSignalStrength();

      // increment sample count
      sampleCount++;
    }
  }

  success = device.stopScanning();
  //Serial.println(success ? "\nSuccessfully stopped scanning." : "\nFailed to stop scanning.");
  //printCollectedLidarData();
  analizeLidarData();
  resetLidarValues();
}

void setupLidar()
{
  Serial.println("Setting up lidar..");
  // reset the sensor
  device.reset();
  delay(50);
  Serial.flush();
  lidarAvailable = device.setMotorSpeed(MOTOR_SPEED_CODE_10_HZ);
  Serial.println(lidarAvailable ? "\nSuccessfully set motor speed." : "\nFailed to set motor speed");
  // Set the sample rate (codes available for 500, 750 and 1000 HZ)
  bool bSuccess = device.setSampleRate(SAMPLE_RATE_CODE_1000_HZ);
  Serial.println(bSuccess ? "\nSuccessfully set sample rate." : "\nFailed to set sample rate.");

  //bool success = device.startScanning();
  //Serial.println(success ? "\nSuccessfully initiated scanning." : "\nFailed to start scanning.");
}

// Resets the variables
void resetLidarValues()
{
  scanCount = 0;
  sampleCount = 0;
}

void analizeLidarData()
{
  int indexOfFirstSyncReading = 0;
  // skip the trailing readings from the first partial scan
  while (!syncValues[indexOfFirstSyncReading])
  {
    indexOfFirstSyncReading++;
  }

  for (int i = indexOfFirstSyncReading; i < sampleCount; i++)
  {
    if (syncValues[i])
    {
      //Serial.println("\n----------------------NEW SCAN----------------------");
    }
    //Serial.println("Angle: " + String(angles[i], 3) + ", Distance: " + String(distances[i]) + ", Signal Strength: " + String(signalStrengths[i]));
    //Serial.print(String(distances[i]) + ",");
    //Serial.println(String(distances[i]) + ", "+ String(angles[i], 3));

    // values 1 and 0 seem to be noise values, so skip those.
    // TODO Maybe figure out why this happens?
    if (distances[i] <= 1)
    {
      continue;
    }

    // NOTE: angle increments counter-clockwise for the lidar
    if (angles[i] < 12.5 || angles[i] >= 337.5)
    {
      obstacleDirections[DIR_FRONT] = min(distances[i], obstacleDirections[DIR_FRONT]);
    }
    else if (angles[i] >= 12.5 && angles[i] < 67.5)
    {
      obstacleDirections[DIR_FRONT_LEFT] = min(distances[i], obstacleDirections[DIR_FRONT_LEFT]);
    }
    else if (angles[i] >= 67.5 && angles[i] < 112.5)
    {
      obstacleDirections[DIR_LEFT] = min(distances[i], obstacleDirections[DIR_LEFT]);
    }
    else if (angles[i] >= 112.5 && angles[i] < 157.5)
    {
      //obstacleDirections[DIR_BACK_LEFT] = min(distances[i], obstacleDirections[DIR_BACK_LEFT]);
    }
    else if (angles[i] >= 157.5 && angles[i] < 202.5)
    {
      //obstacleDirections[DIR_BACK] = min(distances[i], obstacleDirections[DIR_BACK]);
    }
    else if (angles[i] >= 202.5 && angles[i] < 247.5)
    {
      //obstacleDirections[DIR_BACK_RIGHT] = min(distances[i], obstacleDirections[DIR_BACK_RIGHT]);
    }
    else if (angles[i] >= 247.5 && angles[i] < 292.5)
    {
      obstacleDirections[DIR_RIGHT] = min(distances[i], obstacleDirections[DIR_RIGHT]);
    }
    else if (angles[i] >= 292.5 && angles[i] < 337.5)
    {
      obstacleDirections[DIR_FRONT_RIGHT] = min(distances[i], obstacleDirections[DIR_FRONT_RIGHT]);
    }
  }
}

void readOrionSerial()
{
  if (Serial1.available() < sizeof(teensyOrionMsgType))
  {
    return;
  }

  Serial1.readBytes((char *)&incomingMotorMsg, sizeof(teensyOrionMsgType));

  orionReceiveInterval = millis() - orionReceiveSerialStamp;
  orionReceiveSerialStamp = millis();

  // Serial.printf("orionReceiveInterval: %i \n", orionReceiveInterval);
  // Serial.printf("orion loopTime: %i \n", incomingMotorMsg.loopTime);

  if (incomingMotorMsg.confirm == CONFIRM_SYNC_ERROR)
  {
    Serial.printf("<!Teensy> Orion says it had sync error \n");
  }
  else if (incomingMotorMsg.confirm != CONFIRM_CORRECT)
  {
    Serial.print("<!Teensy> Orion out of sync Confirm=(");
    Serial.print(incomingMotorMsg.confirm);
    Serial.println(")");
    Serial.printf("expecting %i bytes in message \n", sizeof(teensyOrionMsgType));

    int flushDuration = min(25, orionReceiveInterval / 2);
    delay(flushDuration);
    // empty the incoming serial buffer
    while (Serial1.available() > 0)
    {
      Serial1.read();
    }
    // delay(5);
    return;
  }

  calculateRotationAngle();
  calculateDistance();
  // Serial.printf("loopTime: %i, \t", loopTime);
  // Serial.printf("rotationAngle: %f, \t distance: %f \n", rotationAngle, distance);

  // Serial.printf("motorStats received: \n");
  // for(int i = 0; i < 4; i++)
  // {
  //   Serial.printf("motor %i, speed: %f \t position: %f \n", i, incomingMotorMsg.motorSpeed[i], incomingMotorMsg.motorPosition[i]);
  // }
}

void readWebSerial()
{
  if (Serial.available() < (int)sizeof(webSerialMsgType))
  {
    return;
  }
  Serial.readBytes((char *)&webMsg, sizeof(webSerialMsgType));

  if (webMsg.confirm != CONFIRM_CORRECT)
  {
    Serial.print("<!> Out of sync with webSerial! (Confirm=");
    Serial.print(webMsg.confirm);
    Serial.println(")");
    Serial.printf("I expect %i bytes\n", sizeof(webSerialMsgType));
    // empty the input buffer from web serial
    // this is a ugly hack which might help to get it synced again?
    // TODO something better, or be sure about if this is ok.
    while (Serial.available() > 0)
    {
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

  // Serial.print("  \\  ");
  // Serial.print(webMsg.rotationTarget);
  // Serial.print("  //  ");
  // Serial.println(webMsg.distanceTarget);

  // make sure received values are in correct range
  webMsg.driveAngle = fmod((fmod(webMsg.driveAngle, float(2.0 * PI)) + 2.0 * PI), float(2.0 * PI));
  webMsg.driveSpeed = max(min(webMsg.driveSpeed, 1), -1);
  webMsg.rotationSpeed = max(min(webMsg.rotationSpeed, MAX_ROTATIONSPEED), -1.0 * MAX_ROTATIONSPEED);

  // TODO do not allow the robot to drive into obstacles
  // avoidObstacles();

  //copy clickToDrive parameters
  rotationTarget = float(webMsg.rotationTarget);
  distanceTarget = 100.0 * min(webMsg.distanceTarget, 5.0);

  // TODO: implement this in videoclick logic client side instead!
  // if (distanceTarget < 0.5f)
  // {
  //   distanceTarget = 0.0f;
  // }

  if (rotationTarget != 0.f || distanceTarget != 0.f)
  {
    // calculateClickToDrive();
    initializeClickToDrive();
  }
  else
  // Only send a command if it was not meant as a click-to-drive
  // or if there is also a key command at the same time.
  // if ((rotationTarget == 0 && distanceTarget == 0)
  // // || (webMsg.driveAngle != 0 && webMsg.driveSpeed != 0 && webMsg.rotationSpeed != 0)
  // )
  {
    CTDIsActive = false;
    //copy received values to motormessage
    outgoingMotorMsg.driveAngle = webMsg.driveAngle;
    outgoingMotorMsg.driveSpeed = webMsg.driveSpeed;
    outgoingMotorMsg.rotationSpeed = webMsg.rotationSpeed;

    // write to orion
    Serial1.write((const char *)&outgoingMotorMsg, sizeof(teensyOrionMsgType));
  }

  //Save in which direction we are turning
  //We use this info for our ugly hack to adjust for the PID taking longer time to change direction
  // if (webMsg.driveSpeed != 0)
  // {
  //   latestRotationDirection = 0;
  //   latestRotationDirection += webMsg.rotationSpeed;
  // }
  // else if (webMsg.rotationSpeed)
  // {
  //   //scale to 1
  //   latestRotationDirection = 2 * webMsg.rotationSpeed;
  // }
  // else if (distanceTarget != 0)
  // {
  //   latestRotationDirection = 0;
  // }
  // else if (rotationTarget != 0)
  // {
  //   latestRotationDirection = (rotationTarget / abs(rotationTarget));
  // }
  // Serial.printf("latestRotationDirection: %f \n", latestRotationDirection);

  // limit the servo values to [0, 180] range
  webMsg.pitch = min(max(webMsg.pitch, 0), 180);
  webMsg.yaw = min(max(webMsg.yaw, 0), 180);
  webMsg.height = min(max(webMsg.height, 0), 180);

  // write to servos
  servoPitch.write(webMsg.pitch);
  servoYaw.write(webMsg.yaw);
  servoHeight.write(webMsg.height);
}

void avoidObstacles()
{
  // if distance in the drive direction is close by, drive slow
  // if distance in the drive direction is critically close by, full stop.

  // 0 forward
  // 0.5 pi left
  // pi backward
  // 1.5 pi right

  // directions are (in order): forward, forward right, right, backward right,
  // backward, backward left, left, forward left

  // find which direction (0-7 clockwise) corresponds to the driveAngle(0-2PI counterclockwise)
  int direction = round((2.0f - webMsg.driveAngle / PI) * 4.0f);

  if (direction > 7)
  {
    direction = 0;
  }

  if (obstacleDirections[direction] < CRITICAL_DISTANCE)
  {
    Serial.print("Direction ");
    Serial.print(direction);
    Serial.print(" not allowed.");
    webMsg.driveSpeed = 0.0f;
  }
  else if (obstacleDirections[direction] < MEDIUM_DISTANCE)
  {
    Serial.print("Direction ");
    Serial.print(direction);
    Serial.print(" at half speed.");
    // TODO Is this a good idea? Going half the speed they wanted.
    webMsg.driveSpeed /= 2.0f;
  }
}