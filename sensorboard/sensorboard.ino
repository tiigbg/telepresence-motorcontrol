#include <ScanPacket.h>
#include <Sweep.h>

#include <Servo.h>

// this tracks how far away an obstacle is detected
// in 8 directions.
// directions are (in order): forward, forward right, right, backward right,
// backward, backward left, left, forward left
#define MEDIUM_DISTANCE 60
#define CRITICAL_DISTANCE 20
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

struct webSerialMsgType {
  float driveAngle = 0.0;
  float driveSpeed = 0.0;
  float rotationSpeed = 0.0;
  uint16_t pitch = 90;
  uint16_t yaw = 90;
  uint16_t height = 90;
  uint16_t confirm = CONFIRM_CORRECT;
} webMsg;

struct teensyOrionMsgType {
  uint16_t confirm = CONFIRM_CORRECT;
  // float driveAngle = 0.0;
  // float driveSpeed = 0.0;
  // float rotationSpeed = 0.0;
} motorMsg;




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

void setup() {                
  pinMode(ledPin, OUTPUT);
  blink(50);
  blink(300);
  blink(50);
  blink(300);

  Serial.begin(9600); // USB
  Serial1.begin(9600); // Orion board
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
  
  lidarEnabled = digitalRead(lidarEnabledPin);
  if(lidarEnabled) {
    ledOn();
    setupLidar();
    ledOff();
    resetLidarValues();
  }

  Serial.println("Teensy started ok");
  blink(200);
  blink(200);
  blink(200);
}

void loop() {
  readOrionSerial();
  delay(1000);
  Serial1.write((const char *) &motorMsg, sizeof(teensyOrionMsgType));

  // readWebSerial();

  // reset readings
  // for(int i = 0; i < 8; i++) {
  //   obstacleDirections[i] = MAX_DISTANCE;
  // }

  // readPingSensors();
  // readLidar();
  // readBumpers();

  // blink(5);
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
  ledOn();
  delay(delayTime);
  ledOff();
  delay(delayTime);
}

void ledOn() {
  digitalWrite(ledPin, HIGH);
}

void ledOff() {
  digitalWrite(ledPin, LOW);
}

void readLidar() {
  if(!lidarAvailable || !lidarEnabled)
  {
    return;
  }
  //Serial.println("Reading lidar.");
  bool success = device.startScanning();
  if(!success)
  {
    return;
  }
  //Serial.println(success ? "\nSuccessfully initiated scanning..." : "\nFailed to start scanning.");
  while(scanCount < nrOfScansToMake)
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

void setupLidar() {
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

void analizeLidarData() {
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
    if(distances[i] <= 1)
    {
      continue;
    }

    // NOTE: angle increments counter-clockwise for the lidar
    if(angles[i] < 12.5 || angles[i] >= 337.5)
    {
      obstacleDirections[DIR_FRONT] = min(distances[i], obstacleDirections[DIR_FRONT]);
    }else if(angles[i] >= 12.5 && angles[i] < 67.5)
    {
      obstacleDirections[DIR_FRONT_LEFT] = min(distances[i], obstacleDirections[DIR_FRONT_LEFT]);
    }else if(angles[i] >= 67.5 && angles[i] < 112.5)
    {
      obstacleDirections[DIR_LEFT] = min(distances[i], obstacleDirections[DIR_LEFT]);
    }else if(angles[i] >= 112.5 && angles[i] < 157.5)
    {
      //obstacleDirections[DIR_BACK_LEFT] = min(distances[i], obstacleDirections[DIR_BACK_LEFT]);
    }else if(angles[i] >= 157.5 && angles[i] < 202.5)
    {
      //obstacleDirections[DIR_BACK] = min(distances[i], obstacleDirections[DIR_BACK]);
    }else if(angles[i] >= 202.5 && angles[i] < 247.5)
    {
      //obstacleDirections[DIR_BACK_RIGHT] = min(distances[i], obstacleDirections[DIR_BACK_RIGHT]);
    }else if(angles[i] >= 247.5 && angles[i] < 292.5)
    {
      obstacleDirections[DIR_RIGHT] = min(distances[i], obstacleDirections[DIR_RIGHT]);
    }else if(angles[i] >= 292.5 && angles[i] < 337.5)
    {
      obstacleDirections[DIR_FRONT_RIGHT] = min(distances[i], obstacleDirections[DIR_FRONT_RIGHT]);
    }
  }
}

void readOrionSerial() {
  if(Serial1.available() == 0) {
    return;
  }
  // Serial.flush();
  // we are out of sync with orion!
  Serial.println("<!> Out of sync with Orion! [");
  String input = "";
  while(Serial1.available() > 0) {
    input += (char)Serial1.read();
  }
  Serial.print(input);
  Serial.println("]");
  //Serial1.flush();
  blink(10);
  blink(10);

  // write to orion
  // Serial1.write((const char *) &motorMsg, sizeof(teensyOrionMsgType));
}

void readWebSerial() {
  if(Serial.available() < (int) sizeof(webSerialMsgType)) {
    return;
  }
  Serial.readBytes((char *) &webMsg, sizeof(webSerialMsgType));
  
  if(webMsg.confirm != CONFIRM_CORRECT) {
    Serial.print("<!> Out of sync with webSerial! (Confirm=");
    Serial.print(webMsg.confirm);
    Serial.println(")");
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
  webMsg.rotationSpeed = max(min(webMsg.rotationSpeed, MAX_ROTATIONSPEED), -1.0*MAX_ROTATIONSPEED);

  // TODO do not allow the robot to drive into obstacles
  avoidObstacles();

  // write to orion
  Serial1.write((const char *) &motorMsg, sizeof(teensyOrionMsgType));
  
  // limit the servo values to [0, 180] range
  webMsg.pitch = min(max(webMsg.pitch, 0), 180);
  webMsg.yaw = min(max(webMsg.yaw, 0), 180);
  webMsg.height = min(max(webMsg.height, 0), 180);

  // write to servos
  servoPitch.write(webMsg.pitch);
  servoYaw.write(webMsg.yaw);
  servoHeight.write(webMsg.height);
}

void avoidObstacles() {
  // if distance in the drive direction is close by, drive slow
  // if distance in the drive direction is critically close by, full stop.
  
  // 0 forward
  // 0.5 pi left
  // pi backward
  // 1.5 pi right

  // directions are (in order): forward, forward right, right, backward right,
  // backward, backward left, left, forward left

  // find which direction (0-7 clockwise) corresponds to the driveAngle(0-2PI counterclockwise)
  int direction = ceil((2.0f*PI - webMsg.driveAngle) * 4);
  if(direction > 7) {
    direction = 0;
  }
  
  if(obstacleDirections[direction] < CRITICAL_DISTANCE) {
    webMsg.driveSpeed = 0.0f;
  } elseif(obstacleDirections[direction] < MEDIUM_DISTANCE) {
    webMsg.driveSpeed /= 2.0f;
  }

}