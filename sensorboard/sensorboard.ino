#include <ScanPacket.h>
#include <Sweep.h>

#include <Servo.h>

// this tracks how far away an obstacle is detected
// in 8 directions.
// directions are (in order): forward, forward right, right, backward right,
// backward, backward left, left, forward left
#define MAX_DISTANCE 999
//int obstacleDirections[] = {MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE};
#define DIR_FRONT 0
#define DIR_FRONT_RIGHT 1
#define DIR_RIGHT 2
#define DIR_BACK_RIGHT 3
#define DIR_BACK 4
#define DIR_BACK_LEFT 5
#define DIR_LEFT 6
#define DIR_FRONT_LEFT 7

int ledPin = 13;

// === Serial ===

unsigned long lastSerialSend = millis();
unsigned long maxSerialSendTime = 500;
bool serialSynced = false;

#define CONFIRM_CORRECT 123
#define CONFIRM_WRONG 456
struct teensyOrionServoMsgType {
  uint16_t confirm = CONFIRM_CORRECT;
  uint16_t pitch = 90;
  uint16_t yaw = 90;
  uint16_t height = 90;
} servoMsg;

struct teensyOrionDistanceMsgType {
  uint16_t confirm = CONFIRM_CORRECT;
  uint16_t obstacleDirections[8] = {MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE};
} distanceMsg;




// === Bumpers ===

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

  Serial.begin(115200); // USB
  Serial1.begin(9600); // Orion board
  Serial3.begin(115200); // sweep lidar device

  Serial.println("Teensy starting up..");
  //Serial1.println("Hi Makeblock, Teensy is starting..");

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

  servoPitch.write(servoMsg.pitch);
  servoYaw.write(servoMsg.yaw);
  servoHeight.write(servoMsg.height);
  
  
  // TODO find a way to either reduce the timeout or to skip this
  // ledOn();
  // setupLidar();
  // ledOff();
  // resetLidarValues();

  syncWithOrion();

  Serial.println("Teensy started ok");
  blink(200);
  blink(200);
  blink(200);
}

void loop() {
  if(Serial1.available() > 0)
  {
    readOrionSerial();
  }

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
  }

  // reset readings
  for(int i = 0; i < 8; i++) {
    distanceMsg.obstacleDirections[i] = MAX_DISTANCE;
  }

  readPingSensors();
  readLidar();
  readBumpers();

  printDistances();

  blink(50);
}

void printDistances() {
  if(millis() < lastSerialSend + maxSerialSendTime)
  {
    return;
  }
  lastSerialSend = millis();

  /*
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
  */
 Serial.println("Sending");
 Serial.println(distanceMsg.confirm);
 Serial.print("sending size:");
 Serial.println(sizeof(teensyOrionDistanceMsgType));
 Serial1.write((const char *) &distanceMsg, sizeof(teensyOrionDistanceMsgType));

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
    distanceMsg.obstacleDirections[DIR_FRONT] = 0;
  }
  if(bumperValueFR || bumperValueRF)
  {
    distanceMsg.obstacleDirections[DIR_FRONT_RIGHT] = 0;
  }
  if(bumperValueRF || bumperValueRB)
  {
    distanceMsg.obstacleDirections[DIR_RIGHT] = 0;
  }
  if(bumperValueRB || bumperValueBR)
  {
    distanceMsg.obstacleDirections[DIR_BACK_RIGHT] = 0;
  }
  if(bumperValueBR || bumperValueBL)
  {
    distanceMsg.obstacleDirections[DIR_BACK] = 0;
  }
  if(bumperValueBL || bumperValueLB)
  {
    distanceMsg.obstacleDirections[DIR_BACK_LEFT] = 0;
  }
  if(bumperValueLB || bumperValueLF)
  {
    distanceMsg.obstacleDirections[DIR_LEFT] = 0;
  }
  if(bumperValueLF || bumperValueFL)
  {
    distanceMsg.obstacleDirections[DIR_FRONT_LEFT] = 0;
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
  if(!lidarAvailable)
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
      distanceMsg.obstacleDirections[DIR_FRONT] = min(distances[i], distanceMsg.obstacleDirections[DIR_FRONT]);
    }else if(angles[i] >= 12.5 && angles[i] < 67.5)
    {
      distanceMsg.obstacleDirections[DIR_FRONT_LEFT] = min(distances[i], distanceMsg.obstacleDirections[DIR_FRONT_LEFT]);
    }else if(angles[i] >= 67.5 && angles[i] < 112.5)
    {
      distanceMsg.obstacleDirections[DIR_LEFT] = min(distances[i], distanceMsg.obstacleDirections[DIR_LEFT]);
    }else if(angles[i] >= 112.5 && angles[i] < 157.5)
    {
      //distanceMsg.obstacleDirections[DIR_BACK_LEFT] = min(distances[i], distanceMsg.obstacleDirections[DIR_BACK_LEFT]);
    }else if(angles[i] >= 157.5 && angles[i] < 202.5)
    {
      //distanceMsg.obstacleDirections[DIR_BACK] = min(distances[i], distanceMsg.obstacleDirections[DIR_BACK]);
    }else if(angles[i] >= 202.5 && angles[i] < 247.5)
    {
      //distanceMsg.obstacleDirections[DIR_BACK_RIGHT] = min(distances[i], distanceMsg.obstacleDirections[DIR_BACK_RIGHT]);
    }else if(angles[i] >= 247.5 && angles[i] < 292.5)
    {
      distanceMsg.obstacleDirections[DIR_RIGHT] = min(distances[i], distanceMsg.obstacleDirections[DIR_RIGHT]);
    }else if(angles[i] >= 292.5 && angles[i] < 337.5)
    {
      distanceMsg.obstacleDirections[DIR_FRONT_RIGHT] = min(distances[i], distanceMsg.obstacleDirections[DIR_FRONT_RIGHT]);
    }
  }
}

// Prints the collected lidar data to the console
// (only prints the complete scans, ignores the first partial)
void printCollectedLidarData()
{
  //Serial.println("\nPrinting info for the collected scans (NOT REAL-TIME):");

  int indexOfFirstSyncReading = 0;
  // don't print the trailing readings from the first partial scan
  while (!syncValues[indexOfFirstSyncReading])
  {
    indexOfFirstSyncReading++;
  }
  // print the readings for all the complete scans
  for (int i = indexOfFirstSyncReading; i < sampleCount; i++)
  {
    if (syncValues[i])
    {
      //Serial.println("\n----------------------NEW SCAN----------------------");
    }
    //Serial.println("Angle: " + String(angles[i], 3) + ", Distance: " + String(distances[i]) + ", Signal Strength: " + String(signalStrengths[i]));
    //Serial.print(String(distances[i]) + ",");
    Serial.println(String(distances[i]) + ", "+ String(angles[i], 3));
  }
  //Serial.println("0");
}

void readOrionSerial() {
  /*
  String incomingString = "";
  
  while(Serial1.available() > 0)
  {
    incomingString += (char)Serial1.read();
    delay(1);
  }
  Serial.print("From orion:");
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

  
  String firstValue = incomingString.substring(0, firstBreakCharIndex);
  String secondValue = incomingString.substring(firstBreakCharIndex+1, secondBreakCharIndex);
  String thirdValue = incomingString.substring(secondBreakCharIndex+1);

  Serial.println("("+incomingString.substring(firstBreakCharIndex+1, secondBreakCharIndex)+")");
  int servoPitchValue = incomingString.substring(0, firstBreakCharIndex).toInt();
  int servoYawValue = incomingString.substring(firstBreakCharIndex+1, secondBreakCharIndex).toInt();
  int servoHeightValue = incomingString.substring(secondBreakCharIndex+1).toInt();

  servoPitchValue = min(max(servoPitchValue, servoPitchMin), servoPitchMax);
  servoYawValue = min(max(servoYawValue, servoYawMin), servoYawMax);
  servoHeightValue = min(max(servoHeightValue, servoHeightMin), servoHeightMax);
  */

  if(Serial1.available() < (int) sizeof(teensyOrionServoMsgType)) {
    return;
  }
  Serial1.readBytes((char *)&servoMsg, sizeof(teensyOrionServoMsgType) );
  
  if(servoMsg.confirm != CONFIRM_CORRECT)
  {
    Serial.print("<!> Out of sync! (Confirm=");
    Serial.print(servoMsg.confirm);
    Serial.println(")");
    // we are out of sync! Go into re-sync mode.
    serialSynced = false;
    // Send a reply with a wrong confirm value, to make sure orion also goes into re-sync mode
    distanceMsg.confirm = CONFIRM_WRONG;
    delay(1000);
    Serial.println("Sending a wrong response");
    Serial1.write((const char *) &distanceMsg, sizeof(teensyOrionDistanceMsgType));
    delay(1000);
    syncWithOrion();

    return;
  }

  // limit the servo values to [0, 180] range
  servoMsg.pitch = min(max(servoMsg.pitch, 0), 180);
  servoMsg.yaw = min(max(servoMsg.yaw, 0), 180);
  servoMsg.height = min(max(servoMsg.height, 0), 180);

  Serial.print("Writing servos:");
  Serial.print(servoMsg.pitch);
  Serial.print(" // ");
  Serial.print(servoMsg.yaw);
  Serial.print(" // ");
  Serial.println(servoMsg.height);
  servoPitch.write(servoMsg.pitch);
  servoYaw.write(servoMsg.yaw);
  servoHeight.write(servoMsg.height);
}

void syncWithOrion() {
  // wait for sync char from Orion
  Serial.print("Waiting for Orion sync char");
  while(!serialSynced) {
    while(Serial1.available()) {
      uint8_t c = Serial1.read();
      if(c == 's') {
        serialSynced = true;
      }
    }
    Serial.print(".");
    delay(300);
    blink(300);
  }
  // empty the buffer
  delay(10);
  while(Serial1.available()> 0) {
    Serial1.read();
  }
  delay(10);
  // Send sync char to Orion
  Serial1.write('s');
  distanceMsg.confirm = CONFIRM_CORRECT;
  // Make sure we don't send over serial immediately after
  lastSerialSend = millis();
  Serial.println("Synced.");
}