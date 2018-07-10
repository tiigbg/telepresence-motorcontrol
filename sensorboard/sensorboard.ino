#include <ScanPacket.h>
#include <Sweep.h>

#include <Servo.h>

// this tracks how far away an obstacle is detected
// in 8 directions.
// directions are (in order): forward, forward right, right, backward right,
// backward, backward left, left, forward left
#define MAX_DISTANSE 999
int obstacleDirections[] = {MAX_DISTANSE, MAX_DISTANSE, MAX_DISTANSE, MAX_DISTANSE, MAX_DISTANSE, MAX_DISTANSE, MAX_DISTANSE, MAX_DISTANSE};
#define DIR_FRONT 0
#define DIR_FRONT_RIGHT 1
#define DIR_RIGHT 2
#define DIR_BACK_RIGHT 3
#define DIR_BACK 4
#define DIR_BACK_LEFT 5
#define DIR_LEFT 6
#define DIR_FRONT_LEFT 7

int ledPin = 13;

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

Servo servoPitch, servoYaw, servoHeight;

// === Lidar ===

Sweep device(Serial3);
// keeps track of how many scans have been collected
uint8_t scanCount = 0;
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
  
  Serial.begin(115200); // USB
  Serial1.begin(115200); // Orion board
  Serial3.begin(115200); // sweep device
  resetLidar();

  
  
  Serial.println("Teensy started ok");
  blink(200);
  blink(200);
  blink(200);
}

void loop() {
  /*
  while(Serial1.available() > 0)
  {
    Serial.print(char(Serial1.read()));
  }
  */

  // reset readings
  for(int i = 0; i < 8; i++) {
    obstacleDirections[i] = MAX_DISTANSE;
  }

  readBumpers();
  readPingSensors();

  // Lidar
  readLidar();
  // ---

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

  blink(50);
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
  digitalWrite(ledPin, HIGH);
  delay(delayTime);
  digitalWrite(ledPin, LOW);
  delay(delayTime);
}

void readLidar() {
  Serial.println("Reading lidar.");
  bool success = device.startScanning();
  Serial.println(success ? "\nSuccessfully initiated scanning..." : "\nFailed to start scanning.");
  while(scanCount < 3)
  {
    success = false;
    ScanPacket reading = device.getReading(success);
    if (success)
    {
      // check if this reading was the very first reading of a new 360 degree scan
      if (reading.isSync())
        scanCount++;

      // don't collect more than 3 scans
      if (scanCount > 3)
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
  Serial.println(success ? "\nSuccessfully stopped scanning." : "\nFailed to stop scanning.");
  printCollectedLidarData();
}

// Resets the variables
void resetLidar()
{
  Serial.println("Resetting lidar.");
  scanCount = 0;
  sampleCount = 0;
  // reset the sensor
  device.reset();
  delay(50);
  Serial.flush();

  //bool success = device.setMotorSpeed(MOTOR_SPEED_CODE_5_HZ);
  //Serial.println(success ? "\nSuccessfully set motor speed." : "\nFailed to set motor speed");
}

// Prints the collected data to the console
// (only prints the complete scans, ignores the first partial)
void printCollectedLidarData()
{
  Serial.println("\nPrinting info for the collected scans (NOT REAL-TIME):");

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
      Serial.println("\n----------------------NEW SCAN----------------------");
    }
    //Serial.println("Angle: " + String(angles[i], 3) + ", Distance: " + String(distances[i]) + ", Signal Strength: " + String(signalStrengths[i]));
    Serial.print(distances[i] + ",");
  }
}