#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Wire.h>

#define HAND 4    // Sets Digital 2 pin as hand sensor
#define FOREARM 3 // Sets Digital 3 pin as forearm sensor
#define BACK 2    // Sets Digital 4 pin as back sensor

#define NODEVICE 0
#define HAND_ID 1
#define FOREARM_ID 2
#define BACK_ID 3

#define NODEVICE_STRING "0"
#define HAND_ID_STRING "1"
#define FOREARM_ID_STRING "2"
#define BACK_ID_STRING "3"

#define STACK_SIZE 1200

#define NACK 0
#define ACK 1
#define HELLO 2
#define MESSAGE 3
#define TERMINATE 4

#define NACK_STRING "0"
#define ACK_STRING "1"
#define HELLO_STRING "2"
#define MESSAGE_STRING "3"
#define TERMINATE_STRING "4"

char debugBuffer[500];
char sendBuffer[1000];
char receiveBuffer[30];

short gyroX=0;
short gyroY=0;
short gyroZ=0;

int handshake=0;
int val=0;

typedef struct packet {
  char packetId;
  char deviceId;
  char deviceId2;
  char deviceId3;
  float data[18];
} dataPacket;

const int MPU = 0x68; // MPU6050 I2C addresses

//power variables
// Constants
const int CURRENT_PORT = A0;           // Input pin for measuring Vout
const int VOLTAGE_PORT = A1;
const int RS = 10;                      // Shunt resistor value (in ohms)
const int VOLTAGE_REF = 5;              // Reference voltage for analog read
const int RL = 10000;                   // Load resistor value (in ohms)
const int resistance_ina169  = 1000;    // Resistor multiplier due to internal of sensor (in ohms)

// Global Variables
float rawCurrentReading;      // Variable to store value from analog read
float scaledCurrentReading;   // Variable to store the scaled value from the analog value
float current;
float RS2 = 0.1;

float voltageReading;
float scaledVoltage;
float voltage;
float x;
float y;

unsigned long energy;
unsigned long timeVal;
unsigned long prevTime;
unsigned long currTime;
float energyVal;
float power;

int dataCount=0;

struct SensorDataStructure {
  float AccX;  // Accelerometer x-axis value for MPU6050
  float AccY;  // Accelerometer y-axis value for MPU6050
  float AccZ;  // Accelerometer z-axis value for MPU6050
  float GyroX; // Gyrometer x-axis value for MPU6050
  float GyroY; // Gyrometer y-axis value for MPU6050
  float GyroZ; // Gyrometer z-axis value for MPU6050
} HandSensorData, ForearmSensorData, BackSensorData, SensorData;

// function headers
void calcPower();
void transferDataFloatsToPacket(float *, char);
void sendPackets(void *);
unsigned int serialize(char *, dataPacket *);
int startHandshake();
void ReadMPUValues();
void PrintMPUValues(SensorDataStructure);
void CallibrateMPUValues();
void UpdateMPUSensorData();
void DeactivateSensors();
void ExecuteSensor(int, SensorDataStructure);

//testing
float sensorData = 1.0;
// Queue<dataPacket> queue = Queue<dataPacket>(10);
dataPacket messagePacket;
char packetNum = 1;
int newMessageCount = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(HAND, OUTPUT);        // Sets hand digital pin as output pin
  pinMode(FOREARM, OUTPUT);     // Sets forearm digital pin as output pin
  pinMode(BACK, OUTPUT);        // Sets back digital pin as output pin
  
  Wire.begin();                 // Initiates I2C communication
  Wire.beginTransmission(MPU);  // Begins communication with the MPU
  Wire.write(0x6B);             // Access the power management register
  Wire.write(0x00);             // Wakes up the MPU
  Wire.endTransmission(true);   // Communication done

  Serial.begin(115200);
  Serial3.begin(115200);

  int ishandShakeSuccess = startHandshake();
  if (ishandShakeSuccess == 0) {
    Serial.println("handshake failed");
    return;
  }
  else if (ishandShakeSuccess == 1) {
    Serial.println("handshake success");
  }

  //delay(10);

  //sendPackets();
  
  xTaskCreate(sendPackets, "sendPackets", STACK_SIZE, NULL, 2, NULL);
//  //xTaskCreate(receiveData, "receiveData", STACK_SIZE, NULL, 2, NULL);
  vTaskStartScheduler();
}

void calcPower() {
//  scaledVoltage = 2.0;
//  current = 2.0;
//  energyVal = 2.0;
//  power = 2.0;
  
  // Read sensor value from INA169
  rawCurrentReading = analogRead(CURRENT_PORT);

  // Scale the value to supply voltage that is 5V
  scaledCurrentReading = (rawCurrentReading * VOLTAGE_REF) / 1023; //send

  // Is = (Vout x 1k) / (RS x RL)
  current = (scaledCurrentReading) / (RS2 * 10);
//  Serial.print("Current: ");
//  Serial.print(current, 9);
//  Serial.println(" A");


  voltageReading = analogRead(VOLTAGE_PORT);
  scaledVoltage = (voltageReading * 5.0 * 2) / 1023;

//  Serial.print("Voltage: ");
//  Serial.print(scaledVoltage, 3);
//  Serial.println(" V");

  currTime = millis();
  power = scaledVoltage * scaledCurrentReading;
  energyVal += (current * scaledVoltage *  (currTime - prevTime)/ 1000) / 3600;

  prevTime = currTime;
//
//  Serial.print("Energy: ");
//  Serial.print(energyVal, 3);
//  Serial.println(" Wh");
//
//  Serial.print("Power: ");
//  Serial.print(power, 3);
//  Serial.println(" W");
}

void transferDataFloatsToPacket(float *arr, char deviceId) {
  int startingIndex = 0;
  if (deviceId == HAND_ID) {
    arr[startingIndex] = HandSensorData.AccX;
    startingIndex++;
    arr[startingIndex] = HandSensorData.AccY;
    startingIndex++;
    arr[startingIndex] = HandSensorData.AccZ;
    startingIndex++;
    arr[startingIndex] = HandSensorData.GyroX;
    startingIndex++;
    arr[startingIndex] = HandSensorData.GyroY;
    startingIndex++;
    arr[startingIndex] = HandSensorData.GyroZ;
    startingIndex++;
  }
  else if (deviceId == FOREARM_ID) {
    startingIndex = 6;
    arr[startingIndex] = ForearmSensorData.AccX;
    startingIndex++;
    arr[startingIndex] = ForearmSensorData.AccY;
    startingIndex++;
    arr[startingIndex] = ForearmSensorData.AccZ;
    startingIndex++;
    arr[startingIndex] = ForearmSensorData.GyroX;
    startingIndex++;
    arr[startingIndex] = ForearmSensorData.GyroY;
    startingIndex++;
    arr[startingIndex] = ForearmSensorData.GyroZ;
    startingIndex++;
  }
  else if (deviceId == BACK_ID) {
    startingIndex = 12;
    arr[startingIndex] = BackSensorData.AccX;
    startingIndex++;
    arr[startingIndex] = BackSensorData.AccY;
    startingIndex++;
    arr[startingIndex] = BackSensorData.AccZ;
    startingIndex++;
    arr[startingIndex] = BackSensorData.GyroX;
    startingIndex++;
    arr[startingIndex] = BackSensorData.GyroY;
    startingIndex++;
    arr[startingIndex] = BackSensorData.GyroZ;
    startingIndex++;
  }
}

void ExecuteSensor(int value, SensorDataStructure sds) {
  DeactivateSensors();
  digitalWrite(value, LOW);  // Activates sensor
  //delay(100);
  ReadMPUValues();
  UpdateMPUSensorData();
  PrintMPUValues(sds);
}

void sendPackets(void *p) {
//  TickType_t xLastWakeTime;
//  const TickType_t xFrequency = 1;
//
// // Initialise the xLastWakeTime variable with the current time.
//  xLastWakeTime = xTaskGetTickCount();
  while(1) {
    //TickType_t xCurrWakeTime = xTaskGetTickCount();

    ExecuteSensor(HAND, HandSensorData);
    ExecuteSensor(FOREARM, ForearmSensorData);
    ExecuteSensor(BACK, BackSensorData);
  
    // convert to dataPacket
    dataPacket messagePacket;
    messagePacket.packetId = MESSAGE;
  
    messagePacket.deviceId = HAND_ID;
    messagePacket.deviceId2 = FOREARM_ID;
    messagePacket.deviceId3 = BACK_ID;
  
    transferDataFloatsToPacket(messagePacket.data, HAND_ID);
    transferDataFloatsToPacket(messagePacket.data, FOREARM_ID);
    transferDataFloatsToPacket(messagePacket.data, BACK_ID);
  
    calcPower();
    
      // serialise into packets and send
    serialize(sendBuffer, &messagePacket);
    int bytesWritten = Serial3.write(sendBuffer);
    //Serial.println(bytesWritten);
  
    // receive ack from RPi
    int numBytes = 0;
    char packetLength = -1;
    unsigned char packetid = 0;
    char checksum = -1;
    int loopCounter = 0;
    int ackFailed = 0;
    while (numBytes < 3) {
      //Serial.println("waiting for ack for message");
  
      if (Serial3.available() > 0) {
        if (numBytes == 0)
          packetLength = (char)Serial3.read();
        else if (numBytes == 1)
          packetid = (unsigned char)Serial3.read();
        else if (numBytes == 2)
          checksum = (char)Serial3.read();
        numBytes++;
      }
  
      loopCounter++;
      if (loopCounter == 30) {
        //Serial.println("waited for ack too long");
        ackFailed = 1;
        break;
      }
   }
    
    // checksum correct
    if (checksum == packetid) {
      // ack received
      if (packetid != ACK) {
        ackFailed = 1;
      }
      else if (packetid == ACK)
        dataCount++;
    }
    else {
      ackFailed = 1;
    }  
  
    if (ackFailed == 1) {
      //Serial.println("ack failed!");
      // Serial3.write(sendBuffer);
    }
  
    //Serial.println(dataCount);
    delay(5);
//    // Wait for the next cycle.
    //vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  // sendPackets();
}

unsigned int serialize(char *buf, dataPacket *p) {
  strcpy(buf, "");
  int checksum = 0;
  char sensorDataBuf[7];
  float valueToConvert = 0.0;
  float test = 0.15;
  char packetIdFromPacket = p->packetId;
  char deviceIdFromPacket = p->deviceId;
  char deviceId2FromPacket = p->deviceId2;
  char deviceId3FromPacket = p->deviceId3;

  // serialising ack packet to send
  if (packetIdFromPacket == ACK) {
    strcat(buf, ACK_STRING);
    strcat(buf, "1");
  }

  // serialising message packet
  else if (packetIdFromPacket == MESSAGE) {
    strcat(buf, "#");
    strcat(buf, MESSAGE_STRING);
    strcat(buf, HAND_ID_STRING);
    strcat(buf, FOREARM_ID_STRING);
    strcat(buf, BACK_ID_STRING);
    
    strcat(buf, "(");

  // semicolon differentiates individual axes values for each sensor
    for (int i=0; i<18; i++) {
      valueToConvert = (p->data)[i];
      checksum ^= (int)valueToConvert;
      dtostrf(valueToConvert, 1, 2, sensorDataBuf);
  
      strcat(buf, sensorDataBuf);
  
      // differentiate by sensor
      if ((i +1) % 6 == 0) {
        strcat(buf, "(");
      }
      else {
        strcat(buf, ";");
      }
    }

    // voltage, current, power, energy
    //checksum ^= (int)scaledVoltage;
    dtostrf(scaledVoltage, 1, 2, sensorDataBuf);
    strcat(buf, sensorDataBuf);
    strcat(buf, "(");
    
    //checksum ^= (int)current;
    dtostrf(current, 1, 2, sensorDataBuf);
    strcat(buf, sensorDataBuf);
    strcat(buf, "(");

    //checksum ^= (int)power;
    dtostrf(power, 1, 2, sensorDataBuf);
    strcat(buf, sensorDataBuf);
    strcat(buf, "(");

    //checksum ^= (int)energyVal;
    dtostrf(energyVal, 1, 2, sensorDataBuf);
    strcat(buf, sensorDataBuf);
    strcat(buf, "(");

    // evaluate and append checksum
    dtostrf(checksum, 1, 0, sensorDataBuf);
    strcat(buf, sensorDataBuf);
  }  
//  Serial.print("checksum: ");
//  Serial.println(checksum);
  strcat(buf, "\n");
}

int startHandshake(){

  //Serial.println("Starting handshake..");
  int handShakeStage = 0;

  char nonDataBuf[4];

  dataPacket helloPacket;
  helloPacket.packetId = 2;

  int numBytes = 0;
  char packetLength = -1;
  unsigned char packetid = 0;
  char checksum = -1;
  // receive hello from RPi (packet only contains length, packet id and checksum)
  while (numBytes < 3) {
    Serial.println("in loop waiting for hello");

    if (Serial3.available() > 0) {
      if (numBytes == 0)
        packetLength = (char)Serial3.read();
      else if (numBytes == 1)
        packetid = (unsigned char)Serial3.read();
      else if (numBytes == 2)
        checksum = (char)Serial3.read();
      numBytes++;
    }
 }

  Serial.println("some bytes received");
//  dprintf("packetLength: %d", packetLength);
//  dprintf("packetid: %d", packetid);
//  dprintf("checksum: %d", checksum);

  // checksum correct
  if (checksum == packetid) {
    // hello received
    if (packetid != HELLO) {
      return 0;
    }
  }
  else {
    return 0;
  }  
  
  Serial.println("hello received!");
  

  // send ack to RPi
  dataPacket ackPacket;
  ackPacket.packetId = ACK;
  
  serialize(sendBuffer, &ackPacket);
  Serial3.write(sendBuffer);

  Serial.println("Ack sent!");

  // receive ack from RPi
  numBytes = 0;
  packetLength = -1;
  packetid = 0;
  checksum = -1;
  int loopCounter = 0;
  while (numBytes < 3) {
    Serial.println("in loop waiting for ack");

    if (Serial3.available() > 0) {
      if (numBytes == 0)
        packetLength = (char)Serial3.read();
      else if (numBytes == 1)
        packetid = (unsigned char)Serial3.read();
      else if (numBytes == 2)
        checksum = (char)Serial3.read();
      numBytes++;
    }

    loopCounter++;
    if (loopCounter == 50)
      break;
 }

//  dprintf("packetLength: %d", packetLength);
//  dprintf("packetid: %d", packetid);
//  dprintf("checksum: %d", checksum);
  // checksum correct
  if (checksum == packetid) {
    // ack received
    if (packetid != ACK) {
      return 0;
    }
  }
  else {
    return 0;
  }  
  
  Serial.println("ack received!");

  

  return 1;
}
  
void ReadMPUValues() {
  Wire.beginTransmission(MPU);      // Begins communication with the MPU
  Wire.write(0x3B);                 // Register 0x3B upper 8 bits of x-axis acceleration data
  Wire.endTransmission(false);      // End communication
  Wire.requestFrom(MPU, 12, true);  // Request 12 registers

//  // testing with sample data
//  SensorData.AccX  = sensorData; // Reads in raw x-axis acceleration data
//  SensorData.AccY  = sensorData; // Reads in raw y-axis acceleration data
//  SensorData.AccZ  = sensorData; // Reads in raw z-axis acceleration data
//  SensorData.GyroX = sensorData; // Reads in raw x-axis gyroscope data
//  SensorData.GyroY = sensorData; // Reads in raw y-axis gyroscope data
//  SensorData.GyroZ = sensorData; // Reads in raw z-axis gyroscope data
//  sensorData += 2.0;
  
  SensorData.AccX  = Wire.read() << 8 | Wire.read(); // Reads in raw x-axis acceleration data
  SensorData.AccY  = Wire.read() << 8 | Wire.read(); // Reads in raw y-axis acceleration data
  SensorData.AccZ  = Wire.read() << 8 | Wire.read(); // Reads in raw z-axis acceleration data
  Wire.read(); Wire.read();                          // Reads in raw temperature data
  SensorData.GyroX = Wire.read() << 8 | Wire.read(); // Reads in raw x-axis gyroscope data
  SensorData.GyroY = Wire.read() << 8 | Wire.read(); // Reads in raw y-axis gyroscope data
  SensorData.GyroZ = Wire.read() << 8 | Wire.read(); // Reads in raw z-axis gyroscope data
}

void PrintMPUValues(SensorDataStructure SDS) {
  Serial.print("Accelerometer Values: [x = "); Serial.print(SDS.AccX);
  Serial.print(", y = "); Serial.print( SDS.AccY);
  Serial.print(", z = "); Serial.print(SDS.AccZ); Serial.println("]"); 
  Serial.print("Gyrorometer Values:   [x = "); Serial.print(SDS.GyroX);
  Serial.print(", y = "); Serial.print(SDS.GyroY);
  Serial.print(", z = "); Serial.print(SDS.GyroZ); Serial.println("]");
  Serial.println();
}

void CallibrateMPUValues() {
  SensorData.AccX = SensorData.AccX/16384.0;
  SensorData.AccY = SensorData.AccY/16384.0;
  SensorData.AccZ = SensorData.AccZ/16384.0;
  SensorData.GyroX = SensorData.GyroX/131.0;
  SensorData.GyroY = SensorData.GyroY/131.0;
  SensorData.GyroZ = SensorData.GyroZ/131.0;
}

void UpdateMPUSensorData() {
  CallibrateMPUValues();
  if (!digitalRead(HAND)) {
    HandSensorData = SensorData;
    //Serial.println("Hand MPU6050 Readings");
  } else if (!digitalRead(FOREARM)) {
    ForearmSensorData = SensorData;
    //Serial.println("Forearm MPU6050 Readings");
  } else if (!digitalRead(BACK)) {
    BackSensorData = SensorData;
    //Serial.println("Back MPU6050 Readings");
  }
}

void DeactivateSensors() {
  digitalWrite(HAND, HIGH);     // Deactivates hand sensor
  digitalWrite(FOREARM, HIGH);  // Deactivates forearm sensor
  digitalWrite(BACK, HIGH);     // Deactivates back sensor
}
