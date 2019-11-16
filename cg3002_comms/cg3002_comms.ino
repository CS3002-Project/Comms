#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Wire.h>

#define HAND 4        // Sets Digital 2 pin as hand sensor
#define FOREARM 3     // Sets Digital 3 pin as forearm sensor
#define BACK 2        // Sets Digital 4 pin as back sensor

#define RS 0.1        // Shunt resistor value
#define RL 10000      // Load resistor value
#define REFVOLTAGE 5  // Reference Voltage for Analog Read
#define RINA169 1000  // Resistor Multiplier due to Internal of Sensor

#define NODEVICE 0
#define HAND_ID 1
#define FOREARM_ID 2
#define BACK_ID 3

#define NODEVICE_STRING "0"
#define HAND_ID_STRING "1"
#define FOREARM_ID_STRING "2"
#define BACK_ID_STRING "3"

#define STACK_SIZE 1000

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

char sendBuffer[150];

const int MPU = 0x68;         // MPU6050 I2C addresses
const int CurrentPort = A14;  // Analog 14 input pin for measuring Vout
const int VoltagePort = A15;  // Analog 15 input pin for measuring voltage divider

float rawCurrentReading;      // Variable to store raw current reading from analog read
float rawVoltageReading;      // Variable to store raw volatge reading from analog read
float scaledCurrentReading;   // Variable to store the scaled value from the analog value

unsigned long prevTime, currTime; // Time to calculate energy

typedef struct packet {
  char packetId;  // Packet ID
  char deviceId;  // Device ID for Hand Sensor
  char deviceId2; // Device ID for Forearm Sensor
  char deviceId3; // Device ID for Back Sensor
  float data[18]; // Data for Sensors
} dataPacket;

struct SensorDataStructure {
  float AccX;  // Accelerometer x-axis value for MPU6050
  float AccY;  // Accelerometer y-axis value for MPU6050
  float AccZ;  // Accelerometer z-axis value for MPU6050
  float GyroX; // Gyrometer x-axis value for MPU6050
  float GyroY; // Gyrometer y-axis value for MPU6050
  float GyroZ; // Gyrometer z-axis value for MPU6050
} HandSensorData, ForearmSensorData, BackSensorData, SensorData;

struct PowerDataStructure {
  float Current;  // Current value for system
  float Voltage;  // Voltage value for system
  float Energy;   // Energy value for system
  float Power;    // Power value for system
} PowerData;

// Handshake
boolean handshake = false;
boolean received = false;
int reply = 0;

void setup() {
  pinMode(HAND, OUTPUT);        // Sets hand digital pin as output pin
  pinMode(FOREARM, OUTPUT);     // Sets forearm digital pin as output pin
  pinMode(BACK, OUTPUT);        // Sets back digital pin as output pin

  Wire.begin();                 // Initiates I2C communication
  Wire.beginTransmission(MPU);  // Begins communication with the MPU
  Wire.write(0x6B);             // Access the power management register
  Wire.write(0x00);             // Wakes up the MPU
  Wire.endTransmission(true);   // Communication done

  Wire.beginTransmission(MPU);  // Begins communication with the MPU
  Wire.write(0x1C);             // Access Accelerometer Scale Register
  Wire.write(0x00);             // Set Accelerometer Scale
  Wire.endTransmission(true);   // Communication done

  Wire.beginTransmission(MPU);  // Begins communication with the MPU
  Wire.write(0x1B);             // Access Gyroscope Scale Register
  Wire.write(0x00);             // Set Gyroscope Range
  Wire.endTransmission(true);   // Communication done

  Serial.begin(115200);         // Initialize serial port baud rate to 115200
  Serial2.begin(115200);        // Initialize serial port 2 baud rate to 115200

  startHandshake();
  
  xTaskCreate(sendPackets, "sendPackets", STACK_SIZE, NULL, 2, NULL);
  vTaskStartScheduler();
}

void startHandshake() {
  while (!received) {
    Serial.println("Start Handshake");
    if (Serial2.available() > 0) {
      reply = Serial2.read();
      if (reply == 'H' ) {
        Serial2.write('A');
        Serial.println("Ack Handshake");
        received = true;
        reply = 0;
        break;
      }
    }
  }
  while (received  && !handshake) {
    if (Serial2.available() > 0) {
      reply = Serial2.read();
      if (reply == 'A') {
        Serial.println("Handshake Complete");
        handshake = true;
        break;
      }
    }
  }
}

void sendPackets(void *p) {
  while(1) {
    TickType_t xCurrWakeTime = xTaskGetTickCount();

    ExecuteAllSensors();
  
    // Convert to dataPacket
    dataPacket messagePacket;
    messagePacket.packetId = MESSAGE;
  
    messagePacket.deviceId = HAND_ID;
    messagePacket.deviceId2 = FOREARM_ID;
    messagePacket.deviceId3 = BACK_ID;
  
    transferDataFloatsToPacket(messagePacket.data, HAND_ID);
    transferDataFloatsToPacket(messagePacket.data, FOREARM_ID);
    transferDataFloatsToPacket(messagePacket.data, BACK_ID);
  
    CalculatePowerData();
    
    // Serialise into packets and send
    serialize(sendBuffer, &messagePacket);
    int bytesWritten = Serial2.write(sendBuffer);

    vTaskDelayUntil(&xCurrWakeTime,  10 / portTICK_PERIOD_MS);
  }
}

void transferDataFloatsToPacket(float *arr, char deviceId) {
  int startingIndex = 0;
  if (deviceId == HAND_ID) {
    arr[startingIndex++] = HandSensorData.AccX;
    arr[startingIndex++] = HandSensorData.AccY;
    arr[startingIndex++] = HandSensorData.AccZ;
    arr[startingIndex++] = HandSensorData.GyroX;
    arr[startingIndex++] = HandSensorData.GyroY;
    arr[startingIndex++] = HandSensorData.GyroZ;
  } else if (deviceId == FOREARM_ID) {
    startingIndex = 6;
    arr[startingIndex++] = ForearmSensorData.AccX;
    arr[startingIndex++] = ForearmSensorData.AccY;
    arr[startingIndex++] = ForearmSensorData.AccZ;
    arr[startingIndex++] = ForearmSensorData.GyroX;
    arr[startingIndex++] = ForearmSensorData.GyroY;
    arr[startingIndex++] = ForearmSensorData.GyroZ;
  } else if (deviceId == BACK_ID) {
    startingIndex = 12;
    arr[startingIndex++] = BackSensorData.AccX;
    arr[startingIndex++] = BackSensorData.AccY;
    arr[startingIndex++] = BackSensorData.AccZ;
    arr[startingIndex++] = BackSensorData.GyroX;
    arr[startingIndex++] = BackSensorData.GyroY;
    arr[startingIndex++] = BackSensorData.GyroZ;
  }
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

  // Serialising ACK packet to send
  if (packetIdFromPacket == ACK) {
    strcat(buf, ACK_STRING);
    strcat(buf, "1");
  }
  // Serialising message packet
  else if (packetIdFromPacket == MESSAGE) {
    strcat(buf, "#");
    strcat(buf, MESSAGE_STRING);
    strcat(buf, HAND_ID_STRING);
    strcat(buf, FOREARM_ID_STRING);
    strcat(buf, BACK_ID_STRING);
    strcat(buf, "(");

    // Semicolon differentiates individual axes values for each sensor
    for (int i=0; i<18; i++) {
      valueToConvert = (p->data)[i];
      checksum ^= (int)valueToConvert;
      dtostrf(valueToConvert, 1, 2, sensorDataBuf);
      strcat(buf, sensorDataBuf);
  
      // Differentiate by sensor
      if ((i +1) % 6 == 0)  strcat(buf, "(");
      else  strcat(buf, ";");
    }

    // Checksum for Voltage, Current, Power, Energy
    // Checksum ^= (int)PowerData.Voltage;
    dtostrf(PowerData.Voltage, 1, 2, sensorDataBuf);
    strcat(buf, sensorDataBuf);
    strcat(buf, "(");
    
    // Checksum ^= (int)PowerData.Current;
    dtostrf(PowerData.Current, 1, 2, sensorDataBuf);
    strcat(buf, sensorDataBuf);
    strcat(buf, "(");

    // Checksum ^= (int)PowerData.Power;
    dtostrf(PowerData.Power, 1, 2, sensorDataBuf);
    strcat(buf, sensorDataBuf);
    strcat(buf, "(");

    // Checksum ^= (int)PowerData.Energy;
    dtostrf(PowerData.Energy, 1, 2, sensorDataBuf);
    strcat(buf, sensorDataBuf);
    strcat(buf, "(");

    // Evaluate and append checksum
    dtostrf(checksum, 1, 0, sensorDataBuf);
    strcat(buf, sensorDataBuf);
  }  
  strcat(buf, "\n");
}

// Sensor Code Section  
void ReadMPUValues() {
  Wire.beginTransmission(MPU);      // Begins communication with the MPU
  Wire.write(0x3B);                 // Register 0x3B upper 8 bits of x-axis acceleration data
  Wire.endTransmission(false);      // End communication
  Wire.requestFrom(MPU, 14, true);  // Request 14 registers
  
  SensorData.AccX  = Wire.read() << 8 | Wire.read(); // Reads in raw x-axis acceleration data
  SensorData.AccY  = Wire.read() << 8 | Wire.read(); // Reads in raw y-axis acceleration data
  SensorData.AccZ  = Wire.read() << 8 | Wire.read(); // Reads in raw z-axis acceleration data
  Wire.read(); Wire.read();                          // Reads in raw temperature data
  SensorData.GyroX = Wire.read() << 8 | Wire.read(); // Reads in raw x-axis gyroscope data
  SensorData.GyroY = Wire.read() << 8 | Wire.read(); // Reads in raw y-axis gyroscope data
  SensorData.GyroZ = Wire.read() << 8 | Wire.read(); // Reads in raw z-axis gyroscope data
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

void ExecuteSensor(int value, SensorDataStructure sds) {
  DeactivateSensors();
  digitalWrite(value, LOW);  // Activates sensor
  ReadMPUValues();
  UpdateMPUSensorData();
  //PrintMPUValues(sds);
}

void ExecuteAllSensors() {
  ExecuteSensor(HAND, HandSensorData);
  ExecuteSensor(FOREARM, ForearmSensorData);
  ExecuteSensor(BACK, BackSensorData);
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

// Power Code Section
void ReadCurrent() {
  rawCurrentReading = analogRead(CurrentPort);                    // Read sensor value from INA169
  scaledCurrentReading = (rawCurrentReading * REFVOLTAGE) / 1023; // Scale the value to supply voltage that is 5V
  PowerData.Current = (scaledCurrentReading) / (RS * 10);         // Is = (Vout x 1k) / (RS x RL)
}

void ReadVoltage() {
  rawVoltageReading = analogRead(VoltagePort);
  PowerData.Voltage = (rawVoltageReading * 5.0 * 2) / 1023;
}

void ReadPower() {
  PowerData.Power = PowerData.Voltage * scaledCurrentReading;
}

void ReadEnergy() {
  currTime = millis();
  PowerData.Energy += (PowerData.Current * PowerData.Voltage *  (currTime - prevTime) / 1000) / 3600;
  prevTime = currTime;
}

void CalculatePowerData() {
  ReadCurrent();
  ReadVoltage();
  ReadPower();
  ReadEnergy();
  //PrintPowerValues(PowerData);
}

void PrintPowerValues(PowerDataStructure PDS) {
  Serial.print("Current: "); Serial.print(PDS.Current, 9); Serial.print(" A, ");
  Serial.print("Voltage: "); Serial.print(PDS.Voltage, 3); Serial.print(" V, ");
  Serial.print("Energy: "); Serial.print(PDS.Energy, 3); Serial.print(" Wh, ");
  Serial.print("Power: "); Serial.print(PDS.Power, 3); Serial.println(" W");
}

void loop() { 
}
