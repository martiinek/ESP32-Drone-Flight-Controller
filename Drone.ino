#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <BME280I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <ESP32Servo.h>
#include <BluetoothSerial.h>
#include <EEPROM.h>

//library objects
TaskHandle_t GetRadioTask;
Servo escLF;
Servo escRF;
Servo escRB;
Servo escLB;
BME280I2C bme;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
BluetoothSerial SerialBT;
//motor pins
#define esc1 33  //right back
#define esc2 32  //right front
#define esc3 26  //left back
#define esc4 25  //left front
//radio input
RF24 radio(5, 4);
const uint64_t address[] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
short data[6];
short altitude;
double seaLevelPres = 1013.25;
bool FLMOD = false;
//IMU
const int MPU_addr = 0x68;
double xAcc, yAcc, zAcc, xGyro, yGyro, zGyro;
double pitchInput, rollInput, yawInput, altitudeInput;
double gyroMicros, deltaGyroTime;
double pitchInputAcc, rollInputAcc;
double pitchInputGyro, rollInputGyro;
double rollGyroOffset, pitchGyroOffset, rollAccOffset, pitchAccOffset;
int counterAltitude = 0;
float declinationAngle = 0.0986;
double heading, headingOffset;
double altitudeInputOffset;
//kalman's altitude filter
const double QTemp = 0.91;   //Procesní šum
const double RTemp = 0.125;  //Měřící šum
double xTemp = 0;            //Stavová proměnná
double pTemp = 1;            //Kovariance stavu
double kTemp;                //Kalmanovo zesílení
const double QPres = 0.043;  //Procesní šum
const double RPres = 0.007;  //Měřící šum
double xPres = 0;            //Stavová proměnná
double pPres = 1;            //Kovariance stavu
double kPres;                //Kalmanovo zesílení
//pid
double pitchKp = 0, pitchKi = 0, pitchKd = 0;
double rollKp = pitchKp, rollKi = pitchKi, rollKd = pitchKd;
double yawKp = 3, yawKi = 0.02, yawKd = 0;
double pitchSetpoint = 0, rollSetpoint = 0, yawSetpoint = 0, altitudeSetpoint = 0;  //radio transmitter values
double pitchLastError, rollLastError, yawLastError;
double pitchIntegral, rollIntegral, yawIntegral;
//integral windup comp
double pitchOutputMax = 30, rollOutputMax = 30, yawOutputMax = 10;
double pitchOutputMin = -30, rollOutputMin = -30, yawOutputMin = -10;
//pid outputs
double pitchOutput, rollOutput, yawOutput;
//eeprom
byte bytes[8];
//time
unsigned long currentMicros;
unsigned long previousMicros;
unsigned long previousMicrosTunning;
unsigned long currentMillisRadioError;
unsigned long previousMillisRadioError;
unsigned long currentMillisBluetooth;
unsigned long previousMillisBluetooth;
const unsigned long interval = 4000;
const unsigned long tunningInterval = 100000;
const unsigned long bluetoothInterval = 100;
const unsigned long radioInterval = 30;
//loop bools
bool tunning = false;
bool flightError = false;
//motor speeds
double sp1 = 0, sp2 = 0, sp3 = 0, sp4 = 0;
//functions initialization
void InitInput();
void ReadSensors(double& pitchInput, double& rollInput, double& yawInput, double& altitudeInput);
double CalculatePID(double Input, double Kp, double Ki, double Kd, double Setpoint, double& LastError, double& Integral, double OutputMin, double OutputMax);
void GetRadio(double& yawSetpoint, double& rollSetpoint, double& pitchSetpoint, double& altitudeSetpoint, double& seaLevelPres, bool& FLMOD);
void Bluetooth(double& pitchKp, double& pitchKi, double& pitchKd);
void ControlMotors();


//variables for loop time messasuring

/*unsigned long startTime;
unsigned long endTime;
unsigned long loopDuration;
startTime = micros();
endTime = micros();
loopDuration = endTime - startTime;
Serial.print("Loop duration: ");
Serial.println(loopDuration);*/


void setup() {
  InitInput();
  delay(500);
}

void loop() {
  currentMicros = micros();
  if ((currentMicros - previousMicros) >= interval) {  // ↓ 3.5 ms
    previousMicros = currentMicros;
    ReadSensors(pitchInput, rollInput, yawInput, altitudeInput);
    yawOutput = CalculatePID(yawInput, yawKp, yawKi, yawKd, yawSetpoint, yawLastError, yawIntegral, yawOutputMin, yawOutputMax);
    if ((currentMicros - previousMicrosTunning) >= tunningInterval) {
      previousMicrosTunning = currentMicros;
      tunning = false;
    } else tunning = false;
    pitchOutput = CalculatePID(pitchInput, pitchKp, pitchKi, pitchKd, pitchSetpoint, pitchLastError, pitchIntegral, pitchOutputMin, pitchOutputMax);
    rollOutput = CalculatePID(rollInput, rollKp, rollKi, rollKd, rollSetpoint, rollLastError, rollIntegral, rollOutputMin, rollOutputMax);
    if (pitchInput < -45 || rollInput < -45 || pitchInput > 45 || rollInput > 45) flightError = true;
    ControlMotors();
  }


  //   --- SERIAL PRINTS ---

  /*Serial.print("altitudeSetpoint ");
  Serial.print(altitudeSetpoint);
  Serial.print(" | altitudeOutput ");
  Serial.println(altitudeOutput);*/

  /*Serial.println(pitchOutput);
    Serial.println(rollOutput);
    Serial.println(yawOutput);
    Serial.println(altitudeOutput);
    Serial.print("Altitude sea level pressure");
    Serial.println(seaLevelPres);
    Serial.print("altitudeSetpoint");
    Serial.println(altitudeSetpoint);
    Serial.print("pitchSetpoint");
    Serial.println(pitchSetpoint);
    Serial.print("rollSetpoint");
    Serial.println(rollSetpoint);
    Serial.print("yawSetpoint");
    Serial.println(yawSetpoint);
    Serial.print("Pitch");
    Serial.println(pitchInput);
    Serial.print("Roll");
    Serial.println(rollInput);
    Serial.print("Yaw");
    Serial.println(yawInput);
    Serial.print("Altitude");
    Serial.println(altitudeInput);
    Serial.print("Altitude Offset");
    Serial.println(altitudeInputOffset);*/
}

void GetRadioTaskFunction(void* pvParameters) {
  for (;;) {
    GetRadio(yawSetpoint, rollSetpoint, pitchSetpoint, altitudeSetpoint, seaLevelPres, FLMOD);
    Bluetooth(pitchKp, pitchKi, pitchKd);
    vTaskDelay(radioInterval / portTICK_PERIOD_MS);
  }
}
