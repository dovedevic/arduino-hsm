#include "MPU9250.h"

#define TEMP_SENSE_1 A0
#define TEMP_SENSE_2 A1
#define TEMP_SENSE_3 A2
#define TEMP_SENSE_4 A3
#define TEMP_SENSE_5 A6
#define TEMP_SENSE_6 A7
const int TEMP_SENSORS[] = {TEMP_SENSE_1, TEMP_SENSE_2, TEMP_SENSE_3, TEMP_SENSE_4, TEMP_SENSE_5, TEMP_SENSE_6};
#define TEMP_SENSOR_COUNT 6

#define TEMP_NOISE_REDUCTION_SAMPLES 5
#define IMU_NOISE_REDUCTION_SAMPLES 1
#define TEMPERATURE_DIFFERENTIAL_THRESHOLD 5
#define TEMPERATURE_DIFFERENTIAL_THRESHOLD_MAX 30
#define TEMPERATURE_THRESHOLD_MAX 120
#define TEMPERATURE_THRESHOLD_MIN 40
#define MAGNETOMETER_THRESHOLD 150
#define MAGNETOMETER_THRESHOLD_MAX 400
#define GYROSCOPE_THRESHOLD 1.0
#define GYROSCOPE_THRESHOLD_MAX 4.0

float TEMP_SENSE_NOISE_BUFFER[TEMP_SENSOR_COUNT][TEMP_NOISE_REDUCTION_SAMPLES];
float IMU_NOISE_BUFFER[3][IMU_NOISE_REDUCTION_SAMPLES];

float denoisedTemperatureReadings[TEMP_SENSOR_COUNT];
float readingsSummary[4];  // min, average, max, maxdiff
int __noiseCheckIndex = 0;
int __imuNoiseCheckIndex = 0;

#define AD0_I2C_ADDRESS 0x68
MPU9250 IMU(Wire, AD0_I2C_ADDRESS);

#define CRITICAL_FLAG_PIN 13
#define FLAG_PIN 11
#define VOLTAGE_PIN 10

#define DEBUG_MODE (true)
#define DEBUG_BUZZER 12
#define DEBUG_BUZZER_CRITICAL 2000
#define DEBUG_BUZZER_LOCAL 1000

#define OK_STATE 0
#define LOCAL_ERROR 1
#define CRITICAL_ERROR 2

bool FLAG_LOW = false;
bool FLAG_HIGH = false;


void setup() {
  Serial.begin(9600);

  for (int sensor = 0; sensor < TEMP_SENSOR_COUNT; sensor++) { pinMode(TEMP_SENSORS[sensor], INPUT); }
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CRITICAL_FLAG_PIN, OUTPUT);
  pinMode(FLAG_PIN, OUTPUT);
  pinMode(VOLTAGE_PIN, INPUT);

  if(DEBUG_MODE){
    pinMode(DEBUG_BUZZER, OUTPUT);
    noTone(DEBUG_BUZZER);
  }
  int imuStatus = IMU.begin();
  if (imuStatus < 0) {
    FLAG_LOW = true;
    FLAG_HIGH = true;
    return;
  }
  
  prepopulateTemperatureSensors();
  int errorLevel = performSelfCheck();
  
  if (errorLevel == CRITICAL_ERROR){
    FLAG_LOW = true;
    FLAG_HIGH = true;
  } else if (errorLevel == LOCAL_ERROR){
    FLAG_LOW = true;
    FLAG_HIGH = false;
  } else {
    FLAG_LOW = false;
    FLAG_HIGH = false;
  }
}

void prepopulateTemperatureSensors(){
  for (int sample = 0; sample < TEMP_NOISE_REDUCTION_SAMPLES - 1; sample++) {
    readTemperatures();
  }
}

void readTemperatures() {
  for (int sensor = 0; sensor < TEMP_SENSOR_COUNT; sensor++) {
    TEMP_SENSE_NOISE_BUFFER[sensor][__noiseCheckIndex] = ((((analogRead(TEMP_SENSORS[sensor]) * 5.0) / 1024.0) - 0.5) * 180) + 32.0;
  }
  __noiseCheckIndex = (__noiseCheckIndex + 1) % TEMP_SENSOR_COUNT;
}

void getSummaryFloatArraySummary(float* readings, int c){
  if (c >= 2) {
    float minimum = min(readings[0], readings[1]);
    float maximim = max(readings[0], readings[1]);
    float sum = readings[0] + readings[1];
    for (int i = 2; i < c; i++) {
      minimum = min(minimum, readings[i]);
      maximim = max(maximim, readings[i]);
      sum += readings[i];
    }
    readingsSummary[0] = minimum;
    readingsSummary[1] = sum / c;
    readingsSummary[2] = maximim;
    readingsSummary[3] = maximim - minimum;
  } else if (c == 1) {
    readingsSummary[0] = readings[0];
    readingsSummary[1] = readings[0];
    readingsSummary[2] = readings[0];
    readingsSummary[3] = readings[0];
  } else {
    readingsSummary[0] = 0.0;
    readingsSummary[1] = 0.0;
    readingsSummary[2] = 0.0;
    readingsSummary[3] = 0.0;
  }
}

void readIMU() {
  IMU.readSensor();
  IMU_NOISE_BUFFER[0][__imuNoiseCheckIndex] = sqrt(IMU.getAccelX_mss() * IMU.getAccelX_mss() + IMU.getAccelY_mss() * IMU.getAccelY_mss() + IMU.getAccelZ_mss() * IMU.getAccelZ_mss());
  IMU_NOISE_BUFFER[1][__imuNoiseCheckIndex] = sqrt(IMU.getGyroX_rads() * IMU.getGyroX_rads() + IMU.getGyroY_rads() * IMU.getGyroY_rads() + IMU.getGyroZ_rads() * IMU.getGyroZ_rads());
  IMU_NOISE_BUFFER[2][__imuNoiseCheckIndex] = sqrt(IMU.getMagX_uT() * IMU.getMagX_uT() + IMU.getMagY_uT() * IMU.getMagY_uT() + IMU.getMagZ_uT() * IMU.getMagZ_uT());
  
  __imuNoiseCheckIndex = (__imuNoiseCheckIndex + 1) % IMU_NOISE_REDUCTION_SAMPLES;
}

int performSelfCheck() {
  readTemperatures();
  readIMU();

  for (int sensor = 0; sensor < TEMP_SENSOR_COUNT; sensor++) {
    getSummaryFloatArraySummary(TEMP_SENSE_NOISE_BUFFER[sensor], TEMP_NOISE_REDUCTION_SAMPLES);
    denoisedTemperatureReadings[sensor] = readingsSummary[1];
  }
  getSummaryFloatArraySummary(IMU_NOISE_BUFFER[0], IMU_NOISE_REDUCTION_SAMPLES);
  float netAccel = readingsSummary[1];
  getSummaryFloatArraySummary(IMU_NOISE_BUFFER[1], IMU_NOISE_REDUCTION_SAMPLES);
  float netGyro = readingsSummary[1];
  getSummaryFloatArraySummary(IMU_NOISE_BUFFER[2], IMU_NOISE_REDUCTION_SAMPLES);
  float netMagn = readingsSummary[1];
  
  getSummaryFloatArraySummary(denoisedTemperatureReadings, TEMP_SENSOR_COUNT);
  float minimum = readingsSummary[0];
  float average = readingsSummary[1];
  float maximum = readingsSummary[2];
  float maxdiff = readingsSummary[3];

  if(DEBUG_MODE){
    Serial.print("[DBG] ");
    Serial.print("av: "); Serial.print(average);
    Serial.print(" mn: "); Serial.print(minimum);
    Serial.print(" mx: "); Serial.print(maximum);
    Serial.print(" df: "); Serial.print(maxdiff);
    Serial.print(" an: "); Serial.print(netAccel);
    Serial.print(" ax: "); Serial.print(IMU.getAccelX_mss());
    Serial.print(" ay: "); Serial.print(IMU.getAccelY_mss());
    Serial.print(" az: "); Serial.print(IMU.getAccelZ_mss());
    Serial.print(" gn: "); Serial.print(netGyro);
    Serial.print(" gx: "); Serial.print(IMU.getGyroX_rads());
    Serial.print(" gy: "); Serial.print(IMU.getGyroY_rads());
    Serial.print(" gz: "); Serial.print(IMU.getGyroZ_rads());
    Serial.print(" mn: "); Serial.print(netMagn);
    Serial.print(" mx: "); Serial.print(IMU.getMagX_uT());
    Serial.print(" my: "); Serial.print(IMU.getMagY_uT());
    Serial.print(" mz: "); Serial.print(IMU.getMagZ_uT());
    Serial.println();
  }

  if (maxdiff > TEMPERATURE_DIFFERENTIAL_THRESHOLD_MAX) {
    Serial.println("[GBL] Encountered a critical error... bricking device.");
    Serial.print("[GBL] Allowed critical differential is "); Serial.print(TEMPERATURE_DIFFERENTIAL_THRESHOLD_MAX); Serial.print(", measured "); Serial.println(maxdiff);
    return CRITICAL_ERROR;
  } else if (maxdiff > TEMPERATURE_DIFFERENTIAL_THRESHOLD) {
    Serial.println("[GBL] Encountered a local error... waiting for resolution.");
    Serial.print("[GBL] Allowed local differential is "); Serial.print(TEMPERATURE_DIFFERENTIAL_THRESHOLD); Serial.print(", measured "); Serial.println(maxdiff);
    return LOCAL_ERROR;
  } else if (maximum > TEMPERATURE_THRESHOLD_MAX) {
    Serial.println("[GBL] Encountered a critical error... bricking device.");
    Serial.print("[GBL] Allowed critical max temperature is "); Serial.print(TEMPERATURE_THRESHOLD_MAX); Serial.print(", measured "); Serial.println(maximum);
    return CRITICAL_ERROR;
  } else if (minimum < TEMPERATURE_THRESHOLD_MIN) {
    Serial.println("[GBL] Encountered a critical error... bricking device.");
    Serial.print("[GBL] Allowed critical min temperature is "); Serial.print(TEMPERATURE_THRESHOLD_MIN); Serial.print(", measured "); Serial.println(minimum);
    return CRITICAL_ERROR;
  } else if (netMagn > MAGNETOMETER_THRESHOLD_MAX) {
    Serial.println("[GBL] Encountered a critical error... bricking device.");
    Serial.print("[GBL] Allowed critical field strength is "); Serial.print(MAGNETOMETER_THRESHOLD_MAX); Serial.print(", measured "); Serial.println(netMagn);
    return CRITICAL_ERROR;
  } else if (netMagn > MAGNETOMETER_THRESHOLD) {
    Serial.println("[GBL] Encountered a local error... waiting for resolution.");
    Serial.print("[GBL] Allowed local field strength is "); Serial.print(MAGNETOMETER_THRESHOLD); Serial.print(", measured "); Serial.println(netMagn);
    return LOCAL_ERROR;
  } else if (netGyro > GYROSCOPE_THRESHOLD_MAX) {
    Serial.println("[GBL] Encountered a critical error... bricking device.");
    Serial.print("[GBL] Allowed critical rotation is "); Serial.print(GYROSCOPE_THRESHOLD_MAX); Serial.print(", measured "); Serial.println(netGyro);
    return CRITICAL_ERROR;
  } else if (netGyro > GYROSCOPE_THRESHOLD) {
    Serial.println("[GBL] Encountered a local error... waiting for resolution.");
    Serial.print("[GBL] Allowed local rotation is "); Serial.print(GYROSCOPE_THRESHOLD); Serial.print(", measured "); Serial.println(netGyro);
    return LOCAL_ERROR;
  } else if (digitalRead(VOLTAGE_PIN) == HIGH) {
    Serial.println("[GBL] Encountered a local error... waiting for resolution.");
    Serial.println("[GBL] The HSM has transitioned to internal power");
    return LOCAL_ERROR;
  }
  return OK_STATE;
}

void loop() {
  if (FLAG_HIGH) {
    if (DEBUG_MODE) tone(DEBUG_BUZZER, DEBUG_BUZZER_CRITICAL);
    return;
  }

  int errorLevel = performSelfCheck();
  if (errorLevel == CRITICAL_ERROR){
    if (!FLAG_HIGH) {
      if (DEBUG_MODE) tone(DEBUG_BUZZER, DEBUG_BUZZER_CRITICAL);
      digitalWrite(CRITICAL_FLAG_PIN, HIGH);
      digitalWrite(FLAG_PIN, HIGH);
    }
    FLAG_LOW = true;
    FLAG_HIGH = true;
    return;
  } else if (errorLevel == LOCAL_ERROR){
    if (!FLAG_LOW) {
      if (DEBUG_MODE) tone(DEBUG_BUZZER, DEBUG_BUZZER_LOCAL);
      digitalWrite(FLAG_PIN, HIGH);
    }
    FLAG_LOW = true;
    FLAG_HIGH = false;
  } else {
    if (FLAG_LOW) {
      if (DEBUG_MODE) noTone(DEBUG_BUZZER);
      digitalWrite(CRITICAL_FLAG_PIN, LOW);
      digitalWrite(FLAG_PIN, LOW);
    }
    FLAG_LOW = false;
    FLAG_HIGH = false;
  }
  
}
