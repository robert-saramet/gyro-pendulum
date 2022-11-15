#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "NXP_FXOS_FXAS.h"

Adafruit_NXPSensorFusion filter;
Adafruit_Sensor_Calibration_EEPROM cal;

#define FILTER_UPDATE_RATE_HZ 400
#define PRINT_EVERY_N_UPDATES 8

uint32_t timestamp;

float  pitch, offset, angle;

#define TOUCH_PIN 2
#define TOUCH_TRESHOLD 17

bool serverMode = false;
bool firstRun = true;
int id = 0;

#define MAX_ROWS 10
long t0[MAX_ROWS], t1[MAX_ROWS], tX[MAX_ROWS];
float u0[MAX_ROWS], u1[MAX_ROWS], uX[MAX_ROWS];
float dps[MAX_ROWS];

#define MIN_MOVEMENT 2  //deg
#define DIR_CHECK_INTERVAL 20  //ms

long ledTime = millis();
bool ledState = true;

void setupIMU() {
  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  }

  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Wire.setClock(400000); // 400KHz
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TOUCH_PIN, INPUT);
  setupIMU();
}

void blinkLed() {
  if(millis() - ledTime > 1000) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    ledTime = millis();
  }
}

void pollIMU() {
  float gx, gy, gz;
  static uint8_t counter = 0;

  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
    return;
  }
  timestamp = millis();
  
  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);

  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;
  
  filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  if (counter++ <= PRINT_EVERY_N_UPDATES) {
    return;
  }
  // reset the counter
  counter = 0;

  pitch = filter.getPitch();
}

void warmup() {
  long startTime = millis();
  while(millis() - startTime < 2000) {
    pollIMU();
  }
  angle = 0;
  offset = pitch - angle;
  firstRun = false;
  digitalWrite(LED_BUILTIN, HIGH);
}

bool touched() {
  return (touchRead(TOUCH_PIN) <= TOUCH_TRESHOLD);
}

float getAngle() {
  pollIMU();
  angle = pitch - offset;
  return angle;
}

bool loopIMU() {
  if(++id > MAX_ROWS) {
    return 1;
  }
  
  u0[id] = getAngle();
  t0[id] = millis();

  float temp = angle;
  delay(DIR_CHECK_INTERVAL);
  while(abs(getAngle()) < abs(temp) - MIN_MOVEMENT) {
    temp = getAngle();
    delay(DIR_CHECK_INTERVAL);
  }
  delay(3*DIR_CHECK_INTERVAL);
  temp = getAngle();
  while(abs(getAngle()) > abs(temp) - MIN_MOVEMENT) {
    temp = getAngle();
    delay(DIR_CHECK_INTERVAL);
  }
  
  u1[id] = angle;
  t1[id] = millis();
  uX[id] = u1[id] - u0[id];
  tX[id] = t1[id] - t0[id];
  dps[id] = 1000/tX[id] * uX[id];
  
  return 0;
}

void loopServer() {
  blinkLed();
}

void loop() {
  if(firstRun) {
    warmup();
    return;
  }

  if(serverMode) {
    while(!touched()) {
      loopServer();
    }
    serverMode = false;
    while(touched()) {
      delay(10);
    }
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
    id = 0;
    while(!touched()) {
      bool done = loopIMU();
      if(done) {
        break;
      }
    }
    serverMode = true;
  }
}
