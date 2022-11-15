#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <AsyncUDP.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "NXP_FXOS_FXAS.h"

Adafruit_NXPSensorFusion filter;
Adafruit_Sensor_Calibration_EEPROM cal;

#define FILTER_UPDATE_RATE_HZ 400
#define PRINT_EVERY_N_UPDATES 20

uint32_t timestamp;

float  pitch, offset, angle;

#define PACKET_DELAY 50

const char *ssid = "Pendul";
const char *password = "Nota10/10";

AsyncUDP udp;

bool firstRun = true;
long packetTime = millis();
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

void setupUDP() {
  if(udp.listen(1234)) {
       /**/
        });
    }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  WiFi.softAP(ssid, password);
  setupIMU();
  setupUDP();
}

void blinkLed() {
  if (millis() - ledTime > 1000) {
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

float getAngle() {
  pollIMU();
  angle = pitch - offset;
  return abs(angle);
}

void warmup() {
  long startTime = millis();
  while (millis() - startTime < 2000) {
    pollIMU();
  }
  angle = 0;
  offset = pitch - angle;
  firstRun = false;
  digitalWrite(LED_BUILTIN, !HIGH);
}

void loop() {
  if (firstRun) {
    warmup();
    return;
  }
  pollIMU();
  if (millis() - packetTime >= PACKET_DELAY) {
    char udpBuf[20];
    char timeBuf[8];
    char angleBuf[8];
    itoa(int(millis()), timeBuf, 10);
    itoa(abs(getAngle()), angleBuf, 10);
    strcpy(udpBuf, timeBuf);
    strcat(udpBuf, "/");
    strcat(udpBuf, angleBuf);
    Serial.println(udpBuf);
    udp.broadcastTo(udpBuf, 1234);
    packetTime = millis();
  }
}
