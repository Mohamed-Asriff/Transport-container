#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <math.h>

// ==========================================
// WIFI
// ==========================================
const char* WIFI_SSID = "flip6";
const char* WIFI_PASSWORD = "0987654321";

// ==========================================
// MQTT
// ==========================================
const char* MQTT_SERVER = "broker.emqx.io";
const int MQTT_PORT = 1883;

const char* MQTT_TOPIC =
  "securetrack/container/CT-001/telemetry";

const char* DEVICE_UID = "ESP32-CT-001";
const char* CONTAINER_CODE = "CT-001";

// ==========================================
// RFID PINS
// ==========================================
#define RFID_RST   4
#define RFID_SDA   15
#define RFID_SCK   7
#define RFID_MOSI  6
#define RFID_MISO  5

// ==========================================
// SERVO
// ==========================================
#define SERVO_PIN 16
#define LOCK_ANGLE 0
#define OPEN_ANGLE 90

// ==========================================
// GPS
// ==========================================
#define GPS_RX_PIN 18
#define GPS_TX_PIN 17

// ==========================================
// A7670C SIM
// ==========================================
#define SIM_RX_PIN 11
#define SIM_TX_PIN 12
#define SIM_BAUD 115200

const char PHONE_NUMBER[] = "+94754558689";

// ==========================================
// MPU6050
// ==========================================
#define MPU_SDA_PIN 37
#define MPU_SCL_PIN 38
#define MPU_ADDRESS 0x68

const unsigned long MPU_SAMPLE_INTERVAL = 20;

const float SHOCK_THRESHOLD_G = 0.75;
const float IMPACT_THRESHOLD_G = 2.2;

// ==========================================
// INTERVALS
// ==========================================
const unsigned long MQTT_PUBLISH_INTERVAL = 10000;
const unsigned long SMS_INTERVAL = 10000;
const unsigned long RFID_DEBOUNCE = 700;

// ==========================================
// OBJECTS
// ==========================================
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

MFRC522 rfid(RFID_SDA, RFID_RST);
TinyGPSPlus gps;

// ==========================================
// STATE
// ==========================================
bool servoReady = false;
bool isLocked = false;

bool tamperDetected = false;
bool tamperAlertSent = false;

String lastRFID = "";

unsigned long lastMQTTPublish = 0;
unsigned long lastSMSTime = 0;
unsigned long lastRFIDTap = 0;
unsigned long lastWiFiAttempt = 0;
unsigned long lastMQTTAttempt = 0;

// ==========================================
// MPU STATE
// ==========================================
bool mpuReady = false;

float previousAx = 0;
float previousAy = 0;
float previousAz = 0;

bool previousMPUReading = false;

unsigned long lastMPURead = 0;
unsigned long previousMPUTime = 0;

// ==========================================
// SERVO LEDC
// ==========================================

void writeServoAngle(int angle) {

  if (!servoReady) {
    return;
  }

  angle = constrain(angle, 0, 180);

  int pulseUS =
    map(angle, 0, 180, 1000, 2000);

  uint32_t duty =
    ((uint32_t)pulseUS * 16383UL)
    / 20000UL;

  ledcWrite(SERVO_PIN, duty);
}

void setupServo() {

  Serial.println("Starting Servo LEDC PWM...");

  bool ok =
    ledcAttach(
      SERVO_PIN,
      50,
      14
    );

  if (!ok) {
    Serial.println("Servo PWM FAILED");
    servoReady = false;
    return;
  }

  servoReady = true;

  writeServoAngle(OPEN_ANGLE);

  isLocked = false;

  Serial.println("Servo PWM READY");
  Serial.println("Initial Status: UNLOCKED");
}

// ==========================================
// MPU FUNCTIONS
// ==========================================

void writeMPURegister(byte reg, byte value) {

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool readMPURegisters(
  byte reg,
  byte count,
  byte* data
) {

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  byte received =
    Wire.requestFrom(
      (uint8_t)MPU_ADDRESS,
      (uint8_t)count
    );

  if (received != count) {
    return false;
  }

  for (byte i = 0; i < count; i++) {
    data[i] = Wire.read();
  }

  return true;
}

void initializeMPU() {

  Serial.println("\nInitializing MPU6050...");

  Wire.begin(
    MPU_SDA_PIN,
    MPU_SCL_PIN
  );

  Wire.setClock(100000);

  delay(100);

  byte whoAmI = 0;

  if (!readMPURegisters(
        0x75,
        1,
        &whoAmI
      )) {

    Serial.println(
      "MPU6050 NOT DETECTED!"
    );

    mpuReady = false;
    return;
  }

  Serial.print("MPU WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);

  // Your board previously behaved unusually,
  // so direct register communication is what matters.
  writeMPURegister(0x6B, 0x00);

  delay(100);

  writeMPURegister(0x1C, 0x00);
  writeMPURegister(0x1A, 0x03);

  delay(100);

  mpuReady = true;

  Serial.println("MPU READY");
}

bool readMPUAcceleration(
  float &ax,
  float &ay,
  float &az
) {

  byte data[6];

  if (!readMPURegisters(
        0x3B,
        6,
        data
      )) {

    return false;
  }

  int16_t rawAx =
    (int16_t)((data[0] << 8) | data[1]);

  int16_t rawAy =
    (int16_t)((data[2] << 8) | data[3]);

  int16_t rawAz =
    (int16_t)((data[4] << 8) | data[5]);

  ax = rawAx / 16384.0;
  ay = rawAy / 16384.0;
  az = rawAz / 16384.0;

  return true;
}

// ==========================================
// WIFI
// ==========================================

void connectWiFi() {

  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  if (
    millis() - lastWiFiAttempt
    < 5000
  ) {
    return;
  }

  lastWiFiAttempt = millis();

  Serial.println("Connecting WiFi...");

  WiFi.mode(WIFI_STA);

  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );
}

// ==========================================
// MQTT
// ==========================================

void connectMQTT() {

  if (
    WiFi.status()
    != WL_CONNECTED
  ) {
    return;
  }

  if (mqtt.connected()) {
    return;
  }

  if (
    millis() - lastMQTTAttempt
    < 5000
  ) {
    return;
  }

  lastMQTTAttempt = millis();

  String clientId =
    "SecureTrack-ESP32-";

  clientId += String(
    (uint32_t)(
      ESP.getEfuseMac()
      & 0xFFFFFFFF
    ),
    HEX
  );

  Serial.println("Connecting MQTT...");

  if (
    mqtt.connect(
      clientId.c_str()
    )
  ) {

    Serial.println("MQTT CONNECTED!");

  } else {

    Serial.print(
      "MQTT FAILED, state = "
    );

    Serial.println(
      mqtt.state()
    );
  }
}

// ==========================================
// GPS
// ==========================================

void readGPS() {

  while (
    Serial2.available() > 0
  ) {
    gps.encode(
      Serial2.read()
    );
  }
}

// ==========================================
// RFID UID
// ==========================================

String getRFIDUID() {

  String uid = "";

  for (
    byte i = 0;
    i < rfid.uid.size;
    i++
  ) {

    if (
      rfid.uid.uidByte[i] < 0x10
    ) {
      uid += "0";
    }

    String part =
      String(
        rfid.uid.uidByte[i],
        HEX
      );

    part.toUpperCase();

    uid += part;

    if (
      i < rfid.uid.size - 1
    ) {
      uid += ":";
    }
  }

  return uid;
}

// ==========================================
// MQTT JSON
// ==========================================

String createMQTTJSON() {

  bool gpsValid =
    gps.location.isValid();

  String json = "{";

  json += "\"deviceUid\":\"";
  json += DEVICE_UID;
  json += "\",";

  json += "\"containerId\":\"";
  json += CONTAINER_CODE;
  json += "\",";

  json += "\"latitude\":";

  if (gpsValid) {
    json += String(
      gps.location.lat(),
      6
    );
  } else {
    json += "null";
  }

  json += ",";

  json += "\"longitude\":";

  if (gpsValid) {
    json += String(
      gps.location.lng(),
      6
    );
  } else {
    json += "null";
  }

  json += ",";

  json += "\"gpsValid\":";
  json += gpsValid
    ? "true"
    : "false";

  json += ",";

  json += "\"isLocked\":";
  json += isLocked
    ? "true"
    : "false";

  json += ",";

  json += "\"lastRFID\":\"";
  json += lastRFID;
  json += "\",";

  json += "\"tamperDetected\":";
  json += tamperDetected
    ? "true"
    : "false";

  json += "}";

  return json;
}

// ==========================================
// MQTT PUBLISH
// ==========================================

void publishTelemetry() {

  if (!mqtt.connected()) {
    return;
  }

  String json =
    createMQTTJSON();

  Serial.println();
  Serial.println(
    "Publishing MQTT telemetry:"
  );

  Serial.println(json);

  bool sent =
    mqtt.publish(
      MQTT_TOPIC,
      json.c_str()
    );

  if (sent) {

    Serial.println(
      "MQTT DATA PUBLISHED!"
    );

  } else {

    Serial.println(
      "MQTT PUBLISH FAILED!"
    );
  }
}

// ==========================================
// TAMPER
// ==========================================

void checkTamper() {

  if (!mpuReady) {
    return;
  }

  if (
    millis() - lastMPURead
    < MPU_SAMPLE_INTERVAL
  ) {
    return;
  }

  lastMPURead = millis();

  float ax, ay, az;

  if (!readMPUAcceleration(
        ax,
        ay,
        az
      )) {

    return;
  }

  unsigned long currentTime =
    millis();

  if (!previousMPUReading) {

    previousAx = ax;
    previousAy = ay;
    previousAz = az;

    previousMPUTime =
      currentTime;

    previousMPUReading =
      true;

    return;
  }

  unsigned long elapsed =
    currentTime
    - previousMPUTime;

  float deltaX =
    ax - previousAx;

  float deltaY =
    ay - previousAy;

  float deltaZ =
    az - previousAz;

  previousAx = ax;
  previousAy = ay;
  previousAz = az;

  previousMPUTime =
    currentTime;

  if (elapsed > 100) {
    return;
  }

  float shock =
    sqrt(
      deltaX * deltaX +
      deltaY * deltaY +
      deltaZ * deltaZ
    );

  float totalAcceleration =
    sqrt(
      ax * ax +
      ay * ay +
      az * az
    );

  if (!isLocked) {
    return;
  }

  if (tamperAlertSent) {
    return;
  }

  if (
    shock >= SHOCK_THRESHOLD_G ||
    totalAcceleration >= IMPACT_THRESHOLD_G
  ) {

    Serial.println(
      "!!! TAMPERING DETECTED !!!"
    );

    tamperDetected = true;
    tamperAlertSent = true;

    // Immediate MQTT update
    publishTelemetry();
  }
}

// ==========================================
// RFID
// ==========================================

void checkRFID() {

  if (
    !rfid.PICC_IsNewCardPresent()
  ) {
    return;
  }

  if (
    !rfid.PICC_ReadCardSerial()
  ) {
    return;
  }

  if (
    millis() - lastRFIDTap
    < RFID_DEBOUNCE
  ) {

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    return;
  }

  lastRFIDTap = millis();

  lastRFID =
    getRFIDUID();

  Serial.print("RFID UID: ");
  Serial.println(lastRFID);

  if (!isLocked) {

    writeServoAngle(
      LOCK_ANGLE
    );

    isLocked = true;

    tamperDetected = false;
    tamperAlertSent = false;

    Serial.println(
      "CONTAINER LOCKED"
    );

  } else {

    writeServoAngle(
      OPEN_ANGLE
    );

    isLocked = false;

    tamperDetected = false;
    tamperAlertSent = false;

    Serial.println(
      "CONTAINER UNLOCKED"
    );
  }

  // Immediate MQTT update
  publishTelemetry();

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// ==========================================
// SETUP
// ==========================================

void setup() {

  Serial.begin(115200);

  delay(1500);

  Serial.println();
  Serial.println(
    "SECURETRACK STARTING"
  );

  // RFID
  SPI.begin(
    RFID_SCK,
    RFID_MISO,
    RFID_MOSI,
    RFID_SDA
  );

  rfid.PCD_Init();

  Serial.println("RFID READY");

  // GPS
  Serial2.begin(
    9600,
    SERIAL_8N1,
    GPS_RX_PIN,
    GPS_TX_PIN
  );

  Serial.println("GPS READY");

  // SIM
  Serial1.begin(
    SIM_BAUD,
    SERIAL_8N1,
    SIM_RX_PIN,
    SIM_TX_PIN
  );

  Serial.println("SIM READY");

  // MPU
  initializeMPU();

  // Servo
  setupServo();

  // MQTT
  mqtt.setServer(
    MQTT_SERVER,
    MQTT_PORT
  );

  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(30);

  // WiFi
  WiFi.mode(WIFI_STA);
  connectWiFi();

  Serial.println(
    "SETUP COMPLETED"
  );
}

// ==========================================
// LOOP
// ==========================================

void loop() {

  readGPS();

  checkRFID();

  checkTamper();

  // WiFi reconnect
  if (
    WiFi.status()
    != WL_CONNECTED
  ) {

    connectWiFi();
  }

  // MQTT reconnect
  if (
    WiFi.status()
    == WL_CONNECTED
  ) {

    if (!mqtt.connected()) {
      connectMQTT();
    }

    if (mqtt.connected()) {
      mqtt.loop();
    }
  }

  // Periodic MQTT telemetry
  if (
    millis() - lastMQTTPublish
    >= MQTT_PUBLISH_INTERVAL
  ) {

    lastMQTTPublish =
      millis();

    publishTelemetry();
  }

  delay(10);
}