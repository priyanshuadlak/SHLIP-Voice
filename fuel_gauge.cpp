/*
  Prosthetic Hand + BLE + Buttons

  Modes: RELAX, PINCH, GRASP, POINTING
  Buttons: RELAX, PINCH, GRASP, POINT, INC, DEC
  BLE Commands from app:
    "RELAX", "PINCH", "GRASP", "POINTING"
    "INC", "DEC"
    "EMERGENCY_STOP"
*/
#include <Wire.h>
#include "Adafruit_MAX1704X.h"
#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Fuel gauge object
Adafruit_MAX17048 maxlipo;

// ================== Servo pins ==================
#define SERVO1_PIN 7   // Thumb
#define SERVO2_PIN 8   // Index
#define SERVO3_PIN 9   // Fingers

Servo servoThumb;
Servo servoIndex;
Servo servoFingers;

// ================== Button pins ==================
#define BTN_RELAX 12
#define BTN_PINCH 13
#define BTN_GRASP 14
#define BTN_POINT 15
#define BTN_INC   16
#define BTN_DEC   17

// ================== Mode default angles ==================
const int DEFAULT_RELAX_T = 180;
const int DEFAULT_RELAX_I = 180;
const int DEFAULT_RELAX_F = 180;

const int DEFAULT_PINCH_T = 30;
const int DEFAULT_PINCH_I = 40;
const int DEFAULT_PINCH_F = 120;

const int DEFAULT_GRASP_T = 0;     // thumb fixed for strong grip
const int DEFAULT_GRASP_I = 60;
const int DEFAULT_GRASP_F = 45;

const int DEFAULT_POINT_T = 120;
const int DEFAULT_POINT_I = 180;
const int DEFAULT_POINT_F = 0;

// Movement params
const int MOVE_STEPS = 25;
const int MOVE_DURATION_MS = 500;

// =============== State variables ===============
enum Mode { RELAX, PINCH, GRASP, POINTING };
Mode currentMode = RELAX;

int angleThumb, angleIndex, angleFingers;
int targetThumb, targetIndex, targetFingers;

// Button last states
int prevRelaxState;
int prevPinchState;
int prevGraspState;
int prevPointState;
int prevIncState;
int prevDecState;

// =============== BATTERY + BLE UUIDs ===============
const int BATTERY_PIN = 4;  // ⚠ apne ESP32-S3 ADC pin se connect karo (e.g. GPIO4)

#define SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define COMMAND_CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BATTERY_CHAR_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"

BLEServer* pServer = nullptr;
BLECharacteristic* pCommandChar = nullptr;
BLECharacteristic* pBatteryChar = nullptr;
bool deviceConnected = false;
unsigned long lastBatteryUpdate = 0;

// ---------- Forward declarations ----------
void moveAllSmooth(int durationMs);
void setMode(Mode m);
void adjustAnglesBy(int delta);
int clampAngle(int a);
void printStatus();
void handleBleCommand(const String& cmd);
int readBatteryPercent();
void sendBattery();

// ================== BLE CALLBACKS ==================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    deviceConnected = true;
    Serial.println("✅ BLE device connected");
  }

  void onDisconnect(BLEServer* server) override {
    deviceConnected = false;
    Serial.println("❌ BLE device disconnected");
    // re-advertise
    server->getAdvertising()->start();
  }
};

class MyCommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    String cmd = characteristic->getValue().c_str();
    cmd.trim();
    if (cmd.length() == 0) return;

    Serial.print("📩 BLE command received: ");
    Serial.println(cmd);

    handleBleCommand(cmd);
  }
};

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(100);

  // Servos
  servoThumb.attach(SERVO1_PIN);
  servoIndex.attach(SERVO2_PIN);
  servoFingers.attach(SERVO3_PIN);

  // Buttons
  pinMode(BTN_RELAX, INPUT_PULLUP);
  pinMode(BTN_PINCH, INPUT_PULLUP);
  pinMode(BTN_GRASP, INPUT_PULLUP);
  pinMode(BTN_POINT, INPUT_PULLUP);
  pinMode(BTN_INC,   INPUT_PULLUP);
  pinMode(BTN_DEC,   INPUT_PULLUP);

  prevRelaxState = digitalRead(BTN_RELAX);
  prevPinchState = digitalRead(BTN_PINCH);
  prevGraspState = digitalRead(BTN_GRASP);
  prevPointState = digitalRead(BTN_POINT);
  prevIncState   = digitalRead(BTN_INC);
  prevDecState   = digitalRead(BTN_DEC);

  // Initial mode: RELAX
  angleThumb = targetThumb = DEFAULT_RELAX_T;
  angleIndex = targetIndex = DEFAULT_RELAX_I;
  angleFingers = targetFingers = DEFAULT_RELAX_F;
  servoThumb.write(angleThumb);
  servoIndex.write(angleIndex);
  servoFingers.write(angleFingers);

  pinMode(BATTERY_PIN, INPUT);

  // ======== BLE init (IMPORTANT: NAME MATCHES FILTER: Prosthetic / ESP32 / Hand) ========
  BLEDevice::init("Prosthetic ESP32 Hand");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* service = pServer->createService(SERVICE_UUID);

  // Command characteristic (WRITE)
  pCommandChar = service->createCharacteristic(
    COMMAND_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pCommandChar->setCallbacks(new MyCommandCallbacks());

  // Battery characteristic (NOTIFY)
  pBatteryChar = service->createCharacteristic(
    BATTERY_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  pBatteryChar->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising* advertising = pServer->getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  Serial.println("🔊 BLE Advertising started: Prosthetic ESP32 Hand");
  printStatus();
}

// ================== LOOP ==================
void loop() {
  // ----- Buttons -----
  int curRelax = digitalRead(BTN_RELAX);
  int curPinch = digitalRead(BTN_PINCH);
  int curGrasp = digitalRead(BTN_GRASP);
  int curPoint = digitalRead(BTN_POINT);
  int curInc   = digitalRead(BTN_INC);
  int curDec   = digitalRead(BTN_DEC);

  if (prevRelaxState == HIGH && curRelax == LOW) { setMode(RELAX); delay(150); }
  if (prevPinchState == HIGH && curPinch == LOW) { setMode(PINCH); delay(150); }
  if (prevGraspState == HIGH && curGrasp == LOW) { setMode(GRASP); delay(150); }
  if (prevPointState == HIGH && curPoint == LOW) { setMode(POINTING); delay(150); }

  if (prevIncState == HIGH && curInc == LOW) {
    if (currentMode != RELAX) adjustAnglesBy(+25);
    delay(150);
  }
  if (prevDecState == HIGH && curDec == LOW) {
    if (currentMode != RELAX) adjustAnglesBy(-25);
    delay(150);
  }

  prevRelaxState = curRelax;
  prevPinchState = curPinch;
  prevGraspState = curGrasp;
  prevPointState = curPoint;
  prevIncState   = curInc;
  prevDecState   = curDec;

  // ----- Battery BLE notify every 5 sec -----
  if (deviceConnected) {
    unsigned long now = millis();
    if (now - lastBatteryUpdate > 5000) {
      lastBatteryUpdate = now;
      sendBattery();
    }
  }

  delay(10);
}

// ================== MODE FUNCTIONS ==================
void setMode(Mode m) {
  currentMode = m;

  switch (m) {
    case RELAX:
      Serial.println(F("Mode: RELAX"));
      targetThumb = DEFAULT_RELAX_T;
      targetIndex = DEFAULT_RELAX_I;
      targetFingers = DEFAULT_RELAX_F;
      moveAllSmooth(400);
      break;

    case PINCH:
      Serial.println(F("Mode: PINCH"));
      targetThumb = DEFAULT_PINCH_T;
      targetIndex = DEFAULT_PINCH_I;
      targetFingers = DEFAULT_PINCH_F;
      moveAllSmooth(MOVE_DURATION_MS);
      break;

    case GRASP:
      Serial.println(F("Mode: GRASP"));
      targetThumb = DEFAULT_GRASP_T;
      targetIndex = DEFAULT_GRASP_I;
      targetFingers = DEFAULT_GRASP_F;
      moveAllSmooth(MOVE_DURATION_MS);
      break;

    case POINTING:
      Serial.println(F("Mode: POINTING"));
      targetThumb = DEFAULT_POINT_T;
      targetIndex = DEFAULT_POINT_I;
      targetFingers = DEFAULT_POINT_F;
      moveAllSmooth(MOVE_DURATION_MS);
      break;
  }

  printStatus();
}

void adjustAnglesBy(int delta) {
  if (currentMode == GRASP) {
    targetThumb = DEFAULT_GRASP_T; // thumb fixed
    targetIndex   = clampAngle(targetIndex   + delta);
    targetFingers = clampAngle(targetFingers + delta);
  } else {
    targetThumb   = clampAngle(targetThumb   + delta);
    targetIndex   = clampAngle(targetIndex   + delta);
    targetFingers = clampAngle(targetFingers + delta);
  }

  moveAllSmooth(MOVE_DURATION_MS);
  printStatus();
}

void moveAllSmooth(int durationMs) {
  int steps = MOVE_STEPS;
  int sleepTime = durationMs / steps;

  for (int s = 1; s <= steps; s++) {
    float p = (float)s / steps;

    int curT = angleThumb   + (targetThumb   - angleThumb)   * p;
    int curI = angleIndex   + (targetIndex   - angleIndex)   * p;
    int curF = angleFingers + (targetFingers - angleFingers) * p;

    servoThumb.write(curT);
    servoIndex.write(curI);
    servoFingers.write(curF);

    delay(sleepTime);
  }

  servoThumb.write(targetThumb);
  servoIndex.write(targetIndex);
  servoFingers.write(targetFingers);

  angleThumb = targetThumb;
  angleIndex = targetIndex;
  angleFingers = targetFingers;
}

int clampAngle(int a) {
  if (a < 0)   return 0;
  if (a > 180) return 180;
  return a;
}

void printStatus() {
  Serial.print(F("Mode = "));
  switch (currentMode) {
    case RELAX:    Serial.print("RELAX"); break;
    case PINCH:    Serial.print("PINCH"); break;
    case GRASP:    Serial.print("GRASP"); break;
    case POINTING: Serial.print("POINTING"); break;
  }
  Serial.print(F(" | Angles = "));
  Serial.print(angleThumb); Serial.print(",");
  Serial.print(angleIndex); Serial.print(",");
  Serial.println(angleFingers);
}

// ================== BLE COMMAND HANDLER ==================
void handleBleCommand(const String& cmd) {
  if (cmd == "RELAX") {
    setMode(RELAX);
  } else if (cmd == "PINCH") {
    setMode(PINCH);
  } else if (cmd == "GRASP") {
    setMode(GRASP);
  } else if (cmd == "POINTING") {
    setMode(POINTING);
  } else if (cmd == "INC") {
    if (currentMode != RELAX) adjustAnglesBy(+25);
  } else if (cmd == "DEC") {
    if (currentMode != RELAX) adjustAnglesBy(-25);
  } else if (cmd == "EMERGENCY_STOP") {
    Serial.println("⛔ EMERGENCY STOP (no movement)");
    // Yaha chaaho to servo detach() kar sakte ho
  } else {
    Serial.print("❓ Unknown BLE cmd: ");
    Serial.println(cmd);
  }
}

// ================== BATTERY FUNCTIONS ==================
int readBatteryPercent() {
  int raw = analogRead(BATTERY_PIN);
  float voltage = (raw * 3.3) / 4095.0;  // basic mapping

  // Example mapping 3.3V–4.2V => 0–100%
  int percent = map((int)(voltage * 100), 330, 420, 0, 100);
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  return percent;
}

void sendBattery() {
  if (!deviceConnected || pBatteryChar == nullptr) return;

  int percent = readBatteryPercent();
  String data = "BAT:" + String(percent);

  Serial.print("🔋 Sending battery: ");
  Serial.println(data);

  pBatteryChar->setValue(data.c_str());
  pBatteryChar->notify();
}