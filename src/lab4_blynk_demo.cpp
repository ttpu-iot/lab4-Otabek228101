#include <Arduino.h>
#include <WiFi.h>
#define BLYNK_FIRMWARE_VERSION "0.1.0"
#define BLYNK_TEMPLATE_ID "TMPL2rkMzqzyl"
#define BLYNK_TEMPLATE_NAME "IoT25 ESP32 Template"
#define BLYNK_AUTH_TOKEN "QaW1JxlrB6rwLC0YXZutzip6dwYsv7HZ"
#define BLYNK_PRINT Serial
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <Servo.h>
const int RED_PIN    = 26;
const int GREEN_PIN  = 27;
const int BLUE_PIN   = 14;
const int YELLOW_PIN = 12;

const int BUTTON_PIN = 25;  
const int BUZZER_PIN = 32;
const int SERVO_PIN  = 5;
const int LIGHT_PIN  = 33;

hd44780_I2Cexp lcd;  
const int LCD_COLS = 16;
const int LCD_ROWS = 2;

char ssid[] = "Wokwi-GUEST";
char pass[] = "";

BlynkTimer timer;

Servo servo;
const int BUZZER_CH   = 0;
const int BUZZER_RES  = 10;   
const int BUZZER_DUTY = 512;  
const unsigned long BUZZER_MAX_MS = 3000;

volatile bool buzzerOn = false;
int buzzerFreq = 0;
unsigned long buzzerStartMs = 0;
uint16_t lastLight = 0;

bool lastButtonRaw = false;
bool buttonStable = false;
unsigned long lastDebounceMs = 0;
const unsigned long DEBOUNCE_MS = 40;

void lcdClearLine(uint8_t row) {
  lcd.setCursor(0, row);
}

void showEvent(const String& msg) {
  lcdClearLine(0);
  lcd.setCursor(0, 0);
  if (msg.length() <= 16) lcd.print(msg);
  else lcd.print(msg.substring(0, 16));
}

void updateLightLine() {
  char buf[17];
  snprintf(buf, sizeof(buf), "L=%u", (unsigned)lastLight);
  lcdClearLine(1);
  lcd.setCursor(0, 1);
  lcd.print(buf);
}

void stopBuzzer() {
  buzzerOn = false;
  ledcWriteTone(BUZZER_CH, 0);
  ledcWrite(BUZZER_CH, 0);
}

void startBuzzer() {
  buzzerOn = true;
  buzzerStartMs = millis();
  if (buzzerFreq > 0) {
    ledcWriteTone(BUZZER_CH, buzzerFreq);
    ledcWrite(BUZZER_CH, BUZZER_DUTY);
  } else {
    ledcWriteTone(BUZZER_CH, 0);
    ledcWrite(BUZZER_CH, 0);
  }
}

void applyBuzzerFreq(int f) {
  buzzerFreq = f;
  if (buzzerOn) {
    if (buzzerFreq > 0) {
      ledcWriteTone(BUZZER_CH, buzzerFreq);
      ledcWrite(BUZZER_CH, BUZZER_DUTY);
    } else {
      ledcWriteTone(BUZZER_CH, 0);
      ledcWrite(BUZZER_CH, 0);
    }
  }
}

BLYNK_WRITE(V1) { int v = param.asInt(); digitalWrite(RED_PIN, v); showEvent(String("Red: ") + (v?"ON":"OFF")); }
BLYNK_WRITE(V2) { int v = param.asInt(); digitalWrite(GREEN_PIN, v); showEvent(String("Green: ") + (v?"ON":"OFF")); }
BLYNK_WRITE(V5) { int v = param.asInt(); digitalWrite(BLUE_PIN, v); showEvent(String("Blue: ") + (v?"ON":"OFF")); }
BLYNK_WRITE(V6) { int v = param.asInt(); digitalWrite(YELLOW_PIN, v); showEvent(String("Yellow: ") + (v?"ON":"OFF")); }

BLYNK_WRITE(V7) { int f = param.asInt(); applyBuzzerFreq(f); showEvent(String("Buz Hz: ") + f); }

BLYNK_WRITE(V3) {
  int v = param.asInt();
  if (v == 1) { startBuzzer(); showEvent(String("Buz: ON ") + buzzerFreq + "Hz"); }
  else { stopBuzzer(); showEvent("Buz: OFF"); }
}

BLYNK_WRITE(V8) {
  int ang = constrain(param.asInt(), 0, 180);
  servo.write(ang);
  showEvent(String("Servo: ") + ang + "deg");
}

BLYNK_WRITE(InternalPinOTA) {
  String url = param.asString();

  showEvent("OTA..."); 

  Serial.printf("OTA update requested: %s\n", url.c_str());
  Serial.println("Starting OTA update...");

  Blynk.disconnect();

  HTTPClient http;
  http.begin(url);

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("HTTP GET failed, code: %d\n", httpCode);
    http.end();
    Blynk.connect();
    showEvent("OTA FAIL");
    return;
  }

  int contentLength = http.getSize();
  if (contentLength <= 0) {
    Serial.println("Invalid content length.");
    http.end();
    Blynk.connect();
    showEvent("OTA FAIL");
    return;
  }

  bool canBegin = Update.begin(contentLength);
  if (!canBegin) {
    Serial.println("Not enough space for OTA.");
    http.end();
    Blynk.connect();
    showEvent("OTA FAIL");
    return;
  }

  WiFiClient *client = http.getStreamPtr();
  int written = Update.writeStream(*client);

  if (written != contentLength) {
    Serial.printf("OTA written %d / %d bytes\n", written, contentLength);
    Update.abort();
    http.end();
    Blynk.connect();
    showEvent("OTA FAIL");
    return;
  }

  if (!Update.end()) {
    Serial.printf("Update error #%d\n", Update.getError());
    http.end();
    Blynk.connect();
    showEvent("OTA FAIL");
    return;
  }

  if (!Update.isFinished()) {
    Serial.println("Update failed.");
    http.end();
    Blynk.connect();
    showEvent("OTA FAIL");
    return;
  }

  Serial.println("Update successfully completed. Rebooting...");
  http.end();

  showEvent("OTA OK"); 
  delay(200);
  ESP.restart();
}

void readLightAndPush() {
  lastLight = analogRead(LIGHT_PIN);
  Blynk.virtualWrite(V9, lastLight);
  updateLightLine();
}

void pollButtonDebounced() {
  bool raw = digitalRead(BUTTON_PIN);

  if (raw != lastButtonRaw) {
    lastDebounceMs = millis();
    lastButtonRaw = raw;
  }

  if ((millis() - lastDebounceMs) > DEBOUNCE_MS) {
    if (buttonStable != raw) {
      buttonStable = raw;
      Blynk.virtualWrite(V4, buttonStable ? 1 : 0);

      if (buttonStable) showEvent("Btn: pressed");
      else showEvent("Btn: released");
    }
  }
}

void setup(void) {
  Serial.begin(115200);
  Serial.println("Starting EX2 (EX1 + OTA)...");

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(LIGHT_PIN, INPUT);

  ledcSetup(BUZZER_CH, 2000, BUZZER_RES);
  ledcAttachPin(BUZZER_PIN, BUZZER_CH);
  stopBuzzer();

  servo.attach(SERVO_PIN);
  servo.write(0);

  int status = lcd.begin(LCD_COLS, LCD_ROWS);
  if (status) {
    Serial.println("LCD init failed!");
    hd44780::fatalError(status);
  }
  lcd.clear();
  showEvent("Initializing...");
  lastLight = analogRead(LIGHT_PIN);
  updateLightLine();

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  timer.setInterval(5000L, readLightAndPush);
  timer.setInterval(50L, pollButtonDebounced);
}

void loop(void) {
  Blynk.run();
  timer.run();

  if (buzzerOn && (millis() - buzzerStartMs >= BUZZER_MAX_MS)) {
    stopBuzzer();
    Blynk.virtualWrite(V3, 0);
    showEvent("Buz: auto OFF");
  }
}
