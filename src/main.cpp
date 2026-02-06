#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <BH1750.h>
#include <Adafruit_NeoMatrix.h>
#include <LittleFS.h>

/* ================= CONFIG ================= */
// BH1750 uses digital pins as power supply (no breadboard available)
#define BH_PWR_PIN  27
#define BH_GND_PIN  26
#define SDA_PIN     21
#define SCL_PIN     22

#define TRIG_PIN    13
#define ECHO_PIN    14
#define LED_PIN     15

#define MATRIX_W    8
#define MATRIX_H    8

// Brightness calibration
#define ABSOLUTE_MAX_BRIGHT 200 
#define MIN_VISIBLE_BRIGHT  4.0f 

#define SENSOR_INTERVAL     30
#define ULTRASONIC_TIMEOUT  23500 

const char* ssid     = "wifi";
const char* password = "password";
const char* hostname = "led"; 

/* ================= OBJECTS ================= */
AsyncWebServer server(80);
BH1750 lightMeter;
Adafruit_NeoMatrix matrix(MATRIX_W, MATRIX_H, LED_PIN,
  NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB + NEO_KHZ800);

/* ================= STATE ================= */
struct {
  float lux = 0.0f;
  float dist = 0.0f;
  int luxThresh = 300;
  int maxDist = 200;
  bool manual = false;
  uint8_t r = 255, g = 255, b = 255;
  uint8_t brightness = 200;
  float currentBright = 0.0f;
  unsigned long lastRead = 0;
} state;

constexpr float SMOOTHING = 0.04f;

/* ================= HELPERS ================= */
inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long d = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT); // time for ultrasonic travel to and fro.
  return d ? d * 0.0343f * 0.5f : state.maxDist; // time x speed (cm/microsecond) /2 - distance from sensor to wall.
}

/* ================= API ================= */
void handleData(AsyncWebServerRequest *r) {
  char json[180];
  snprintf(json, sizeof(json),
    "{\"lx\":%.1f,\"ds\":%.1f,\"tp\":%.1f,\"md\":%d,\"al\":%d,\"ad\":%d,\"br\":%d}",
    state.lux, state.dist, temperatureRead(), state.manual, 
    state.luxThresh, state.maxDist, state.brightness
  );
  r->send(200, "application/json", json);
}

void handleSet(AsyncWebServerRequest *r) {
  if (r->hasParam("m")) state.manual = r->getParam("m")->value().toInt();
  if (r->hasParam("b")) state.brightness = r->getParam("b")->value().toInt();
  if (r->hasParam("al")) state.luxThresh = r->getParam("al")->value().toInt();
  if (r->hasParam("ad")) state.maxDist = r->getParam("ad")->value().toInt();

  if (r->hasParam("c")) {
    uint32_t col = strtoul(r->getParam("c")->value().c_str(), nullptr, 16);
    state.r = col >> 16;
    state.g = col >> 8;
    state.b = col;
    state.manual = true; 
  }
  r->send(200, "OK");
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(1000);

  // initialize LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
    return;
  }
  Serial.println("LittleFS mounted successfully");

  // setup BH1750 power pins (digital pins used as VCC/GND)
  pinMode(BH_PWR_PIN, OUTPUT);
  pinMode(BH_GND_PIN, OUTPUT);
  digitalWrite(BH_PWR_PIN, HIGH);
  digitalWrite(BH_GND_PIN, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);

  matrix.begin();
  matrix.setBrightness(0);
  matrix.fillScreen(matrix.Color(255, 255, 255));
  matrix.show();

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname);
  WiFi.begin(ssid, password);

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(200);
  }
  Serial.println("\nWiFi connected!");
  Serial.print("http://"); Serial.print(hostname); Serial.println(".local");

  if (MDNS.begin(hostname)) Serial.println("mDNS active");

  // serving html file from littleFS
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  
  // API endpoints
  server.on("/data", HTTP_GET, handleData);
  server.on("/set", HTTP_GET, handleSet);
  
  server.begin();
}

/* ================= LOOP ================= */
void loop() {
  unsigned long now = millis();
  
  if (now - state.lastRead < SENSOR_INTERVAL) return;
  state.lastRead = now;

  // Read sensors
  float l = lightMeter.readLightLevel();
  if (l >= 0) state.lux = l;
  state.dist = readDistanceCM();

  float targetBright;

  if (state.manual) {
    // manual mode
    if (state.brightness == 0) {
      targetBright = 0.0f;
    } else {
      float linear = state.brightness / 255.0f; 
      float corrected = linear * linear;
      targetBright = MIN_VISIBLE_BRIGHT + (corrected * (ABSOLUTE_MAX_BRIGHT - MIN_VISIBLE_BRIGHT));
    }
    matrix.fillScreen(matrix.Color(state.r, state.g, state.b));
    state.currentBright = targetBright;

  } else {
    // auto mode
    float lf = mapf(constrain(state.lux, 0, state.luxThresh), 0, state.luxThresh, 1.0f, 0.0f);
    float df = mapf(constrain(state.dist, 0, state.maxDist), 0, state.maxDist, 0.1f, 1.0f);
    float combined = lf * df; 
    
    if (combined <= 0.01f) {
      targetBright = 0.0f;
    } else {
      float corrected = combined * combined;
      targetBright = MIN_VISIBLE_BRIGHT + (corrected * (ABSOLUTE_MAX_BRIGHT - MIN_VISIBLE_BRIGHT));
    }
    
    matrix.fillScreen(matrix.Color(255, 255, 255));
    state.currentBright += (targetBright - state.currentBright) * SMOOTHING;
  }

  if (targetBright == 0.0f && state.currentBright < 1.0f) {
    state.currentBright = 0.0f;
  }
  
  matrix.setBrightness((uint8_t)state.currentBright);
  matrix.show();
}
