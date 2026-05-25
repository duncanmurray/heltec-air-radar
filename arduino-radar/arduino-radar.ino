#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

constexpr int OLED_WIDTH = 128;
constexpr int OLED_HEIGHT = 64;
constexpr int OLED_ADDR = 0x3C;
constexpr int OLED_SDA = 4;
constexpr int OLED_SCL = 15;
constexpr int OLED_RST = 16;
constexpr int OLED_ROTATION = 1;

constexpr int PROGRAM_BUTTON = 0;
constexpr int BATTERY_PIN = 37;

constexpr double HOME_LAT = 51.500000;
constexpr double HOME_LON = -0.120000;
const char* HOME_PLACE = "HOME";

constexpr float RANGE_KM = 15.0f;
constexpr int QUERY_RADIUS_NM = 8;
constexpr uint32_t FETCH_MS = 30000;
constexpr uint32_t FIRST_FETCH_DELAY_MS = 5000;
constexpr uint32_t HTTP_TIMEOUT_MS = 10000;
constexpr uint32_t WIFI_TIMEOUT_MS = 12000;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;
constexpr uint32_t BUTTON_LONG_MS = 900;
constexpr int MAX_TARGETS = 20;

constexpr int RADAR_CX = 32;
constexpr int RADAR_CY = 52;
constexpr int RADAR_R = 27;

struct Target {
  float distanceKm;
  float dxKm;
  float dyKm;
  char callsign[8];
  char type[8];
  int altitudeFt;
  int headingDeg;
  int speedKt;
  uint32_t fetchedAtMs;
  uint16_t seenSeconds;
};

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RST);
SemaphoreHandle_t targetMutex;
Target targets[MAX_TARGETS];
int targetCount = 0;
bool dataStale = true;
bool fetchInProgress = false;
uint32_t lastGoodFetchMs = 0;

bool detailMode = false;
int selectedTarget = 0;
int sweepAngle = 0;

bool buttonRaw = false;
bool buttonStable = false;
bool longSent = false;
uint32_t buttonChangedMs = 0;
uint32_t buttonPressedMs = 0;

float batteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  return raw * 3.6f / 4095.0f * 2.0f;
}

bool connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < WIFI_TIMEOUT_MS) {
    delay(100);
  }

  Serial.print("wifi ");
  Serial.println(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "failed");
  return WiFi.status() == WL_CONNECTED;
}

String aircraftUrl() {
  String url = "https://api.adsb.lol/v2/point/";
  url += String(HOME_LAT, 6);
  url += "/";
  url += String(HOME_LON, 6);
  url += "/";
  url += String(QUERY_RADIUS_NM);
  return url;
}

void kmFromCenter(double lat, double lon, float& dx, float& dy, float& distance) {
  dy = (lat - HOME_LAT) * 111.32f;
  dx = (lon - HOME_LON) * 111.32f * cos(HOME_LAT * DEG_TO_RAD);
  distance = sqrt(dx * dx + dy * dy);
}

void projectedOffset(const Target& target, float& dx, float& dy) {
  uint32_t ageMs = millis() - target.fetchedAtMs + target.seenSeconds * 1000UL;
  float km = target.speedKt * 1.852f * (ageMs / 1000.0f) / 3600.0f;
  float angle = target.headingDeg * DEG_TO_RAD;
  dx = target.dxKm + sin(angle) * km;
  dy = target.dyKm + cos(angle) * km;
}

const char* compassFromOffset(float dx, float dy) {
  static const char* dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  float angle = atan2(dx, dy) * RAD_TO_DEG;
  if (angle < 0) angle += 360.0f;
  return dirs[((int)((angle + 22.5f) / 45.0f)) & 7];
}

const char* compassFromDegrees(int angle) {
  static const char* dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  angle = (angle % 360 + 360) % 360;
  return dirs[((angle + 22) / 45) & 7];
}

int compareTargets(const void* a, const void* b) {
  const Target* ta = static_cast<const Target*>(a);
  const Target* tb = static_cast<const Target*>(b);
  return (ta->distanceKm > tb->distanceKm) - (ta->distanceKm < tb->distanceKm);
}

bool parseAircraft(const String& payload, Target* outTargets, int& outCount) {
  DynamicJsonDocument doc(32768);
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.print("json failed ");
    Serial.println(err.c_str());
    return false;
  }

  JsonArray aircraft = doc["ac"].as<JsonArray>();
  outCount = 0;
  uint32_t now = millis();

  for (JsonObject item : aircraft) {
    if (outCount >= MAX_TARGETS) break;
    if (item["lat"].isNull() || item["lon"].isNull()) continue;

    float dx = 0;
    float dy = 0;
    float distance = 0;
    kmFromCenter(item["lat"].as<double>(), item["lon"].as<double>(), dx, dy, distance);
    if (distance > RANGE_KM) continue;

    Target& target = outTargets[outCount++];
    target.distanceKm = distance;
    target.dxKm = dx;
    target.dyKm = dy;
    strlcpy(target.callsign, item["flight"] | item["hex"] | "ICAO", sizeof(target.callsign));
    strlcpy(target.type, item["t"] | item["hex"] | "", sizeof(target.type));
    target.altitudeFt = item["alt_baro"].is<int>() ? item["alt_baro"].as<int>() : item["alt_geom"] | 0;
    float heading = item["track"].is<float>() ? item["track"].as<float>() : item["true_heading"] | 0.0f;
    float speed = item["gs"].is<float>() ? item["gs"].as<float>() : item["tas"] | 0.0f;
    float seen = item["seen_pos"].is<float>() ? item["seen_pos"].as<float>() : item["seen"] | 0.0f;
    target.headingDeg = ((int)roundf(heading) % 360 + 360) % 360;
    target.speedKt = (int)roundf(speed);
    target.seenSeconds = (uint16_t)max(0, (int)roundf(seen));
    target.fetchedAtMs = now;
  }

  qsort(outTargets, outCount, sizeof(Target), compareTargets);
  return true;
}

bool readHttpBody(WiFiClientSecure& client, String& body) {
  uint32_t deadline = millis() + HTTP_TIMEOUT_MS;
  String headers;

  while (client.connected() && millis() < deadline) {
    while (client.available()) {
      char c = client.read();
      headers += c;
      if (headers.endsWith("\r\n\r\n")) goto headersDone;
    }
    delay(1);
  }
  return false;

headersDone:
  bool chunked = headers.indexOf("Transfer-Encoding: chunked") >= 0 ||
                 headers.indexOf("transfer-encoding: chunked") >= 0;
  body.reserve(8192);

  if (!chunked) {
    while (client.connected() && millis() < deadline) {
      while (client.available()) body += (char)client.read();
      delay(1);
    }
    return body.length() > 0;
  }

  for (;;) {
    String sizeLine = client.readStringUntil('\n');
    sizeLine.trim();
    if (!sizeLine.length()) {
      if (millis() > deadline) return false;
      delay(1);
      continue;
    }
    int chunkSize = (int)strtol(sizeLine.c_str(), nullptr, 16);
    if (chunkSize <= 0) break;

    int remaining = chunkSize;
    while (remaining > 0 && millis() < deadline) {
      int available = client.available();
      if (!available) {
        delay(1);
        continue;
      }
      int count = min(available, remaining);
      while (count--) {
        body += (char)client.read();
        remaining--;
      }
    }
    client.read();
    client.read();
    if (millis() >= deadline) return false;
  }

  return body.length() > 0;
}

bool fetchPayload(String& payload) {
  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(HTTP_TIMEOUT_MS / 1000);

  const char* host = "api.adsb.lol";
  if (!client.connect(host, 443, HTTP_TIMEOUT_MS)) {
    Serial.println("connect failed");
    return false;
  }

  String path = "/v2/point/";
  path += String(HOME_LAT, 6);
  path += "/";
  path += String(HOME_LON, 6);
  path += "/";
  path += String(QUERY_RADIUS_NM);

  client.print("GET ");
  client.print(path);
  client.print(" HTTP/1.1\r\nHost: ");
  client.print(host);
  client.print("\r\nUser-Agent: heltec-arduino-radar\r\nConnection: close\r\n\r\n");

  bool ok = readHttpBody(client, payload);
  client.stop();
  return ok;
}

void fetchTask(void*) {
  delay(FIRST_FETCH_DELAY_MS);

  for (;;) {
    if (!connectWiFi()) {
      dataStale = true;
      vTaskDelay(pdMS_TO_TICKS(10000));
      continue;
    }

    fetchInProgress = true;
    Serial.println("fetch flights");

    Target freshTargets[MAX_TARGETS];
    int freshCount = 0;
    bool ok = false;
    String payload;

    if (fetchPayload(payload)) {
      ok = parseAircraft(payload, freshTargets, freshCount);
    } else {
      Serial.println("http failed");
    }

    if (ok) {
      xSemaphoreTake(targetMutex, portMAX_DELAY);
      memcpy(targets, freshTargets, sizeof(Target) * freshCount);
      targetCount = freshCount;
      dataStale = freshCount == 0;
      lastGoodFetchMs = freshCount ? millis() : lastGoodFetchMs;
      if (selectedTarget >= targetCount) selectedTarget = 0;
      xSemaphoreGive(targetMutex);
      Serial.printf("targets %d\n", freshCount);
    } else {
      dataStale = true;
    }

    fetchInProgress = false;
    vTaskDelay(pdMS_TO_TICKS(FETCH_MS));
  }
}

void drawTarget(const Target& target) {
  float dx = 0;
  float dy = 0;
  projectedOffset(target, dx, dy);

  int x = RADAR_CX + (int)(dx / RANGE_KM * RADAR_R);
  int y = RADAR_CY - (int)(dy / RANGE_KM * RADAR_R);
  display.drawPixel(x, y, SSD1306_WHITE);
  display.drawPixel(x + 1, y, SSD1306_WHITE);
  display.drawPixel(x, y + 1, SSD1306_WHITE);

  float nose = (target.headingDeg - 90) * DEG_TO_RAD;
  display.drawPixel(x + (int)(cos(nose) * 3), y + (int)(sin(nose) * 3), SSD1306_WHITE);
}

void drawRadar() {
  Target localTargets[MAX_TARGETS];
  int localCount = 0;
  bool stale = false;
  bool fetching = fetchInProgress;

  xSemaphoreTake(targetMutex, portMAX_DELAY);
  localCount = targetCount;
  stale = dataStale;
  memcpy(localTargets, targets, sizeof(Target) * localCount);
  xSemaphoreGive(targetMutex);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(HOME_PLACE);
  display.setCursor(48, 0);
  display.printf("%02d", localCount);

  display.drawCircle(RADAR_CX, RADAR_CY, RADAR_R, SSD1306_WHITE);
  display.drawCircle(RADAR_CX, RADAR_CY, RADAR_R / 2, SSD1306_WHITE);
  display.drawLine(RADAR_CX - RADAR_R, RADAR_CY, RADAR_CX + RADAR_R, RADAR_CY, SSD1306_WHITE);
  display.drawLine(RADAR_CX, RADAR_CY - RADAR_R, RADAR_CX, RADAR_CY + RADAR_R, SSD1306_WHITE);

  float sweep = sweepAngle * DEG_TO_RAD;
  display.drawLine(
    RADAR_CX,
    RADAR_CY,
    RADAR_CX + (int)(cos(sweep) * RADAR_R),
    RADAR_CY + (int)(sin(sweep) * RADAR_R),
    SSD1306_WHITE
  );

  for (int i = 0; i < localCount; i++) drawTarget(localTargets[i]);

  if (localCount > 0) {
    float dx = 0;
    float dy = 0;
    projectedOffset(localTargets[0], dx, dy);
    float distance = sqrt(dx * dx + dy * dy);
    display.setCursor(0, 98);
    display.print(localTargets[0].callsign);
    display.setCursor(0, 108);
    display.printf("%02dkm %s", (int)distance, compassFromOffset(dx, dy));
    display.setCursor(0, 118);
    display.printf("%dft", localTargets[0].altitudeFt);
  } else {
    display.setCursor(fetching ? 12 : 8, 114);
    display.print(fetching ? "FETCH" : (stale ? "NODATA" : "SCAN"));
  }

  display.display();
  sweepAngle = (sweepAngle + 12) % 360;
}

void drawDetail() {
  Target target;
  int count = 0;

  xSemaphoreTake(targetMutex, portMAX_DELAY);
  count = targetCount;
  if (selectedTarget >= count) selectedTarget = 0;
  target = targets[selectedTarget];
  xSemaphoreGive(targetMutex);

  float dx = 0;
  float dy = 0;
  projectedOffset(target, dx, dy);
  float distance = sqrt(dx * dx + dy * dy);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(16, 0);
  display.printf("%02d/%02d", selectedTarget + 1, count);
  display.setCursor(0, 18);
  display.print(target.callsign);
  display.setCursor(0, 30);
  display.print(target.type);
  display.setCursor(0, 48);
  display.printf("%02dkm %s", (int)distance, compassFromOffset(dx, dy));
  display.setCursor(0, 60);
  display.printf("%dft", target.altitudeFt);
  display.setCursor(0, 78);
  display.printf("%s %03d", compassFromDegrees(target.headingDeg), target.headingDeg);
  display.setCursor(0, 90);
  display.printf("%dkt", target.speedKt);
  display.setCursor(0, 116);
  display.print("HOLD=RAD");
  display.display();
}

String readButtonEvent() {
  uint32_t now = millis();
  bool pressed = digitalRead(PROGRAM_BUTTON) == LOW;

  if (pressed != buttonRaw) {
    buttonRaw = pressed;
    buttonChangedMs = now;
  }

  if (now - buttonChangedMs > BUTTON_DEBOUNCE_MS && pressed != buttonStable) {
    buttonStable = pressed;
    if (pressed) {
      buttonPressedMs = now;
      longSent = false;
    } else if (!longSent) {
      return "short";
    }
  }

  if (buttonStable && !longSent && now - buttonPressedMs > BUTTON_LONG_MS) {
    longSent = true;
    return "long";
  }

  return "";
}

void setup() {
  Serial.begin(115200);
  pinMode(PROGRAM_BUTTON, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("oled failed");
    for (;;) delay(1000);
  }
  display.setRotation(OLED_ROTATION);
  display.clearDisplay();
  display.display();

  targetMutex = xSemaphoreCreateMutex();
  connectWiFi();
  xTaskCreatePinnedToCore(fetchTask, "fetch", 12288, nullptr, 1, nullptr, 0);
}

void loop() {
  String event = readButtonEvent();
  if (event == "short" && targetCount > 0) {
    if (!detailMode) selectedTarget = 0;
    else selectedTarget = (selectedTarget + 1) % targetCount;
    detailMode = true;
  } else if (event == "long") {
    detailMode = false;
  }

  if (detailMode && targetCount > 0) drawDetail();
  else {
    detailMode = false;
    drawRadar();
  }

  delay(120);
}
