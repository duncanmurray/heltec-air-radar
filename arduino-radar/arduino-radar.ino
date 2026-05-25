#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/TomThumb.h>

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

enum AircraftArt {
  ART_HELICOPTER,
  ART_LIGHT_PROP,
  ART_TWIN_PROP,
  ART_BUSINESS_JET,
  ART_AIRLINER,
  ART_HEAVY_JET,
  ART_CARGO,
  ART_MILITARY,
  ART_GLIDER,
  ART_UNKNOWN
};

struct Target {
  float distanceKm;
  float dxKm;
  float dyKm;
  char hex[8];
  char callsign[8];
  char type[8];
  char category[4];
  char model[18];
  char readableType[14];
  char operatorName[20];
  char route[12];
  int altitudeFt;
  int headingDeg;
  int speedKt;
  uint32_t fetchedAtMs;
  uint32_t enrichedAtMs;
  uint16_t seenSeconds;
  bool enriched;
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
uint32_t detailScrollStartMs = 0;
int enrichRequestedIndex = -1;
uint32_t lastEnrichRequestMs = 0;

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

const char* broadAircraftType(const Target& target);

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
    memset(&target, 0, sizeof(Target));
    target.distanceKm = distance;
    target.dxKm = dx;
    target.dyKm = dy;
    strlcpy(target.hex, item["hex"] | "", sizeof(target.hex));
    strlcpy(target.callsign, item["flight"] | item["hex"] | "ICAO", sizeof(target.callsign));
    strlcpy(target.type, item["t"] | item["hex"] | "", sizeof(target.type));
    strlcpy(target.category, item["category"] | "", sizeof(target.category));
    strlcpy(target.model, target.type, sizeof(target.model));
    strlcpy(target.readableType, broadAircraftType(target), sizeof(target.readableType));
    strlcpy(target.operatorName, "Unknown", sizeof(target.operatorName));
    strlcpy(target.route, "Unknown", sizeof(target.route));
    target.altitudeFt = item["alt_baro"].is<int>() ? item["alt_baro"].as<int>() : item["alt_geom"] | 0;
    float heading = item["track"].is<float>() ? item["track"].as<float>() : item["true_heading"] | 0.0f;
    float speed = item["gs"].is<float>() ? item["gs"].as<float>() : item["tas"] | 0.0f;
    float seen = item["seen_pos"].is<float>() ? item["seen_pos"].as<float>() : item["seen"] | 0.0f;
    target.headingDeg = ((int)roundf(heading) % 360 + 360) % 360;
    target.speedKt = (int)roundf(speed);
    target.seenSeconds = (uint16_t)max(0, (int)roundf(seen));
    target.fetchedAtMs = now;
    target.enrichedAtMs = 0;
    target.enriched = false;
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

bool fetchPayload(const char* host, const String& path, String& payload) {
  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(HTTP_TIMEOUT_MS / 1000);

  if (!client.connect(host, 443, HTTP_TIMEOUT_MS)) {
    Serial.println("connect failed");
    return false;
  }

  client.print("GET ");
  client.print(path);
  client.print(" HTTP/1.1\r\nHost: ");
  client.print(host);
  client.print("\r\nUser-Agent: heltec-arduino-radar\r\nConnection: close\r\n\r\n");

  bool ok = readHttpBody(client, payload);
  client.stop();
  return ok;
}

String aircraftPath() {
  String path = "/v2/point/";
  path += String(HOME_LAT, 6);
  path += "/";
  path += String(HOME_LON, 6);
  path += "/";
  path += String(QUERY_RADIUS_NM);
  return path;
}

String compactId(const char* value) {
  String id = value;
  id.trim();
  id.replace(" ", "");
  return id;
}

void parseRoutePayload(const String& payload, Target& target) {
  DynamicJsonDocument doc(12288);
  if (deserializeJson(doc, payload)) return;

  JsonObject route = doc["response"]["flightroute"];
  if (route.isNull()) return;

  const char* airline = route["airline"]["name"] | "";
  const char* origin = route["origin"]["iata_code"] | route["origin"]["icao_code"] | "";
  const char* destination = route["destination"]["iata_code"] | route["destination"]["icao_code"] | "";

  if (airline[0]) strlcpy(target.operatorName, airline, sizeof(target.operatorName));
  if (origin[0] && destination[0]) {
    snprintf(target.route, sizeof(target.route), "%s>%s", origin, destination);
  }
}

void parseAircraftPayload(const String& payload, Target& target) {
  DynamicJsonDocument doc(12288);
  if (deserializeJson(doc, payload)) return;

  JsonObject aircraft = doc["response"]["aircraft"];
  if (aircraft.isNull()) return;

  const char* model = aircraft["type"] | "";
  const char* owner = aircraft["registered_owner"] | "";
  const char* icaoType = aircraft["icao_type"] | "";

  if (model[0]) strlcpy(target.model, model, sizeof(target.model));
  else if (icaoType[0]) strlcpy(target.model, icaoType, sizeof(target.model));

  if (owner[0] && strcmp(target.operatorName, "Unknown") == 0) {
    strlcpy(target.operatorName, owner, sizeof(target.operatorName));
  }
}

void enrichTarget(Target& target) {
  String payload;
  String callsign = compactId(target.callsign);
  String hex = compactId(target.hex);

  if (callsign.length()) {
    String path = "/v0/callsign/" + callsign;
    if (fetchPayload("api.adsbdb.com", path, payload)) parseRoutePayload(payload, target);
  }

  payload = "";
  if (hex.length()) {
    String path = "/v0/aircraft/" + hex;
    if (fetchPayload("api.adsbdb.com", path, payload)) parseAircraftPayload(payload, target);
  }

  strlcpy(target.readableType, broadAircraftType(target), sizeof(target.readableType));
  target.enrichedAtMs = millis();
  target.enriched = true;
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

    if (fetchPayload("api.adsb.lol", aircraftPath(), payload)) {
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

void enrichTask(void*) {
  for (;;) {
    int index = -1;
    Target target;

    if (detailMode && targetCount > 0) {
      xSemaphoreTake(targetMutex, portMAX_DELAY);
      index = selectedTarget;
      if (index >= targetCount) index = 0;
      target = targets[index];
      xSemaphoreGive(targetMutex);

      bool old = target.enriched && millis() - target.enrichedAtMs < 3600000UL;
      if (!old) {
        Serial.print("enrich ");
        Serial.println(target.callsign);
        enrichTarget(target);

        xSemaphoreTake(targetMutex, portMAX_DELAY);
        if (index < targetCount && strcmp(targets[index].hex, target.hex) == 0) {
          targets[index] = target;
        }
        xSemaphoreGive(targetMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void drawTarget(const Target& target) {
  float dx = 0;
  float dy = 0;
  projectedOffset(target, dx, dy);
  float distance = sqrt(dx * dx + dy * dy);
  if (distance > RANGE_KM) return;

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

bool startsWithAny(const char* value, const char* const prefixes[], int count) {
  for (int i = 0; i < count; i++) {
    if (strncmp(value, prefixes[i], strlen(prefixes[i])) == 0) return true;
  }
  return false;
}

bool containsAny(const char* value, const char* const needles[], int count) {
  for (int i = 0; i < count; i++) {
    if (strstr(value, needles[i])) return true;
  }
  return false;
}

bool isMilitaryLike(const Target& target) {
  static const char* const prefixes[] = {"RCH", "RRR", "NATO", "GAF", "FAF", "IAM", "ASY", "CNV", "LAGR"};
  return startsWithAny(target.callsign, prefixes, sizeof(prefixes) / sizeof(prefixes[0])) ||
         strcmp(target.category, "A7") == 0;
}

bool isHelicopterLike(const Target& target) {
  static const char* const prefixes[] = {"H", "R44", "R66", "EC", "AS", "B06", "B47", "S76", "S92"};
  return startsWithAny(target.type, prefixes, sizeof(prefixes) / sizeof(prefixes[0]));
}

bool isLightPropLike(const Target& target) {
  static const char* const types[] = {"C150", "C152", "C172", "C182", "C206", "PA28", "P28A", "SR20", "SR22", "DA40", "DA42", "BE36"};
  return startsWithAny(target.type, types, sizeof(types) / sizeof(types[0])) ||
         strcmp(target.category, "A1") == 0 ||
         strcmp(target.category, "A2") == 0;
}

bool isTwinPropLike(const Target& target) {
  static const char* const types[] = {"BE20", "BE30", "BE40", "B350", "C208", "DHC6", "AT43", "AT45", "AT72", "DH8", "E120", "JS41"};
  return startsWithAny(target.type, types, sizeof(types) / sizeof(types[0]));
}

bool isBusinessJetLike(const Target& target) {
  static const char* const types[] = {"C25", "C5", "C56", "C68", "CL30", "CL35", "CL60", "E35", "E45", "FA", "GLF", "GLEX", "LJ", "H25B"};
  return startsWithAny(target.type, types, sizeof(types) / sizeof(types[0]));
}

bool isHeavyJetLike(const Target& target) {
  static const char* const types[] = {"B74", "B77", "B78", "A33", "A34", "A35", "A38", "MD11", "DC10"};
  return startsWithAny(target.type, types, sizeof(types) / sizeof(types[0])) ||
         strcmp(target.category, "A5") == 0;
}

bool isCargoLike(const Target& target) {
  static const char* const prefixes[] = {"FDX", "UPS", "GTI", "BOX", "DHK", "BCS", "CLX", "ABR", "NPT"};
  return startsWithAny(target.callsign, prefixes, sizeof(prefixes) / sizeof(prefixes[0]));
}

bool isGliderLike(const Target& target) {
  static const char* const types[] = {"GLID", "ASW", "DG", "LS", "VENT", "SZD"};
  return startsWithAny(target.type, types, sizeof(types) / sizeof(types[0])) ||
         strcmp(target.category, "B1") == 0;
}

AircraftArt aircraftArtFor(const Target& target) {
  if (isHelicopterLike(target)) return ART_HELICOPTER;
  if (isMilitaryLike(target)) return ART_MILITARY;
  if (isGliderLike(target)) return ART_GLIDER;
  if (isCargoLike(target)) return ART_CARGO;
  if (isTwinPropLike(target)) return ART_TWIN_PROP;
  if (isBusinessJetLike(target)) return ART_BUSINESS_JET;
  if (isLightPropLike(target)) return ART_LIGHT_PROP;
  if (isHeavyJetLike(target)) return ART_HEAVY_JET;
  if (target.type[0]) return ART_AIRLINER;
  return ART_UNKNOWN;
}

const char* broadAircraftType(const Target& target) {
  switch (aircraftArtFor(target)) {
    case ART_HELICOPTER: return "Helicopter";
    case ART_LIGHT_PROP: return "Light prop";
    case ART_TWIN_PROP: return "Twin prop";
    case ART_BUSINESS_JET: return "Biz jet";
    case ART_AIRLINER: return "Airliner";
    case ART_HEAVY_JET: return "Heavy jet";
    case ART_CARGO: return "Cargo";
    case ART_MILITARY: return "Military";
    case ART_GLIDER: return "Glider";
    default: return "Aircraft";
  }
}

void drawAirlinerArt(int y) {
  display.fillRoundRect(9, y + 12, 42, 7, 3, SSD1306_WHITE);
  display.fillTriangle(51, y + 12, 61, y + 15, 51, y + 18, SSD1306_WHITE);
  display.fillTriangle(22, y + 12, 5, y + 3, 29, y + 12, SSD1306_WHITE);
  display.fillTriangle(24, y + 18, 8, y + 27, 31, y + 18, SSD1306_WHITE);
  display.fillTriangle(12, y + 12, 4, y + 5, 12, y + 18, SSD1306_WHITE);
  display.drawPixel(20, y + 14, SSD1306_BLACK);
  display.drawPixel(26, y + 14, SSD1306_BLACK);
  display.drawPixel(32, y + 14, SSD1306_BLACK);
  display.drawPixel(38, y + 14, SSD1306_BLACK);
}

void drawHeavyJetArt(int y) {
  display.fillRoundRect(5, y + 11, 49, 9, 4, SSD1306_WHITE);
  display.fillTriangle(54, y + 11, 63, y + 15, 54, y + 19, SSD1306_WHITE);
  display.fillTriangle(24, y + 11, 3, y + 1, 34, y + 12, SSD1306_WHITE);
  display.fillTriangle(25, y + 19, 5, y + 29, 35, y + 19, SSD1306_WHITE);
  display.fillTriangle(11, y + 11, 2, y + 3, 13, y + 20, SSD1306_WHITE);
  display.fillCircle(19, y + 21, 2, SSD1306_WHITE);
  display.fillCircle(43, y + 21, 2, SSD1306_WHITE);
  display.drawPixel(25, y + 14, SSD1306_BLACK);
  display.drawPixel(32, y + 14, SSD1306_BLACK);
  display.drawPixel(39, y + 14, SSD1306_BLACK);
}

void drawPropArt(int y) {
  display.fillRoundRect(14, y + 13, 34, 6, 3, SSD1306_WHITE);
  display.fillTriangle(48, y + 13, 56, y + 16, 48, y + 19, SSD1306_WHITE);
  display.fillTriangle(24, y + 13, 8, y + 4, 30, y + 13, SSD1306_WHITE);
  display.fillTriangle(24, y + 18, 8, y + 27, 30, y + 18, SSD1306_WHITE);
  display.fillTriangle(16, y + 13, 7, y + 8, 16, y + 19, SSD1306_WHITE);
  display.drawCircle(58, y + 16, 4, SSD1306_WHITE);
  display.drawLine(58, y + 8, 58, y + 24, SSD1306_WHITE);
  display.drawLine(50, y + 16, 63, y + 16, SSD1306_WHITE);
  display.drawPixel(24, y + 15, SSD1306_BLACK);
  display.drawPixel(29, y + 15, SSD1306_BLACK);
}

void drawHelicopterArt(int y) {
  display.drawLine(8, y + 4, 56, y + 4, SSD1306_WHITE);
  display.drawLine(32, y + 4, 32, y + 10, SSD1306_WHITE);
  display.fillRoundRect(17, y + 12, 27, 12, 5, SSD1306_WHITE);
  display.drawLine(44, y + 17, 59, y + 12, SSD1306_WHITE);
  display.drawLine(59, y + 8, 59, y + 16, SSD1306_WHITE);
  display.drawLine(55, y + 12, 63, y + 12, SSD1306_WHITE);
  display.drawLine(22, y + 24, 17, y + 29, SSD1306_WHITE);
  display.drawLine(39, y + 24, 44, y + 29, SSD1306_WHITE);
  display.drawLine(13, y + 29, 48, y + 29, SSD1306_WHITE);
  display.drawPixel(26, y + 16, SSD1306_BLACK);
  display.drawPixel(31, y + 16, SSD1306_BLACK);
}

void drawMilitaryArt(int y) {
  display.fillTriangle(8, y + 15, 62, y + 8, 50, y + 15, SSD1306_WHITE);
  display.fillTriangle(8, y + 15, 62, y + 22, 50, y + 15, SSD1306_WHITE);
  display.fillTriangle(24, y + 14, 4, y + 4, 32, y + 14, SSD1306_WHITE);
  display.fillTriangle(24, y + 16, 4, y + 26, 32, y + 16, SSD1306_WHITE);
  display.fillTriangle(13, y + 14, 5, y + 8, 13, y + 22, SSD1306_WHITE);
  display.drawPixel(51, y + 15, SSD1306_BLACK);
}

void drawGenericAircraftArt(int y) {
  display.fillRoundRect(10, y + 13, 42, 6, 3, SSD1306_WHITE);
  display.fillTriangle(52, y + 13, 60, y + 16, 52, y + 19, SSD1306_WHITE);
  display.fillTriangle(28, y + 13, 10, y + 4, 34, y + 13, SSD1306_WHITE);
  display.fillTriangle(28, y + 18, 10, y + 27, 34, y + 18, SSD1306_WHITE);
  display.fillTriangle(14, y + 13, 6, y + 8, 14, y + 19, SSD1306_WHITE);
}

void drawTwinPropArt(int y) {
  display.fillRoundRect(12, y + 13, 38, 6, 3, SSD1306_WHITE);
  display.fillTriangle(50, y + 13, 58, y + 16, 50, y + 19, SSD1306_WHITE);
  display.fillTriangle(25, y + 13, 6, y + 4, 34, y + 13, SSD1306_WHITE);
  display.fillTriangle(25, y + 18, 6, y + 27, 34, y + 18, SSD1306_WHITE);
  display.fillTriangle(15, y + 13, 7, y + 8, 15, y + 19, SSD1306_WHITE);
  display.drawCircle(16, y + 11, 3, SSD1306_WHITE);
  display.drawCircle(16, y + 21, 3, SSD1306_WHITE);
  display.drawLine(13, y + 11, 19, y + 11, SSD1306_WHITE);
  display.drawLine(13, y + 21, 19, y + 21, SSD1306_WHITE);
}

void drawBusinessJetArt(int y) {
  display.fillRoundRect(9, y + 13, 43, 6, 3, SSD1306_WHITE);
  display.fillTriangle(52, y + 13, 61, y + 16, 52, y + 19, SSD1306_WHITE);
  display.fillTriangle(26, y + 13, 10, y + 5, 33, y + 13, SSD1306_WHITE);
  display.fillTriangle(27, y + 18, 12, y + 26, 34, y + 18, SSD1306_WHITE);
  display.fillTriangle(10, y + 13, 4, y + 6, 13, y + 19, SSD1306_WHITE);
  display.fillCircle(17, y + 20, 2, SSD1306_WHITE);
  display.fillCircle(23, y + 20, 2, SSD1306_WHITE);
  display.drawPixel(37, y + 15, SSD1306_BLACK);
}

void drawCargoArt(int y) {
  display.fillRoundRect(5, y + 10, 47, 11, 2, SSD1306_WHITE);
  display.fillTriangle(52, y + 10, 62, y + 15, 52, y + 20, SSD1306_WHITE);
  display.fillTriangle(24, y + 10, 5, y + 2, 35, y + 10, SSD1306_WHITE);
  display.fillTriangle(25, y + 20, 6, y + 28, 36, y + 20, SSD1306_WHITE);
  display.fillTriangle(10, y + 10, 2, y + 3, 12, y + 21, SSD1306_WHITE);
  display.drawRect(14, y + 13, 10, 5, SSD1306_BLACK);
  display.drawRect(28, y + 13, 10, 5, SSD1306_BLACK);
}

void drawGliderArt(int y) {
  display.drawLine(4, y + 14, 60, y + 14, SSD1306_WHITE);
  display.drawLine(28, y + 14, 58, y + 8, SSD1306_WHITE);
  display.drawLine(28, y + 14, 58, y + 20, SSD1306_WHITE);
  display.fillRoundRect(12, y + 13, 25, 4, 2, SSD1306_WHITE);
  display.fillTriangle(37, y + 13, 45, y + 15, 37, y + 17, SSD1306_WHITE);
  display.drawLine(13, y + 13, 6, y + 7, SSD1306_WHITE);
  display.drawLine(13, y + 16, 6, y + 23, SSD1306_WHITE);
}

void drawAircraftArt(const Target& target, int y) {
  switch (aircraftArtFor(target)) {
    case ART_HELICOPTER: drawHelicopterArt(y); break;
    case ART_LIGHT_PROP: drawPropArt(y); break;
    case ART_TWIN_PROP: drawTwinPropArt(y); break;
    case ART_BUSINESS_JET: drawBusinessJetArt(y); break;
    case ART_AIRLINER: drawAirlinerArt(y); break;
    case ART_HEAVY_JET: drawHeavyJetArt(y); break;
    case ART_CARGO: drawCargoArt(y); break;
    case ART_MILITARY: drawMilitaryArt(y); break;
    case ART_GLIDER: drawGliderArt(y); break;
    default: drawGenericAircraftArt(y); break;
  }
}

void drawDetailRow(int& y, const char* label, String value, int valueLines = 1) {
  value.trim();
  if (!value.length()) value = "Unknown";
  display.setCursor(0, y + 5);
  display.print(label);
  display.print(" ");
  int firstLineChars = max(0, 16 - (int)strlen(label) - 1);
  display.print(value.substring(0, firstLineChars));
  y += 7;
  for (int i = 0; i < valueLines; i++) {
    int start = firstLineChars + i * 16;
    if (start >= value.length()) break;
    display.setCursor(0, y + 5);
    display.print(value.substring(start, start + 16));
    y += 7;
  }
  y += 1;
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
  display.setFont(NULL);
  display.setCursor(16, 0);
  display.printf("%02d/%02d", selectedTarget + 1, count);
  drawAircraftArt(target, 9);
  display.setFont(&TomThumb);
  int rowY = 41;
  drawDetailRow(rowY, "CALL", String(target.callsign), 0);
  drawDetailRow(rowY, "MODEL", String(target.model), 0);
  drawDetailRow(rowY, "TYPE", String(target.readableType), 0);
  drawDetailRow(rowY, "OPER", String(target.operatorName), 1);
  drawDetailRow(rowY, "POS", String((int)distance) + "km " + compassFromOffset(dx, dy), 0);
  drawDetailRow(rowY, "ALT", String(target.altitudeFt) + "ft", 0);
  drawDetailRow(rowY, "HDG", String(compassFromDegrees(target.headingDeg)) + " " + String(target.headingDeg), 0);
  drawDetailRow(rowY, "SPD", String(target.speedKt) + "kt", 0);
  drawDetailRow(rowY, "ROUTE", String(target.route), 0);
  display.setFont(NULL);
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
  xTaskCreatePinnedToCore(enrichTask, "enrich", 12288, nullptr, 1, nullptr, 0);
}

void loop() {
  String event = readButtonEvent();
  if (event == "short" && targetCount > 0) {
    if (!detailMode) selectedTarget = 0;
    else selectedTarget = (selectedTarget + 1) % targetCount;
    detailMode = true;
    detailScrollStartMs = millis();
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
