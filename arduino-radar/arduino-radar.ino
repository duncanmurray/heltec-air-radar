#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/TomThumb.h>
#include "aircraft_bitmaps.h"

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
constexpr uint32_t DETAIL_ART_SCROLL_MS = 4000;
constexpr uint32_t DETAIL_TEXT_HOLD_MS = 14000;
constexpr uint32_t ACTIVE_FRAME_MS = 120;
constexpr uint32_t STATIC_FRAME_MS = 500;
constexpr uint32_t SPLASH_MS = 3200;
constexpr uint8_t OLED_CONTRAST = 0x8F;
constexpr uint32_t CPU_MHZ = 80;
constexpr int MAX_TARGETS = 20;
constexpr int ENRICH_CACHE_SIZE = 24;
constexpr uint32_t ENRICH_CACHE_TTL_MS = 6UL * 60UL * 60UL * 1000UL;

constexpr int RADAR_CX = 32;
constexpr int RADAR_CY = 52;
constexpr int RADAR_R = 27;

enum AircraftArt {
  ART_HELICOPTER,
  ART_LIGHT_PROP,
  ART_TWIN_PROP,
  ART_BUSINESS_JET,
  ART_REGIONAL_JET,
  ART_NARROWBODY_JET,
  ART_AIRLINER,
  ART_HEAVY_JET,
  ART_CARGO,
  ART_MILITARY,
  ART_GLIDER,
  ART_BALLOON,
  ART_ULTRALIGHT,
  ART_SKYDIVER,
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

struct EnrichCacheEntry {
  char key[16];
  char callsign[8];
  char model[18];
  char readableType[14];
  char operatorName[20];
  char route[12];
  uint32_t storedAtMs;
  bool valid;
};

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RST);
SemaphoreHandle_t targetMutex;
SemaphoreHandle_t cacheMutex;
Target targets[MAX_TARGETS];
EnrichCacheEntry enrichCache[ENRICH_CACHE_SIZE];
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
  WiFi.setSleep(true);
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
    strlcpy(target.type, item["t"] | "", sizeof(target.type));
    strlcpy(target.category, item["category"] | "", sizeof(target.category));
    strlcpy(target.model, target.type, sizeof(target.model));
    strlcpy(target.readableType, broadAircraftType(target), sizeof(target.readableType));
    strlcpy(target.operatorName, "Unknown", sizeof(target.operatorName));
    strlcpy(target.route, "Unknown", sizeof(target.route));
    loadEnrichCache(target);
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

void cacheKeyFor(const Target& target, char* key, size_t keySize) {
  String id = compactId(target.hex);
  if (!id.length()) id = compactId(target.callsign);
  strlcpy(key, id.c_str(), keySize);
}

void applyCacheEntry(Target& target, const EnrichCacheEntry& entry) {
  if (entry.callsign[0]) strlcpy(target.callsign, entry.callsign, sizeof(target.callsign));
  if (entry.model[0]) strlcpy(target.model, entry.model, sizeof(target.model));
  if (entry.readableType[0]) strlcpy(target.readableType, entry.readableType, sizeof(target.readableType));
  if (entry.operatorName[0]) strlcpy(target.operatorName, entry.operatorName, sizeof(target.operatorName));
  if (entry.route[0]) strlcpy(target.route, entry.route, sizeof(target.route));
  target.enrichedAtMs = entry.storedAtMs;
  target.enriched = true;
}

bool loadEnrichCache(Target& target) {
  char key[16];
  cacheKeyFor(target, key, sizeof(key));
  if (!key[0]) return false;

  bool hit = false;
  xSemaphoreTake(cacheMutex, portMAX_DELAY);
  for (int i = 0; i < ENRICH_CACHE_SIZE; i++) {
    if (!enrichCache[i].valid || strcmp(enrichCache[i].key, key) != 0) continue;
    if (millis() - enrichCache[i].storedAtMs > ENRICH_CACHE_TTL_MS) {
      enrichCache[i].valid = false;
      break;
    }
    applyCacheEntry(target, enrichCache[i]);
    hit = true;
    break;
  }
  xSemaphoreGive(cacheMutex);
  return hit;
}

void storeEnrichCache(const Target& target) {
  char key[16];
  cacheKeyFor(target, key, sizeof(key));
  if (!key[0]) return;

  xSemaphoreTake(cacheMutex, portMAX_DELAY);
  int slot = -1;
  uint32_t oldest = UINT32_MAX;
  for (int i = 0; i < ENRICH_CACHE_SIZE; i++) {
    if (enrichCache[i].valid && strcmp(enrichCache[i].key, key) == 0) {
      slot = i;
      break;
    }
    if (!enrichCache[i].valid) {
      slot = i;
      break;
    }
    if (enrichCache[i].storedAtMs < oldest) {
      oldest = enrichCache[i].storedAtMs;
      slot = i;
    }
  }

  EnrichCacheEntry& entry = enrichCache[slot];
  memset(&entry, 0, sizeof(entry));
  strlcpy(entry.key, key, sizeof(entry.key));
  strlcpy(entry.callsign, target.callsign, sizeof(entry.callsign));
  strlcpy(entry.model, target.model, sizeof(entry.model));
  strlcpy(entry.readableType, target.readableType, sizeof(entry.readableType));
  strlcpy(entry.operatorName, target.operatorName, sizeof(entry.operatorName));
  strlcpy(entry.route, target.route, sizeof(entry.route));
  entry.storedAtMs = millis();
  entry.valid = true;
  xSemaphoreGive(cacheMutex);
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
  if (loadEnrichCache(target)) {
    Serial.print("cache ");
    Serial.println(target.callsign);
    return;
  }

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
  storeEnrichCache(target);
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

  static const int8_t arrowheads[8][4][2] = {
    {{0, -2}, {0, -1}, {-1, 0}, {1, 0}},
    {{2, -2}, {1, -1}, {0, -1}, {1, 0}},
    {{2, 0}, {1, 0}, {0, -1}, {0, 1}},
    {{2, 2}, {1, 1}, {1, 0}, {0, 1}},
    {{0, 2}, {0, 1}, {-1, 0}, {1, 0}},
    {{-2, 2}, {-1, 1}, {0, 1}, {-1, 0}},
    {{-2, 0}, {-1, 0}, {0, -1}, {0, 1}},
    {{-2, -2}, {-1, -1}, {-1, 0}, {0, -1}}
  };
  int dir = ((target.headingDeg + 22) / 45) & 7;
  for (int i = 0; i < 4; i++) {
    display.drawPixel(x + arrowheads[dir][i][0], y + arrowheads[dir][i][1], SSD1306_WHITE);
  }
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

char asciiUpper(char c) {
  if (c >= 'a' && c <= 'z') return c - 32;
  return c;
}

bool containsIgnoreCase(const char* value, const char* needle) {
  if (!needle[0]) return true;
  for (int i = 0; value[i]; i++) {
    int j = 0;
    while (needle[j] && value[i + j] && asciiUpper(value[i + j]) == asciiUpper(needle[j])) j++;
    if (!needle[j]) return true;
  }
  return false;
}

bool containsAnyIgnoreCase(const char* value, const char* const needles[], int count) {
  for (int i = 0; i < count; i++) {
    if (containsIgnoreCase(value, needles[i])) return true;
  }
  return false;
}

bool equalsAny(const char* value, const char* const choices[], int count) {
  for (int i = 0; i < count; i++) {
    if (strcmp(value, choices[i]) == 0) return true;
  }
  return false;
}

bool isMilitaryLike(const Target& target) {
  static const char* const prefixes[] = {"RCH", "RRR", "NATO", "GAF", "FAF", "IAM", "ASY", "CNV", "LAGR"};
  static const char* const words[] = {"AIR FORCE", "ARMY", "NAVY", "MILITARY", "NATO"};
  return startsWithAny(target.callsign, prefixes, sizeof(prefixes) / sizeof(prefixes[0])) ||
         containsAnyIgnoreCase(target.operatorName, words, sizeof(words) / sizeof(words[0])) ||
         containsAnyIgnoreCase(target.model, words, sizeof(words) / sizeof(words[0]));
}

bool isHelicopterLike(const Target& target) {
  static const char* const prefixes[] = {"H", "R44", "R66", "EC", "AS", "B06", "B47", "S76", "S92"};
  return startsWithAny(target.type, prefixes, sizeof(prefixes) / sizeof(prefixes[0])) ||
         strcmp(target.category, "A7") == 0;
}

bool isLightPropLike(const Target& target) {
  static const char* const types[] = {"C150", "C152", "C172", "C182", "C206", "PA28", "P28A", "SR20", "SR22", "DA40", "DA42", "BE36"};
  static const char* const words[] = {"CESSNA", "PIPER", "CIRRUS", "DIAMOND", "SKYHAWK", "CHEROKEE"};
  return startsWithAny(target.type, types, sizeof(types) / sizeof(types[0])) ||
         containsAnyIgnoreCase(target.model, words, sizeof(words) / sizeof(words[0]));
}

bool isTwinPropLike(const Target& target) {
  static const char* const types[] = {"BE20", "BE30", "BE40", "B350", "C208", "DHC6", "AT43", "AT45", "AT72", "DH8", "E120", "JS41"};
  static const char* const words[] = {"TURBOPROP", "DASH 8", "ATR", "KING AIR", "TWIN OTTER"};
  return startsWithAny(target.type, types, sizeof(types) / sizeof(types[0])) ||
         containsAnyIgnoreCase(target.model, words, sizeof(words) / sizeof(words[0]));
}

bool isBusinessJetLike(const Target& target) {
  static const char* const types[] = {"C25", "C5", "C56", "C68", "CL30", "CL35", "CL60", "E35", "E45", "FA", "GLF", "GLEX", "LJ", "H25B"};
  static const char* const words[] = {"GULFSTREAM", "CITATION", "LEARJET", "FALCON", "GLOBAL", "CHALLENGER", "EMBRAER PHENOM"};
  return startsWithAny(target.type, types, sizeof(types) / sizeof(types[0])) ||
         containsAnyIgnoreCase(target.model, words, sizeof(words) / sizeof(words[0]));
}

bool isRegionalJetLike(const Target& target) {
  static const char* const types[] = {"CRJ", "E13", "E14", "E17", "E19", "E29", "F70", "F100"};
  static const char* const words[] = {"REGIONAL JET", "ERJ", "EMBRAER 1", "EMBRAER 19", "EMBRAER 17", "CRJ", "FOKKER"};
  return startsWithAny(target.type, types, sizeof(types) / sizeof(types[0])) ||
         containsAnyIgnoreCase(target.model, words, sizeof(words) / sizeof(words[0]));
}

bool isNarrowbodyJetLike(const Target& target) {
  static const char* const types[] = {"A318", "A319", "A320", "A321", "B37", "B38", "B39", "B3X", "B73"};
  static const char* const words[] = {"A320", "A321", "737", "NARROW"};
  return startsWithAny(target.type, types, sizeof(types) / sizeof(types[0])) ||
         containsAnyIgnoreCase(target.model, words, sizeof(words) / sizeof(words[0]));
}

bool isHeavyJetLike(const Target& target) {
  static const char* const types[] = {"B74", "B77", "B78", "A33", "A34", "A35", "A38", "MD11", "DC10"};
  static const char* const words[] = {"WIDEBODY", "747", "767", "777", "787", "A330", "A340", "A350", "A380", "MD-11", "DC-10"};
  return startsWithAny(target.type, types, sizeof(types) / sizeof(types[0])) ||
         containsAnyIgnoreCase(target.model, words, sizeof(words) / sizeof(words[0]));
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

bool isBalloonLike(const Target& target) {
  static const char* const typePrefixes[] = {"BALL", "BAL", "LTA", "SHIP"};
  static const char* const modelWords[] = {"BALLOON", "AIRSHIP", "LIGHTER"};
  return startsWithAny(target.type, typePrefixes, sizeof(typePrefixes) / sizeof(typePrefixes[0])) ||
         containsAny(target.model, modelWords, sizeof(modelWords) / sizeof(modelWords[0])) ||
         containsAny(target.readableType, modelWords, sizeof(modelWords) / sizeof(modelWords[0])) ||
         strcmp(target.category, "B2") == 0;
}

bool isUltralightLike(const Target& target) {
  static const char* const typePrefixes[] = {"ULAC", "ULTR", "PARA", "GYRO"};
  static const char* const words[] = {"ULTRALIGHT", "MICROLIGHT", "HANG", "PARAGLIDER"};
  return startsWithAny(target.type, typePrefixes, sizeof(typePrefixes) / sizeof(typePrefixes[0])) ||
         containsAny(target.model, words, sizeof(words) / sizeof(words[0])) ||
         containsAny(target.readableType, words, sizeof(words) / sizeof(words[0])) ||
         strcmp(target.category, "B4") == 0;
}

bool isSkydiverLike(const Target& target) {
  static const char* const words[] = {"SKYDIVER", "PARACHUTE", "PARACHUTIST"};
  return containsAny(target.model, words, sizeof(words) / sizeof(words[0])) ||
         containsAny(target.readableType, words, sizeof(words) / sizeof(words[0])) ||
         strcmp(target.category, "B3") == 0;
}

AircraftArt aircraftArtFor(const Target& target) {
  if (isSkydiverLike(target)) return ART_SKYDIVER;
  if (isBalloonLike(target)) return ART_BALLOON;
  if (isUltralightLike(target)) return ART_ULTRALIGHT;
  if (isHelicopterLike(target)) return ART_HELICOPTER;
  if (isMilitaryLike(target)) return ART_MILITARY;
  if (isGliderLike(target)) return ART_GLIDER;
  if (isCargoLike(target)) return ART_CARGO;
  if (isTwinPropLike(target)) return ART_TWIN_PROP;
  if (isBusinessJetLike(target)) return ART_BUSINESS_JET;
  if (isRegionalJetLike(target)) return ART_REGIONAL_JET;
  if (isLightPropLike(target)) return ART_LIGHT_PROP;
  if (isHeavyJetLike(target)) return ART_HEAVY_JET;
  if (isNarrowbodyJetLike(target)) return ART_NARROWBODY_JET;
  return ART_UNKNOWN;
}

const char* categoryAircraftType(const Target& target) {
  if (strcmp(target.category, "A1") == 0) return "Light acft";
  if (strcmp(target.category, "A2") == 0) return "Small acft";
  if (strcmp(target.category, "A3") == 0) return "Large acft";
  if (strcmp(target.category, "A4") == 0) return "Large acft";
  if (strcmp(target.category, "A5") == 0) return "Heavy acft";
  if (strcmp(target.category, "A6") == 0) return "High perf";
  return nullptr;
}

const char* broadAircraftType(const Target& target) {
  AircraftArt art = aircraftArtFor(target);
  switch (art) {
    case ART_HELICOPTER: return "Helicopter";
    case ART_LIGHT_PROP: return "Light prop";
    case ART_TWIN_PROP: return "Twin prop";
    case ART_BUSINESS_JET: return "Biz jet";
    case ART_REGIONAL_JET: return "Reg jet";
    case ART_NARROWBODY_JET: return "Narrowbody";
    case ART_AIRLINER: return "Airliner";
    case ART_HEAVY_JET: return "Widebody";
    case ART_CARGO: return "Cargo";
    case ART_MILITARY: return "Military";
    case ART_GLIDER: return "Glider";
    case ART_BALLOON: return "Balloon";
    case ART_ULTRALIGHT: return "Ultralight";
    case ART_SKYDIVER: return "Skydiver";
    default: break;
  }

  const char* categoryType = categoryAircraftType(target);
  if (categoryType) return categoryType;
  return "Aircraft";
}

const uint8_t* modelBitmapForAircraft(const Target& target) {
  static const char* const a320Types[] = {"A318", "A319", "A320", "A321"};
  static const char* const a330Types[] = {"A332", "A333", "A337", "A338", "A339"};
  static const char* const a340Types[] = {"A342", "A343", "A345", "A346"};
  static const char* const a380Types[] = {"A388"};
  static const char* const b737Types[] = {"B37M", "B38M", "B39M", "B3XM", "B732", "B733", "B734", "B735", "B736", "B737", "B738", "B739"};
  static const char* const b747Types[] = {"B741", "B742", "B743", "B744", "B748", "BLCF"};
  static const char* const b767Types[] = {"B762", "B763", "B764"};
  static const char* const b777Types[] = {"B772", "B773", "B77L", "B77W"};
  static const char* const b787Types[] = {"B788", "B789", "B78X"};
  static const char* const c130Types[] = {"C130", "C30J"};
  static const char* const crjTypes[] = {"CRJ1", "CRJ2", "CRJ7", "CRJ9", "CRJX"};
  static const char* const dh8Types[] = {"DH8A", "DH8B", "DH8C", "DH8D"};
  static const char* const e195Types[] = {"E190", "E195", "E290", "E295"};
  static const char* const erjTypes[] = {"E135", "E145", "E170", "E175"};
  static const char* const f100Types[] = {"F100", "F70"};
  static const char* const fa7xTypes[] = {"FA7X", "FA8X"};
  static const char* const glf5Types[] = {"GLF4", "GLF5", "GLF6", "GLEX"};
  static const char* const learTypes[] = {"LJ31", "LJ35", "LJ40", "LJ45", "LJ55", "LJ60", "LJ70", "LJ75"};
  static const char* const md11Types[] = {"MD11"};

  if (equalsAny(target.type, a320Types, sizeof(a320Types) / sizeof(a320Types[0]))) return AIRCRAFT_A320_BMP;
  if (equalsAny(target.type, a330Types, sizeof(a330Types) / sizeof(a330Types[0]))) return AIRCRAFT_A330_BMP;
  if (equalsAny(target.type, a340Types, sizeof(a340Types) / sizeof(a340Types[0]))) return AIRCRAFT_A340_BMP;
  if (equalsAny(target.type, a380Types, sizeof(a380Types) / sizeof(a380Types[0]))) return AIRCRAFT_A380_BMP;
  if (equalsAny(target.type, b737Types, sizeof(b737Types) / sizeof(b737Types[0]))) return AIRCRAFT_B737_BMP;
  if (equalsAny(target.type, b747Types, sizeof(b747Types) / sizeof(b747Types[0]))) return AIRCRAFT_B747_BMP;
  if (equalsAny(target.type, b767Types, sizeof(b767Types) / sizeof(b767Types[0]))) return AIRCRAFT_B767_BMP;
  if (equalsAny(target.type, b777Types, sizeof(b777Types) / sizeof(b777Types[0]))) return AIRCRAFT_B777_BMP;
  if (equalsAny(target.type, b787Types, sizeof(b787Types) / sizeof(b787Types[0]))) return AIRCRAFT_B787_BMP;
  if (equalsAny(target.type, c130Types, sizeof(c130Types) / sizeof(c130Types[0]))) return AIRCRAFT_C130_BMP;
  if (equalsAny(target.type, crjTypes, sizeof(crjTypes) / sizeof(crjTypes[0]))) return AIRCRAFT_CRJ_BMP;
  if (equalsAny(target.type, dh8Types, sizeof(dh8Types) / sizeof(dh8Types[0]))) return AIRCRAFT_DH8_BMP;
  if (equalsAny(target.type, e195Types, sizeof(e195Types) / sizeof(e195Types[0]))) return AIRCRAFT_E195_BMP;
  if (equalsAny(target.type, erjTypes, sizeof(erjTypes) / sizeof(erjTypes[0]))) return AIRCRAFT_ERJ_BMP;
  if (equalsAny(target.type, f100Types, sizeof(f100Types) / sizeof(f100Types[0]))) return AIRCRAFT_F100_BMP;
  if (equalsAny(target.type, fa7xTypes, sizeof(fa7xTypes) / sizeof(fa7xTypes[0]))) return AIRCRAFT_FA7X_BMP;
  if (equalsAny(target.type, glf5Types, sizeof(glf5Types) / sizeof(glf5Types[0]))) return AIRCRAFT_GLF5_BMP;
  if (equalsAny(target.type, learTypes, sizeof(learTypes) / sizeof(learTypes[0]))) return AIRCRAFT_LEARJET_BMP;
  if (equalsAny(target.type, md11Types, sizeof(md11Types) / sizeof(md11Types[0]))) return AIRCRAFT_MD11_BMP;
  return nullptr;
}

const uint8_t* categoryBitmapForAircraft(const Target& target) {
  if (strcmp(target.category, "A1") == 0) return AIRCRAFT_CAT_A1_BMP;
  if (strcmp(target.category, "A2") == 0) return AIRCRAFT_CAT_A2_BMP;
  if (strcmp(target.category, "A3") == 0) return AIRCRAFT_CAT_A3_BMP;
  if (strcmp(target.category, "A4") == 0) return AIRCRAFT_CAT_A4_BMP;
  if (strcmp(target.category, "A5") == 0) return AIRCRAFT_CAT_A5_BMP;
  if (strcmp(target.category, "B3") == 0) return AIRCRAFT_CAT_B3_BMP;
  return nullptr;
}

const uint8_t* bitmapForAircraft(const Target& target) {
  const uint8_t* modelBitmap = modelBitmapForAircraft(target);
  if (modelBitmap) return modelBitmap;
  const uint8_t* categoryBitmap = categoryBitmapForAircraft(target);
  if (categoryBitmap) return categoryBitmap;

  switch (aircraftArtFor(target)) {
    case ART_HELICOPTER: return AIRCRAFT_HELICOPTER_BMP;
    case ART_LIGHT_PROP: return AIRCRAFT_LIGHT_PROP_BMP;
    case ART_TWIN_PROP: return AIRCRAFT_TWIN_PROP_BMP;
    case ART_BUSINESS_JET: return AIRCRAFT_BUSINESS_JET_BMP;
    case ART_REGIONAL_JET: return AIRCRAFT_ERJ_BMP;
    case ART_NARROWBODY_JET: return AIRCRAFT_AIRLINER_BMP;
    case ART_AIRLINER: return AIRCRAFT_AIRLINER_BMP;
    case ART_HEAVY_JET: return AIRCRAFT_HEAVY_JET_BMP;
    case ART_CARGO: return AIRCRAFT_CARGO_BMP;
    case ART_MILITARY: return AIRCRAFT_MILITARY_BMP;
    case ART_GLIDER: return AIRCRAFT_GLIDER_BMP;
    case ART_BALLOON: return AIRCRAFT_BALLOON_BMP;
    case ART_ULTRALIGHT: return AIRCRAFT_ULTRALIGHT_BMP;
    case ART_SKYDIVER: return AIRCRAFT_CAT_B3_BMP;
    default: return AIRCRAFT_UNKNOWN_BMP;
  }
}

void drawAircraftArt(const Target& target, int y) {
  display.drawBitmap(4, y, bitmapForAircraft(target), AIRCRAFT_BMP_W, AIRCRAFT_BMP_H, SSD1306_WHITE);
}

void drawCenteredText(const char* text, int y) {
  int16_t x1 = 0;
  int16_t y1 = 0;
  uint16_t w = 0;
  uint16_t h = 0;
  display.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  int x = max(0, (display.width() - (int)w) / 2);
  display.setCursor(x, y);
  display.print(text);
}

void drawScrollingAircraft(const Target& target, int index, int count, uint32_t phaseMs) {
  int screenW = display.width();
  int screenH = display.height();
  int travel = screenH + AIRCRAFT_BMP_H + 8;
  int x = (screenW - AIRCRAFT_BMP_W) / 2;
  int y = screenH + 4 - (int)((uint64_t)phaseMs * travel / DETAIL_ART_SCROLL_MS);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setFont(NULL);
  char pageText[8];
  snprintf(pageText, sizeof(pageText), "%02d/%02d", index + 1, count);
  drawCenteredText(pageText, 0);
  display.setCursor(0, 12);
  display.print(target.callsign[0] ? target.callsign : "Unknown");
  display.drawBitmap(x, y, bitmapForAircraft(target), AIRCRAFT_BMP_W, AIRCRAFT_BMP_H, SSD1306_WHITE);
  display.setFont(&TomThumb);
  display.setCursor(0, screenH - 8);
  display.print(target.model[0] ? target.model : broadAircraftType(target));
  display.setFont(NULL);
  display.display();
}

void drawDetailWaiting() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setFont(NULL);
  drawCenteredText("DETAIL", 4);
  display.drawFastHLine(6, 18, display.width() - 12, SSD1306_WHITE);
  drawCenteredText(fetchInProgress ? "FETCHING" : "NO DATA", 34);
  drawCenteredText("AIRCRAFT", 50);
  display.setFont(&TomThumb);
  drawCenteredText(fetchInProgress ? "updating..." : "waiting...", 74);
  display.setFont(NULL);
  display.display();
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
  if (count > 0) {
    if (selectedTarget >= count) selectedTarget = 0;
    target = targets[selectedTarget];
  }
  xSemaphoreGive(targetMutex);

  if (count == 0) {
    drawDetailWaiting();
    return;
  }

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

  uint32_t cycleMs = DETAIL_ART_SCROLL_MS + DETAIL_TEXT_HOLD_MS;
  uint32_t phaseMs = (millis() - detailScrollStartMs) % cycleMs;
  if (phaseMs < DETAIL_ART_SCROLL_MS) {
    drawScrollingAircraft(target, selectedTarget, count, phaseMs);
    return;
  }

  display.setFont(&TomThumb);
  int rowY = 17;
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

void drawSplashScreen() {
  uint32_t started = millis();
  int frame = 0;

  while (millis() - started < SPLASH_MS) {
    int progress = min(52, (int)((millis() - started) * 52UL / SPLASH_MS));
    int planeX = -30 + (int)((millis() - started) * 92UL / SPLASH_MS);
    int planeY = 47 - (int)(sin(frame * 0.22f) * 4.0f);
    bool fading = millis() - started > SPLASH_MS - 650;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setFont(NULL);

    if (!fading || (frame & 1) == 0) {
      display.setTextSize(2);
      drawCenteredText("AIR", 6);
      drawCenteredText("RADAR", 24);
      display.setTextSize(1);
    }

    display.drawLine(2, 55, 20, 43, SSD1306_WHITE);
    display.drawLine(62, 55, 44, 43, SSD1306_WHITE);
    display.drawLine(6, 63, 26, 48, SSD1306_WHITE);
    display.drawLine(58, 63, 38, 48, SSD1306_WHITE);

    if (!fading || (frame & 2) == 0) {
      if (planeX < display.width() && planeX + 18 > 0) {
        int bodyX = max(0, planeX);
        int bodyW = min(18, display.width() - bodyX);
        display.drawFastHLine(bodyX, planeY, bodyW, SSD1306_WHITE);
      }
      display.fillTriangle(planeX + 18, planeY - 2, planeX + 28, planeY + 2, planeX + 18, planeY + 6, SSD1306_WHITE);
      display.fillTriangle(planeX + 8, planeY, planeX - 4, planeY - 8, planeX + 12, planeY, SSD1306_WHITE);
      display.fillTriangle(planeX + 8, planeY + 5, planeX - 4, planeY + 13, planeX + 12, planeY + 5, SSD1306_WHITE);
      display.drawFastVLine(planeX + 1, planeY, 6, SSD1306_WHITE);
    }

    display.setFont(&TomThumb);
    drawCenteredText("NEARBY FLIGHTS", 76);
    display.setFont(NULL);
    display.drawRect(5, 91, 54, 8, SSD1306_WHITE);
    display.fillRect(6, 92, progress, 6, SSD1306_WHITE);
    display.setFont(&TomThumb);
    drawCenteredText("PRG cycle", 108);
    drawCenteredText("HOLD radar", 120);
    display.setFont(NULL);

    display.display();
    delay(90);
    frame++;
  }

  for (int pass = 0; pass < 4; pass++) {
    for (int y = pass; y < display.height(); y += 4) {
      display.drawFastHLine(0, y, display.width(), SSD1306_BLACK);
    }
    display.display();
    delay(85);
  }
}

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(CPU_MHZ);
  btStop();
  pinMode(PROGRAM_BUTTON, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("oled failed");
    for (;;) delay(1000);
  }
  display.setRotation(OLED_ROTATION);
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(OLED_CONTRAST);
  display.clearDisplay();
  display.display();
  drawSplashScreen();

  targetMutex = xSemaphoreCreateMutex();
  cacheMutex = xSemaphoreCreateMutex();
  connectWiFi();
  xTaskCreatePinnedToCore(fetchTask, "fetch", 12288, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(enrichTask, "enrich", 12288, nullptr, 1, nullptr, 0);
}

void loop() {
  uint32_t frameDelayMs = ACTIVE_FRAME_MS;
  String event = readButtonEvent();
  if (event == "short" && targetCount > 0) {
    if (!detailMode) selectedTarget = 0;
    else selectedTarget = (selectedTarget + 1) % targetCount;
    detailMode = true;
    detailScrollStartMs = millis();
  } else if (event == "long") {
    detailMode = false;
  }

  if (detailMode) {
    drawDetail();
    int count = 0;
    xSemaphoreTake(targetMutex, portMAX_DELAY);
    count = targetCount;
    xSemaphoreGive(targetMutex);
    if (count == 0) {
      frameDelayMs = STATIC_FRAME_MS;
    } else {
      uint32_t cycleMs = DETAIL_ART_SCROLL_MS + DETAIL_TEXT_HOLD_MS;
      uint32_t phaseMs = (millis() - detailScrollStartMs) % cycleMs;
      if (phaseMs >= DETAIL_ART_SCROLL_MS) frameDelayMs = STATIC_FRAME_MS;
    }
  } else {
    drawRadar();
  }

  delay(frameDelayMs);
}
