

// VERSION 5 --> DHT20 RAIN ALERT

// ===================== SCOOTER SECURITY SYSTEM MAIN ===================== //
// Theft detection (IMU + BLE) + AWS MQTT notification + DHT20 humidity alert
// Fully working version — NO BLE scanning unless movement detected
// ======================================================================== //

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include <SparkFunLSM6DSO.h>
#include <NimBLEDevice.h>
#include "certs.h"       // your AWS CA, CERT, PRIVATE KEY
#include <Adafruit_AHTX0.h>   // DHT20 / AHT20 humidity & temp sensor

// ===================== WIFI CONFIG ===================== //
const char* WIFI_SSID     = "Scootracker";
const char* WIFI_PASSWORD = "@1234Cats";

// ===================== AWS IOT CONFIG ===================== //
const char* AWS_IOT_ENDPOINT  = "a3d2bbsbu015hf-ats.iot.us-east-2.amazonaws.com";
const int   AWS_IOT_PORT      = 8883;
const char* AWS_IOT_CLIENT_ID = "Scooter01Client";

const char* AWS_IOT_TOPIC     = "scooter/alerts/unauthorized";  // theft
const char* WEATHER_TOPIC     = "scooter/weather/rain_alert";   // humidity event
const char* DEVICE_ID         = "scooter01";

WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);

// ===================== IMU ===================== //
LSM6DSO myIMU;
const float MOVE_THRESHOLD = 0.50f;   // sensitivity
const int   MOTION_SAMPLES = 5;       // smooth noise

float g_lastAx_g=0,g_lastAy_g=0,g_lastAz_g=0,g_lastAvgDev=0;
int   g_lastRssi = -200;

#define LSM6DSO_ADDR 0x6B
#define CTRL1_XL     0x10
#define OUTX_L_A     0x28

bool readAccelG(float* ax_g, float* ay_g, float* az_g) {
  Wire.beginTransmission(LSM6DSO_ADDR);
  Wire.write(OUTX_L_A);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(LSM6DSO_ADDR, 6) != 6) return false;

  int16_t x = Wire.read() | (Wire.read() << 8);
  int16_t y = Wire.read() | (Wire.read() << 8);
  int16_t z = Wire.read() | (Wire.read() << 8);

  const float SENS = 0.000061f;  // g/LSB
  *ax_g = x * SENS;
  *ay_g = y * SENS;
  *az_g = z * SENS;
  return true;
}

// ===================== WEATHER ===================== //
Adafruit_AHTX0 aht;
bool  g_weatherOk=false;
float g_tempC=0,g_humidity=0;

// ===================== BLE SECURITY ===================== //
const char* TARGET_NAME    = "ScooterOwnerPhone";
const int   BLE_SCAN_TIME  = 3;
const int   RSSI_THRESHOLD = -75;

// ===================== OUTPUT HARDWARE ===================== //
const int LED_PIN=12, BUZZER_PIN=27;
const int BUZZER_CHANNEL=0;
const int BUZZER_FREQ=2048;
const int ALARM_BURST_MS=2000;

// =================================================================== //
//                          WIFI + AWS                                 //
// =================================================================== //
void connectWiFi(){
  Serial.print("WiFi → Connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  while(WiFi.status()!=WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println("\nWiFi ✓  " + WiFi.localIP().toString());
}

void setupAWS(){
  secureClient.setCACert(AWS_CERT_CA);
  secureClient.setCertificate(AWS_CERT_CRT);
  secureClient.setPrivateKey(AWS_CERT_PRIVATE);
  mqttClient.setServer(AWS_IOT_ENDPOINT,AWS_IOT_PORT);
}

void connectAWS(){
  if(WiFi.status()!=WL_CONNECTED) connectWiFi();

  Serial.print("AWS → Connecting...");
  while(!mqttClient.connected()){
    if(mqttClient.connect(AWS_IOT_CLIENT_ID)){
      Serial.println(" Connected ✓");
      return;
    }
    Serial.print(".");
    delay(2000);
  }
}

// =================================================================== //
//                        AWS MQTT PUBLISHERS                          //
// =================================================================== //
void publishUnauthorizedAlert(){
  if(!mqttClient.connected()) connectAWS();

  String msg="{\"deviceId\":\""+String(DEVICE_ID)+"\","
             "\"event\":\"unauthorized_movement\","
             "\"ax\":"+String(g_lastAx_g,3)+","
             "\"ay\":"+String(g_lastAy_g,3)+","
             "\"az\":"+String(g_lastAz_g,3)+","
             "\"deviation\":"+String(g_lastAvgDev,3)+","
             "\"rssi\":"+String(g_lastRssi)+"}";

  mqttClient.publish(AWS_IOT_TOPIC,msg.c_str());
  Serial.println("📡 Theft Alert Published → AWS");
}

void publishRainAlert() {
  if (!mqttClient.connected()) connectAWS();
  mqttClient.loop();

  String msg = "{\"deviceId\":\"" + String(DEVICE_ID) + "\","
               "\"event\":\"rain_detected\","
               "\"tempC\":" + String(g_tempC, 1) + ","
               "\"humidity\":" + String(g_humidity, 1) + "}";

  Serial.print("Rain topic: ");
  Serial.println(WEATHER_TOPIC);
  Serial.print("Rain payload: ");
  Serial.println(msg);

  bool ok = mqttClient.publish(WEATHER_TOPIC, msg.c_str());
  if (ok)
  {
    Serial.println("☁ Rain Alert Published → AWS (OK)");
    delay(500);
  } 
  else    Serial.println("☁ Rain Alert FAILED to publish");
}

// =================================================================== //
//                          MOTION (WORKING VERSION)                   //
// =================================================================== //

bool isScooterMoving() {
  const float G   = 1.0f;   // using g units
  const float MAG_MIN_GOOD = 0.2f;   // sanity check for magnitude

  float devSum = 0.0f;
  float lastX = 0, lastY = 0, lastZ = 0;

  for (int i = 0; i < MOTION_SAMPLES; i++) {
    float ax_g, ay_g, az_g;
    if (!readAccelG(&ax_g, &ay_g, &az_g)) return false;

    lastX = ax_g;
    lastY = ay_g;
    lastZ = az_g;

    float mag_g = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    float dev_g = fabs(mag_g - G);
    devSum += dev_g;

    delay(5);
  }

  g_lastAx_g   = lastX;
  g_lastAy_g   = lastY;
  g_lastAz_g   = lastZ;
  g_lastAvgDev = devSum / MOTION_SAMPLES;

  float magLast = sqrt(g_lastAx_g * g_lastAx_g +
                       g_lastAy_g * g_lastAy_g +
                       g_lastAz_g * g_lastAz_g);

  Serial.print("Accel (g): X=");
  Serial.print(g_lastAx_g, 3);
  Serial.print(" Y=");
  Serial.print(g_lastAy_g, 3);
  Serial.print(" Z=");
  Serial.print(g_lastAz_g, 3);
  Serial.print(" | AvgDev(g)=");
  Serial.print(g_lastAvgDev, 4);
  Serial.print(" | Mag(g)=");
  Serial.println(magLast, 3);

  if (magLast < MAG_MIN_GOOD) {
    Serial.println("IMU reading invalid → treat as no motion");
    return false;
  }

  const float MOVE_THRESHOLD_G = 0.10f;  // tweak as needed
  bool moving = g_lastAvgDev > MOVE_THRESHOLD_G;

  if (moving) 
    Serial.println("Movement detected (above threshold).");
  return moving;
}
// =================================================================== //
//                         BLE AUTHENTICATION                          //
// =================================================================== //
bool isOwnerNearbyBLE(int* outRssi){
  if(outRssi)*outRssi=-200;

  NimBLEScan* scan=NimBLEDevice::getScan();
  scan->setActiveScan(true);
  NimBLEScanResults r=scan->start(BLE_SCAN_TIME,false);

  for(int i=0;i<r.getCount();i++){
    auto dev=r.getDevice(i);
    if(!dev.haveName())continue;

    int rssi=dev.getRSSI();
    if(String(dev.getName().c_str())==TARGET_NAME){
      if(outRssi)*outRssi=rssi;
      return(rssi>RSSI_THRESHOLD);  // true if close
    }
  }
  return false;
}

// =================================================================== //
//                                ALARM                                 //
// =================================================================== //
void runAlarmBurst() {
  ledcWriteTone(BUZZER_CHANNEL, BUZZER_FREQ);
  for (int i = 0; i < 20; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
    mqttClient.loop();
  }
  ledcWriteTone(BUZZER_CHANNEL, 0);
  digitalWrite(LED_PIN, LOW);
}

// =================================================================== //
//                                SETUP                                //
// =================================================================== //


void setup(){
  Serial.begin(115200);

  pinMode(LED_PIN,OUTPUT);
  pinMode(BUZZER_PIN,OUTPUT);
  ledcSetup(BUZZER_CHANNEL,BUZZER_FREQ,8);
  ledcAttachPin(BUZZER_PIN,BUZZER_CHANNEL);

  Wire.begin(21, 22);
  Wire.beginTransmission(LSM6DSO_ADDR);
  Wire.write(CTRL1_XL);
  Wire.write(0b01000000); // 104 Hz, ±2 g
  Wire.endTransmission();
  connectWiFi(); setupAWS(); connectAWS();
  myIMU.begin();
  aht.begin(); g_weatherOk=true;

  NimBLEDevice::init("");
  Serial.println("=== System Ready ===");
}

// =================================================================== //
//                                LOOP                                 //
// =================================================================== //
void loop() {
  if (!mqttClient.connected()) connectAWS();
  mqttClient.loop();

  // --------- THEFT SYSTEM (ONLY activates on motion) --------- //
  if (isScooterMoving()) {
    Serial.println("Checking for owner phone via BLE...");

    int rssi = -200;
    bool ownerNearby = isOwnerNearbyBLE(&rssi);
    g_lastRssi = rssi;

    if (ownerNearby) {
      Serial.print("Owner nearby (RSSI=");
      Serial.print(g_lastRssi);
      Serial.println(" dBm) → no alarm.");
      ledcWriteTone(BUZZER_CHANNEL, 0);
      digitalWrite(LED_PIN, LOW);
    } else {
      Serial.print("Owner NOT found (RSSI=");
      Serial.print(g_lastRssi);
      Serial.println(" dBm) → trigger alarm + alert.");

      Serial.println("🚨 UNAUTHORIZED MOVEMENT! Triggering alarm and publishing alert...");
      publishUnauthorizedAlert();

      // keep alarming until owner returns
      while (true) {
        int rBack = -200;
        Serial.println("Alarm active. Scanning for owner phone...");
        if (isOwnerNearbyBLE(&rBack)) {
          g_lastRssi = rBack;
          Serial.print("Owner returned (RSSI=");
          Serial.print(g_lastRssi);
          Serial.println(" dBm) → alarm off ✓");
          ledcWriteTone(BUZZER_CHANNEL, 0);
          digitalWrite(LED_PIN, LOW);
          break;
        }
        runAlarmBurst();
      }
    }
    delay(600);  // cooldown after a movement event
  } else {
    // idle theft path – ensure alarm is OFF
    ledcWriteTone(BUZZER_CHANNEL, 0);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  // --------- WEATHER (periodic & independent) --------- //
  static unsigned long lastWeather = 0;
  if (millis() - lastWeather > 30000) {  // 30s test interval
    lastWeather = millis();
    sensors_event_t h, t;
    if (aht.getEvent(&h, &t)) {
      g_tempC    = t.temperature;
      g_humidity = h.relative_humidity;
      Serial.print("Weather → T=");
      Serial.print(g_tempC);
      Serial.print(" C  H=");
      Serial.print(g_humidity);
      Serial.println(" %");

      if (g_humidity > 55) {   // low for testing; raise later (e.g., >85)
        publishRainAlert();
      }
    }
  }
}


// VERSION 5
