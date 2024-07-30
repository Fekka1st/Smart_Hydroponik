#include <Arduino.h>
#include <EEPROM.h>
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include "GravityTDS.h"

// Define Firebase dan WiFi
#define WIFI_SSID "" // nama wifi
#define WIFI_PASSWORD "" // nama password wifi
#define API_KEY "" // token api firebase
#define DATABASE_URL "" // url api firebase

// Define PIN-PIN
#define PH_SENSOR_PIN 34
#define TDS_SENSOR_PIN 35
#define DHT_PIN 19
#define DHT_TYPE DHT22
#define PUMP_MAIN_PIN 17

#define PUMP_NUTRIENT_A_PIN 16
#define PUMP_NUTRIENT_B_PIN 15
#define PUMP_PH_UP_PIN 2
#define PUMP_PH_DOWN_PIN 4
#define EEPROM_SIZE 512

// Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

DHT dht(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
GravityTDS gravityTds;
unsigned long sendDataPrevMillis = 0;
bool signupOK = false;
unsigned long dataInterval = 5000; // Interval pengiriman data dalam milidetik ini contoh 5 detik
unsigned long lastDataSendTime = 0;
unsigned long pumpPrevMillis = 0;
unsigned long pumpPHPrevMillis = 0;
unsigned long pumpNutrientPrevMillis = 0;
const unsigned long pumpInterval = 1000; // 1 detik jeda pompa, dibuat karena tekanan sedot pompa tinggi sesuaikan dengan kondisi air atau tangki  rekomendasi 1 detik - 10 detik

bool showSensorData = true; 

// Firebase paths
String settingsPath = "smarthydroponic/sistem/settings";
String sensorDataPath = "smarthydroponic/sistem/sensorData";
String pompaControlDataPath = "smarthydroponic/sistem/PompaControl";

// Variable Store Data
float pHValue, pHValuereal, temperatureData, humidity;
int TDSValue = 0;
float getPHMin, getPHMax, getTDSMin, getTDSMax;
bool phUp, phDown, nutrisiPompa, pompaUtama, getMode;
int samples = 10;
float adc_resolution = 4096;
int measurings=0;

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  pinMode(PUMP_NUTRIENT_A_PIN, OUTPUT);
  pinMode(PUMP_NUTRIENT_B_PIN, OUTPUT);
  pinMode(PUMP_PH_UP_PIN, OUTPUT);
  pinMode(PUMP_PH_DOWN_PIN, OUTPUT);
  pinMode(PUMP_MAIN_PIN, OUTPUT);

  gravityTds.setPin(TDS_SENSOR_PIN);
  gravityTds.setAref(3.3);
  gravityTds.setAdcRange(4096);
  
  digitalWrite(PUMP_MAIN_PIN, LOW);
  gravityTds.begin();
  dht.begin();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Hydroponic");
  lcd.setCursor(0, 1);
  lcd.print("Ferry Aditya H");
  delay(3000);
  lcd.clear();

  connectToWiFi();
  setupFirebase();
}

void connectToWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    lcd.setCursor(0, 0);
    lcd.print("Connecting WIFI");
    delay(300);
  }
  lcd.clear();
  Serial.println("\nConnected with IP: " + WiFi.localIP().toString());
}

void setupFirebase() {
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Connected to Firebase");
    lcd.setCursor(0, 0);
    lcd.print("Connected");
    lcd.setCursor(0, 1);
    lcd.print("Firebase");
    delay(2000);
    lcd.clear();
    signupOK = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

float ph (float voltage) {
    return 7 + ((2.5 - voltage) / 0.18);
}

void readSensors() {
  temperatureData = dht.readTemperature();
  humidity = dht.readHumidity();
  gravityTds.setTemperature(25);
  gravityTds.update();
  TDSValue = gravityTds.getTdsValue();
  pHValuereal = analogRead(PH_SENSOR_PIN);
  for (int i = 0; i < samples; i++)
    {
        measurings += analogRead(PH_SENSOR_PIN);
        delay(10);
    }
  pHValue = 6;
  float voltage = 3.3 / adc_resolution * measurings/samples;
  pHValuereal = ph(voltage);
  Serial.printf("Temp: %.2f, Hum: %.2f, TDS: %.2f, pH: %.2f\n", temperatureData, humidity, TDSValue, pHValuereal);
}



void sendSensorData() {
  Firebase.RTDB.setFloat(&fbdo, sensorDataPath + "/pHReal", pHValuereal);
  minimalisinoise();
}

void minimalisinoise(){
  if (pHValue >= 3 || pHValue <= 11 ){
  Firebase.RTDB.setFloat(&fbdo, sensorDataPath + "/pH", pHValue);
  }
  if (TDSValue >= 200 || TDSValue <= 1000 ) {
  Firebase.RTDB.setInt(&fbdo, sensorDataPath + "/TDS", TDSValue);
  }
  if (temperatureData >= 20 || temperatureData <= 50 ) {
   Firebase.RTDB.setFloat(&fbdo, sensorDataPath + "/Temp", temperatureData);
  }
   if (temperatureData >= 20 || temperatureData <= 100 ){
   Firebase.RTDB.setFloat(&fbdo, sensorDataPath + "/humidity", humidity);
  }
}

void getDataSettings() {
  Firebase.RTDB.getString(&fbdo, settingsPath + "/pHmax") ? getPHMax = fbdo.stringData().toFloat() : Serial.println(fbdo.errorReason());
  Firebase.RTDB.getString(&fbdo, settingsPath + "/pHmin") ? getPHMin = fbdo.stringData().toFloat() : Serial.println(fbdo.errorReason());
  Firebase.RTDB.getString(&fbdo, settingsPath + "/TDSmax") ? getTDSMax = fbdo.stringData().toFloat() : Serial.println(fbdo.errorReason());
  Firebase.RTDB.getString(&fbdo, settingsPath + "/TDSmin") ? getTDSMin = fbdo.stringData().toFloat() : Serial.println(fbdo.errorReason());
}

void controlPumpsAutomatically() {
  unsigned long currentMillis = millis();
  // Kontrol pompa pH
  if (pHValue >= 3 && pHValue <= 11) {
    if (currentMillis - pumpPHPrevMillis >= pumpInterval) {
      pumpPHPrevMillis = currentMillis;

      if (pHValue < getPHMin) {
        controlPump(PUMP_PH_UP_PIN, PUMP_PH_DOWN_PIN, true, false, pompaControlDataPath + "/PompaPHUP");
      } else if (pHValue > getPHMax) {
        controlPump(PUMP_PH_UP_PIN, PUMP_PH_DOWN_PIN, false, true, pompaControlDataPath + "/PompaPHDown");
      } else {
        controlPump(PUMP_PH_UP_PIN, PUMP_PH_DOWN_PIN, false, false, pompaControlDataPath + "/PompaPHUP");
        controlPump(PUMP_PH_UP_PIN, PUMP_PH_DOWN_PIN, false, false, pompaControlDataPath + "/PompaPHDown");
      }

      delay(1000); // Delay 1 detik sebelum mematikan pompa pH
      digitalWrite(PUMP_PH_UP_PIN, LOW);
      digitalWrite(PUMP_PH_DOWN_PIN, LOW);
    }
  }

  // Kontrol pompa nutrisi
  if (TDSValue >= 200 && TDSValue <= 1000) {
    if (currentMillis - pumpNutrientPrevMillis >= pumpInterval) {
      pumpNutrientPrevMillis = currentMillis;

      if (TDSValue < getTDSMin) {
        controlNutrientPumps(true, pompaControlDataPath + "/NutrisiMIX");
      } else if (TDSValue > getTDSMax) {
        controlNutrientPumps(false, pompaControlDataPath + "/NutrisiMIX");
      } else {
        Firebase.RTDB.setBool(&fbdo, pompaControlDataPath + "/NutrisiMIX", false);
      }

      delay(1000); // Delay 1 detik sebelum mematikan pompa nutrisi
      digitalWrite(PUMP_NUTRIENT_A_PIN, LOW);
      digitalWrite(PUMP_NUTRIENT_B_PIN, LOW);
    }
  }
}

void controlPump(uint8_t pumpPin1, uint8_t pumpPin2, bool state1, bool state2, const String& path) {
  digitalWrite(pumpPin1, state1 ? HIGH : LOW);
  digitalWrite(pumpPin2, state2 ? HIGH : LOW);
  delay(3000);
  digitalWrite(pumpPin1, LOW);
  digitalWrite(pumpPin2, LOW);
  Firebase.RTDB.setBool(&fbdo, path, state1 || state2);
}

void controlNutrientPumps(bool state, const String& path) {
  digitalWrite(PUMP_NUTRIENT_A_PIN, state ? HIGH : LOW);
  digitalWrite(PUMP_NUTRIENT_B_PIN, state ? HIGH : LOW);
  delay(3000);
  digitalWrite(PUMP_NUTRIENT_A_PIN, LOW);
  digitalWrite(PUMP_NUTRIENT_B_PIN, LOW);
  Firebase.RTDB.setBool(&fbdo, path, state);
}

void getPompaData() {
  Firebase.RTDB.getString(&fbdo, pompaControlDataPath + "/PompaPHUP") ? phUp = (fbdo.stringData() == "true") : 0;
  Firebase.RTDB.getString(&fbdo, pompaControlDataPath + "/PompaPHDown") ? phDown = (fbdo.stringData() == "true") : 0;
  Firebase.RTDB.getString(&fbdo, pompaControlDataPath + "/NutrisiMIX") ? nutrisiPompa = (fbdo.stringData() == "true") : 0;
}

void controlPumpsManually() {
  getPompaData();
  unsigned long currentMillis = millis();
  
  if (currentMillis - pumpPrevMillis >= pumpInterval) {
    pumpPrevMillis = currentMillis;

    digitalWrite(PUMP_NUTRIENT_A_PIN, nutrisiPompa ? HIGH : LOW);
    digitalWrite(PUMP_NUTRIENT_B_PIN, nutrisiPompa ? HIGH : LOW);
    digitalWrite(PUMP_PH_UP_PIN, phUp ? HIGH : LOW);
    digitalWrite(PUMP_PH_DOWN_PIN, phDown ? HIGH : LOW);
    
    delay(1000); // Delay 1 detik sebelum mematikan pompa
    digitalWrite(PUMP_NUTRIENT_A_PIN, LOW);
    digitalWrite(PUMP_NUTRIENT_B_PIN, LOW);
    digitalWrite(PUMP_PH_UP_PIN, LOW);
    digitalWrite(PUMP_PH_DOWN_PIN, LOW);
  }
}

void mainPompa() {
  Firebase.RTDB.getString(&fbdo, pompaControlDataPath + "/PompaMain") ? pompaUtama = (fbdo.stringData() == "true") : 0;
  digitalWrite(PUMP_MAIN_PIN, pompaUtama ? LOW : HIGH);
}

void loop() {
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 1500 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();
    readSensors();
    if (millis() - lastDataSendTime > dataInterval) {
      sendSensorData();
      lastDataSendTime = millis();
    }
    mainPompa();
    Firebase.RTDB.getString(&fbdo, settingsPath + "/mode") ? getMode = (fbdo.stringData() == "true") : 0;
    getMode ? controlPumpsAutomatically() : controlPumpsManually();
  }
  LCD();
}

void LCD() {
    lcd.setCursor(0, 0);
    lcd.print("pH:"); lcd.print(pHValuereal);
    lcd.setCursor(8, 0);
    lcd.print("TDS:"); lcd.print(TDSValue);
    lcd.setCursor(0, 1);
    lcd.print("Temp:"); lcd.print(temperatureData);
    lcd.setCursor(9, 1);
    lcd.print("hum:"); lcd.print(humidity);
    lcd.clear();
}
