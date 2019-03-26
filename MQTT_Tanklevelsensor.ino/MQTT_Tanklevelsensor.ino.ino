#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <NewPingESP8266.h>

#define FORCE_DEEPSLEEP
#define TIME_TO_SLEEP  1        //Time ESP32 will go to sleep (in Minutes), max 70
#define TRIGGER_PIN  D6         // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     D7         // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200        // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define DHTTYPE DHT22
#define DHTPIN 4

//Pin Declarations
const int gasPin = A0;                    // Luftqualitäts-Pin = A0

const char* host = "192.168.178.4";
const char* ssid = "Chickenpocks";
const char* password = "0s7v1e4n2g2a6";

//Variables
float t = 0.0;
float h = 0.0;
float hic = 0.0;
float dwp = 0.0;
float diff = 1.0;
float gas = 0.0;
float gasdiff = 10.0;
unsigned int batt;
double battV;
bool dhtValue = false;

DHT dht(DHTPIN, DHTTYPE);
NewPingESP8266 sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPingESP8266 setup of pins and maximum distance.

WiFiClient net;
PubSubClient mqtt(net);

void connect();


bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}

float calcDewpoint(float humi, float temp) {
  float k;
  k = log(humi / 100) + (17.62 * temp) / (243.12 + temp);
  return 243.12 * k / (17.62 - k);
}


void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Booting...");

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);

  mqtt.setServer(host, 1883);

  dht.begin();
  delay(2000);
  connect();

  Serial.println("Setup completed...");
}

void connect() {
  // Loop until we're reconnected
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqtt.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqtt.publish("Tank/state", "ready");
      // ... and resubscribe
      mqtt.subscribe("Tank/back");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void messageReceived(String topic, String payload, char * bytes, unsigned int length) {
  Serial.print("incoming: ");
  Serial.print(topic);
  Serial.print(" - ");
  Serial.print(payload);
  Serial.println();
}

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  temp = (int) (4 * temp + .5);
  return (double) temp / 4;
}

void climate() {
  Serial.print("Start reading Climate Values.");
  float nh = dht.readHumidity();
  float nt = dht.readTemperature();
  float nhic = dht.computeHeatIndex(nt, nh, false);
  float ndwp = calcDewpoint(nh, nt);
  float ngas = analogRead(gasPin);
  int dhtValue1 = 0;
  if (isnan(nh) || isnan(nt)) {
    Serial.println("Failed to read from DHT sensor!");
  }
  if (checkBound(nt, t, diff)) { // Temperature
    t = nt;
    if (t > 0.0) {
      dhtValue1 = dhtValue1 + 1;
      mqtt.publish("Tank/temp", String(t).c_str(), true);
    }
    Serial.print(".");
  }
  if (checkBound(nh, h, diff)) { // Humidity
    h = nh;  
    if (h > 0.0) {
      dhtValue1 = dhtValue1 + 1;
      mqtt.publish("Tank/humidity", String(h).c_str(), true);
    }
    Serial.print(".");
  }

  if (checkBound(nhic, hic, diff)) { // Heat index
    hic = nhic;
    mqtt.publish("Tank/heatindex", String(hic).c_str(), true);
    if (hic > 0.0) {
      dhtValue1 = dhtValue1 + 1;
    }
    Serial.print(".");
  }

  if (checkBound(ndwp, dwp, diff)) { // Dewpoint
    dwp = ndwp;
    if (dwp > 0.0) {
      dhtValue1 = dhtValue1 + 1;
      mqtt.publish("Tank/dewpoint", String(dwp).c_str(), true);
    }
    Serial.print(".");
  }

  if (checkBound(ngas, gas, gasdiff)) {
    gas = ngas;
    if (gas = 0.0) {
      dhtValue1 = dhtValue1 + 1;
      mqtt.publish("Tank/luftquali", String(gas).c_str(), true);
    }
    if (dhtValue1 > 4) {
      dhtValue = true;
    }
    else {
      dhtValue = false;
    }
    Serial.println(".");
  }
  Serial.println("Temperature: " + String(t) + "*C;" + " Humidity: " + String(h) + "%;" + " Heat index: " + String(hic) + "*C;" + " DewPoint: " + String(dwp) + "*C;" + " AirQuality: " + String(gas) + "pph");
  Serial.println("-------------------------------------");
}

void levelsensor() {
  int maxlevel = 180;                 // Variable to store the maximal Distance in centimeters from the bottom of the Tank to the maximal fillment
  int distanceToSensor = 4;          // Distance from MaxLevel to Sensor 
  int maxFillment = 1500;             // Varaible to store the max. Tank fillment in liters
  int DistanceSum = 0;
  Serial.print("Starting Fillment measuring:.");

  for (int i = 1; i <= 10; i++) {
    Serial.print(".");
    int Distance = 0;
    unsigned int uS = sonar.ping();   // Send ping, get ping time in microseconds (uS).
    Distance = sonar.convert_cm(uS);  // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
    DistanceSum = DistanceSum + Distance;
    delay(500);
  }
  Serial.println(".");
  int DistanceAverage = DistanceSum / 10;
  int Level = maxlevel - (DistanceAverage - distanceToSensor);           // Variable to calculate the height of the Level
  int liter = (maxFillment / maxlevel) * Level;     // Finally Value to store the effectiv fillment in liters to post over Mqtt
  Serial.println("Distance: " + String(DistanceAverage) + "cm; " + " Level: " + String(Level) + "cm; " + " liter: " + String(liter) + "liter");
  mqtt.publish("Tank/content", String(liter).c_str(), true);
  mqtt.publish("Tank/level", String(Level).c_str(), true);
  Serial.println("-------------------------------------");
}

void battState() {
  batt = analogRead(A0)/1.75;
  battV = mapDouble(batt, 0, 1023, 0.0, 6.6);
  mqtt.publish("Tank/batt", String(battV).c_str(), true);
  Serial.println("Batt Volt: " + String(battV));
  Serial.println("-------------------------------------");
}

void loop() {
  if (!mqtt.connected()) {
    connect();
  }
  mqtt.loop();
  Serial.println("Reprogramming Window...");
  delay(15000); // time to make reprogramming possible
  battState();
  climate();
  levelsensor();
  delay(3000); // secure that the mqtt-Messages from all Functions gets sent before going to deep sleep


#ifdef FORCE_DEEPSLEEP
  if (battV < 4.8) { // Handle longer sleep times if Battery is too low
    Serial.println("Force deepsleep for " + String (TIME_TO_SLEEP*2)+ " min!");
    String tts = "Force deepsleep for " + String (TIME_TO_SLEEP*2) + " min!";
    mqtt.publish("Tank/state", String(tts).c_str(), true);
    delay(2000);
    ESP.deepSleep((TIME_TO_SLEEP/2) * 1000000); //send IFTTT low_bat warning
  } 
  else if (battV < 4.0){
    Serial.println("Force deepsleep for " + String (TIME_TO_SLEEP) + " min!");
    String tts = "Force deepsleep for " + String (TIME_TO_SLEEP) + " min!";
    mqtt.publish("Tank/state", String(tts).c_str(), true);
    delay(2000);
    ESP.deepSleep(TIME_TO_SLEEP * 1000000);
  }
#endif
}
