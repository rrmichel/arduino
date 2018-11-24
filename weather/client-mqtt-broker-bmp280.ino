#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <PubSubClient.h>


const char* ssid = "...";
const char* password = "...";
const char* espName = "home1";

// Endpoint settings
const char* mqtt_server = "10.14.10.115";
const char* mqtt_user = "user";
const char* mqtt_password = "pass";
const char* mqtt_clientId = "home1";

// Available Topics
const char* topic_altitude = "losant/5bf8715184a2990008742e15/altitude";
const char* topic_temperature = "losant/5bf8715184a2990008742e15/state";
const char* topic_pressure = "losant/5bf8715184a2990008742e15/pressure";

// Minutes to sleep between updates
int minutes2sleep = 1;

ESP8266WiFiMulti WiFiMulti;
WiFiClient wifiClient;
PubSubClient client(wifiClient);

#define SERIAL_BAUD 115200
#define FORCE_DEEPSLEEP
//#define DEBUG
#define RETAINED false

//BME280I2C bme;
Adafruit_BMP280 bme;

/**
 * Le Setup
 */
void setup() {
  Serial.begin(SERIAL_BAUD);
  //while(!Serial) {} // Wait
  Wire.begin();

  splashScreen();
  delay(1000); //reprogramming

  Serial.println("---");
  Serial.println("Searching for sensor:");
  Serial.print("Result: ");

  while(!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }
/*
  switch(bme.chipModel()) {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }
*/
  startWIFI();
}


/**
 * Looping Louie
 */
void loop() {
  runMQTT();
  sendSensorData();

  delay(500); //wait
  goToBed(minutes2sleep); //sending into deep sleep
}


/**
 * Building http-POST-request and send all necessary data              
 */
void sendSensorData () {
  float temp(NAN), alt(NAN), pres(NAN);

  temp = bme.readTemperature();
  alt = bme.readAltitude(1013.25);
  pres = bme.readPressure();
  
  // client.publish(topic_temperature, String(temp).c_str(), RETAINED);
  // client.publish(topic_altitude, String(alt).c_str(), RETAINED);
  // client.publish(topic_pressure, String(pres).c_str(), RETAINED);

  StaticJsonBuffer<300> JSONbuffer;
  JsonObject& JSONencoder = JSONbuffer.createObject();
  JsonObject& data = JSONencoder.createNestedObject("data");

  data["temperature"] = temp;
  data["pressure"] = pres / 100;
  data["altitude"] = alt;
  
  char JSONmessageBuffer[100];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);
 
  if (client.publish(topic_temperature, JSONmessageBuffer, RETAINED) == true) {
    Serial.println("Success sending message");
  } else {
    Serial.println("Error sending message");
  }

  #ifdef DEBUG
    Serial.print("temp: ");
    Serial.println(String(temp).c_str());
    Serial.print("alt: ");
    Serial.println(String(alt).c_str());
    Serial.print("pres: ");
    Serial.println(String(pres).c_str());
  #endif
}


/**
 * Establish WiFi-Connection
 * 
 * If connection times out (threshold 50 sec) 
 * device will sleep for 5 minutes and will restart afterwards.
 */
void startWIFI() {
  Serial.println("---");
  WiFi.mode(WIFI_STA);
  Serial.println("(Re)Connecting to Wifi-Network with following credentials:");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Key: ");
  Serial.println(password);
  Serial.print("Device-Name: ");
  Serial.println(espName);
  
  WiFi.hostname(espName);
  WiFiMulti.addAP(ssid, password);

  int tryCnt = 0;
  
  while (WiFiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    tryCnt++;

    if (tryCnt > 100) {
      Serial.println("");
      Serial.println("Could not connect to WiFi. Sending device to bed.");
      goToBed(5);
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  delay(300);
}


/**
 * Establish MQTT-Connection
 * 
 * If connection fails, device will sleep for 5 minutes and will restart afterwards.
 */
void runMQTT() {
  Serial.println("---");
  Serial.println("Starting MQTT-Client with following credentials:");
  Serial.print("Host: ");
  Serial.println(mqtt_server);
  Serial.print("User: ");
  Serial.println(mqtt_user);
  Serial.print("Password: ");
  Serial.println(mqtt_password);
  Serial.print("ClientId: ");
  Serial.println(mqtt_clientId);

  client.setServer(mqtt_server, 1883);
  
  while (!client.connected()) {
    Serial.print("Attempting connection... ");
    // Attempt to connect
    if (client.connect(mqtt_clientId, mqtt_user, mqtt_password)) {
      Serial.println("Success.");
      client.loop();
    } else {
      Serial.println("Failed.");
      Serial.println("Could not connect to MQTT-Server. Sending device to bed.");
      goToBed(5);
    }
  }
}


/**
 * Sending device into deep sleep
 */
void goToBed (int minutes) {
  #ifdef FORCE_DEEPSLEEP
    Serial.print("Uaaah. I'm tired. Going back to bed for ");
    Serial.print(minutes);
    Serial.println(" minutes. Good night!");
    Serial.println("---");
    ESP.deepSleep(minutes * 60 * 1000000);
    delay(100);
  #endif
}


/**
 * Dump some information on startup.
 */
void splashScreen() {
  for (int i=0; i<=5; i++) Serial.println();
  Serial.println("#######################################");
  Serial.print("# DeviceName: ");
  Serial.println(espName);
  Serial.print("# Configured Endpoint: ");
  Serial.println(mqtt_server);
  Serial.println("#######################################");
  for (int i=0; i<2; i++) Serial.println();
}
