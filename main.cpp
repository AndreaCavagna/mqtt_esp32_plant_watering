
#include <WiFi.h>
#include <ESP32Ping.h>
#include <AsyncMqttClient.h>
#include "time.h"

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

#define WIFI_SSID "WIFI NAME"
#define WIFI_PASSWORD "WIFI PASSWORD"

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 123, 16) // ADD YOUR MQTT BROKER HERE 
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

// Temperature MQTT Topics
#define MQTT_PUB_MOISTURE_VALUE "esp32/bedroom_plant/moistureValue"
#define MQTT_PUB_MOISTURE_PERCENT  "esp32/bedroom_plant/moisturePercent"
#define MQTT_PUB_WIFI_LINK "esp32/bedroom_plant/wifi_rssi"
#define MQTT_PUB_PING_TIME "esp32/bedroom_plant/ping_time"
#define MQTT_PUB_MOISTURE_OFFSCALE_VALUE "esp32/bedroom_plant/moistureOffscaleValue"


#define MQTT_PUB_STATUS_WATERING "esp32/bedroom_plant/status/activateWatering"
#define MQTT_PUB_STATUS_WATERING_TIME "esp32/bedroom_plant/status/wateringTime"
#define MQTT_PUB_LAST_WATERED "esp32/bedroom_plant/status/lastWatered"
#define MQTT_PUB_STATUS_SOIL_MOISTURE_MIN_VALUE "esp32/bedroom_plant/status/minMoisture"
#define MQTT_PUB_STATUS_AIR_VOLTAGE_VALUE "esp32/bedroom_plant/status/AIR_VOLTAGE_VALUE"
#define MQTT_PUB_STATUS_WATER_VOLTAGE_VALUE "esp32/bedroom_plant/status/WATER_VOLTAGE_VALUE"

char buf[26];
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

// Variables to hold sensor readings
float temp;
float hum;
float dew_point;
int wifi_link;
float ping_time;

int AIR_VOLTAGE_VALUE = 2100;
int WATER_VOLTAGE_VALUE = 1150;
int SensorPin = 34;
int LEDPin = 14;
int WATER_PUMP_PIN = 26;
int LEDPinGND = 12;
int soilMoistureValue = 0;
int soilmoisturepercent = 0;
int soilMoistureOffscaleValue = 0;
int SOIL_MOISTURE_MIN_VALUE = 30;
int WATERING_TIME_SEC = 20;

bool wateringTriggered = false;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings


char* get_local_time()
{
  struct tm local_time;
  if (!getLocalTime(&local_time)) {
    Serial.println("non riesco a connettermi al server");
    char empty[2];
    empty[0] = '0';
    empty[1] = '/n';
    return empty;
  }

  strftime(buf, 26, "%A, %H:%M", &local_time);
  //strftime(buf, 26, "%Y-%m-%d %H:%M:%S", &local_time);
  puts(buf);
  Serial.println("printging buffer");
  Serial.println(buf);
  return buf;
}

int getWifiStrength(int points) {
  long rssi = 0;
  long averageRSSI = 0;

  for (int i = 0; i < points; i++) {
    rssi += WiFi.RSSI();
    delay(50);
  }

  averageRSSI = rssi / points;
  return averageRSSI;
}


void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  mqttClient.subscribe("esp32/bedroom_plant/set/#", 0);

}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  //Serial.println("Publish received.");
  //Serial.print("  topic: ");
  //Serial.println(topic);
  //Serial.print("  Payload: ");
  //Serial.println(payload[0]);
  //Serial.print("  qos: ");
  //Serial.println(properties.qos);
  //Serial.print("  dup: ");
  //Serial.println(properties.dup);
  //Serial.print("  retain: ");
  //Serial.println(properties.retain);
  //Serial.print("  len: ");
  //Serial.println(len);
  //Serial.print("  index: ");
  //Serial.println(index);
  //Serial.print("  total: ");
  //Serial.println(total);

  if ( strcmp(topic, "esp32/bedroom_plant/set/activateWatering") == 0)
  {
    if ( payload[0] == 't') {
      wateringTriggered = true;
      
      mqttClient.publish(MQTT_PUB_STATUS_WATERING, 1, true, "true");
    } else {
      wateringTriggered = false;
      digitalWrite(WATER_PUMP_PIN, LOW);
      mqttClient.publish(MQTT_PUB_STATUS_WATERING, 1, true, "false");
    }
  }

  if ( strcmp(topic, "esp32/bedroom_plant/set/wateringTime") == 0)
  {
    WATERING_TIME_SEC = atoi(payload);
    char WATERING_TIME_SEC_trunc[10];
    sprintf(WATERING_TIME_SEC_trunc, "%d", WATERING_TIME_SEC);
    mqttClient.publish(MQTT_PUB_STATUS_WATERING_TIME, 1, true, WATERING_TIME_SEC_trunc);
  }

  if ( strcmp(topic, "esp32/bedroom_plant/set/minMoisture") == 0)
  {
    SOIL_MOISTURE_MIN_VALUE = atoi(payload);
    char SOIL_MOISTURE_MIN_VALUE_trunc[10];
    sprintf(SOIL_MOISTURE_MIN_VALUE_trunc, "%d", SOIL_MOISTURE_MIN_VALUE);
    mqttClient.publish(MQTT_PUB_STATUS_SOIL_MOISTURE_MIN_VALUE, 1, true, SOIL_MOISTURE_MIN_VALUE_trunc);
  }

  if ( strcmp(topic, "esp32/bedroom_plant/set/AIR_VOLTAGE_VALUE") == 0)
  {
    AIR_VOLTAGE_VALUE = atoi(payload);
    char AIR_VOLTAGE_VALUE_trunc[10];
    sprintf(AIR_VOLTAGE_VALUE_trunc, "%d", AIR_VOLTAGE_VALUE);
    mqttClient.publish(MQTT_PUB_STATUS_AIR_VOLTAGE_VALUE, 1, true, AIR_VOLTAGE_VALUE_trunc);
  }

  if ( strcmp(topic, "esp32/bedroom_plant/set/WATER_VOLTAGE_VALUE") == 0)
  {
    WATER_VOLTAGE_VALUE = atoi(payload);
    char WATER_VOLTAGE_VALUE_trunc[10];
    sprintf(WATER_VOLTAGE_VALUE_trunc, "%d", WATER_VOLTAGE_VALUE);
    mqttClient.publish(MQTT_PUB_STATUS_WATER_VOLTAGE_VALUE, 1, true, WATER_VOLTAGE_VALUE_trunc);
  }

}

/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
  }
  void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  }*/

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void initializeMQTTStatuses() {

  mqttClient.publish(MQTT_PUB_STATUS_WATERING, 1, true, "false");
  
  delay(250);
  buf[0] = 'N';
  buf[1] = 'e';
  buf[2] = 'v';
  buf[3] = 'e';
  buf[4] = 'r';
  buf[5] = '\n';

  
  mqttClient.publish(MQTT_PUB_LAST_WATERED, 1, true, buf );
  mqttClient.publish("esp32/bedroom_plant/set/activateWatering", 1, true, "false" );

  char WATERING_TIME_SEC_trunc[10];
  sprintf(WATERING_TIME_SEC_trunc, "%d", WATERING_TIME_SEC);
  mqttClient.publish(MQTT_PUB_STATUS_WATERING_TIME, 1, true, WATERING_TIME_SEC_trunc);

  char SOIL_MOISTURE_MIN_VALUE_trunc[10];
  sprintf(SOIL_MOISTURE_MIN_VALUE_trunc, "%d", SOIL_MOISTURE_MIN_VALUE);
  mqttClient.publish(MQTT_PUB_STATUS_SOIL_MOISTURE_MIN_VALUE, 1, true, SOIL_MOISTURE_MIN_VALUE_trunc);

  char AIR_VOLTAGE_VALUE_trunc[10];
  sprintf(AIR_VOLTAGE_VALUE_trunc, "%d", AIR_VOLTAGE_VALUE);
  mqttClient.publish(MQTT_PUB_STATUS_AIR_VOLTAGE_VALUE, 1, true, AIR_VOLTAGE_VALUE_trunc);

  char WATER_VOLTAGE_VALUE_trunc[10];
  sprintf(WATER_VOLTAGE_VALUE_trunc, "%d", WATER_VOLTAGE_VALUE);
  mqttClient.publish(MQTT_PUB_STATUS_WATER_VOLTAGE_VALUE, 1, true, WATER_VOLTAGE_VALUE_trunc);

  wateringTriggered = false;
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode(LEDPinGND, OUTPUT);
  pinMode(LEDPin, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(WATER_PUMP_PIN, OUTPUT);

  digitalWrite(LEDPinGND, LOW);
  digitalWrite(LEDPin, LOW);
  digitalWrite(27, LOW);
  digitalWrite(WATER_PUMP_PIN, LOW);

  Serial.println("System has started");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  //mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");
  connectToWifi();
  delay(5000);
  initializeMQTTStatuses();

}

void loop() {

  float total_value = 0;
  int repetitions = 60;
  int off_scale_values = 0;
  soilMoistureOffscaleValue = 0;

  // cosi homeassistant non ha il valore vuoto
  mqttClient.publish(MQTT_PUB_STATUS_WATERING, 1, true, "false");
  mqttClient.publish(MQTT_PUB_LAST_WATERED, 1, true, buf);

  for (int i = 0; i < repetitions ; i++) {
    soilMoistureValue = analogRead(SensorPin);
    
    if ( soilMoistureValue > AIR_VOLTAGE_VALUE + 500 ||  soilMoistureValue < WATER_VOLTAGE_VALUE - 2000 ) {
      off_scale_values += 1;
      soilMoistureOffscaleValue += soilMoistureValue;
    } else if ( soilMoistureValue < WATER_VOLTAGE_VALUE) {
      total_value +=  WATER_VOLTAGE_VALUE;
    } else if (soilMoistureValue > AIR_VOLTAGE_VALUE) {
      total_value +=  AIR_VOLTAGE_VALUE;
    } else {
      total_value += soilMoistureValue;
    };
    Serial.println("Moisture value");
    Serial.println(soilMoistureValue);

    if (wateringTriggered == true) {
      Serial.println("PUMPING");
      mqttClient.publish(MQTT_PUB_STATUS_WATERING, 1, true, "true");
      digitalWrite(WATER_PUMP_PIN, HIGH);
      delay(WATERING_TIME_SEC * 1000);
      wateringTriggered = false;
      mqttClient.publish(MQTT_PUB_STATUS_WATERING, 1, true, "false");
      digitalWrite(WATER_PUMP_PIN, LOW);
      get_local_time();
      mqttClient.publish(MQTT_PUB_LAST_WATERED, 1, true, buf);
    };
    delay(5000);

  }

  if ( repetitions - off_scale_values == 0 ) {
    soilMoistureValue = -1;
    soilmoisturepercent = -1;
    for (int i = 0; i < 500; i++) {
      digitalWrite(LEDPin, LOW);
      delay(500);
      digitalWrite(LEDPin, HIGH);
    };
  } else {
    soilMoistureValue = total_value / (repetitions - off_scale_values );
    soilmoisturepercent = map(soilMoistureValue, AIR_VOLTAGE_VALUE, WATER_VOLTAGE_VALUE, 0, 100);
  }

  if (soilmoisturepercent < SOIL_MOISTURE_MIN_VALUE) {
    digitalWrite(LEDPin, HIGH);
  } else {
    digitalWrite(LEDPin, LOW);
  }


  if (Ping.ping("192.168.123.1")) {
    ping_time = Ping.averageTime();
  } else {
    delay(60000);
    ESP.restart();
  };

  int random_restart = random(100);

  if (random_restart <= 1){
    ESP.restart();
  }

  wifi_link = getWifiStrength(10);

  char soilMoistureValue_trunc[10];
  sprintf(soilMoistureValue_trunc, "%d", soilMoistureValue);

  char soilMoistureOffscaleValue_trunc[10];
  sprintf(soilMoistureOffscaleValue_trunc, "%d", soilMoistureOffscaleValue);

  char soilmoisturepercent_trunc[10];
  sprintf(soilmoisturepercent_trunc, "%d", soilmoisturepercent);

  char wifi_link_trunc[10];
  sprintf(wifi_link_trunc, "%d", wifi_link);

  char ping_time_trunc[10];
  sprintf(ping_time_trunc, "%.1f", ping_time);

  // Publish an MQTT message on topic esp32/dht/temperature
  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_MOISTURE_VALUE, 1, true, soilMoistureValue_trunc);
  //Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_MOISTURE_VALUE, packetIdPub1);
  //Serial.printf("Message: %.2f \n", soilMoistureValue_trunc);
  delay(100);


  // Publish an MQTT message on topic esp32/dht/humidity
  // uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(hum).c_str());
  uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_MOISTURE_PERCENT, 1, true, soilmoisturepercent_trunc);
  //Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_MOISTURE_PERCENT, packetIdPub2);
  //Serial.printf("Message: %.2f \n", soilmoisturepercent_trunc);
  delay(100);

  uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_MOISTURE_OFFSCALE_VALUE, 1, true, soilMoistureOffscaleValue_trunc);
  //Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_MOISTURE_PERCENT, packetIdPub2);
  //Serial.printf("Message: %.2f \n", soilmoisturepercent_trunc);
  delay(100);


  uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_WIFI_LINK, 1, true, wifi_link_trunc);
  //Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_WIFI_LINK, packetIdPub4);
  //Serial.printf("Message: %.2f \n", wifi_link);
  delay(100);


  uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_PING_TIME, 1, true, ping_time_trunc);
  //Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_PING_TIME, packetIdPub5);
  //Serial.printf("Message: %.2f \n", ping_time);
}