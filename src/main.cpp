#include "Sht40Class.h"
#include "MqttClass.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <WiFi.h>

using namespace std;

/*
Load WiFi SSID & Pass and MQTT IP Address:
  const char* ssid = "WiFi_SSID";
  const char* pass = "WiFi_Pass";
  const char* mqttServer = "192.168.0.10";
*/
#include "networkCredentials.h"

#define DEBUG_MODE
#define DEBUG_MODE_MQTT

#define BAUDRATE 115200

#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_ADDRESS_SHT40 0x45

#define RELAY_PIN 19

unsigned long mqttLastEvent;
unsigned long mqttInterval = 60000; //ms
unsigned long reconnectInterval = 5000; //ms

int relay;

/*
0 = Living Room
1 = Work Room
2 = Bed Room
3 = Bath Room
4 = Outside
*/
/*
const char* roomNames[] = {
  "Obyvak",
  "Pracovna",
  "Loznice",
  "Koupelna",
  "Venkovni"
};
const int roomsCount = 5;
*/
const int defaultRoom = 4;  // LIVING ROOM

const char* mqttPublishTopics[] = {
  "/home/temp/therm_LVR_in",
  "/home/temp/therm_WKR_in",
  "/home/temp/therm_BDR_in",
  "/home/temp/therm_BHR_in",
  "/home/temp/therm_OUT_in",
  "/home/temp/heating_mode_in",
  "/home/electro/switchboard_in"
};
const char* mqttSubscribeTopicsRooms[] = {
  "/home/temp/therm_LVR_out",
  "/home/temp/therm_WKR_out",
  "/home/temp/therm_BDR_out",
  "/home/temp/therm_BHR_out",
  "/home/temp/therm_OUT_out"
};
const char* mqttSubscribeTopicsOthers[] = {
  "/home/temp/heating_mode_out",
  "/home/electro/switchboard_out",
  "/home/time"
};

const char* mqttHumidityKeys[] = {
  "humidity_LVR",
  "humidity_WKR",
  "humidity_BDR",
  "humidity_BHR",
  "humidity_OUT"
};
const char* mqttTempActKeys[] = {
  "temp_act_LVR",
  "temp_act_WKR",
  "temp_act_BDR",
  "temp_act_BHR",
  "temp_act_OUT"
};
const char* mqttTempSetKeys[] = {
  "temp_set_LVR",
  "temp_set_WKR",
  "temp_set_BDR",
  "temp_set_BHR",
  "temp_set_OUT"
};
const char* mqttRelayKeys[] = {
  "relay_LVR",
  "relay_WKR",
  "relay_BDR",
  "relay_BHR",
  "relay_OUT"
};
const char* mqttOtherKeys[] = {
  "heating_enabled",
  "signal_RC",
  "hours",
  "minutes"
};

Sht40Class sht40(I2C_ADDRESS_SHT40);

void MqttCallback(char* topic, byte* message, unsigned long length);
MqttClass mqtt(mqttServer, MqttCallback);

void setup()
{
  delay(100);
  
  #ifdef DEBUG_MODE
  Serial.begin(BAUDRATE);
  while(!Serial);
  Serial.println();
  Serial.println("Thermostat LVR");
  Serial.println();
  #endif

  //--------------------------------------------------------------------------
  // WiFi Init
  #ifdef DEBUG_MODE
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  #endif

  WiFi.begin(ssid, pass);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    #ifdef DEBUG_MODE
    Serial.print(".");
    #endif
  }

  #ifdef DEBUG_MODE
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  #endif

  int subscribeCount1, subscribeCount2, publishCount;
  for(const char* topic : mqttSubscribeTopicsRooms)
    subscribeCount1++;

  for(const char* topic : mqttSubscribeTopicsOthers)
    subscribeCount2++;

  for(const char* topic : mqttPublishTopics)
    publishCount++;
  
  mqtt.setSubscribeTopics(
    mqttSubscribeTopicsRooms, subscribeCount1,
    mqttSubscribeTopicsOthers, subscribeCount2,
    false
  );
  mqtt.subscribe();
  
  mqtt.setPublishTopics(mqttPublishTopics, publishCount, defaultRoom);

  //--------------------------------------------------------------------------
  // Sensor SHT40
  #ifdef DEBUG_MODE
  Serial.println("SHT40 test");
  #endif
  if(!sht40.init())
  {
    #ifdef DEBUG_MODE
    Serial.println("Couldn't find SHT40");
    #endif
    delay(60000);
    ESP.restart();
  }
  #ifdef DEBUG_MODE
  Serial.println("Found SHT40 sensor");
  Serial.print("Serial number 0x");
  Serial.println(sht40.getSerial(), HEX);
  #endif

  delay(500);

  sht40.getData();
  #ifdef DEBUG_MODE
  Serial.println(String(sht40.getHumidity()) + "%rH");
  Serial.println(String(sht40.getTemperature()) + "°C");
  #endif

  //--------------------------------------------------------------------------
  // Relay
  pinMode(RELAY_PIN, OUTPUT);

  mqttLastEvent = millis() - mqttInterval + 2000; //2 seconds to call mqtt publish

  delay(500);
}

void loop()
{
  mqtt.loop();

  unsigned long now = millis();
  if(now - mqttLastEvent > mqttInterval)
  {
    mqttLastEvent = now;

    sht40.getData();

    #ifdef DEBUG_MODE
    Serial.println(String(sht40.getHumidity()) + "%rH");
    Serial.println(String(sht40.getTemperature()) + "°C");
    #endif  
    
    if(mqtt.getSynced())
    {
      mqtt.publish(
        mqttPublishTopics[defaultRoom],
        mqttTempActKeys[defaultRoom], sht40.getTemperature(),
        mqttHumidityKeys[defaultRoom], sht40.getHumidity(),
        mqttRelayKeys[defaultRoom], relay
      );
    }
    else
    {
      mqtt.publish(
        mqttPublishTopics[defaultRoom],
        mqttTempActKeys[defaultRoom], sht40.getTemperature(),
        mqttHumidityKeys[defaultRoom], sht40.getHumidity()
      );
    }
  }
}

/*----------------------------------------------------------------------------
  MQTT CallBack Function
 -----------------------------------------------------------------------------*/
void MqttCallback(char* topic, byte* message, unsigned long length)
{
  #ifdef DEBUG_MODE_MQTT
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  #endif

  String json;

  for(int i = 0; i < length; i++)
  {
    json += (char)message[i];
    #ifdef DEBUG_MODE_MQTT
    Serial.print((char)message[i]);
    #endif
  }
  #ifdef DEBUG_MODE_MQTT
  Serial.println();
  #endif

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json);

  if(error)
  {
    #ifdef DEBUG_MODE_MQTT
    Serial.println();
    Serial.println("Json deserialization failed:");
    Serial.println(error.c_str());
    #endif
    return;
  }

  if(strcmp(topic, mqttSubscribeTopicsRooms[defaultRoom]) == 0)
  {
    if(doc[mqttRelayKeys[defaultRoom]].is<int>())
    {
      int relayValue = doc[mqttRelayKeys[defaultRoom]];
      if(relay != relayValue)
      {
        relay = relayValue;
        digitalWrite(RELAY_PIN, relay);
      }

      if(!mqtt.getSynced())
      {
        mqtt.setSynced();
      }
      #ifdef DEBUG_MODE_MQTT
      String str = String(mqttRelayKeys[defaultRoom]) + " = " + relay;
      Serial.println(str);
      Serial.println(mqtt.getSynced());
      #endif
    }
  }
}