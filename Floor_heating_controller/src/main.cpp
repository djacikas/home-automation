// Floor heating controller made to send data through MQTT
// Made by djacikas
// v1.0
//
// Dallas sensors:
// 0 - Manifold supply water temp
// 1 - Manifold return water temp
// 2 - Living room supply water temp
// 3 - Living room return water temp
// 4 - Bedroom supply water temp
// 5 - Bedroom return water temp
// 6 - Room supply water temp
// 7 - Room return water temp
// 8 - Bathroom supply water temp
// 9 - Bathroom return water temp
//
//

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <MQTT.h>
#include <Bounce2.h>
#include <OneWire.h>
#include <DallasTemperature.h>

void connect();
void messageReceived(String &topic, String &payload);
void setRelay(String topic, String payload, int index);

const char ssid[] = "ssid";
const char pass[] = "pass";

WiFiClient net;
MQTTClient client;

const char mqttBrokerAddress[13] = "192.168.88.5";
String mqttTopicFloorHeatingController = "/myhome/devices/floorheatingcontroller";

#define RELAY_ON 1
#define RELAY_OFF 0
#define CHANNELS 4

Bounce * relays = new Bounce[CHANNELS];
int relayPins[CHANNELS]  = {4, 14, 12, 13}; //D2, D5, D6, D7
bool relaysStates[CHANNELS];

int livingRoomRelayIndex = 0;
int bedroomRelayIndex = 1;
int childRoomRelayIndex = 2;
int bathroomRelayIndex = 3;

unsigned long lastRelayRequest = 0;
int delayInMillisForRelays = 300000;

unsigned long lastTempRequest = 0;
int delayInMillisForDallas = 60000;

#define ONE_WIRE_BUS 5 //D1 GPIO5
#define TEMPERATURE_PRECISION 12 // 9, 10, 11, 12

int numberOfDallasSensors; 

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DeviceAddress dallasManifoldSupply = { 0x28, 0xF6, 0x3C, 0x75, 0xD0, 0x01, 0x3C, 0x92 };
DeviceAddress dallasManifoldReturn = { 0x28, 0x6A, 0xF4, 0x75, 0xD0, 0x01, 0x3C, 0x42 };

DeviceAddress dallasLivingRoomSupply = { 0x28, 0x58, 0x04, 0x75, 0xD0, 0x01, 0x3C, 0x67 };
DeviceAddress dallasLivingRoomReturn = { 0x28, 0xDC, 0x8C, 0x75, 0xD0, 0x01, 0x3C, 0xA4 };

DeviceAddress dallasBedroomSupply = { 0x28, 0xE9, 0x49, 0x75, 0xD0, 0x01, 0x3C, 0x52 };
DeviceAddress dallasBedroomReturn = { 0x28, 0x03, 0xE2, 0x75, 0xD0, 0x01, 0x3C, 0xF7 };

DeviceAddress dallasChildRoomSupply = { 0x28, 0x5F, 0x46, 0x75, 0xD0, 0x01, 0x3C, 0x88 };
DeviceAddress dallasChildRoomReturn = { 0x28, 0x1F, 0x04, 0x75, 0xD0, 0x01, 0x3C, 0x97 };

DeviceAddress dallasBathroomSupply = { 0x28, 0x83, 0x03, 0x75, 0xD0, 0x01, 0x3C, 0x0A };
DeviceAddress dallasBathroomReturn = { 0x28, 0xFF, 0x70, 0x94, 0x71, 0x16, 0x04, 0xA9 };

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  // EEPROM.begin(512);  //Initialize EEPROM
  sensors.begin();

  client.begin(mqttBrokerAddress, net);
  client.onMessage(messageReceived);

  connect();

  numberOfDallasSensors = sensors.getDeviceCount();

  Serial.print("Found ");
  Serial.print(numberOfDallasSensors, DEC);
  Serial.println(" devices.");

  sensors.setResolution(TEMPERATURE_PRECISION);

  for (int i = 0; i < CHANNELS; i++) {
    relays[i].attach( relayPins[i] , INPUT_PULLUP  );       //setup the bounce instance for the current button
    relays[i].interval(25);              // interval in ms

    // Then set relay pins in output mode
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], RELAY_OFF);
  }
}

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect("floorheatingcontroller", "public", "public")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  client.subscribe("/myhome/livingroom/heating");
  client.subscribe("/myhome/bedroom/heating");
  client.subscribe("/myhome/childroom/heating");
  client.subscribe("/myhome/bathroom/heating");
}

void messageReceived(String &topic, String &payload) {
  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.

  if (topic == "/myhome/livingroom/heating")
  {
    setRelay(topic, payload, livingRoomRelayIndex);
  }
  else if (topic == "/myhome/bedroom/heating")
  {
    setRelay(topic, payload, bedroomRelayIndex);
  }
  else if (topic == "/myhome/childroom/heating")
  {
    setRelay(topic, payload, childRoomRelayIndex);
  }
  else if (topic == "/myhome/bathroom/heating")
  {
    setRelay(topic, payload, bathroomRelayIndex);
  }
  else {
    Serial.println("Topic " + String(topic) + " wasn't described. Payload: " + String(payload));
  }
}

void setRelay(String topic, String payload, int index){
  digitalWrite(relayPins[index], payload == "on" ? RELAY_ON : RELAY_OFF);
  Serial.println(String(topic) + ": " + String(payload));
  lastRelayRequest = millis();
}

void loop()
{ 
  client.loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!client.connected()) {
    connect();
  }

  if (millis() - lastRelayRequest >= delayInMillisForRelays) // waited long enough??
  {
    
    Serial.println("millis: " + String(millis()));
    Serial.println("lastRelayRequest: " + String(lastRelayRequest));

    for (int i = 0; i < CHANNELS; i++)
    {
      digitalWrite(relayPins[i], RELAY_OFF);

      switch (i)
      {
      case 0:
        client.publish(mqttTopicFloorHeatingController + "/myhome/livingroom/heating", "off");
        break;
      case 1:
        client.publish(mqttTopicFloorHeatingController + "/myhome/bedroom/heating", "off");
        break;
      case 2:
        client.publish(mqttTopicFloorHeatingController + "/myhome/childroom/heating", "off");
        break;
      case 3:
        client.publish(mqttTopicFloorHeatingController + "/myhome/bathroom/heating", "off");
        break;

      default:
        break;
      }
    }

    lastRelayRequest = millis();
  }

  if (millis() - lastTempRequest >= delayInMillisForDallas) // waited long enough??
  {
    sensors.requestTemperatures();
    
    float manifoldTempSupply = sensors.getTempC(dallasManifoldSupply);
    float manifoldTempReturn = sensors.getTempC(dallasManifoldReturn);
    client.publish(mqttTopicFloorHeatingController + "/manifold/temperature/supply", String(manifoldTempSupply));
    client.publish(mqttTopicFloorHeatingController + "/manifold/temperature/return", String(manifoldTempReturn));

    float livingRoomTempSupply = sensors.getTempC(dallasLivingRoomSupply);
    float livingRoomTempReturn = sensors.getTempC(dallasLivingRoomReturn);
    client.publish(mqttTopicFloorHeatingController + "/livingroom/temperature/supply", String(livingRoomTempSupply));
    client.publish(mqttTopicFloorHeatingController + "/livingroom/temperature/return", String(livingRoomTempReturn));
    
    float bedroomTempSupply = sensors.getTempC(dallasBedroomSupply);
    float bedroomTempReturn = sensors.getTempC(dallasBedroomReturn);
    client.publish(mqttTopicFloorHeatingController + "/bedroom/temperature/supply", String(bedroomTempSupply));
    client.publish(mqttTopicFloorHeatingController + "/bedroom/temperature/return", String(bedroomTempReturn));
    
    float childRoomTempSupply = sensors.getTempC(dallasChildRoomSupply);
    float childRoomTempReturn = sensors.getTempC(dallasChildRoomReturn);
    client.publish(mqttTopicFloorHeatingController + "/childroom/temperature/supply", String(childRoomTempSupply));
    client.publish(mqttTopicFloorHeatingController + "/childroom/temperature/return", String(childRoomTempReturn));
    
    float bathroomTempSupply = sensors.getTempC(dallasBathroomSupply);
    float bathroomTempReturn = sensors.getTempC(dallasBathroomReturn);
    client.publish(mqttTopicFloorHeatingController + "/bathroom/temperature/supply", String(bathroomTempSupply));
    client.publish(mqttTopicFloorHeatingController + "/bathroom/temperature/return", String(bathroomTempReturn));
    
    lastTempRequest = millis();
  }
}