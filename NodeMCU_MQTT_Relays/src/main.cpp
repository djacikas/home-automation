// Made by DJacikas
// v1.0

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <MQTT.h>
#include <Bounce2.h>
#include <EEPROM.h>

void StoreStateToEEPROM(int channel);

const char ssid[] = "MikroTik-A29991";
const char pass[] = "karsticeburekai";

WiFiClient net;
MQTTClient client;

#define NODE_ID 1

#define RELAY_ON 1
#define RELAY_OFF 0
#define CHANNELS 3

Bounce * buttons = new Bounce[CHANNELS];
int BUTTON_PINS[CHANNELS] = {5, 4, 14};
int relayPins[CHANNELS]  = {12, 13, 15};
bool states[CHANNELS];

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect("arduino", "public", "public")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  client.subscribe("/lights-in");
  // client.unsubscribe("/hello");
}

void messageReceived(String &topic, String &payload) {
  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.

  int nodeId = payload.substring(11, 12).toInt();
  int relay = payload.substring(21, 22).toInt();
  int state = payload.substring(31, 32).toInt();

  Serial.println("payload: " + String(payload));
  Serial.println("nodeId: " + String(nodeId));
  Serial.println("relay: " + String(relay));
  Serial.println("state: " + String(state));

  if(nodeId == NODE_ID && state != states[relay]?RELAY_ON:RELAY_OFF){
    buttons[relay].update();
    states[relay] = !states[relay];
    digitalWrite(relayPins[relay], states[relay]);

    StoreStateToEEPROM(relay);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  EEPROM.begin(512);  //Initialize EEPROM

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported
  // by Arduino. You need to set the IP address directly.
  client.begin("192.168.88.5", net);
  client.onMessage(messageReceived);

  connect();

  for (int i = 0; i < CHANNELS; i++) {
    buttons[i].attach( BUTTON_PINS[i] , INPUT_PULLUP  );       //setup the bounce instance for the current button
    buttons[i].interval(25);              // interval in ms

    // Then set relay pins in output mode
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], RELAY_OFF);

    //TODO read states from eeprom
    states[i] = EEPROM.read(i);
    digitalWrite(relayPins[i], states[i]?RELAY_ON:RELAY_OFF);
  }
}

void loop() {
  client.loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!client.connected()) {
    connect();
  }

  for (int i = 0; i < CHANNELS; i++)  {
    // Update the Bounce instance :
    buttons[i].update();
    // If it fell, flag the need to toggle the relay
    if ( buttons[i].fell() ) {
      states[i] = !states[i];
      digitalWrite(relayPins[i], states[i]);
      client.publish("/lights-out", "{\"node_id\":" + String(NODE_ID) + ",\"relay\":" + String(i) + ",\"state\":"+ String(states[i]) + "}");

      StoreStateToEEPROM(i);
    }
  }
}

void StoreStateToEEPROM(int channel){
    EEPROM.write(channel, states[channel]);
    EEPROM.commit();    //Store data to EEPROM
}