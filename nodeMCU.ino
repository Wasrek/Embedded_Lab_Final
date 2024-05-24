#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = "";
const char* password = "";
const char* mqttServer = "mqtt.netpie.io";
const int mqttPort = 1883;
const char* mqttClientID = "";
const char* mqttUsername = "";
const char* mqttPassword = "";

#define RX_PIN D5
#define TX_PIN D6

SoftwareSerial mySerial(RX_PIN, TX_PIN);

WiFiClient espClient;
PubSubClient client(espClient);

String receivedData = "";
bool receiving = false;

void clearSerialBuffer() {
  while (mySerial.available()) {
    mySerial.read();
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqttClientID, mqttUsername, mqttPassword)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  reconnect();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  while (mySerial.available() || receiving) {
    char c = (char)mySerial.read();
    if (c == '<') {
      receiving = true;
      receivedData = ""; // Clear the buffer
    } else if (c == '>') {
      receiving = false;
      Serial.print("Received complete data: ");
      Serial.println(receivedData);

      // Deserialize JSON data
       const size_t capacity = JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(3) + 60;
      StaticJsonDocument<capacity> doc;
      DeserializationError error = deserializeJson(doc, receivedData);

      if (!error) {
        // Handle the received JSON data
        Serial.print(F("Message send: "));
        Serial.println((doc.as<String>()).c_str());

        // Publish the JSON data to NETPIE shadow
        if (client.publish("@shadow/data/update", receivedData.c_str())) {
          Serial.println("Message published successfully");
        } else {
          Serial.println("Message publish failed");
        }
      } else {
        Serial.print("deserializeJson() failed: ");
        //Serial.println(error.c_str());
      }
    } else if (receiving) {
      receivedData += c; // Append to buffer
    }
  }

  delay(1000);
}