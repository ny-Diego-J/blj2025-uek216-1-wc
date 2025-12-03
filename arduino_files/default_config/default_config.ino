#include <WiFi.h>
#include <ESP32MQTTClient.h>
#include "esp_idf_version.h"

const char *ssid = "GuestWLANPortal";
const char *server = "mqtt://10.10.2.127:1883";

const char *sub_topic = "Lasertag/target1/light";     // empfängt
const char *pub_topic = "zuerich/wc/01/info";       // sendet

const char *client_id = "lasarpointer";

ESP32MQTTClient client;

bool laserHit = false;
#define TRIG 4
#define ECHO 5

void setup() {
  Serial.begin(115200);

  setup_wifi();
  WiFi.setSleep(false);

  pinMode(27, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);


  client.setURI(server);
  client.setMqttClientName(client_id);
  client.loopStart();
}

void loop() {

  // Trigger-Puls
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // Echo messen
  long duration = pulseIn(ECHO, HIGH);

  // Distanz berechnen (falls der Sensor Ultraschall nutzt)
  float distance = duration * 0.034 / 2; // cm

  Serial.print("Distanz: ");
  Serial.print(distance);
  Serial.println(" cm");

  // MQTT Publish
if (client.isConnected()) {
    String msg = String(distance);
    client.publish(pub_topic, msg.c_str());
}

if ( distance > 120){
  digitalWrite(27, HIGH);
} else {
  digitalWrite(27, LOW);
}


  delay(200);
}
void setup_wifi() {
  Serial.print("Connecting to ");
  Serial.print(ssid);
  WiFi.begin(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);


    Serial.print(".");
  }
  Serial.println("done!");
}

void onMqttConnect(esp_mqtt_client_handle_t client_handle) {
  if (client.isMyTurn(client_handle)) {
    client.subscribe(sub_topic, [](const std::string &payload) {

      Serial.printf("Empfangen %s: %s\n", sub_topic, payload.c_str());

    });
  }
}

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
esp_err_t handleMQTT(esp_mqtt_event_handle_t event) {
  client.onEventCallback(event);
  return ESP_OK;
}
#else
void handleMQTT(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
  auto *event = static_cast<esp_mqtt_event_handle_t>(event_data);
  client.onEventCallback(event);
}
#endif

