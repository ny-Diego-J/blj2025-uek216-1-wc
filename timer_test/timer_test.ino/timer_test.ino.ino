#include <WiFi.h>
#include <ESP32MQTTClient.h>
#include "esp_idf_version.h"
#include <MaxLedControl.h>
#include <cstdint>
#include <cmath>
 
const char *ssid = "GuestWLANPortal";
const char *server = "mqtt://10.10.2.127:1883";
 
const char *sub_topic = "Lasertag/target1/light";        // empfängt
const char *pub_topic = "zuerich/wc/01/etc/test1";            // sendet
const char *pub_topic2 = "zuerich/wc/01/etc/test2";  // sendet
 
const char *client_id = "lasarpointer";
 
ESP32MQTTClient client;
 
bool laserHit = false;
#define TRIG_PIN 4
#define ECHO_PIN 5
#define DIN_PIN 23
#define CLK_PIN 18
#define CS_PIN 2
uint64_t TIMER_MS = 1 * 60 * 1000;

uint64_t start_time;
uint64_t countdown_ms; 
float distance_cm;
 
LedControl display = LedControl(DIN_PIN, CLK_PIN, CS_PIN, 1);

void mqttPublisher(void* pvParameters){
    (void)pvParameters;

    char msg[32];

    while (1){
    // MQTT Publish
    if (client.isConnected()) {
        snprintf(msg, 16, "%.2f", distance_cm);
        client.publish(pub_topic, msg);

        snprintf(msg, 16, "%llu", countdown_ms);
        client.publish(pub_topic2, msg);
        //String msg = String(distance_cm);
        //client.publish(pub_topic, msg.c_str());
        //client.publish(pub_topic2, String(countdown_ms).c_str());
    }
    else{
        Serial.println("disconnected :((((");

        setup_wifi();
    }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void displayString(const String &s) {
  for (uint16_t i = 0; i < (uint16_t)s.length(); i++) {
    display.setChar(0, 8 - i - 1, s[i], false);
  }
  Serial.println(s);
}
 
float getUltrasonicDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
 
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
 
  float cm = duration * 0.034 / 2;  // cm

  if (cm == 0.f || cm > 500.f){
    cm = NAN;
  }
 
  return cm;
}
 
String stringLeftPad(const String &s, const uint16_t len, const char pad) {
  uint16_t pad_len = len - (uint16_t)s.length();
 
  String pad_str;
  for (uint16_t i = 0; i < pad_len; i++) {
    pad_str += pad;
  }
 
  return pad_str + s;
}

void setup() {
  Serial.begin(115200);
    
  setup_wifi();
  WiFi.setSleep(false);
  client.setURI(server);
  client.setMqttClientName(client_id);
  client.loopStart();
 
  pinMode(27, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
 
  display.begin(15);
  display.clear();

  start_time = millis();
  countdown_ms = TIMER_MS;

  // create publisher thread
  xTaskCreate(
    mqttPublisher,
    "mqttPublisher",
    4096,
    NULL,
    1,
    NULL
  );
}
 
void loop() {
  uint64_t elapsed_ms = millis() - start_time;
  String s;

  if (elapsed_ms > TIMER_MS) {
    Serial.print("alarm");
    s = "alarm";
    countdown_ms = 0;
  } else {
    digitalWrite(27, LOW);
    countdown_ms = TIMER_MS - elapsed_ms;
    uint64_t countdown_s = countdown_ms / 1000;
 
    uint16_t minutes = countdown_s / 60;
    uint16_t seconds = countdown_s % 60;
 
    s += stringLeftPad(String(minutes), 2, '0');
    s += ' ';
    s += stringLeftPad(String(seconds), 2, '0');
  }
 
  Serial.println(s);
 
  displayString(s);

  distance_cm = getUltrasonicDistanceCm();
 
  Serial.println(distance_cm);
 
  if (distance_cm < 10) {
    start_time = millis();
  }
 
  Serial.print("Distanz: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  delay(100);
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