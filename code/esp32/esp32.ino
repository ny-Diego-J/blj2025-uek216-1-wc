#include <WiFi.h>
#include <ESP32MQTTClient.h>
#include "esp_idf_version.h"
#include <MaxLedControl.h>
#include <cstdint>
#include <cmath>

#define WC_ID "02"
#define WC_PATH "zuerich/wc/"
#define WC_ID_PATH WC_PATH WC_ID

#define TRIG_PIN 4
#define ECHO_PIN 5
#define DIN_PIN 23
#define CLK_PIN 18
#define CS_PIN 2

const char *ssid = "GuestWLANPortal";
const char *server = "mqtt://10.10.2.127:1883";


const char *sub_max_time = WC_ID_PATH "/max_time_ms";
const char *pub_time_remaining = WC_ID_PATH "/time_remaining_ms";
const char *pub_in_use = WC_ID_PATH "/in_use";
const char *pub_door_distance = WC_ID_PATH "/etc/door_distance_cm";
 
const char *client_id = "wc" WC_ID;


const float opened_threshold_cm = 85;

uint64_t max_time_ms = 15 * 60 * 1000; // default 15 minutes
uint64_t start_time_ms;
uint64_t countdown_ms; // milliseconds left
float door_distance_cm;
 
LedControl display = LedControl(DIN_PIN, CLK_PIN, CS_PIN, 1);
ESP32MQTTClient client;

void mqttPublisher(void* pvParameters){
    (void)pvParameters;

    char msg[32];

    while (1){
      // MQTT Publish
      if (client.isConnected()) {
          snprintf(msg, 32, "%f", door_distance_cm);
          client.publish(pub_door_distance, msg);

          snprintf(msg, 32, "%llu", countdown_ms);
          client.publish(pub_time_remaining, msg);
      }
      else{
          Serial.println("disconnected :((((");

          setup_wifi();
      }
      
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void countdownDisplayer(void* pvParameters){
  (void)pvParameters;

  char s[32];
  while (1){
    uint64_t elapsed_ms = millis() - start_time_ms;

    if (elapsed_ms > max_time_ms) {
      // countdown reached 0
      Serial.print("alarm");
      strncpy(s, "alarm", 32);
      countdown_ms = 0;
    } 
    else {
      countdown_ms = max_time_ms - elapsed_ms;
      uint64_t countdown_s = countdown_ms / 1000;

      uint16_t minutes = countdown_s / 60;
      uint16_t seconds = countdown_s % 60;

      snprintf(s, 32, "%.2d %.2d", minutes, seconds);
    }

    Serial.println(s);

    display.clearDisplay(0);
 
    displayString(s);

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void displayString(char* s) {
  uint16_t len = (uint16_t)strlen(s);

  for (uint16_t i = 0; i < len; i++) {
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

  // 0 means no signal received
  if (cm == 0.f || cm > 500.f){
    cm = 500.f;
  }
 
  return cm;
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

  start_time_ms = millis();
  countdown_ms = max_time_ms;

  // create publisher thread
  xTaskCreate(
    mqttPublisher,
    "mqttPublisher",
    4096,
    NULL,
    1,
    NULL
  );
  // create displayer thread
  xTaskCreate(
    countdownDisplayer,
    "countdownDisplayer",
    4096,
    NULL,
    1,
    NULL
  );
}
 
void loop() {
  door_distance_cm = getUltrasonicDistanceCm();
 
  Serial.println(door_distance_cm);
 
  if (door_distance_cm < opened_threshold_cm) {
    // reset timer
    start_time_ms = millis();
  }
 
  Serial.print("distance: ");
  Serial.print(door_distance_cm);

  delay(300);
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
  client.subscribe(std::string(sub_max_time), [](const std::string &payload) {
    max_time_ms = std::stoull(payload.c_str(), NULL, 10);
    Serial.println(max_time_ms);
  });
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