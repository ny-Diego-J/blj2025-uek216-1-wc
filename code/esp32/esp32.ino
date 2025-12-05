#include <WiFi.h>
#include <ESP32MQTTClient.h>
#include "esp_idf_version.h"
#include <MaxLedControl.h>
#include <cstdint>
#include <cmath>

#define WC_ID "01"
#define WC_PATH "zuerich/wc/"
#define WC_ID_PATH WC_PATH WC_ID

#define TRIG_PIN 4
#define ECHO_PIN 5
#define DIN_PIN 23
#define CLK_PIN 18
#define CS_PIN 2
#define OPENED_THRESHOLD_CM 85

#define NUM_SINES 1
#define MAX_WAVEFORM (10.f * PI);

const char *ssid = "GuestWLANPortal";
const char *server = "mqtt://10.10.2.127:1883";


const char *sub_max_time = WC_ID_PATH "/max_time_ms";
const char *pub_time_remaining = WC_ID_PATH "/time_remaining_ms";
const char *pub_in_use = WC_ID_PATH "/in_use";
const char *pub_door_distance = WC_ID_PATH "/etc/door_distance_cm";
 
const char *client_id = "wc" WC_ID;

float freqs[NUM_SINES] = {1.f};
float amps[NUM_SINES] = {10.f};
float delta = 0.015f;
float t = 0.f;



uint64_t max_time_ms = 15 * 60 * 1000; // default 15 minutes
int _alarm = false;
uint64_t start_time_ms;
uint64_t countdown_ms; // milliseconds left
float door_distance_cm;


uint32_t xor_mask = 67676767;

LedControl display = LedControl(DIN_PIN, CLK_PIN, CS_PIN, 1);
ESP32MQTTClient client;

float getWaveformX(){

  float val = 0.f;
  for (uint16_t i = 0; i < NUM_SINES; i++){
    val += amps[i] * sin(freqs[i] * t);
  }

  t += delta;

  t = fmod(t, 2 * PI);

  return val / MAX_WAVEFORM;
}

void playFrequency(uint16_t freq_ms, uint16_t time_ms){
  uint64_t start = millis();
  uint64_t end = start + time_ms;

  while (millis() < end){
      digitalWrite(27, HIGH);
      vTaskDelay(freq_ms / portTICK_PERIOD_MS);
      digitalWrite(27, LOW);
      vTaskDelay(freq_ms / portTICK_PERIOD_MS);
  }
}

void playPWM(uint16_t duty, uint16_t cycle, uint16_t time_ms){
  uint64_t start = millis();
  uint64_t end = start + time_ms;

  while (millis() < end){
    uint64_t start_duty = millis();
    uint64_t end_duty = start_duty + duty;
    while (millis() < end_duty){
      digitalWrite(27, HIGH);
      yield();
      //vTaskDelay(1 * portTICK_PERIOD_MS);
      digitalWrite(27, LOW);
      yield();
      //vTaskDelay(1 * portTICK_PERIOD_MS);
    }
    vTaskDelay((1 + cycle - duty) * portTICK_PERIOD_MS);
  }
}

void alarmThread(void* pvParameters){
  (void)(pvParameters);

  while (true){
    while (_alarm){
      float w = getWaveformX();

      uint32_t duty = (int32_t)((w + 1) * 3);

      playPWM(duty, 7, 50);

      //digitalWrite(27, HIGH);
      //vTaskDelay(duty / portTICK_PERIOD_MS);
      //digitalWrite(27, LOW);
      //vTaskDelay(duty / portTICK_PERIOD_MS);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


// Xor is its own inverse, so this works for decipher and cipher
uint32_t cipher(void* x){
  return (*(uint32_t*)x) ^ xor_mask;
}

void mqttPublisher(void* pvParameters){
    (void)pvParameters;

    char msg[32];

    while (1){
      // cipher and MQTT Publish
      if (client.isConnected()) {
          //Serial.printf("raw %f, xored %lu", door_distance_cm, (unsigned long)encoded);
          snprintf(msg, 32, "%lu", (unsigned long) cipher(&door_distance_cm));
          client.publish(pub_door_distance, msg);

          snprintf(msg, 32, "%lu", (unsigned long) cipher(&countdown_ms));
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
      _alarm = true;
      Serial.print("71nnE UP");
      strncpy(s, "71nnE UP", 32);
      countdown_ms = 0;
    }
    else {
      _alarm = false;
      countdown_ms = max_time_ms - elapsed_ms;
      uint64_t countdown_s = countdown_ms / 1000;

      uint16_t minutes = countdown_s / 60;
      uint16_t seconds = countdown_s % 60;

      snprintf(s, 32, "%.2d %.2d", minutes, seconds);
    }

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
  while (!client.isConnected()) {
    delay(10);
  }

  // publish default max_time
  //char msg[32];
  //snprintf(msg, 32, "%llu", cipher(&max_time_ms));
  //client.publish(sub_max_time, msg);
 
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
  xTaskCreate(
    alarmThread,
    "alarmThread",
    2048,
    NULL,
    1,
    NULL
  );
}
 
void loop() {
  door_distance_cm = getUltrasonicDistanceCm();
 
  Serial.println(door_distance_cm);
 
  if (door_distance_cm < OPENED_THRESHOLD_CM) {
    // reset timer
    start_time_ms = millis();
  }
 
  Serial.print("distance: ");
  Serial.print(door_distance_cm);
  Serial.println(max_time_ms);

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
    uint32_t ciphered = (uint32_t) std::stoull(payload.c_str(), NULL, 10);
    max_time_ms = cipher(&ciphered);
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