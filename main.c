#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// ───── Pin Definitions ─────
const int BEACON_LED        = 5;
const int ALERT_LED         = 4;
const int BUTTON_PIN        = 18;
const int DOSIMETER_PIN     = 34;
const int SENSOR_TRACE_PIN  = 15;  // mark sensorTask execution
const int LIDAR_TRACE_PIN   = 2;   // mark lidarTask execution

// ───── Parameters ─────
const int MAX_EVENTS         = 10;
const int EXPOSURE_THRESHOLD = 3000;  // correct constant

// ───── Wi‑Fi Credentials ─────
const char* WIFI_SSID    = "Wokwi-GUEST";
const char* WIFI_PWD     = "";
const int   WIFI_CHANNEL = 6;

// ───── FreeRTOS primitives ─────
SemaphoreHandle_t sensor_alert_sem;
SemaphoreHandle_t button_event_sem;
SemaphoreHandle_t serial_mutex;
QueueHandle_t    sensor_queue;

volatile bool system_mode        = false;
volatile bool alert_state        = false;
volatile int  latest_sensor_value = 0;

WebServer console(80);

// ───── Forward declarations ─────
void commsTask(void* pv);
void heartbeatTask(void* pv);
void modeLedTask(void* pv);
void sensorTask(void* pv);
void lidarTask(void* pv);
void sensorConsumerTask(void* pv);
void eventResponseTask(void* pv);
void IRAM_ATTR buttonISR();
void sendConsolePage();
void handleRoot();
void handleToggle();

void setup() {
  Serial.begin(115200);

  // trace pins
  pinMode(SENSOR_TRACE_PIN, OUTPUT);
  pinMode(LIDAR_TRACE_PIN,  OUTPUT);
  digitalWrite(SENSOR_TRACE_PIN, LOW);
  digitalWrite(LIDAR_TRACE_PIN,  LOW);

  // synchronization primitives
  sensor_alert_sem = xSemaphoreCreateCounting(MAX_EVENTS, 0);
  button_event_sem = xSemaphoreCreateBinary();
  serial_mutex     = xSemaphoreCreateMutex();
  sensor_queue     = xQueueCreate(20, sizeof(int));

  // button ISR
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  // HTTP routes
  console.on("/",       handleRoot);
  console.on("/toggle", handleToggle);

  // spawn tasks
  xTaskCreate(commsTask,          "Comms",     4096, NULL, 3, NULL);
  xTaskCreate(sensorTask,         "Sensor",    2048, NULL, 4, NULL);
  xTaskCreate(lidarTask,          "LiDAR",     4096, NULL, 2, NULL);
  xTaskCreate(sensorConsumerTask, "Consume",   2048, NULL, 3, NULL);
  xTaskCreate(eventResponseTask,  "Respond",   2048, NULL, 2, NULL);
  xTaskCreate(heartbeatTask,      "Heartbeat", 1024, NULL, 1, NULL);
  xTaskCreate(modeLedTask,        "ModeLED",   1024, NULL, 2, NULL);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// —— commsTask — connect & serve console ——
void commsTask(void* pv) {
  Serial.print("Connecting to WiFi ");
  WiFi.begin(WIFI_SSID, WIFI_PWD, WIFI_CHANNEL);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(100));
    Serial.print(".");
  }
  Serial.println(" Connected! IP=" + WiFi.localIP().toString());
  console.begin();
  for (;;) {
    console.handleClient();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// —— sensorTask — sample every 17 ms —— 
void sensorTask(void* pv) {
  const TickType_t period = pdMS_TO_TICKS(17);
  TickType_t lastWake = xTaskGetTickCount();
  bool lastAlert = false;

  pinMode(DOSIMETER_PIN, INPUT);
  for (;;) {
    // TRACE ON
    digitalWrite(SENSOR_TRACE_PIN, HIGH);

    int reading = analogRead(DOSIMETER_PIN);
    xQueueSend(sensor_queue, &reading, 0);
    if (reading > EXPOSURE_THRESHOLD && !lastAlert) {
      lastAlert = true;
      xSemaphoreGive(sensor_alert_sem);
    } else if (reading <= EXPOSURE_THRESHOLD) {
      lastAlert = false;
    }

    // TRACE OFF
    digitalWrite(SENSOR_TRACE_PIN, LOW);

    vTaskDelayUntil(&lastWake, period);
  }
}

// —— lidarTask — variable execution —— 
void lidarTask(void* pv) {
  for (;;) {
    // TRACE ON
    digitalWrite(LIDAR_TRACE_PIN, HIGH);

    int loops = random(10000, 60000);
    volatile int dummy = 0;
    for (int i = 0; i < loops; i++) dummy += (i & 1);

    // TRACE OFF
    digitalWrite(LIDAR_TRACE_PIN, LOW);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// —— sensorConsumerTask — update latest value ——
void sensorConsumerTask(void* pv) {
  int val;
  while (xQueueReceive(sensor_queue, &val, portMAX_DELAY) == pdTRUE) {
    latest_sensor_value = val;
  }
}

// —— eventResponseTask — blink ALERT_LED ——
void eventResponseTask(void* pv) {
  pinMode(ALERT_LED, OUTPUT);
  for (;;) {
    if (xSemaphoreTake(sensor_alert_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
      xSemaphoreTake(serial_mutex, portMAX_DELAY);
      Serial.println("🚨 ALERT!");
      xSemaphoreGive(serial_mutex);

      for (int i = 0; i < 3; i++) {
        digitalWrite(ALERT_LED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(200));
        digitalWrite(ALERT_LED, LOW);
        vTaskDelay(pdMS_TO_TICKS(200));
      }
    }
    if (xSemaphoreTake(button_event_sem, 0) == pdTRUE) {
      system_mode = !system_mode;
      xSemaphoreTake(serial_mutex, portMAX_DELAY);
      Serial.print("Mode = ");
      Serial.println(system_mode ? "ALERT" : "NORMAL");
      xSemaphoreGive(serial_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// —— heartbeatTask — blink heartbeat ——
void heartbeatTask(void* pv) {
  pinMode(BEACON_LED, OUTPUT);
  for (;;) {
    digitalWrite(BEACON_LED, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(BEACON_LED, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// —— modeLedTask — reflect system_mode ——
void modeLedTask(void* pv) {
  pinMode(BEACON_LED, OUTPUT);
  for (;;) {
    digitalWrite(BEACON_LED, system_mode);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// —— ISR — give semaphore —— 
void IRAM_ATTR buttonISR() {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(button_event_sem, &woken);
  portYIELD_FROM_ISR(woken);
}

// —— Web UI ——
void sendConsolePage() {
  String html = "<html><body><h1>Flight Control Console</h1>";
  html += "<p>Mode: " + String(system_mode ? "ALERT" : "NORMAL") + "</p>";
  html += "<p>Radiation: " + String(latest_sensor_value) + "</p>";
  html += "<p><a href=\"/toggle\">Toggle Mode</a></p>";
  html += "</body></html>";
  console.send(200, "text/html", html);
}
void handleRoot()   { sendConsolePage(); }
void handleToggle(){ xSemaphoreGive(button_event_sem); sendConsolePage(); }
