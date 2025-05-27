#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <esp_http_server.h>

// Wi-Fi Credentials
const char* ssid = "Wifi_name";
const char* password = "Wifi_pass";

// LED Control Pins (Avoid GPIO 12 and 15)
#define GREEN1 14
#define RED1   13
#define GREEN2 2
#define RED2   4

// Status LED
#define BUILTIN_LED 33  // AI-Thinker ESP32-CAM has built-in LED on GPIO 33

WebServer server(80);
bool cameraInitialized = false;

// LED sequence variables
#define NUM_LEDS 4
const int ledPins[NUM_LEDS] = {GREEN1, RED1, GREEN2, RED2};
int currentLedIndex = 0;
unsigned long lastLedChange = 0;
const unsigned long LED_DURATION = 5000; // 5 seconds per LED

// Camera Pin Definitions (AI Thinker ESP32-CAM)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void startCameraServer();
void setupCamera();
void updateTrafficLights();

void setup() {
  Serial.begin(115200);
  Serial.println("\n\nESP32-CAM Traffic Light Controller");
  
  // Setup LED pins
  pinMode(GREEN1, OUTPUT);
  pinMode(RED1, OUTPUT);
  pinMode(GREEN2, OUTPUT);
  pinMode(RED2, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);

  // Initial state: All LEDs off
  digitalWrite(GREEN1, LOW);
  digitalWrite(RED1, LOW);
  digitalWrite(GREEN2, LOW);
  digitalWrite(RED2, LOW);
  digitalWrite(BUILTIN_LED, LOW);  // Turn off onboard LED initially

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));  // Blink LED while connecting
    delay(500);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(BUILTIN_LED, HIGH);  // LED stays on when connected
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed.");
    // Blink LED rapidly to indicate WiFi connection failure
    for (int i = 0; i < 10; i++) {
      digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));
      delay(100);
    }
    digitalWrite(BUILTIN_LED, LOW);
  }
  
  // Always turn on the built-in LED
  digitalWrite(BUILTIN_LED, HIGH);

  // Initialize camera
  setupCamera();

  // Web server routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/startsequence", HTTP_GET, handleStartSequence);
  server.on("/stopsequence", HTTP_GET, handleStopSequence);
  server.on("/testleds", HTTP_GET, handleTestAllLeds);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
  
  if (cameraInitialized) {
    startCameraServer();
    Serial.println("Camera stream ready at: ");
    Serial.print("http://");
    Serial.print(WiFi.localIP());
    Serial.println(":81/stream");
  }
  
  // Initialize LED sequence
  turnOffAllLeds();
  digitalWrite(ledPins[currentLedIndex], HIGH); // Turn on first LED
  lastLedChange = millis();
}

void loop() {
  server.handleClient();
  
  // Handle sequential LED blinking (one by one)
  if (millis() - lastLedChange >= LED_DURATION) {
    // Turn off current LED
    digitalWrite(ledPins[currentLedIndex], LOW);
    
    // Move to next LED
    currentLedIndex = (currentLedIndex + 1) % NUM_LEDS;
    
    // Turn on next LED
    digitalWrite(ledPins[currentLedIndex], HIGH);
    
    // Update timestamp
    lastLedChange = millis();
    
    Serial.print("Switched to LED at pin: ");
    Serial.println(ledPins[currentLedIndex]);
  }
  
  // Keep the built-in LED always on
  if (digitalRead(BUILTIN_LED) == LOW) {
    digitalWrite(BUILTIN_LED, HIGH);
  }
  
  // Monitor WiFi connection and reconnect if necessary
  static unsigned long lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 30000) {  // Check every 30 seconds
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost. Reconnecting...");
      WiFi.reconnect();
    }
  }
}

void turnOffAllLeds() {
  for (int i = 0; i < NUM_LEDS; i++) {
    digitalWrite(ledPins[i], LOW);
  }
}

void testAllLeds() {
  // Turn all LEDs on for 1 second
  for (int i = 0; i < NUM_LEDS; i++) {
    digitalWrite(ledPins[i], HIGH);
  }
  delay(1000);
  
  // Turn all LEDs off
  turnOffAllLeds();
}

void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;  // Higher resolution with PSRAM
    config.jpeg_quality = 10;           // Lower value means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;  // Lower resolution without PSRAM
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x. Please check ribbon and wiring.\n", err);
    cameraInitialized = false;
  } else {
    Serial.println("Camera initialized successfully");
    cameraInitialized = true;
    
    // Apply better initial camera settings
    sensor_t * s = esp_camera_sensor_get();
    if (s) {
      s->set_brightness(s, 1);      // Increase brightness slightly (-2 to 2)
      s->set_contrast(s, 1);        // Increase contrast (range -2 to 2)
      s->set_saturation(s, 0);      // Normal saturation (range -2 to 2)
      s->set_special_effect(s, 0);  // No special effect
      s->set_whitebal(s, 1);        // Enable white balance
      s->set_awb_gain(s, 1);        // Enable AWB gain
      s->set_wb_mode(s, 0);         // Auto white balance
      s->set_gainceiling(s, (gainceiling_t)6); // Set gain ceiling for better low-light
    }
  }
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>ESP32-CAM Sequential LED Control</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: Arial; text-align: center; margin: 0; padding: 20px; }";
  html += "h1 { color: #0066cc; }";
  html += ".button { display: inline-block; padding: 15px 25px; font-size: 18px; cursor: pointer; text-align: center; ";
  html += "text-decoration: none; outline: none; color: #fff; border: none; border-radius: 10px; box-shadow: 0 5px #999; margin: 10px; }";
  html += ".green { background-color: #4CAF50; }";
  html += ".green:hover { background-color: #3e8e41; }";
  html += ".red { background-color: #f44336; }";
  html += ".red:hover { background-color: #da190b; }";
  html += ".blue { background-color: #2196F3; }";
  html += ".blue:hover { background-color: #0b7dda; }";
  html += ".orange { background-color: #FF9800; }";
  html += ".orange:hover { background-color: #e68a00; }";
  html += ".container { max-width: 800px; margin: 0 auto; }";
  html += ".stream { width: 100%; max-width: 640px; height: auto; margin: 20px auto; border: 2px solid #ddd; }";
  html += ".status { padding: 10px; margin: 10px 0; background-color: #f2f2f2; border-radius: 5px; }";
  html += "</style>";
  html += "</head><body>";
  html += "<div class='container'>";
  html += "<h1>ESP32-CAM Sequential LED Controller</h1>";
  
  html += "<div class='status'>";
  html += "<p><strong>Current Status:</strong> Sequential LEDs active (5 seconds per LED)</p>";
  html += "<p><strong>Active LED:</strong> Pin " + String(ledPins[currentLedIndex]) + "</p>";
  html += "</div>";
  
  html += "<div>";
  html += "<a href='/stopsequence' class='button red'>Stop Sequence</a>";
  html += "<a href='/startsequence' class='button green'>Start Sequence</a>";
  html += "<a href='/testleds' class='button orange'>Test All LEDs</a>";
  html += "</div>";
  
  if (cameraInitialized) {
    html += "<div>";
    html += "<h2>Camera View</h2>";
    html += "<img src='http://" + WiFi.localIP().toString() + ":81/stream' class='stream' alt='Camera Stream'>";
    html += "</div>";
  } else {
    html += "<p>Camera not available</p>";
  }
  
  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

void handleStartSequence() {
  // Reset the sequence
  turnOffAllLeds();
  currentLedIndex = 0;
  digitalWrite(ledPins[currentLedIndex], HIGH);
  lastLedChange = millis();
  
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStopSequence() {
  // Turn off all LEDs
  turnOffAllLeds();
  
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleTestAllLeds() {
  // Test all LEDs
  testAllLeds();
  
  // Then restart the sequence
  currentLedIndex = 0;
  digitalWrite(ledPins[currentLedIndex], HIGH);
  lastLedChange = millis();
  
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleNotFound() {
  server.send(404, "text/plain", "404: Not found");
}

// Streaming Server
httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len;
  uint8_t * _jpg_buf;
  char part_buf[64];

  res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  if (res != ESP_OK) {
    return res;
  }

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      break;
    }
    
    _jpg_buf_len = fb->len;
    _jpg_buf = fb->buf;

    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, 64, "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", _jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, "\r\n", 2);
    }
    
    esp_camera_fb_return(fb);
    
    if (res != ESP_OK) {
      break;
    }
    
    // Small delay to avoid hogging the CPU
    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - 0;
    frame_time /= 1000;
    if (frame_time < 35) {
      vTaskDelay(pdMS_TO_TICKS(35 - frame_time));
    }
  }

  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.ctrl_port = 81;
  config.max_open_sockets = 5;
  config.lru_purge_enable = true;

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("Camera HTTP server started");
  } else {
    Serial.println("Failed to start camera HTTP server");
  }
}