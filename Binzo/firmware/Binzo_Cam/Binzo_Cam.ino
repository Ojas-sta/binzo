/*
 * BINZO - ESP32-CAM Firmware
 * Hardware: AI-Thinker ESP32-CAM (OV2640 + PSRAM)
 *
 * Capabilities:
 * - 32x24 luminance grid motion & silhouette shape detection
 * - Optional PIR motion sensor integration on GPIO13
 * - Wi-Fi Station connecting to Main ESP32 SoftAP ("Binzo-Bin", 192.168.4.200)
 * - ESP-NOW broadcast sender streaming "HUMAN,1" / "HUMAN,0" every 500ms
 * - Snapshot HTTP server on port 80 serving live visual diagnostics
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_pins.h"

// Configuration
#define USE_PIR               0       // Set to 1 if external PIR sensor is wired to GPIO13
#define MOTION_THRESHOLD      22      // Average pixel delta threshold to trigger motion
#define ACTIVE_CELLS_TRIGGER  28      // Minimum number of cells with change to count as a human presence
#define GRID_COLS             32
#define GRID_ROWS             24

#define WIFI_SSID             "Binzo-Bin"
#define WIFI_PASS             "binzo1234"
#define CAM_STATIC_IP         192, 168, 4, 200
#define CAM_GATEWAY           192, 168, 4, 1
#define CAM_SUBNET            255, 255, 255, 0

// ESP-NOW Broadcast Address
uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Web Server
WebServer server(80);

// Detection State
bool isHumanDetected = false;
unsigned long lastEspNowSendTime = 0;
const unsigned long ESPNOW_INTERVAL_MS = 500;

// Vision Grids
uint8_t prevGrid[GRID_ROWS][GRID_COLS];
uint8_t currGrid[GRID_ROWS][GRID_COLS];
bool gridInitialized = false;

// =============================================================================
// CAMERA INITIALIZATION
// =============================================================================
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  
  // Use Grayscale QVGA (320x240) for fast direct luminance processing
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 2;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_vflip(s, 1);     // Flip vertical if mounted upside down
    s->set_hmirror(s, 0);
  }

  Serial.println("[CAM] Camera initialized successfully.");
  return true;
}

// =============================================================================
// VISION DETECTION (32x24 Luminance Grid Frame Difference)
// =============================================================================
void processVision() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[CAM] Frame buffer grab failed");
    return;
  }

  // 320x240 downsample to 32x24 (10x10 block stride)
  int blockW = fb->width / GRID_COLS;   // 10
  int blockH = fb->height / GRID_ROWS;  // 10
  int activeCells = 0;

  for (int r = 0; r < GRID_ROWS; r++) {
    for (int c = 0; c < GRID_COLS; c++) {
      int pixelX = c * blockW + (blockW / 2);
      int pixelY = r * blockH + (blockH / 2);
      int index = pixelY * fb->width + pixelX;

      currGrid[r][c] = fb->buf[index];

      if (gridInitialized) {
        int diff = abs((int)currGrid[r][c] - (int)prevGrid[r][c]);
        if (diff > MOTION_THRESHOLD) {
          activeCells++;
        }
      }
    }
  }

  // Save current grid as baseline for next frame
  memcpy(prevGrid, currGrid, sizeof(currGrid));
  gridInitialized = true;

  bool visionMotion = (activeCells >= ACTIVE_CELLS_TRIGGER);

  #if USE_PIR
    bool pirState = (digitalRead(PIR_PIN) == HIGH);
    isHumanDetected = visionMotion || pirState;
  #else
    isHumanDetected = visionMotion;
  #endif

  // Turn status LED ON when human detected (Active LOW on AI-Thinker)
  digitalWrite(STATUS_LED_PIN, isHumanDetected ? LOW : HIGH);

  esp_camera_fb_return(fb);
}

// =============================================================================
// WEB SERVER HANDLERS
// =============================================================================
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>BINZO CAM DIAGNOSTICS</title>";
  html += "<style>body{font-family:Arial,sans-serif;background:#1a1a1a;color:#eee;text-align:center;padding:20px;}";
  html += ".card{background:#2a2a2a;border-radius:10px;padding:20px;display:inline-block;box-shadow:0 4px 10px rgba(0,0,0,0.5);}";
  html += ".status{font-size:20px;font-weight:bold;margin:15px;color:" + String(isHumanDetected ? "#4CAF50" : "#F44336") + ";}";
  html += "img{border-radius:8px;max-width:100%;border:2px solid #555;}";
  html += "button{background:#008CBA;border:none;color:white;padding:10px 20px;border-radius:5px;cursor:pointer;font-size:16px;}";
  html += "</style><script>setInterval(()=>{document.getElementById('snap').src='/capture?r='+Math.random();}, 1000);</script>";
  html += "</head><body><div class='card'>";
  html += "<h2>BINZO — ESP32-CAM Diagnostics</h2>";
  html += "<div class='status'>Detection: " + String(isHumanDetected ? "HUMAN PRESENT" : "CLEAR") + "</div>";
  html += "<img id='snap' src='/capture' width='320' height='240'><br><br>";
  html += "<p>RSSI: " + String(WiFi.RSSI()) + " dBm | IP: " + WiFi.localIP().toString() + "</p>";
  html += "<div style='margin-top:20px;padding:12px 18px;border:1px solid #333;border-radius:14px;background:#111;display:inline-block;max-width:320px;text-align:center;'>";
  html += "<div style='display:flex;align-items:center;justify-content:center;gap:8px;font-size:12px;letter-spacing:0.06em;color:#f8fafc;'>";
  html += "<span>POWERED BY <strong style='color:#00E5FF;'>cl0ud</strong></span>";
  html += "<svg width='20' height='13' viewBox='0 0 608 408' fill='none' xmlns='http://www.w3.org/2000/svg' style='vertical-align:middle;'>";
  html += "<path stroke='#38BDF8' stroke-width='40' stroke-linejoin='round' fill='none' d='M 309.8 1.5 c -23.8 3.0 -45.7 11.0 -65.4 23.8 -16.7 10.8 -31.5 24.8 -42.8 40.3 l -5.5 7.6 -5.1 -1.4 c -7.4 -2.1 -26.8 -2.8 -36.7 -1.3 -39.4 5.7 -70.9 32.3 -83.0 70.0 l -2.7 8.5 -8.8 6.2 c -38.8 27.0 -59.7 66.9 -59.7 113.8 -0.1 22.0 4.3 40.6 14.2 61.0 6.8 14.0 15.6 25.8 28.0 37.8 19.9 19.2 41.2 30.5 68.4 36.4 6.9 1.5 13.4 1.8 40.7 1.8 35.1 -0.0 39.2 -0.4 55.4 -5.7 34.2 -11.0 63.0 -34.9 96.1 -79.7 37.2 -50.3 44.9 -60.4 55.3 -72.2 22.3 -25.5 45.9 -42.4 72.3 -51.7 11.5 -4.1 22.8 -5.6 35.3 -4.9 11.9 0.7 19.5 2.6 29.1 7.2 19.6 9.5 33.7 26.4 40.7 48.5 2.0 6.4 2.3 9.5 2.3 21.5 -0.1 13.2 -0.3 14.5 -3.5 23.9 -7.1 20.9 -21.6 37.0 -41.5 46.2 -14.9 6.9 -13.3 6.7 -83.5 7.2 l -63.5 0.4 -13.5 18.0 c -7.4 9.9 -18.2 23.4 -23.8 30.0 l -10.4 11.9 90.9 -0.4 c 89.0 -0.3 91.0 -0.4 99.3 -2.5 19.3 -4.8 31.9 -10.5 49.1 -22.0 47.4 -31.7 69.8 -91.1 55.6 -147.2 -12.3 -48.2 -49.8 -85.9 -98.0 -98.4 l -9.0 -2.4 -2.3 -9.8 c -2.9 -12.7 -7.1 -24.0 -13.9 -37.3 -22.0 -42.6 -62.3 -73.3 -109.0 -83.1 -11.5 -2.4 -39.4 -3.5 -51.1 -2.0 z'/>";
  html += "</svg>";
  html += "</div>";
  html += "<div style='height:1px;background:#262626;margin:8px 0;'></div>";
  html += "<div style='font-size:10px;letter-spacing:0.1em;color:#888;text-transform:uppercase;'>CO-ENGINEERED BY</div>";
  html += "<div style='font-size:13px;font-weight:bold;color:#fff;margin-top:2px;'><a href='https://github.com/Ojas-sta' target='_blank' rel='noopener noreferrer' style='color:#fff;text-decoration:none;'>OJAS SINGH</a></div>";
  html += "<div style='font-size:9px;color:#666;margin-top:2px;'><a href='https://github.com/Ojas-sta/cl0ud-brand-engineering' target='_blank' rel='noopener noreferrer' style='color:#888;text-decoration:none;'>cl0ud brand engineering</a></div>";
  html += "</div>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

void handleCapture() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }

  uint8_t *jpg_buf = NULL;
  size_t jpg_len = 0;
  bool converted = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_GRAYSCALE, 80, &jpg_buf, &jpg_len);
  esp_camera_fb_return(fb);

  if (!converted) {
    server.send(500, "text/plain", "JPEG conversion failed");
    return;
  }

  server.sendHeader("Content-Type", "image/jpeg");
  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send_P(200, "image/jpeg", (const char *)jpg_buf, jpg_len);
  free(jpg_buf);
}

void handleStatus() {
  String json = "{";
  json += "\"human\":" + String(isHumanDetected ? "true" : "false") + ",";
  json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"uptime_ms\":" + String(millis());
  json += "}";
  server.send(200, "application/json", json);
}

// =============================================================================
// ESP-NOW TRANSMISSION
// =============================================================================
void sendEspNowDetection() {
  const char *msg = isHumanDetected ? "HUMAN,1" : "HUMAN,0";
  esp_err_t result = esp_now_send(broadcastMac, (const uint8_t *)msg, strlen(msg));
  if (result != ESP_OK) {
    // Retry or log if needed
  }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== BINZO ESP32-CAM INITIALIZING ===");

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH); // Off initially (Active LOW)

  #if USE_PIR
    pinMode(PIR_PIN, INPUT);
  #endif

  if (!initCamera()) {
    Serial.println("[ERR] Camera halt!");
    while (true) {
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(100);
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(100);
    }
  }

  // Configure Wi-Fi Station with Static IP
  WiFi.mode(WIFI_STA);
  IPAddress local_IP(CAM_STATIC_IP);
  IPAddress gateway(CAM_GATEWAY);
  IPAddress subnet(CAM_SUBNET);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.printf("[NET] Connecting to AP %s...", WIFI_SSID);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 25) {
    delay(300);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[NET] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[NET] AP connection delayed; proceeding with ESP-NOW broadcast.");
  }

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init Failed!");
  } else {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastMac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      Serial.println("[ESP-NOW] Broadcast Peer Added.");
    }
  }

  // Setup Web Server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/status", HTTP_GET, handleStatus);
  server.begin();
  Serial.println("[HTTP] Server listening on port 80");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  server.handleClient();
  processVision();

  unsigned long currentMillis = millis();
  if (currentMillis - lastEspNowSendTime >= ESPNOW_INTERVAL_MS) {
    lastEspNowSendTime = currentMillis;
    sendEspNowDetection();
  }
}
