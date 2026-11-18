#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>

// -----------------------------------------------------------------------------
// BINZO — ESP32-C3 Super Mini Head & Actuator Subsystem
// Architecture: Powered by cl0ud (https://github.com/Ojas-sta/cl0ud-brand-engineering)
// Direct M5StickV Camera UART + MG90S Lid Servo + HC-SR04 + ESP-NOW Master
// -----------------------------------------------------------------------------
// Pinout:
// GPIO 10: Lid Servo (MG90S)
// GPIO 3:  HC-SR04 TRIG (Trash Level)
// GPIO 4:  HC-SR04 ECHO (Trash Level)
// GPIO 6:  Camera Grove RX (connects to M5 Camera TX / Pin 34)
// GPIO 7:  Camera Grove TX (connects to M5 Camera RX / Pin 35)
// GPIO 8:  I2C SDA (Slave 0x30 fallback)
// GPIO 9:  I2C SCL (Slave 0x30 fallback)
// -----------------------------------------------------------------------------

#define SLAVE_I2C_ADDR   0x30
#define PIN_SDA          8
#define PIN_SCL          9

#define PIN_SERVO        10
#define PIN_TRIG         3
#define PIN_ECHO         4

#define PIN_CAM_RX       6
#define PIN_CAM_TX       7

// Bin dimensions (cm) for fill % calculation
const float BIN_DEPTH_CM = 35.0; // Empty depth
const float BIN_FULL_CM  = 6.0;  // Full depth (near sensor)

Servo lidServo;
volatile uint8_t pendingAngle = 0;
uint8_t currentDistanceCm = 35;
uint8_t currentFillPercent = 0;

// Camera Perception State
bool cameraConnected = false;
bool cameraAttention = false;
uint8_t cameraDwellSec = 0;
unsigned long lastCameraPacketTime = 0;
String camBuffer = "";

// ESP-NOW Control Packet (Shared across Subsystems)
typedef struct __attribute__((packed)) {
    uint8_t  magic;          // 0xB2 (Binzo signature)
    uint8_t  robot_cmd;      // 0 = MOVE (Line Follow), 1 = STOP (Halt)
    uint8_t  lid_state;      // 0 = CLOSED (0 deg), 1 = OPEN (95 deg)
    uint8_t  attention;      // 0 = NO, 1 = YES
    uint8_t  dwell_sec;      // Current dwell in seconds (0 to 10)
    uint8_t  trash_fill_pct; // 0 to 100% from HC-SR04
    uint16_t seq;            // Monotonic sequence counter
} BinzoEspNowPacket;

BinzoEspNowPacket espNowPkt;
uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// I2C Callbacks (Legacy Core2 Fallback)
void onI2CReceive(int len) {
    if (Wire.available()) {
        uint8_t cmd = Wire.read();
        if (cmd == 0x01) {
            pendingAngle = 95; // OPEN LID
        } else if (cmd == 0x02) {
            pendingAngle = 0;  // CLOSE LID
        } else if (cmd == 0x10 && Wire.available()) {
            pendingAngle = constrain(Wire.read(), 0, 180);
        }
    }
}

void onI2CRequest() {
    uint8_t payload[2];
    payload[0] = currentDistanceCm;
    payload[1] = currentFillPercent;
    Wire.write(payload, 2);
}

// Ultrasonic Trash Level Measurement
float measureDistanceCm() {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    long duration = pulseIn(PIN_ECHO, HIGH, 25000); // 25ms timeout (~4m)
    if (duration == 0) return BIN_DEPTH_CM;
    return (duration * 0.0343f) / 2.0f;
}

// Process Direct Camera UART Packets
void processCameraLine(const String& line) {
    if (line.startsWith("BINZO:ATTN,")) {
        // Format: BINZO:ATTN,<flag>,<dwell_sec>,<score>,<h>,<cx>
        int p1 = line.indexOf(',', 11);
        if (p1 > 0) {
            uint8_t flag = line.substring(11, p1).toInt();
            int p2 = line.indexOf(',', p1 + 1);
            float dwell = (p2 > 0) ? line.substring(p1 + 1, p2).toFloat() : 0.0f;

            lastCameraPacketTime = millis();
            cameraConnected = true;
            cameraAttention = (flag == 1);
            cameraDwellSec = (uint8_t)constrain((int)dwell, 0, 10);

            if (flag == 1) {
                // Paying Attention -> OPEN LID (95 deg)
                pendingAngle = 95;
            } else {
                // Not Paying Attention or 10s Timeout -> CLOSE LID (0 deg)
                pendingAngle = 0;
            }
        }
    }
}

void initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    // Lock to Wi-Fi Channel 1 for ultra-low latency & zero-router dependency
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Init Failed!");
        return;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastMac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESP-NOW] Peer Add Failed!");
    } else {
        Serial.println("[ESP-NOW] Broadcast Peer Added on Channel 1!");
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize Camera Grove Hardware UART on GPIO 6 (RX), GPIO 7 (TX)
    Serial1.begin(115200, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);

    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);

    // Initialize Servo on Pin 10
    ESP32PWM::allocateTimer(0);
    lidServo.setPeriodHertz(50);
    lidServo.attach(PIN_SERVO, 500, 2400);
    lidServo.write(0); // Start closed

    // Initialize I2C Slave
    Wire.begin(SLAVE_I2C_ADDR, PIN_SDA, PIN_SCL, 100000);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);

    // Initialize ESP-NOW
    initEspNow();

    memset(&espNowPkt, 0, sizeof(espNowPkt));
    espNowPkt.magic = 0xB2;

    Serial.println("[BINZO-SUPERMINI] Head Subsystem Online: Cam UART + Servo + ESP-NOW");
}

void loop() {
    // 1. Read incoming stream from M5StickV Camera
    while (Serial1.available()) {
        char c = (char)Serial1.read();
        if (c == '\n') {
            camBuffer.trim();
            if (camBuffer.length() > 0) {
                processCameraLine(camBuffer);
            }
            camBuffer = "";
        } else if (c != '\r') {
            camBuffer += c;
            if (camBuffer.length() > 120) camBuffer = ""; // Safety clamp
        }
    }

    // 2. Failsafe: If camera stops sending for > 2000 ms, clear attention & close lid
    if (cameraConnected && (millis() - lastCameraPacketTime > 2000)) {
        cameraAttention = false;
        cameraConnected = false;
        pendingAngle = 0;
    }

    // 3. Actuate servo if angle changed
    static uint8_t activeAngle = 255;
    if (pendingAngle != activeAngle) {
        activeAngle = pendingAngle;
        lidServo.write(activeAngle);
        Serial.printf("[SERVO] Angle -> %d deg | Attention: %d\n", activeAngle, cameraAttention);
    }

    // 4. Measure trash level every 200 ms
    static unsigned long lastMeasure = 0;
    if (millis() - lastMeasure >= 200) {
        lastMeasure = millis();
        float dist = measureDistanceCm();
        currentDistanceCm = (uint8_t)constrain((int)dist, 0, 255);

        float pct = ((BIN_DEPTH_CM - dist) / (BIN_DEPTH_CM - BIN_FULL_CM)) * 100.0f;
        currentFillPercent = (uint8_t)constrain((int)pct, 0, 100);
    }

    // 5. Broadcast ESP-NOW packet at 20 Hz (every 50 ms) to M5Core2
    static unsigned long lastEspNowTx = 0;
    if (millis() - lastEspNowTx >= 50) {
        lastEspNowTx = millis();

        espNowPkt.robot_cmd = cameraAttention ? 1 : 0; // 1 = STOP, 0 = MOVE (Line Follow)
        espNowPkt.lid_state = (activeAngle == 95) ? 1 : 0;
        espNowPkt.attention = cameraAttention ? 1 : 0;
        espNowPkt.dwell_sec = cameraDwellSec;
        espNowPkt.trash_fill_pct = currentFillPercent;
        espNowPkt.seq++;

        esp_now_send(broadcastMac, (uint8_t *)&espNowPkt, sizeof(espNowPkt));
    }

    delay(2);
}
