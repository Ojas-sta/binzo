/**
 * ============================================================================
 * Project: M5Core2 Mecanum Smart Trash Can Robot (4x4 Omnidirectional Swerve Drive)
 * Subsystem: Autonomous Mecanum Drivetrain & Mission FSM
 * Architecture: Powered by cl0ud (https://github.com/Ojas-sta/cl0ud-brand-engineering)
 * Co-Engineered by: Ojas Singh <ojasctc@gmail.com>
 *
 * Hardware Stack:
 *   - M5Stack Core2 (ESP32)
 *   - M5Stack 4-Channel Encoder Motor Module (I2C 0x24) driving 4 Mecanum wheels
 *   - M5StickV / UnitV AI Camera (Kendryte K210 YOLOv2 Tracker via Port C Serial2: RX 13, TX 14)
 *   - 2x TCRT5000 IR Line Follower Sensors (Port B: GPIO 26 Left, GPIO 36 Right)
 *   - MG90S Mouth/Lid Servo (via 4EncoderMotor servo port or GPIO PWM)
 *   - RadioMaster Pocket (USB HID to Laptop) -> UDP over WiFi (Manual RC Override)
 *
 * Mission Flow:
 *   1. LINE PATROL: Follows closed-loop black track using TCRT5000 IR sensors.
 *   2. SIGHTING: M5 Camera detects person via YOLOv2 (conf >= 55%, 3 consecutive frames).
 *   3. APPROACH: Leaves line! Uses MC4 encoder odometry + M5 camera visual servoing (cx).
 *   4. OPEN MOUTH: Arrives in front of person (h >= 120px), halts, opens servo lid to 95°.
 *   5. AWAIT TRASH: Holds mouth open; detects trash deposit or 5s dwell timeout.
 *   6. CLOSE MOUTH: Closes servo lid to 0°, displays "THANK YOU!" on M5Core2 screen.
 *   7. RETURN: Drives in reverse / backtracks using encoder distance until IR detects track.
 *   8. RE-ACQUIRE: Squares up both IR sensors on black line & resumes patrol loop!
 * ============================================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <M5Unified.h>
#include <esp_system.h>
#include "M5Module4EncoderMotor.h"
#include "WeMultipleLineFollower.h"

// ----------------------------------------------------------------------------
// WIFI CONFIGURATION & NETWORK OVERRIDE
// ----------------------------------------------------------------------------
const char* WIFI_SSID     = "DPSIDNT";
const char* WIFI_PASSWORD = "Air@25683";
bool wifiOverridden       = false;

// ----------------------------------------------------------------------------
// NETWORK PORTS & TIMINGS
// ----------------------------------------------------------------------------
const uint16_t UDP_CONTROL_PORT   = 4210;
const uint16_t UDP_DISCOVERY_PORT = 4211;
const uint16_t UDP_LOG_PORT       = 4212;
const unsigned long FAILSAFE_TIMEOUT_MS = 1500;
const unsigned long BEACON_INTERVAL_MS  = 1000;
const unsigned long LOG_INTERVAL_MS     = 400;
const unsigned long UI_REFRESH_MS       = 200;

IPAddress laptopHostIp(0, 0, 0, 0);
unsigned long lastLogTxTime = 0;
RTC_NOINIT_ATTR uint32_t bootCount;

// ----------------------------------------------------------------------------
// HARDWARE PIN & PORT DEFINITIONS
// ----------------------------------------------------------------------------
// Port C (Blue Grove): M5StickV AI Camera Hardware UART2
const uint8_t PIN_M5_CAM_RX = 13; // M5Core2 RX2 <- M5 Camera Grove Pin 34 (TX)
const uint8_t PIN_M5_CAM_TX = 14; // M5Core2 TX2 -> M5 Camera Grove Pin 35 (RX)
HardwareSerial M5CamSerial(2);

// Port B (Black Grove): 5-Channel Multiple Line Follower (OneWire protocol)
const uint8_t PIN_LINE_FOLLOWER = 26; // Grove Port B Pin 26
WeMultipleLineFollower lineFollower(PIN_LINE_FOLLOWER);

// Mouth / Lid Servo (MG90S)
const uint8_t SERVO_CHANNEL      = 0;  // 4EncoderMotor servo port CH0
const uint8_t SERVO_ANGLE_CLOSED = 0;  // Closed mouth (0 deg)
const uint8_t SERVO_ANGLE_OPEN   = 95; // Open mouth (95 deg)

// Motor channel assignments (0 to 3) on M5Module4EncoderMotor
const uint8_t CH_FRONT_LEFT  = 0;
const uint8_t CH_FRONT_RIGHT = 1;
const uint8_t CH_REAR_LEFT   = 2;
const uint8_t CH_REAR_RIGHT  = 3;

// Direction multipliers (set to -1 if any wheel turns reverse)
const int8_t INV_FL = 1;
const int8_t INV_FR = 1;
const int8_t INV_RL = 1;
const int8_t INV_RR = 1;

// ----------------------------------------------------------------------------
// AUTONOMOUS MISSION TUNING & THRESHOLDS
// ----------------------------------------------------------------------------
const int8_t  PATROL_SPEED           = 42;    // Line cruising duty cycle (-127 to 127)
const int8_t  APPROACH_SPEED         = 48;    // Forward speed when approaching human
const float   APPROACH_YAW_KP        = 0.40f; // Visual servoing yaw gain: (cx - 160) * Kp
const int     APPROACH_DEADBAND      = 15;    // Deadband around center (+-15px)
const int     PERSON_STOP_H          = 120;   // Bounding box height to stop in front of person
const int     PERSON_MIN_CONF        = 55;    // Min detection confidence % to trigger approach
const int     PERSON_CONF_FRAMES     = 3;     // Consecutive sightings required
const unsigned long TRASH_DWELL_MS   = 5000;  // Time mouth stays open for trash deposit
const int8_t  RETURN_SPEED           = 42;    // Reverse backtracking speed to find line
const unsigned long PERSON_COOLDOWN  = 6500;  // Cooldown before serving another person

// ----------------------------------------------------------------------------
// AUTONOMOUS STATE MACHINE
// ----------------------------------------------------------------------------
enum AutoMissionState {
    AUTO_STANDBY = 0,       // Standby / Manual RC Override Active
    AUTO_LINE_PATROL,       // Cruising black line loop with IR sensors
    AUTO_APPROACH_PERSON,   // Off-line! Heading towards human via M5 visual servoing
    AUTO_OPEN_MOUTH,        // Arrived: stopping, opening mouth lid (servo 95°)
    AUTO_AWAIT_TRASH,       // Holding mouth open, detecting trash deposit / 5s dwell
    AUTO_CLOSE_MOUTH,       // Closing mouth lid (servo 0°), "Thank you!" on screen
    AUTO_RETURN_LINE,       // Backtracking reverse to black line loop using encoders
    AUTO_REACQUIRE_LINE     // Aligning both IR sensors squarely on line track
};

const char* autoStateNames[] = {
    "STANDBY", "LINE_PATROL", "APPROACH", "OPEN_MOUTH", "AWAIT_TRASH", 
    "CLOSE_MOUTH", "RETURN_LINE", "REACQUIRE"
};

AutoMissionState autoState = AUTO_LINE_PATROL;
bool isAutonomousMode = true; // Auto mission active by default

// Mission tracking variables
int32_t encStart0 = 0, encStart1 = 0, encStart2 = 0, encStart3 = 0;
unsigned long approachStartTime = 0;
unsigned long trashWaitStartTime = 0;
unsigned long returnStartTime = 0;
unsigned long lastPersonServedTime = 0;
int personConfidenceStreak = 0;

// Line sensor runtime state (5-Channel Multiple Line Follower)
uint16_t lineRawValue = 31;
int linePosition      = 0;  // -5..-1 (Right), 0 (Centered), +1..+5 (Left)
int lastKnownLineSide = 0;  // -1: Left, +1: Right
bool isLineDetected   = false;
bool onLineLeft       = false;
bool onLineRight      = false;
bool lineSensorBits[5] = {false, false, false, false, false};

// M5 AI Camera telemetry state
int m5PersonCount = 0;
int m5TrackId     = 0;
int m5Cx          = 160;
int m5Cy          = 120;
int m5W           = 0;
int m5H           = 0;
int m5Conf        = 0;
bool m5HumanDetected = false;
unsigned long lastM5Heartbeat = 0;
bool isM5CamOnline   = false;
String m5SerialBuffer = "";

// Camera Vision Line Following state (from K210 / temu_line_follower)
bool camMecanumActive = false;
unsigned long lastCamLineTime = 0;
int camVx = 0, camVy = 0, camYaw = 0;
int8_t camM0 = 0, camM1 = 0, camM2 = 0, camM3 = 0;

// ----------------------------------------------------------------------------
// GLOBAL OBJECTS & STATE
// ----------------------------------------------------------------------------
M5Module4EncoderMotor motorDriver;
WiFiUDP udpControl;
WiFiUDP udpDiscovery;

unsigned long lastPacketRxTime = 0;
unsigned long lastBeaconTxTime = 0;
unsigned long lastUiRefreshTime = 0;
uint32_t packetCount = 0;
uint32_t lastPacketCount = 0;
float currentPacketRate = 0.0f;

bool isEmergencyStop = false;
bool isArmed = false;
int16_t inVx = 0, inVy = 0, inYaw = 0, inThr = 0;
int8_t targetMotorPWM[4] = {0, 0, 0, 0};
int8_t actualMotorPWM[4] = {0, 0, 0, 0};
bool motorModuleFound = false;

// UI Colors (RGB565)
#define COLOR_BG          0x0842
#define COLOR_PANEL       0x18E3
#define COLOR_ACCENT      0x04FF
#define COLOR_GREEN_ARMED 0x07E0
#define COLOR_RED_ESTOP   0xF800
#define COLOR_ORANGE_FAIL 0xFD20
#define COLOR_TEXT_DIM    0x9CD3

// ----------------------------------------------------------------------------
// FORWARD DECLARATIONS
// ----------------------------------------------------------------------------
void setupWiFi();
void initMotorDriver();
void processIncomingUDP();
void sendDiscoveryBeacon();
void sendTelemetryUDP(const char* statusMsg);
void logDebugUDP(const char* tag, const char* fmt, ...);
void applyMotorSpeeds(int8_t m0, int8_t m1, int8_t m2, int8_t m3);
void driveMecanum(int vx, int vy, int w);
void stopAllMotors(const char* reason = "Stop");
void setMouthAngle(int angle);
void readLineSensors();
void updateM5Camera();
void parseM5Packet(const String& pkt);
void updateAutonomousMission();
void resetDepartureOdometry();
int32_t getAverageDepartureTicks();
void drawStaticUI();
void updateDisplay();
void handleTouch();
const char* getResetReasonStr(esp_reset_reason_t reason);

// ----------------------------------------------------------------------------
// RESET REASON HELPER
// ----------------------------------------------------------------------------
const char* getResetReasonStr(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXTERNAL_PIN";
        case ESP_RST_SW:        return "SW_RESET";
        case ESP_RST_PANIC:     return "PANIC_CRASH";
        case ESP_RST_INT_WDT:   return "INT_WATCHDOG";
        case ESP_RST_TASK_WDT:  return "TASK_WATCHDOG";
        case ESP_RST_WDT:       return "OTHER_WATCHDOG";
        case ESP_RST_DEEPSLEEP: return "DEEP_SLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

// ----------------------------------------------------------------------------
// SEND UDP DEBUG LOGS TO LAPTOP
// ----------------------------------------------------------------------------
void logDebugUDP(const char* tag, const char* fmt, ...) {
    char textBuf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(textBuf, sizeof(textBuf), fmt, args);
    va_end(args);

    Serial.printf("[%s] %s\n", tag, textBuf);
    if (WiFi.status() != WL_CONNECTED) return;

    char udpPacket[240];
    int vbat = M5.Power.getBatteryVoltage();
    snprintf(udpPacket, sizeof(udpPacket), "DBG,%lu,%d,%d,[%s] %s\n",
             millis(), vbat, isArmed ? 1 : 0, tag, textBuf);

    IPAddress target = (laptopHostIp[0] != 0) ? laptopHostIp : IPAddress(255, 255, 255, 255);
    udpControl.beginPacket(target, UDP_LOG_PORT);
    udpControl.write((const uint8_t*)udpPacket, strlen(udpPacket));
    udpControl.endPacket();
}

// ----------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------
void setup() {
    auto cfg = M5.config();
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);
    M5.Display.begin();
    M5.Display.setRotation(1); // Landscape
    M5.Display.fillScreen(BLACK);

    M5.Display.setTextDatum(top_center);
    M5.Display.setFont(&fonts::FreeSansBold12pt7b);
    M5.Display.setTextColor(COLOR_ACCENT);
    M5.Display.drawString("BINZO SMART TRASH CAN", 160, 20);

    M5.Display.setFont(&fonts::Font0);
    M5.Display.setTextColor(0x07FF); // Cyan
    M5.Display.drawString("POWERED BY cl0ud | Co-engineered by Ojas Singh", 160, 42);

    M5.Display.setFont(&fonts::FreeSans9pt7b);
    M5.Display.setTextColor(WHITE);
    M5.Display.drawString("Initializing hardware & camera...", 160, 68);

    Serial.println("\n=============================================");
    Serial.println("  BINZO M5Core2 Mecanum Smart Trash Can Robot");
    Serial.println("  Architecture: Powered by cl0ud");
    Serial.println("  Co-Engineered by: Ojas Singh <ojasctc@gmail.com>");
    Serial.println("=============================================");

    bootCount++;
    esp_reset_reason_t rstReason = esp_reset_reason();
    Serial.printf("[SYSTEM] Boot #%u | Reset Reason: %s\n", bootCount, getResetReasonStr(rstReason));

    // 1. Initialize M5 Camera Hardware UART2 on Port C (RX=13, TX=14)
    M5CamSerial.begin(115200, SERIAL_8N1, PIN_M5_CAM_RX, PIN_M5_CAM_TX);
    Serial.printf("[M5_CAM] Serial2 initialized on Port C (RX=%d, TX=%d) @ 115200 baud\n",
                  PIN_M5_CAM_RX, PIN_M5_CAM_TX);

    // 2. Initialize 5-Channel Multiple Line Follower on Port B (GPIO 26)
    lineFollower.reset(PIN_LINE_FOLLOWER);
    Serial.printf("[LINE] 5-Channel Multiple Line Follower initialized on Port B (Pin %d)\n", 
                  PIN_LINE_FOLLOWER);

    // 3. Initialize 4-Encoder Motor Driver Module (I2C 0x24)
    initMotorDriver();

    // 4. Initial Mouth Lid Position: Closed
    setMouthAngle(SERVO_ANGLE_CLOSED);

    // 5. Connect WiFi for UDP Remote & Telemetry (with Instant Override & Timeout)
    setupWiFi();

    // 6. Start UDP Listeners if connected
    if (WiFi.status() == WL_CONNECTED) {
        udpControl.begin(UDP_CONTROL_PORT);
        udpDiscovery.begin(UDP_DISCOVERY_PORT);
        Serial.printf("[UDP] Listening for control packets on port %d\n", UDP_CONTROL_PORT);
    } else {
        Serial.println("[UDP] WiFi offline / overridden. Autonomous patrol active.");
    }

    drawStaticUI();
    lastPacketRxTime = millis();
}

// ----------------------------------------------------------------------------
// INITIALIZE 4-ENCODER MOTOR MODULE
// ----------------------------------------------------------------------------
void initMotorDriver() {
    Serial.println("[MOTOR] Initializing 4EncoderMotor module on I2C (0x24)...");
    
    uint8_t attempts = 0;
    while (!motorDriver.begin(&Wire1, MODULE_4ENCODERMOTOR_ADDR, 21, 22)) {
        attempts++;
        Serial.printf("[MOTOR] Driver init failed (attempt %d). Retrying...\n", attempts);
        M5.Display.fillRect(0, 80, 320, 30, BLACK);
        M5.Display.setTextColor(COLOR_RED_ESTOP);
        M5.Display.drawString("Motor Module NOT Detected!", 160, 90);
        delay(600);
        if (attempts > 5) {
            Serial.println("[MOTOR] Warning: Motor driver module not responding.");
            break;
        }
    }

    if (attempts <= 5) {
        motorModuleFound = true;
        Serial.println("[MOTOR] 4-Channel Encoder Motor Module online!");
        Wire1.setClock(400000);
        Wire1.setTimeOut(25);

        uint8_t fw = 0;
        if (motorDriver.getFirewareVersion(&fw)) {
            Serial.printf("[MOTOR] Firmware Version: %d\n", fw);
            if (fw >= 3) {
                for (uint8_t i = 0; i < 4; i++) {
                    motorDriver.setSoftStartAndStop(i, false);
                }
            }
        }

        // Configure all 4 channels to direct duty speed control and reset encoders
        for (uint8_t i = 0; i < 4; i++) {
            motorDriver.setMode(i, NORMAL_MODE);
            motorDriver.setMotorSpeed(i, 0);
            motorDriver.setEncoderValue(i, 0);
        }
    }
}

// ----------------------------------------------------------------------------
// SETUP WIFI WITH INSTANT TOUCH/BUTTON OVERRIDE & TIMEOUT
// ----------------------------------------------------------------------------
void setupWiFi() {
    M5.Display.fillRect(0, 95, 320, 145, BLACK);
    M5.Display.setFont(&fonts::FreeSansBold9pt7b);
    M5.Display.setTextColor(COLOR_ACCENT);
    M5.Display.setTextDatum(top_center);
    M5.Display.drawString("CONNECTING TO WIFI", 160, 100);

    M5.Display.setFont(&fonts::Font0);
    M5.Display.setTextColor(WHITE);
    M5.Display.drawString(String("SSID: ") + WIFI_SSID, 160, 124);

    // Draw prominent Touch Override Button
    M5.Display.fillRoundRect(16, 172, 288, 38, 8, 0x8800);
    M5.Display.setTextColor(WHITE);
    M5.Display.setFont(&fonts::FreeSansBold9pt7b);
    M5.Display.setTextDatum(middle_center);
    M5.Display.drawString("[ TAP TO SKIP / OVERRIDE ]", 160, 191);

    Serial.printf("\n[WiFi] Connecting to SSID: '%s'...\n", WIFI_SSID);
    Serial.println("[WiFi] Tap screen or press buttons to OVERRIDE and run OFFLINE.");

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long wifiStart = millis();
    const unsigned long WIFI_TIMEOUT_MS = 10000; // 10-second automatic timeout override
    int dots = 0;

    while (WiFi.status() != WL_CONNECTED) {
        M5.update();

        // 1. Manual User Override (Touch anywhere or Button A/B/C)
        auto touchDetail = M5.Touch.getDetail();
        if (touchDetail.isPressed() || M5.BtnA.isPressed() || M5.BtnB.isPressed() || M5.BtnC.isPressed()) {
            wifiOverridden = true;
            Serial.println("\n[WiFi] User OVERRIDE triggered! Bypassing WiFi connection.");
            M5.Speaker.tone(1200, 100);

            M5.Display.fillRect(0, 95, 320, 145, BLACK);
            M5.Display.setTextColor(COLOR_ORANGE_FAIL);
            M5.Display.setFont(&fonts::FreeSansBold9pt7b);
            M5.Display.drawString("WIFI OVERRIDDEN!", 160, 125);
            M5.Display.setFont(&fonts::Font0);
            M5.Display.setTextColor(WHITE);
            M5.Display.drawString("Running in OFFLINE Autonomous Mode", 160, 155);
            delay(700);
            return;
        }

        // 2. Automatic Timeout Override
        if (millis() - wifiStart > WIFI_TIMEOUT_MS) {
            wifiOverridden = true;
            Serial.println("\n[WiFi] Timeout reached. Overriding network connection -> OFFLINE MODE.");
            M5.Speaker.tone(800, 120);

            M5.Display.fillRect(0, 95, 320, 145, BLACK);
            M5.Display.setTextColor(COLOR_ORANGE_FAIL);
            M5.Display.setFont(&fonts::FreeSansBold9pt7b);
            M5.Display.drawString("WIFI TIMEOUT -> OFFLINE", 160, 125);
            M5.Display.setFont(&fonts::Font0);
            M5.Display.setTextColor(WHITE);
            M5.Display.drawString("Standalone Autonomous Mode Active", 160, 155);
            delay(700);
            return;
        }

        delay(150);
        Serial.print(".");
        dots++;
        String dotStr = "";
        for (int i = 0; i < (dots % 6); i++) dotStr += ".";
        M5.Display.fillRect(0, 140, 320, 24, BLACK);
        M5.Display.setFont(&fonts::Font0);
        M5.Display.setTextColor(COLOR_ACCENT);
        int remainingSec = (int)((WIFI_TIMEOUT_MS - (millis() - wifiStart)) / 1000);
        M5.Display.drawString(String("Connecting") + dotStr + " (" + String(remainingSec) + "s)", 160, 145);
    }

    WiFi.setSleep(false);
    wifiOverridden = false;
    Serial.printf("\n[WiFi] Connected! IP: %s | RSSI: %d dBm\n", 
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());

    M5.Display.fillRect(0, 95, 320, 145, BLACK);
    M5.Display.setFont(&fonts::FreeSansBold9pt7b);
    M5.Display.setTextColor(COLOR_GREEN_ARMED);
    M5.Display.drawString("WiFi Connected!", 160, 125);
    M5.Display.setFont(&fonts::Font0);
    M5.Display.setTextColor(WHITE);
    M5.Display.drawString(String("IP: ") + WiFi.localIP().toString() + " (" + String(WiFi.RSSI()) + " dBm)", 160, 155);
    M5.Speaker.tone(1800, 120);
    delay(600);
}

// ----------------------------------------------------------------------------
// MAIN LOOP
// ----------------------------------------------------------------------------
void loop() {
    M5.update();
    unsigned long now = millis();

    // 0. Dynamic UDP socket state management (handles connect / reconnect / override)
    static bool udpStarted = false;
    if (WiFi.status() == WL_CONNECTED) {
        if (!udpStarted) {
            udpControl.begin(UDP_CONTROL_PORT);
            udpDiscovery.begin(UDP_DISCOVERY_PORT);
            udpStarted = true;
            Serial.printf("[UDP] WiFi connected. Listening on port %d\n", UDP_CONTROL_PORT);
        }
    } else {
        udpStarted = false;
    }

    // 1. Process touch input (E-stop & Mode Toggle)
    handleTouch();

    // 2. Poll M5StickV AI Camera telemetry from Port C UART2
    updateM5Camera();

    // 3. Poll TCRT5000 IR line sensors from Port B
    readLineSensors();

    // 4. Process incoming UDP packets (Manual RadioMaster RC control)
    processIncomingUDP();

    // 5. Execution Arbitration: Manual Radio vs Autonomous Trash Mission
    if (!isEmergencyStop) {
        bool manualRadioActive = isArmed && (inThr > 5 || abs(inVx) > 10 || abs(inVy) > 10 || abs(inYaw) > 10);

        if (manualRadioActive) {
            // Manual Radio takes direct control
            isAutonomousMode = false;
        } else if (isAutonomousMode) {
            // Autonomous Human-Tracking & Trash Mission executes
            updateAutonomousMission();
        } else {
            // Idle standby
            stopAllMotors("Standby");
        }
    } else {
        stopAllMotors("Emergency Stop Active");
    }

    // 6. Broadcast Discovery Beacon every 1 second
    if (now - lastBeaconTxTime >= BEACON_INTERVAL_MS) {
        lastBeaconTxTime = now;
        sendDiscoveryBeacon();
        currentPacketRate = (float)(packetCount - lastPacketCount) / (BEACON_INTERVAL_MS / 1000.0f);
        lastPacketCount = packetCount;
    }

    // 7. Stream Telemetry to Laptop (2.5 Hz)
    if (now - lastLogTxTime >= LOG_INTERVAL_MS) {
        lastLogTxTime = now;
        String statusMsg = isEmergencyStop ? "[ESTOP] Stopped" : 
                           (isAutonomousMode ? (String("[AUTO] ") + autoStateNames[autoState]) : "[MANUAL] RC Active");
        sendTelemetryUDP(statusMsg.c_str());
    }

    // 8. Refresh UI on Screen (5 Hz)
    if (now - lastUiRefreshTime >= UI_REFRESH_MS) {
        lastUiRefreshTime = now;
        updateDisplay();
    }

    // 9. Serial Telemetry Stream (2.5 Hz)
    static unsigned long lastSerialPrintTime = 0;
    if (now - lastSerialPrintTime >= 400) {
        lastSerialPrintTime = now;
        Serial.printf("[AUTO] State: %-11s | LineRaw: %2u, Pos: %2d [S:%d%d%d%d%d] | Cam: human=%d cx=%3d h=%3d | Mot: FL=%3d FR=%3d\n",
                      autoStateNames[autoState], lineRawValue, linePosition,
                      lineSensorBits[0], lineSensorBits[1], lineSensorBits[2], lineSensorBits[3], lineSensorBits[4],
                      m5HumanDetected, m5Cx, m5H,
                      actualMotorPWM[0], actualMotorPWM[1]);
    }

    delay(1);
}

// ----------------------------------------------------------------------------
// AUTONOMOUS HUMAN-TRACKING & TRASH MISSION ENGINE
// ----------------------------------------------------------------------------
void updateAutonomousMission() {
    unsigned long currentMillis = millis();

    switch (autoState) {
        // ---------------------------------------------------------------------
        // 1. LINE FOLLOWING LOOP
        // ---------------------------------------------------------------------
        case AUTO_LINE_PATROL: {
            // Check if M5 AI Camera spots a person (confidence & streak filter)
            bool canServePerson = (m5HumanDetected && m5Conf >= PERSON_MIN_CONF && 
                                  (currentMillis - lastPersonServedTime > PERSON_COOLDOWN));

            if (canServePerson) {
                personConfidenceStreak++;
                if (personConfidenceStreak >= PERSON_CONF_FRAMES) {
                    // Confirmed person! Record departure odometry and leave line
                    stopAllMotors("Human spotted! Leaving line");
                    resetDepartureOdometry();
                    approachStartTime = currentMillis;
                    personConfidenceStreak = 0;
                    M5.Speaker.tone(1400, 150); // Friendly alert chirp
                    logDebugUDP("MISSION", "Human sighted at cx=%d, h=%d! Leaving line to approach...", m5Cx, m5H);
                    autoState = AUTO_APPROACH_PERSON;
                    break;
                }
            } else {
                personConfidenceStreak = 0;
            }

            // Camera Vision Line Follower vs 5-Channel Multiple Line Follower
            if (camMecanumActive && (currentMillis - lastCamLineTime < 350)) {
                // Actively driven by camera's vision movement vectors (handled in parseM5Packet)
            } else if (isLineDetected) {
                // 5-Channel Multiple Line Follower (OneWire Pin 26)
                // linePosition: -4 (far right) to +4 (far left), 0 centered
                int steer = constrain(-linePosition * 7, -32, 32);
                driveMecanum(PATROL_SPEED, 0, steer);
            } else {
                // Both off line: gently turn back to last known side
                int steer = (lastKnownLineSide == 1) ? -28 : 28;
                driveMecanum(PATROL_SPEED - 12, 0, steer);
            }
            break;
        }

        // ---------------------------------------------------------------------
        // 2. LEAVE LINE & APPROACH PERSON (Visual Servoing via M5 Camera cx)
        // ---------------------------------------------------------------------
        case AUTO_APPROACH_PERSON: {
            // Check arrival condition: person is right in front of trash can!
            bool arrivedAtPerson = (m5H >= PERSON_STOP_H);

            if (arrivedAtPerson) {
                stopAllMotors("Arrived at person");
                logDebugUDP("MISSION", "Arrived at person (h=%d)! Opening mouth...", m5H);
                autoState = AUTO_OPEN_MOUTH;
                break;
            }

            // Safety timeout: If approach exceeds 8 seconds or target completely vanished
            if (currentMillis - approachStartTime > 8000 || (!m5HumanDetected && currentMillis - lastM5Heartbeat > 2500)) {
                stopAllMotors("Approach timeout / target lost");
                logDebugUDP("MISSION", "Target lost or timed out. Backtracking to line...");
                returnStartTime = currentMillis;
                autoState = AUTO_RETURN_LINE;
                break;
            }

            // Visual Servoing Yaw Alignment: Center person horizontally in frame (cx = 160)
            int errorX = m5Cx - 160;
            int rot = 0;
            if (abs(errorX) > APPROACH_DEADBAND) {
                rot = constrain((int)(errorX * APPROACH_YAW_KP), -38, 38);
            }

            // Forward speed tapering: slow down smoothly as human grows in frame
            int fwdSpeed = APPROACH_SPEED;
            if (m5H > 75) {
                fwdSpeed = map(constrain(m5H, 75, PERSON_STOP_H), 75, PERSON_STOP_H, APPROACH_SPEED, 22);
            }

            // Mecanum drive: Forward + Yaw rotation
            driveMecanum(fwdSpeed, 0, rot);
            break;
        }

        // ---------------------------------------------------------------------
        // 3. OPEN MOUTH (MG90S Servo Lid to 95°)
        // ---------------------------------------------------------------------
        case AUTO_OPEN_MOUTH:
            stopAllMotors("Mouth opening");
            setMouthAngle(SERVO_ANGLE_OPEN); // Pop open lid
            trashWaitStartTime = currentMillis;
            M5.Speaker.tone(1800, 200);      // Welcome tone
            logDebugUDP("MISSION", "Mouth wide open (95 deg). Awaiting trash deposit...");
            autoState = AUTO_AWAIT_TRASH;
            break;

        // ---------------------------------------------------------------------
        // 4. AWAIT TRASH DEPOSIT
        // ---------------------------------------------------------------------
        case AUTO_AWAIT_TRASH: {
            stopAllMotors("Awaiting trash");

            // Service completed condition:
            // a) Dwell timeout (5 seconds)
            bool dwellFinished = (currentMillis - trashWaitStartTime >= TRASH_DWELL_MS);
            // b) User threw trash and backed away (box height drops)
            bool userSteppedBack = (m5H < 70) && (currentMillis - trashWaitStartTime > 2500);

            if (dwellFinished || userSteppedBack) {
                logDebugUDP("MISSION", "Trash deposit finished (dwell=%d, back=%d). Closing mouth...",
                            dwellFinished, userSteppedBack);
                autoState = AUTO_CLOSE_MOUTH;
            }
            break;
        }

        // ---------------------------------------------------------------------
        // 5. CLOSE MOUTH (MG90S Servo Lid to 0°)
        // ---------------------------------------------------------------------
        case AUTO_CLOSE_MOUTH:
            setMouthAngle(SERVO_ANGLE_CLOSED); // Seal lid
            M5.Speaker.tone(2200, 100);        // "Thank you" chirp
            returnStartTime = currentMillis;
            lastPersonServedTime = currentMillis;
            logDebugUDP("MISSION", "Mouth closed. Backtracking to black line loop...");
            autoState = AUTO_RETURN_LINE;
            break;

        // ---------------------------------------------------------------------
        // 6. RETURN TO LINE (Backtracking & 5-Channel Line Detection)
        // ---------------------------------------------------------------------
        case AUTO_RETURN_LINE: {
            // Check if 5-channel line sensor has crossed the black line
            if (isLineDetected) {
                stopAllMotors("Line detected on reverse");
                logDebugUDP("MISSION", "Black line detected! Squaring up...");
                autoState = AUTO_REACQUIRE_LINE;
                break;
            }

            // Safety timeout: If reverse runs > 7.5 seconds without hitting line
            if (currentMillis - returnStartTime > 7500) {
                stopAllMotors("Return timeout");
                logDebugUDP("MISSION", "Reverse return timed out. Re-engaging patrol search...");
                autoState = AUTO_LINE_PATROL;
                break;
            }

            // Drive in reverse along incoming path
            driveMecanum(-RETURN_SPEED, 0, 0);
            break;
        }

        // ---------------------------------------------------------------------
        // 7. RE-ACQUIRE LINE TRACK
        // ---------------------------------------------------------------------
        case AUTO_REACQUIRE_LINE: {
            // Square up line follower centered on the line (linePosition == 0 or +/-1)
            if (isLineDetected && abs(linePosition) <= 1) {
                stopAllMotors("Centered on line");
                logDebugUDP("MISSION", "Re-aligned squarely on line track. Resuming patrol loop!");
                M5.Speaker.tone(1200, 80);
                autoState = AUTO_LINE_PATROL;
                break;
            } else if (linePosition > 1) {
                // Line is to the left -> rotate CCW
                driveMecanum(0, 0, -20);
            } else if (linePosition < -1) {
                // Line is to the right -> rotate CW
                driveMecanum(0, 0, 20);
            } else {
                // Creep forward into track
                driveMecanum(20, 0, 0);
            }
            break;
        }

        default:
            autoState = AUTO_LINE_PATROL;
            break;
    }
}

// ----------------------------------------------------------------------------
// ODOMETRY HELPERS (MC4 ENCODER MOTORS)
// ----------------------------------------------------------------------------
void resetDepartureOdometry() {
    if (motorModuleFound) {
        encStart0 = motorDriver.getEncoderValue(0);
        encStart1 = motorDriver.getEncoderValue(1);
        encStart2 = motorDriver.getEncoderValue(2);
        encStart3 = motorDriver.getEncoderValue(3);
    }
}

int32_t getAverageDepartureTicks() {
    if (!motorModuleFound) return 0;
    int32_t d0 = abs(motorDriver.getEncoderValue(0) - encStart0);
    int32_t d1 = abs(motorDriver.getEncoderValue(1) - encStart1);
    int32_t d2 = abs(motorDriver.getEncoderValue(2) - encStart2);
    int32_t d3 = abs(motorDriver.getEncoderValue(3) - encStart3);
    return (d0 + d1 + d2 + d3) / 4;
}

// ----------------------------------------------------------------------------
// M5 AI CAMERA PARSER & TELEMETRY
// ----------------------------------------------------------------------------
void updateM5Camera() {
    while (M5CamSerial.available() > 0) {
        char c = (char)M5CamSerial.read();
        if (c == '\n' || c == '\r') {
            if (m5SerialBuffer.length() > 0) {
                parseM5Packet(m5SerialBuffer);
                m5SerialBuffer = "";
            }
        } else if (m5SerialBuffer.length() < 120) {
            m5SerialBuffer += c;
        }
    }
    isM5CamOnline = (millis() - lastM5Heartbeat < 2500);
}

void parseM5Packet(const String& pkt) {
    if (pkt.startsWith("BINZO:P,")) {
        // Format: BINZO:P,<count>,<id>,<cx>,<cy>,<w>,<h>,<conf> or BINZO:P,0
        int c1 = pkt.indexOf(',');
        int c2 = pkt.indexOf(',', c1 + 1);
        if (c2 == -1) {
            m5PersonCount = pkt.substring(c1 + 1).toInt();
            m5HumanDetected = (m5PersonCount > 0);
            lastM5Heartbeat = millis();
        } else {
            int c3 = pkt.indexOf(',', c2 + 1);
            int c4 = pkt.indexOf(',', c3 + 1);
            int c5 = pkt.indexOf(',', c4 + 1);
            int c6 = pkt.indexOf(',', c5 + 1);
            int c7 = pkt.indexOf(',', c6 + 1);

            m5PersonCount = pkt.substring(c1 + 1, c2).toInt();
            m5TrackId     = pkt.substring(c2 + 1, c3).toInt();
            m5Cx          = pkt.substring(c3 + 1, c4).toInt();
            m5Cy          = pkt.substring(c4 + 1, c5).toInt();
            m5W           = pkt.substring(c5 + 1, c6).toInt();
            m5H           = (c7 != -1) ? pkt.substring(c6 + 1, c7).toInt() : pkt.substring(c6 + 1).toInt();
            m5Conf        = (c7 != -1) ? pkt.substring(c7 + 1).toInt() : 80;

            m5HumanDetected = (m5PersonCount > 0);
            lastM5Heartbeat = millis();
        }
    } else if (pkt.startsWith("HUMAN,1")) {
        m5HumanDetected = true;
        m5PersonCount = 1;
        lastM5Heartbeat = millis();
    } else if (pkt.startsWith("HUMAN,0")) {
        m5HumanDetected = false;
        m5PersonCount = 0;
        lastM5Heartbeat = millis();
    } else if (pkt.startsWith("BINZO:MEC,")) {
        // Format: BINZO:MEC,<vx>,<vy>,<yaw>,<m0>,<m1>,<m2>,<m3>
        int c1 = pkt.indexOf(',');
        int c2 = pkt.indexOf(',', c1 + 1);
        int c3 = pkt.indexOf(',', c2 + 1);
        int c4 = pkt.indexOf(',', c3 + 1);
        int c5 = pkt.indexOf(',', c4 + 1);
        int c6 = pkt.indexOf(',', c5 + 1);
        int c7 = pkt.indexOf(',', c6 + 1);

        if (c6 != -1) {
            camVx  = pkt.substring(c1 + 1, c2).toInt();
            camVy  = pkt.substring(c2 + 1, c3).toInt();
            camYaw = pkt.substring(c3 + 1, c4).toInt();
            camM0  = (int8_t)pkt.substring(c4 + 1, c5).toInt();
            camM1  = (int8_t)pkt.substring(c5 + 1, c6).toInt();
            camM2  = (c7 != -1) ? (int8_t)pkt.substring(c6 + 1, c7).toInt() : (int8_t)pkt.substring(c6 + 1).toInt();
            camM3  = (c7 != -1) ? (int8_t)pkt.substring(c7 + 1).toInt() : 0;

            camMecanumActive = true;
            lastCamLineTime = millis();
            lastM5Heartbeat = millis();

            if (isAutonomousMode && !isEmergencyStop && autoState == AUTO_LINE_PATROL) {
                applyMotorSpeeds(camM0 * INV_FL, camM1 * INV_FR, camM2 * INV_RL, camM3 * INV_RR);
            }
        }
    }
}

// ----------------------------------------------------------------------------
// LINE SENSOR READING (5-CHANNEL MULTIPLE LINE FOLLOWER ON PORT B)
// ----------------------------------------------------------------------------
void readLineSensors() {
    lineRawValue = lineFollower.readSensor();

    // S1 (bit 0) to S5 (bit 4): 0 = over black line, 1 = white background
    for (int i = 0; i < 5; i++) {
        lineSensorBits[i] = ((lineRawValue & (1 << i)) == 0);
    }

    switch (lineRawValue) {
        case 15: linePosition =  4; lastKnownLineSide = 1; break;  // 01111: S5 (far left)
        case  7: linePosition =  3; lastKnownLineSide = 1; break;  // 00111: S4, S5
        case 23: linePosition =  2; lastKnownLineSide = 1; break;  // 10111: S4
        case 19: linePosition =  1; lastKnownLineSide = 1; break;  // 10011: S3, S4
        case 27: linePosition =  0; break;                         // 11011: S3 (centered)
        case 25: linePosition = -1; lastKnownLineSide = -1; break; // 11001: S2, S3
        case 29: linePosition = -2; lastKnownLineSide = -1; break; // 11101: S2
        case 28: linePosition = -3; lastKnownLineSide = -1; break; // 11100: S1, S2
        case 30: linePosition = -4; lastKnownLineSide = -1; break; // 11110: S1 (far right)
        case 31: // All off line
            linePosition = (lastKnownLineSide == 1) ? 5 : -5;
            break;
        default:
            if (lineSensorBits[2]) linePosition = 0;
            break;
    }

    isLineDetected = (lineRawValue != 31 && lineRawValue != 0);
    onLineLeft  = lineSensorBits[3] || lineSensorBits[4];
    onLineRight = lineSensorBits[0] || lineSensorBits[1];
}

// ----------------------------------------------------------------------------
// MOUTH / LID SERVO ACTUATION
// ----------------------------------------------------------------------------
void setMouthAngle(int angle) {
    angle = constrain(angle, 0, 180);
    if (motorModuleFound) {
        motorDriver.setServoAngle(SERVO_CHANNEL, (uint8_t)angle);
    }
}

// ----------------------------------------------------------------------------
// MECANUM KINEMATICS & MOTOR DRIVE
// ----------------------------------------------------------------------------
void driveMecanum(int vx, int vy, int w) {
    // vx: Forward/Backward (-127 to 127)
    // vy: Strafe Right/Left (-127 to 127)
    // w:  Yaw Rotation (-127 to 127)
    int m0 = vx + vy + w;
    int m1 = vx - vy - w;
    int m2 = vx - vy + w;
    int m3 = vx + vy - w;

    int maxVal = max(max(abs(m0), abs(m1)), max(abs(m2), abs(m3)));
    if (maxVal > 127) {
        m0 = (m0 * 127) / maxVal;
        m1 = (m1 * 127) / maxVal;
        m2 = (m2 * 127) / maxVal;
        m3 = (m3 * 127) / maxVal;
    }

    applyMotorSpeeds(m0 * INV_FL, m1 * INV_FR, m2 * INV_RL, m3 * INV_RR);
}

void applyMotorSpeeds(int8_t m0, int8_t m1, int8_t m2, int8_t m3) {
    actualMotorPWM[CH_FRONT_LEFT]  = m0;
    actualMotorPWM[CH_FRONT_RIGHT] = m1;
    actualMotorPWM[CH_REAR_LEFT]   = m2;
    actualMotorPWM[CH_REAR_RIGHT]  = m3;

    if (!motorModuleFound) return;

    static int8_t lastSentPWM[4] = {-128, -128, -128, -128};
    static unsigned long lastSentTime = 0;
    bool forceKeepalive = (millis() - lastSentTime > 120);

    bool changed = (m0 != lastSentPWM[0] || m1 != lastSentPWM[1] || 
                    m2 != lastSentPWM[2] || m3 != lastSentPWM[3]);

    if (forceKeepalive || changed) {
        motorDriver.setMotorSpeed(CH_FRONT_LEFT, m0);
        motorDriver.setMotorSpeed(CH_FRONT_RIGHT, m1);
        motorDriver.setMotorSpeed(CH_REAR_LEFT, m2);
        motorDriver.setMotorSpeed(CH_REAR_RIGHT, m3);

        lastSentPWM[0] = m0;
        lastSentPWM[1] = m1;
        lastSentPWM[2] = m2;
        lastSentPWM[3] = m3;
        lastSentTime = millis();
    }
}

void stopAllMotors(const char* reason) {
    for (uint8_t i = 0; i < 4; i++) {
        targetMotorPWM[i] = 0;
        actualMotorPWM[i] = 0;
    }
    if (motorModuleFound) {
        for (uint8_t i = 0; i < 4; i++) {
            motorDriver.setMotorSpeed(i, 0);
        }
    }
}

// ----------------------------------------------------------------------------
// TOUCH INPUT (ESTOP & AUTO MODE TOGGLE)
// ----------------------------------------------------------------------------
void handleTouch() {
    static unsigned long lastTouchTime = 0;
    unsigned long now = millis();

    if (M5.Touch.getCount() > 0) {
        auto detail = M5.Touch.getDetail(0);
        if (now - lastTouchTime > 400 && detail.wasPressed()) {
            lastTouchTime = now;

            // Bottom banner (Y >= 195): Emergency Stop toggle
            if (detail.y >= 195) {
                isEmergencyStop = !isEmergencyStop;
                if (isEmergencyStop) {
                    stopAllMotors("E-Stop Pressed");
                    M5.Speaker.tone(1000, 200);
                } else {
                    M5.Speaker.tone(2000, 100);
                }
            } 
            // Top header bar (Y <= 26 && X >= 170): Network Override / Reconnect Toggle
            else if (detail.y <= 26 && detail.x >= 170) {
                if (WiFi.status() == WL_CONNECTED) {
                    wifiOverridden = true;
                    WiFi.disconnect();
                    Serial.println("[WiFi] User tapped NET status: Network OVERRIDDEN to offline mode.");
                    M5.Speaker.tone(900, 100);
                } else {
                    wifiOverridden = false;
                    WiFi.disconnect();
                    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
                    Serial.printf("[WiFi] User tapped NET status: Reconnecting to '%s'...\n", WIFI_SSID);
                    M5.Speaker.tone(1500, 100);
                }
            }
            // Center card area (26 < Y < 190): Toggle Autonomous Mode vs Manual RC
            else if (detail.y < 190) {
                isAutonomousMode = !isAutonomousMode;
                if (isAutonomousMode) {
                    autoState = AUTO_LINE_PATROL;
                    M5.Speaker.tone(1500, 100);
                } else {
                    stopAllMotors("Manual Mode Selected");
                    M5.Speaker.tone(800, 100);
                }
            }
        }
    }
}

// ----------------------------------------------------------------------------
// PROCESS INCOMING UDP CONTROL PACKETS (MANUAL RC OVERRIDE)
// ----------------------------------------------------------------------------
void processIncomingUDP() {
    char rxBuffer[128];
    bool gotPacket = false;

    while (udpControl.parsePacket() > 0) {
        laptopHostIp = udpControl.remoteIP();
        int len = udpControl.read(rxBuffer, sizeof(rxBuffer) - 1);
        if (len > 0) {
            rxBuffer[len] = '\0';
            gotPacket = true;
        }
    }

    if (!gotPacket) return;

    if (strncmp(rxBuffer, "TRASH,", 6) == 0) {
        uint32_t seq = 0;
        int arm = 0;
        int vx = 0, vy = 0, yaw = 0, thr = 0;
        int m0 = 0, m1 = 0, m2 = 0, m3 = 0;

        int parsed = sscanf(rxBuffer, "TRASH,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                            &seq, &arm, &vx, &vy, &yaw, &thr, &m0, &m1, &m2, &m3);

        if (parsed == 10) {
            lastPacketRxTime = millis();
            packetCount++;

            inVx  = vx;
            inVy  = vy;
            inYaw = yaw;
            inThr = thr;
            isArmed = (arm == 1);

            // If manual radio sticks are moved, manual control drives
            if (!isAutonomousMode && isArmed && !isEmergencyStop) {
                if (thr <= 0 || (thr < 5 && abs(vx) < 5 && abs(vy) < 5 && abs(yaw) < 5)) {
                    stopAllMotors("Neutral Sticks");
                } else {
                    applyMotorSpeeds(constrain(m0 * INV_FL, -127, 127),
                                     constrain(m1 * INV_FR, -127, 127),
                                     constrain(m2 * INV_RL, -127, 127),
                                     constrain(m3 * INV_RR, -127, 127));
                }
            }
        }
    }
}

// ----------------------------------------------------------------------------
// TELEMETRY & BEACON TRANSMISSION
// ----------------------------------------------------------------------------
void sendDiscoveryBeacon() {
    if (WiFi.status() != WL_CONNECTED) return;
    char beaconBuf[128];
    snprintf(beaconBuf, sizeof(beaconBuf), 
             "{\"device\":\"M5_TRASHCAN\",\"mode\":\"%s\",\"state\":\"%s\"}",
             isAutonomousMode ? "AUTO" : "MANUAL", autoStateNames[autoState]);

    udpDiscovery.beginPacket(IPAddress(255, 255, 255, 255), UDP_DISCOVERY_PORT);
    udpDiscovery.write((const uint8_t*)beaconBuf, strlen(beaconBuf));
    udpDiscovery.endPacket();
}

void sendTelemetryUDP(const char* logMsg) {
    if (WiFi.status() != WL_CONNECTED || laptopHostIp[0] == 0) return;
    char logBuf[180];
    int vbat = M5.Power.getBatteryVoltage();
    snprintf(logBuf, sizeof(logBuf), 
             "M5LOG,%lu,%d,%d,%d,%s,Cam=[%d,cx=%d,h=%d],Msg=%s\n",
             millis(), vbat, isAutonomousMode ? 1 : 0, isEmergencyStop ? 1 : 0,
             autoStateNames[autoState], m5PersonCount, m5Cx, m5H, logMsg);

    udpControl.beginPacket(laptopHostIp, UDP_LOG_PORT);
    udpControl.write((const uint8_t*)logBuf, strlen(logBuf));
    udpControl.endPacket();
}

// ----------------------------------------------------------------------------
// STATIC UI
// ----------------------------------------------------------------------------
void drawStaticUI() {
    M5.Display.fillScreen(COLOR_BG);
    M5.Display.fillRect(0, 0, 320, 24, 0x10A2);

    // Left Card: Autonomous Navigation & Face Box
    M5.Display.fillRoundRect(8, 70, 148, 122, 6, COLOR_PANEL);
    M5.Display.drawRoundRect(8, 70, 148, 122, 6, 0x3186);

    // Right Card: Encoders & Sensors Box
    M5.Display.fillRoundRect(164, 70, 148, 122, 6, COLOR_PANEL);
    M5.Display.drawRoundRect(164, 70, 148, 122, 6, 0x3186);
}

// ----------------------------------------------------------------------------
// DYNAMIC UI UPDATE (5 HZ) - INTERACTIVE ROBOT FACE & STATUS
// ----------------------------------------------------------------------------
void updateDisplay() {
    // 1. TOP HEADER STATUS BAR
    M5.Display.fillRect(0, 0, 320, 24, 0x10A2);
    M5.Display.setFont(&fonts::Font0);
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextDatum(top_left);
    M5.Display.drawString(isAutonomousMode ? "[AUTO PATROL ACTIVE]" : "[MANUAL RC MODE]", 8, 6);

    String netStr = (WiFi.status() == WL_CONNECTED) ? "NET:OK" : (wifiOverridden ? "NET:OVR" : "NET:OFF");
    String rightStr = "BAT:" + String(M5.Power.getBatteryLevel()) + "% " + netStr + " CAM:" + (isM5CamOnline ? "OK" : "NO");
    M5.Display.setTextDatum(top_right);
    M5.Display.drawString(rightStr, 312, 6);

    // 2. STATE BANNER
    uint16_t badgeColor = isEmergencyStop ? COLOR_RED_ESTOP : 
                          (isAutonomousMode ? COLOR_ACCENT : COLOR_GREEN_ARMED);
    M5.Display.fillRoundRect(8, 30, 304, 34, 6, badgeColor);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setFont(&fonts::FreeSansBold9pt7b);
    M5.Display.setTextColor(BLACK);

    String bannerText;
    if (isEmergencyStop) {
        bannerText = "EMERGENCY STOPPED";
    } else if (isAutonomousMode) {
        bannerText = String("MISSION: ") + autoStateNames[autoState];
    } else {
        bannerText = "MANUAL RADIO ACTIVE";
    }
    M5.Display.drawString(bannerText, 160, 47);

    // 3. INTERACTIVE ROBOT FACE (Left Card: 8, 70, 148, 122)
    int faceCx = 82;
    int faceCy = 131;
    M5.Display.fillRect(10, 72, 144, 118, COLOR_PANEL);

    if (autoState == AUTO_OPEN_MOUTH || autoState == AUTO_AWAIT_TRASH) {
        // OPEN MOUTH ANIMATION: Happy eyes + big wide open mouth
        M5.Display.fillCircle(faceCx - 24, faceCy - 18, 9, COLOR_ACCENT);
        M5.Display.fillCircle(faceCx + 24, faceCy - 18, 9, COLOR_ACCENT);
        M5.Display.fillRoundRect(faceCx - 30, faceCy + 4, 60, 28, 10, COLOR_ORANGE_FAIL);
        M5.Display.setFont(&fonts::Font0);
        M5.Display.setTextColor(WHITE);
        M5.Display.setTextDatum(middle_center);
        M5.Display.drawString("FEED ME", faceCx, faceCy + 18);
    } else if (autoState == AUTO_CLOSE_MOUTH) {
        // SMILE / THANK YOU ANIMATION
        M5.Display.fillCircle(faceCx - 24, faceCy - 15, 9, COLOR_GREEN_ARMED);
        M5.Display.fillCircle(faceCx + 24, faceCy - 15, 9, COLOR_GREEN_ARMED);
        M5.Display.drawArc(faceCx, faceCy + 10, 24, 20, 20, 160, COLOR_GREEN_ARMED);
        M5.Display.setFont(&fonts::Font0);
        M5.Display.setTextColor(COLOR_GREEN_ARMED);
        M5.Display.setTextDatum(middle_center);
        M5.Display.drawString("THANK YOU!", faceCx, faceCy + 28);
    } else if (autoState == AUTO_APPROACH_PERSON) {
        // EXCITED EYES TRACKING PERSON HORIZONTALLY
        int pupilOffset = constrain((m5Cx - 160) / 16, -10, 10);
        M5.Display.fillCircle(faceCx - 24, faceCy - 8, 12, WHITE);
        M5.Display.fillCircle(faceCx + 24, faceCy - 8, 12, WHITE);
        M5.Display.fillCircle(faceCx - 24 + pupilOffset, faceCy - 8, 5, COLOR_ACCENT);
        M5.Display.fillCircle(faceCx + 24 + pupilOffset, faceCy - 8, 5, COLOR_ACCENT);
        M5.Display.setFont(&fonts::Font0);
        M5.Display.setTextColor(COLOR_ACCENT);
        M5.Display.setTextDatum(middle_center);
        M5.Display.drawString("APPROACHING", faceCx, faceCy + 22);
    } else {
        // NORMAL PATROL BLINKING EYES
        M5.Display.fillRoundRect(faceCx - 32, faceCy - 14, 20, 16, 4, COLOR_ACCENT);
        M5.Display.fillRoundRect(faceCx + 12, faceCy - 14, 20, 16, 4, COLOR_ACCENT);
        M5.Display.drawFastHLine(faceCx - 20, faceCy + 18, 40, COLOR_TEXT_DIM);
        M5.Display.setFont(&fonts::Font0);
        M5.Display.setTextColor(COLOR_TEXT_DIM);
        M5.Display.setTextDatum(middle_center);
        M5.Display.drawString("PATROLLING LINE", faceCx, faceCy + 28);
    }

    // 4. RIGHT CARD (Sensors & Encoders Telemetry: 164, 70, 148, 122)
    M5.Display.fillRect(166, 72, 144, 118, COLOR_PANEL);
    M5.Display.setFont(&fonts::Font0);
    M5.Display.setTextDatum(top_left);
    M5.Display.setTextColor(COLOR_TEXT_DIM);

    M5.Display.drawString("LINE SENSOR (Port B):", 172, 76);
    for (int i = 0; i < 5; i++) {
        uint16_t boxColor = lineSensorBits[i] ? COLOR_GREEN_ARMED : 0x2124;
        M5.Display.fillRoundRect(172 + (i * 16), 88, 12, 12, 2, boxColor);
    }
    M5.Display.setTextColor(isLineDetected ? COLOR_GREEN_ARMED : WHITE);
    M5.Display.drawString("P:" + String(linePosition), 260, 90);

    M5.Display.setTextColor(COLOR_TEXT_DIM);
    M5.Display.drawString("M5 AI CAMERA (Port C):", 172, 110);
    M5.Display.setTextColor(m5HumanDetected ? COLOR_GREEN_ARMED : WHITE);
    M5.Display.drawString(String("Target: ") + (m5HumanDetected ? "HUMAN" : "NONE"), 174, 124);
    M5.Display.drawString(String("cx:") + String(m5Cx) + " h:" + String(m5H), 174, 138);

    M5.Display.setTextColor(COLOR_TEXT_DIM);
    M5.Display.drawString("MC4 ENCODERS:", 172, 156);
    int32_t enc0 = motorModuleFound ? motorDriver.getEncoderValue(0) : 0;
    int32_t enc1 = motorModuleFound ? motorDriver.getEncoderValue(1) : 0;
    M5.Display.setTextColor(WHITE);
    M5.Display.drawString(String("FL:") + String(enc0) + " FR:" + String(enc1), 174, 170);

    // 5. BOTTOM BAR / TOUCH ACTION BUTTON
    uint16_t btnColor = isEmergencyStop ? COLOR_GREEN_ARMED : 0x8800;
    M5.Display.fillRoundRect(8, 196, 304, 28, 5, btnColor);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setFont(&fonts::FreeSansBold9pt7b);
    M5.Display.setTextColor(WHITE);
    M5.Display.drawString(isEmergencyStop ? "TAP TO RESUME" : "[ EMERGENCY STOP ]", 160, 210);

    // Subtle cl0ud / Ojas Singh attribution footer
    M5.Display.setFont(&fonts::Font0);
    M5.Display.setTextColor(0x52AA); // Subtle dim cyan/grey
    M5.Display.drawString("POWERED BY cl0ud  |  Co-engineered by Ojas Singh", 160, 232);
}
