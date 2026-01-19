#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Adafruit_SHT4x.h>
#include "AmpifySoilMoistureAsync.h"
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include <math.h>
#include <esp_task_wdt.h>  // Hardware Watchdog Timer

// ================= HIVEMQ CLOUD CONFIG =================
const char* MQTT_HOST = "8bcddcb207f64292b51cea0423d766b7.s1.eu.hivemq.cloud";
const int MQTT_PORT = 8883;
const char* MQTT_USER = "smartfarm";
const char* MQTT_PASS = "Az056876";

// Auto-generate version from filename and compile date/time
// __FILE__ = ชื่อไฟล์, __DATE__ = วันที่คอมไพล์, __TIME__ = เวลาคอมไพล์
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
const String FIRMWARE_VERSION = "finish5_4_6_0 " + String(__DATE__) + " " + String(__TIME__);

WiFiClientSecure espClient;
PubSubClient mqtt(espClient);

// OTA variables
bool otaInProgress = false;
String otaUrl = "";

// ================= DEBUG CONFIG =================
// ⚠️ ปิด DEBUG เมื่อไม่ได้ต่อ Serial Monitor
// เพื่อป้องกัน Serial buffer overflow ที่ทำให้ WiFi หลุด
#define DEBUG_SERIAL 0          // เปลี่ยนเป็น 1 เมื่อต้องการ debug
#define DEBUG_EVERY_MS 2000
#define DEBUG_SENSOR_DETAIL 0   // เปลี่ยนเป็น 1 เมื่อต้องการดู sensor detail

#if DEBUG_SERIAL
#define DBG(...) Serial.printf(__VA_ARGS__)
#define DBGLN(x) Serial.println(x)
#else
#define DBG(...)
#define DBGLN(x)
#endif

// ==========================================
//          1. PIN DEFINITIONS (Complete Pin Map)
// ==========================================
/*
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║                    ESP32 PIN MAP - HOPS FARM v2                  ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  RELAY OUTPUTS (Active High) - ตามรูป Relay Board:              ║
 * ║    GPIO 5   → R1: เครื่องทำหมอก (Foggy Machine)                  ║
 * ║    GPIO 18  → R2: Solenoid 1 (วาล์วเติมถังหมอก)                  ║
 * ║    GPIO 4   → R3: Solenoid 2 (วาล์วรดน้ำ/ปุ๋ย)                   ║
 * ║    GPIO 15  → R4: Pump A (ปั๊มหลัก - จ่ายน้ำ+เติมถัง)           ║
 * ║    GPIO 23  → R5: Fan (พัดลม)                                    ║
 * ║    GPIO 26  → R6: Light (ไฟปลูก)                                 ║
 * ║    GPIO 12  → R7: Pump B (ปั๊มปุ๋ย)                              ║
 * ║    GPIO 19  → R8: Status Lamp/Buzzer (ไฟแจ้งเตือน)               ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  I2C BUS #1 (SHT40 #1 + LCD 16x2):                               ║
 * ║    GPIO 21  → SDA                                                ║
 * ║    GPIO 22  → SCL                                                ║
 * ║    Address: SHT40=0x44, LCD=0x27 (or 0x3F)                       ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  I2C BUS #2 (SHT40 #2):                                          ║
 * ║    GPIO 32  → SDA                                                ║
 * ║    GPIO 33  → SCL                                                ║
 * ║    Address: SHT40=0x44                                           ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  DIGITAL INPUTS:                                                 ║
 * ║    GPIO 27  → Flow Sensor YF-S201 (Interrupt)                    ║
 * ║    GPIO 14  → Soil Moisture Clock                                ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  ANALOG INPUTS (Input-Only, No Internal Pullup):                 ║
 * ║    GPIO 35  → Soil Moisture Data                                 ║
 * ║    GPIO 36  → ZMPT101B AC Voltage Sensor                         ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  FLOAT SWITCH (ใช้ Internal Pull-up):                            ║
 * ║    GPIO 16  → Float Switch LOW (ตรวจน้ำต่ำ)                      ║
 * ║    GPIO 17  → Float Switch HIGH (ตรวจน้ำเต็ม)                    ║
 * ║    ✅ ต่อตรงได้เลย: GPIO ---- [Float Switch] ---- GND            ║
 * ║    ⚠️ GPIO 25,27 มีปัญหากับ WiFi (ADC2) จึงเปลี่ยนมาใช้ 16,17    ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

// ---- Relay Outputs (Active High) - ตรงกับ Relay Board ----
#define PUMP_FOGGY_PIN 5          // R1: เครื่องทำหมอก (Foggy Machine)
#define SOL_1_PIN 18              // R2: Solenoid 1 (วาล์วเติมน้ำถังหมอก)
#define SOL_2_PIN 4               // R3: Solenoid 2 (วาล์วรดน้ำ/ปุ๋ย)
#define PUMP_A_PIN 15             // R4: Pump A (ปั๊มหลัก - จ่ายน้ำ+เติมถัง)
#define FAN_PIN 23                // R5: พัดลม
#define LIGHT_PIN 26              // R6: ไฟปลูก
#define PUMP_B_PIN 12             // R7: Pump B (ปั๊มปุ๋ย)
#define STATUS_LAMP_RELAY_PIN 19  // R8: Buzzer/ไฟแจ้งเตือน

// ---- I2C Bus #1 (SHT40 #1 + LCD I2C) ----
#define I2C1_SDA 21
#define I2C1_SCL 22
#define LCD_I2C_ADDR 0x27         // LCD Address (0x27 หรือ 0x3F)
#define LCD_COLS 16
#define LCD_ROWS 2
// SHT40 #1 Address: 0x44 (default)

// ---- I2C Bus #2 (SHT40 #2) ----
#define I2C2_SDA 32
#define I2C2_SCL 33
// SHT40 #2 Address: 0x44

// ---- Flow Sensor ----
#define FLOW_SENSOR_PIN 27        // YF-S201 pulse output (interrupt capable)

// ---- Soil Moisture Sensor ----
const int clockPin = 14;          // Soil sensor clock
const int dataPin = 35;           // Soil sensor data (GPIO 35 - ADC1_CH7)

// ---- Water Level Float Switch (52mm Plastic Float Switch) ----
// ติดตั้งแบบ NO (Normally Open): 
//   - ลูกลอยลอยขึ้น (มีน้ำ) = วงจรปิด = LOW (เมื่อใช้ Pull-up)
//   - ลูกลอยตกลง (ไม่มีน้ำ) = วงจรเปิด = HIGH
// 
// ✅ ใช้ GPIO 16, 17 ที่มี Internal Pull-up (ไม่ conflict กับ WiFi)
//    ต่อตรงได้เลย: GPIO ---- [Float Switch] ---- GND
// ⚠️ Flow Sensor ย้ายไป GPIO 27
//
#define FLOAT_LOW_PIN 16          // ลูกลอยตัวล่าง (ตรวจน้ำต่ำ) - มี Internal Pull-up
#define FLOAT_HIGH_PIN 17         // ลูกลอยตัวบน (ตรวจน้ำเต็ม) - มี Internal Pull-up
#define FLOAT_SWITCH_NC false     // true ถ้าใช้ Float Switch แบบ NC

// ---- AC Voltage Sensor ZMPT101B ----
// ใช้ตรวจจับไฟดับ/ไฟตก (Power Fail Detection)
//
// การต่อสาย:
//   ZMPT101B     ESP32
//   ─────────────────────
//   VCC    →    3.3V
//   GND    →    GND
//   OUT    →    GPIO 36 (VP)
//   
//   AC Input (L, N) → ต่อไฟ AC 220V ⚠️ อันตราย!
//
// ⚠️ ข้อควรระวัง:
//   - ขั้วต่อ AC เป็นไฟ 220V อันตรายถึงชีวิต!
//   - ใช้ Terminal Block ที่ปลอดภัย
//   - ติดตั้งในกล่องปิดมิดชิด
//   - ห้ามแตะโมดูลขณะมีไฟ
//
#define ZMPT101B_PIN 36           // ADC1_CH0 - Analog Input (VP)

// ค่า Calibration (ปรับให้ตรงกับโมดูลของคุณ)
// วิธี Calibrate:
//   1. วัดแรงดันจริงด้วยมัลติมิเตอร์ (เช่น 220V)
//   2. ดูค่าที่อ่านได้จาก Serial Monitor
//   3. ปรับค่า ZMPT_CALIBRATION = แรงดันจริง / ค่าที่อ่านได้
//   ตัวอย่าง: ถ้าวัดได้ 220V แต่ Serial แสดง 440 → ZMPT_CALIBRATION = 220/440 = 0.5
#define ZMPT_CALIBRATION 0.426

// ==========================================
//          2. WIFI / GOOGLE SHEET
// ==========================================
#define WIFI_SSID "TP-Link_2770"
#define WIFI_PASSWORD "0982532212"

const char* GOOGLE_SHEET_URL =
  "https://script.google.com/macros/s/AKfycbztXOVShEhbILPb5RGQPaDBiRZ1Is5LFbfkHgZI1lh5K7RQeZyZ5Zk5BkIQvIM9Nw/exec";
String currentErrorMsg = "Normal";

// ==========================================
//          3. SOIL SENSOR CONFIG
// ==========================================
#define MOISTURE_MIN_VALUE 9100
#define MOISTURE_MAX_VALUE 29210
AmpifySoilMoistureAsync moistureSensor(dataPin);

// ==========================================
//          4. RELAY POLARITY (ACTIVE HIGH)
// ==========================================
const bool RELAY_ACTIVE_HIGH = true;
inline void relayWrite(uint8_t pin, bool on) {
  digitalWrite(pin, RELAY_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH));
}

// ==========================================
//          5. PREFERENCES
// ==========================================
Preferences preferences;

// ==========================================
//          6. USER SETTINGS (ปรับตามขอบเขต)
// ==========================================
int SET_WATER_HOUR = 9;
int SET_WATER_MIN = 0;
int SET_WATER_DUR_SEC = 15;

int SET_LIGHT_START_HOUR = 19;  // เปิดไฟ 19:00 น. (ตรงกับกลางคืน)
int SET_LIGHT_DUR_HOUR = 9;     // เปิด 9 ชม. = ปิด 04:00 น.

int SET_FER_DAY_1 = 3;  // 0=Sun..6=Sat
int SET_FER_DAY_2 = 0;
int SET_FER_DUR_SEC = 10;

// ปรับตามขอบเขต 1.3.4 - ควบคุมอุณหภูมิไม่เกิน 30°C
float SET_TEMP_ON = 30.0;
float SET_TEMP_OFF = 28.0;

// ปรับตามขอบเขต 1.3.8 - ควบคุมความชื้น 50-80%
float SET_HUM_ON = 50.0;
float SET_HUM_OFF = 80.0;

int SET_SOIL_LOW = 50;
int SET_SOIL_SCHED_MIN = 70;

// MIST ON/OFF TIME - ไม่ได้ใช้งานแล้ว (Mist ควบคุมตาม RH State Machine)
// คงไว้สำหรับ compatibility กับ Dashboard เท่านั้น
unsigned long mistOnTimeMs = 20000;
unsigned long mistOffTimeMs = 60000;

// ==========================================
//          7. TIMING CONSTANTS
// ==========================================
const unsigned long SENSOR_READ_INTERVAL = 5000;
const unsigned long CONTROL_INTERVAL = 10000;
const unsigned long LOG_INTERVAL = 60000; // ค่าที่ Google sheet ส่ง ทุก 1 นาที 
const unsigned long WIFI_RECONNECT_INTERVAL = 30000;   // ลดจาก 5 นาที เป็น 30 วินาที
const unsigned long WIFI_WATCHDOG_TIMEOUT = 300000;    // 5 นาที → restart WiFi แบบเต็ม
const unsigned long LCD_UPDATE_INTERVAL = 3000;
const unsigned long SENSOR_ERROR_RESTART_MS = 300000;  // 5 นาที → restart ESP32 ถ้า sensor ไม่กลับมา

// ==========================================
// Watchdog 2 ระดับ (Escalation Strategy)
// ==========================================
// ระดับ 1: ส่งไม่ได้ 10 นาที → WiFi.disconnect() (ให้ต่อใหม่)
// ระดับ 2: ส่งไม่ได้ 120 นาที → ESP.restart() (reboot บอร์ด)
const unsigned long WATCHDOG_LEVEL1_TIMEOUT = 600000;    // 10 นาที → restart WiFi
const unsigned long WATCHDOG_LEVEL2_TIMEOUT = 7200000;   // 120 นาที → restart ESP32

// ==========================================
//          8. SAFETY CONSTANTS
// ==========================================
// Pump A Timeout แยกตามงาน
const unsigned long PUMP_A_WATERING_TIMEOUT = 120000;   // 2 นาที สำหรับรดน้ำ (sol2)
const unsigned long PUMP_A_TANKFILL_TIMEOUT = 300000;   // 5 นาที สำหรับเติมถัง (sol1)
const unsigned long PUMP_B_TIMEOUT = 60000;             // 1 นาที สำหรับปั๊มปุ๋ย

const unsigned long SENSOR_NAN_ALARM_MS = 60000;
const unsigned long SENSOR_MISMATCH_MS = 60000;
const unsigned long SENSOR_STUCK_ALARM_MS = 3600000;

const float TEMP_DIFF_MAX = 8.0;
const float HUM_DIFF_MAX = 15.0;

const unsigned long SOIL_BAD_ALARM_MS = 60000;
const unsigned long SOIL_STUCK_ALARM_MS = 7200000;  // 2 ชั่วโมง
const unsigned long SOIL_RAW_TOO_LOW = 1000;
const unsigned long SOIL_RAW_TOO_HIGH = 65000;

// Watering cycle settings
const unsigned long WATER_CYCLE_DURATION = 120000;  // รดน้ำครั้งละ 2 นาที
const unsigned long WATER_WAIT_CHECK = 10000;       // รอ 10 วินาที แล้วเช็ค soil
const int SOIL_TARGET_PERCENT = 70;                 // หยุดรดเมื่อ soil >= 70%
const int SOIL_RESCUE_TRIGGER = 45;                 // รดน้ำ rescue เมื่อ soil < 45%

const int CHECK_START_HOUR = 9;
const int CHECK_END_HOUR = 16;
const unsigned long WATER_COOLDOWN = 3600000;

const float FLOW_CAL_K = 21.25f;
const float FLOW_MIN_LPM = 0.20f;
const unsigned long FLOW_GRACE_MS = 4000;
const unsigned long FLOW_FAIL_MS = 3000;
const float FLOW_NOISE_LPM = 0.10f;
const unsigned long FLOW_NOISE_ALARM_MS = 60000;

// AC Voltage monitoring
const float AC_VOLTAGE_NORMAL_MIN = 180.0;  // ต่ำกว่านี้ถือว่าไฟตก
const float AC_VOLTAGE_NORMAL_MAX = 250.0;  // สูงกว่านี้ถือว่าผิดปกติ
const float AC_VOLTAGE_POWER_FAIL = 80.0;   // ต่ำกว่านี้ถือว่าไฟดับ
const unsigned long POWER_FAIL_ALARM_MS = 3000;  // ต้องต่ำต่อเนื่อง 3 วินาที

// ==========================================
//   CLIMATE CONTROL STATE MACHINE
// ==========================================
// โหมดควบคุมสภาพอากาศ
enum ClimateState {
  STATE_NORMAL,     // ปกติ
  STATE_OVERTEMP,   // อุณหภูมิสูงเกิน (สำคัญสุด)
  STATE_RH_HIGH,    // ความชื้นสูงเกิน
  STATE_RH_LOW      // ความชื้นต่ำเกิน
};

// Thresholds สำหรับแต่ละโหมด (ปรับผ่าน MQTT ได้)
float TEMP_OVERTEMP_ENTER = 29.0;   // เข้า OVERTEMP เมื่อ >= 29°C
float TEMP_OVERTEMP_EXIT = 26.0;    // ออก OVERTEMP เมื่อ <= 26°C
unsigned long OVERTEMP_EXIT_HOLD_MS = 120000;  // ค้าง 2 นาทีก่อนออก

float RH_HIGH_ENTER = 75.0;         // เข้า RH_HIGH เมื่อ >= 75%
float RH_HIGH_EXIT = 70.0;          // ออก RH_HIGH เมื่อ <= 70%

float RH_LOW_ENTER = 55.0;          // เข้า RH_LOW เมื่อ <= 55%
float RH_LOW_EXIT = 68.0;           // ออก RH_LOW เมื่อ >= 68%

float RH_GUARD_MAX = 78.0;          // เพดานความชื้นสูงสุด (ห้ามเปิด humidifier)
float RH_EMERGENCY_OFF = 75.0;      // ปิด humidifier ทันทีถ้าถึง

// Pulse timing สำหรับ OVERTEMP humidifier
unsigned long OVERTEMP_HUMID_ON_MS = 180000;   // 3 นาที ON
unsigned long OVERTEMP_HUMID_OFF_MS = 180000;  // 3 นาที OFF

// Night Fan pulse (สำหรับ RH_HIGH และ RH_LOW ตอนกลางคืน)
unsigned long NIGHT_FAN_ON_MS = 30000;   // 30 วินาที ON
unsigned long NIGHT_FAN_OFF_MS = 300000; // 5 นาที OFF

// Day/Night definition
int DAY_START_HOUR = 7;   // 07:00 เริ่มกลางวัน
int DAY_END_HOUR = 19;    // 19:00 เริ่มกลางคืน

// ==========================================
//          9. GLOBAL VARIABLES
// ==========================================
bool ModeAuto = true;
bool systemError = false;

// ---- Output States ----
// ปั๊ม A (ปั๊มหลัก) - ใช้รดน้ำและเติมถังหมอก
bool pumpA_On = false;

// ปั๊ม B (ปั๊มปุ๋ย)
bool pumpB_On = false;

// Solenoid 1 (วาล์วหมอก/เติมน้ำถัง)
bool sol1_On = false;

// Solenoid 2 (วาล์วรดน้ำ/ปุ๋ย)
bool sol2_On = false;

// ปั๊มหมอก (Foggy)
bool pumpFoggy_On = false;

// พัดลมและไฟ
bool fanOn = false;
bool lightOn = false;

// Water tank level
bool waterLevelLow = false;   // true = น้ำต่ำ
bool waterLevelHigh = false;  // true = น้ำเต็ม
bool tankFillingActive = false;  // กำลังเติมน้ำถังอยู่

// Float Switch Debounce (ป้องกันน้ำกระเพื่อม)
const unsigned long FLOAT_DEBOUNCE_MS = 2000;  // ต้องคงที่ 2 วินาทีก่อนเปลี่ยนสถานะ
bool rawFloatLow = false;           // ค่าดิบจาก sensor
bool rawFloatHigh = false;          // ค่าดิบจาก sensor
unsigned long floatLowChangeMs = 0;   // เวลาที่เริ่มเปลี่ยน
unsigned long floatHighChangeMs = 0;  // เวลาที่เริ่มเปลี่ยน
bool floatLowPending = false;         // รอยืนยันการเปลี่ยน
bool floatHighPending = false;        // รอยืนยันการเปลี่ยน

// Daily flags
bool hasWateredToday = false;
bool hasFertilizedToday = false;

// Timers
unsigned long pumpA_StartTime = 0;
unsigned long pumpB_StartTime = 0;
unsigned long fertilizerStartTime = 0;
unsigned long lastWaterFinishTime = 0;

// Pump A task type (เพื่อใช้ timeout ที่ถูกต้อง)
enum PumpATask {
  PUMP_A_IDLE,
  PUMP_A_WATERING,    // รดน้ำ (sol2)
  PUMP_A_TANKFILL     // เติมถัง (sol1)
};
PumpATask currentPumpATask = PUMP_A_IDLE;

// Sensor values
unsigned long moistureValue = 0;
unsigned long moisturePercentage = 0;
float tempused = NAN, humused = NAN;
float tempmain = NAN, hummain = NAN, temp2 = NAN, hum2 = NAN;

// Sensor health tracking
unsigned long lastTempOkMs = 0;
unsigned long lastHumOkMs = 0;
unsigned long tempMismatchStartMs = 0;
unsigned long humMismatchStartMs = 0;
unsigned long sensorErrorStartMs = 0;    // เวลาที่เริ่ม sensor error (สำหรับ restart)
float lastGoodTemp = NAN;
float lastGoodHum = NAN;
unsigned long tempStuckStartMs = 0;
unsigned long humStuckStartMs = 0;

// Soil health
unsigned long soilBadStartMs = 0;
unsigned long soilStuckStartMs = 0;
int lastSoilPctForStuck = -1;

// Mist pulse machine (for humidity control)
bool mistPulsing = false, mistWaiting = false;
unsigned long mistTimer = 0;

// Flow
volatile long pulseCount = 0;
float flowRateLpm = 0.0f;
float dailyWaterVolume = 0.0;
unsigned long oldTimeFlow = 0;
unsigned long flowLowStart = 0;
unsigned long flowNoiseStartMs = 0;

// AC Voltage monitoring (Non-blocking sampling)
float acVoltage = 0.0;           // แรงดันไฟ AC (Vrms)
float acVoltageFiltered = 0.0;   // ค่า voltage หลัง EMA filter
bool acPowerOk = true;           // สถานะไฟปกติ (true=OK, false=FAIL)
bool powerFailAlarm = false;     // Alarm ไฟดับ (หลังผ่าน debounce)
unsigned long powerFailStartMs = 0;  // เวลาเริ่มตรวจพบไฟต่ำ
unsigned long powerRestoreStartMs = 0;  // เวลาเริ่มตรวจพบไฟกลับ (สำหรับ debounce restore)
bool pumpAlarmActive = false;    // Alarm เฉพาะ Pump (Fan/Mist/Light ยังทำงาน)
bool soilAlarmActive = false;    // Alarm เฉพาะ Soil Sensor (ระบบอื่นยังทำงาน)

// Non-blocking AC Sampling (ใช้ double ป้องกัน overflow)
const uint16_t AC_SAMPLE_COUNT = 800;         // จำนวน sample ต่อรอบ (~40ms ที่ 50Hz)
const uint32_t AC_SAMPLE_INTERVAL_US = 50;    // ช่วงห่างระหว่าง sample (50µs = 20kHz)
const unsigned long AC_CALC_INTERVAL_MS = 100; // คำนวณทุก 100ms
uint16_t acSampleIndex = 0;                   // index ปัจจุบัน
double acSampleSum = 0;                       // ผลรวมของ sample (double ป้องกัน overflow)
double acSampleSumSq = 0;                     // ผลรวมของ sample² (double ป้องกัน overflow)
unsigned long lastAcSampleUs = 0;             // เวลา sample ล่าสุด
unsigned long lastAcCalcMs = 0;               // เวลาคำนวณ RMS ล่าสุด

// Debounce & Hysteresis Constants
const unsigned long POWER_RESTORE_DEBOUNCE_MS = 2000;  // รอ 2 วินาทีก่อนยืนยันไฟกลับ

// Climate State Machine
ClimateState currentClimateState = STATE_NORMAL;
ClimateState prevClimateState = STATE_NORMAL;
unsigned long climateStateEnterMs = 0;      // เวลาเข้าสู่ state ปัจจุบัน
unsigned long overtempExitConditionMs = 0;  // เวลาที่เริ่มถึงเงื่อนไขออก OVERTEMP
bool overtempExitPending = false;           // รอออกจาก OVERTEMP (hold 2 นาที)

// OVERTEMP Humidifier pulse
bool overtempHumidPulseOn = false;
unsigned long overtempHumidTimer = 0;

// Night Fan pulse
bool nightFanPulseOn = false;
unsigned long nightFanTimer = 0;

// RH_LOW Mist hold timer (เปิดต่ออีก 1 นาทีเมื่อ RH >= 68%)
bool rhLowMistHoldActive = false;           // กำลังอยู่ใน hold period
unsigned long rhLowMistHoldStartMs = 0;     // เวลาที่เริ่ม hold
const unsigned long RH_LOW_MIST_HOLD_MS = 60000;  // 1 นาที

// Day/Night tracking
bool isDayTime = true;
int currentHour = -1;

// Cached Time (Non-blocking)
int cachedHour = -1;
int cachedMinute = -1;
int cachedSecond = -1;
int cachedDayOfWeek = -1;
bool cachedTimeReady = false;
unsigned long lastTimeUpdateMs = 0;
const unsigned long TIME_UPDATE_INTERVAL = 1000;  // อัพเดททุก 1 วินาที

// ==========================================
// HTTP Task - Persistent Task + FreeRTOS Queue
// ==========================================
// วิธีนี้เหมาะสำหรับ IoT 24/7:
// - สร้าง Task 1 ครั้งตอน setup (ไม่สร้าง/ลบซ้ำ)
// - ใช้ FreeRTOS Queue ส่งข้อมูล (thread-safe)
// - ไม่มี Memory Leak
// - ไม่ต้องใช้ vTaskDelete
// ==========================================

TaskHandle_t httpTaskHandle = NULL;
QueueHandle_t httpQueue = NULL;                 // FreeRTOS Queue
const int HTTP_QUEUE_SIZE = 10;                 // เก็บได้ 10 payload
const int HTTP_MAX_RETRY = 3;                   // จำนวน retry สูงสุด

unsigned long httpSuccessCount = 0;
unsigned long httpFailCount = 0;
unsigned long httpQueueFullCount = 0;           // นับจำนวนครั้งที่ queue เต็ม

// WiFi Watchdog variables
unsigned long lastWifiConnectedMs = 0;
unsigned long lastHttpSuccessMs = 0;
unsigned long wifiDisconnectCount = 0;
unsigned long wifiRestartCount = 0;

// MQTT Health Tracking Variables
unsigned long lastMqttPublishSuccess = 0;
unsigned long lastMqttActivity = 0;  // เวลาที่มี activity ล่าสุด (send/receive)
unsigned long mqttReconnectCount = 0;
bool mqttEverConnected = false;  // เคยเชื่อมต่อสำเร็จหรือยัง

struct HttpPayload {
  float temp;
  float hum;
  int soil;
  char mode[10];
  char climateState[15];
  float waterVol;
  char alarm[50];
};

// Forward declarations
void httpPersistentTask(void* parameter);
bool sendHttpRequest(HttpPayload* p);

// Loop timers
unsigned long lastControlMs = 0;
unsigned long lastReadMs = 0;
unsigned long lastDatalogsMs = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastDebugMs = 0;

// Previous output states (for publish on change)
bool prevPumpA = false;
bool prevPumpB = false;
bool prevSol1 = false;
bool prevSol2 = false;
bool prevPumpFoggy = false;
bool prevFan = false;
bool prevLight = false;
bool prevSystemError = false;

// ==========================================
//          10. OBJECTS
// ==========================================
// LCD I2C (16x2) - ใช้ I2C Bus #1 ร่วมกับ SHT40 #1
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
Adafruit_SHT4x shtMain = Adafruit_SHT4x();
Adafruit_SHT4x sht2 = Adafruit_SHT4x();
TwoWire I2C_1 = TwoWire(1);  // I2C Bus #2 for SHT40 #2

// ==========================================
//          11. MQTT HELPERS
// ==========================================
inline void publishStatus(const char* topic, bool on) {
  mqtt.publish(topic, on ? "On" : "Off");
}

inline bool wantOnPayload(const char* pl) {
  if (!pl) return false;
  String s(pl);
  s.trim();
  s.toLowerCase();
  return (s == "on" || s == "1" || s == "true");
}

// ปั๊ม A ต้องทำงานเมื่อ: รดน้ำ (sol2) หรือ เติมถังหมอก (sol1)
inline bool needPumpA() {
  return sol1_On || sol2_On;
}

// อ่านสถานะระดับน้ำจาก Float Switch
// 
// การติดตั้ง Float Switch แบบ NO (Normally Open):
// ✅ ใช้ GPIO 16, 17 ที่มี Internal Pull-up (ไม่ conflict กับ WiFi)
// ⚠️ GPIO 25,27 มีปัญหากับ WiFi (ADC2) จึงเปลี่ยนมาใช้ 16,17
//
//   ┌─────────────────────────────────────┐
//   │            ถังน้ำหมอก                │
//   │  ┌───┐                              │
//   │  │ ● │ ← Float HIGH (GPIO 17)       │  ถ้าลอย = น้ำเต็ม
//   │  └───┘                              │
//   │                                     │
//   │                                     │
//   │  ┌───┐                              │
//   │  │ ● │ ← Float LOW (GPIO 16)        │  ถ้าไม่ลอย = น้ำต่ำมาก
//   │  └───┘                              │
//   └─────────────────────────────────────┘
//
// ✅ ต่อตรงได้เลย (ไม่ต้องใช้ Resistor):
//   GPIO 16 ----[Float Switch LOW]---- GND
//   GPIO 17 ----[Float Switch HIGH]---- GND
//
// Logic (NO Switch + Internal Pull-up):
//   - ลูกลอยลอย (มีน้ำถึง) → วงจรปิด → อ่านได้ LOW
//   - ลูกลอยตก (ไม่มีน้ำ) → วงจรเปิด → อ่านได้ HIGH (Pull-up)
//
void readWaterLevel() {
  unsigned long now = millis();
  
  bool floatLowState = digitalRead(FLOAT_LOW_PIN);
  bool floatHighState = digitalRead(FLOAT_HIGH_PIN);
  
  // ถ้าใช้ Float Switch แบบ NC ให้กลับค่า
  if (FLOAT_SWITCH_NC) {
    floatLowState = !floatLowState;
    floatHighState = !floatHighState;
  }
  
  // NO Switch + Internal Pull-up:
  // LOW = ลูกลอยลอย (มีน้ำถึงระดับนั้น)
  // HIGH = ลูกลอยตก (ไม่มีน้ำถึงระดับนั้น)
  
  // อ่านค่าดิบ
  bool newRawLow = (floatLowState == HIGH);   // true = น้ำต่ำ
  bool newRawHigh = (floatHighState == LOW);  // true = น้ำเต็ม
  
  // ============ Debounce Float LOW ============
  if (newRawLow != waterLevelLow) {
    // ค่าเปลี่ยน - เริ่มนับ debounce
    if (!floatLowPending) {
      floatLowPending = true;
      floatLowChangeMs = now;
      DBG("[FLOAT] LOW pending: %d -> %d\n", waterLevelLow, newRawLow);
    } else if (now - floatLowChangeMs >= FLOAT_DEBOUNCE_MS) {
      // ค้างครบ 2 วินาที - ยืนยันการเปลี่ยน
      waterLevelLow = newRawLow;
      floatLowPending = false;
      DBG("[FLOAT] LOW confirmed: %d\n", waterLevelLow);
    }
  } else {
    // ค่ากลับมาเหมือนเดิม - ยกเลิก pending
    floatLowPending = false;
  }
  
  // ============ Debounce Float HIGH ============
  if (newRawHigh != waterLevelHigh) {
    // ค่าเปลี่ยน - เริ่มนับ debounce
    if (!floatHighPending) {
      floatHighPending = true;
      floatHighChangeMs = now;
      DBG("[FLOAT] HIGH pending: %d -> %d\n", waterLevelHigh, newRawHigh);
    } else if (now - floatHighChangeMs >= FLOAT_DEBOUNCE_MS) {
      // ค้างครบ 2 วินาที - ยืนยันการเปลี่ยน
      waterLevelHigh = newRawHigh;
      floatHighPending = false;
      DBG("[FLOAT] HIGH confirmed: %d\n", waterLevelHigh);
    }
  } else {
    // ค่ากลับมาเหมือนเดิม - ยกเลิก pending
    floatHighPending = false;
  }
}

// ==========================================
//   อ่านค่าแรงดันไฟ AC จาก ZMPT101B (Non-blocking)
// ==========================================
// ใช้วิธี Non-blocking sampling + คำนวณ Mean/Variance
// เพื่อตัด DC offset อัตโนมัติ (แม่นยำกว่าใช้ค่าคงที่ 2048)
//
// ปรับปรุง:
//   - ใช้ double ป้องกัน overflow
//   - เพิ่ม acPowerOk สำหรับสถานะไฟ
//   - EMA Filter ทำให้ค่านิ่ง
//
void sampleACVoltage() {
  unsigned long nowUs = micros();
  
  // Sample ทุก 50µs (20kHz sampling rate)
  if ((nowUs - lastAcSampleUs) >= AC_SAMPLE_INTERVAL_US) {
    lastAcSampleUs = nowUs;
    
    int adcValue = analogRead(ZMPT101B_PIN);  // 0-4095
    acSampleSum += (double)adcValue;
    acSampleSumSq += (double)adcValue * (double)adcValue;
    acSampleIndex++;
  }
}

void calculateACVoltage() {
  unsigned long now = millis();
  
  // คำนวณทุก 100ms หรือเมื่อครบ samples
  if (acSampleIndex >= AC_SAMPLE_COUNT || (now - lastAcCalcMs >= AC_CALC_INTERVAL_MS && acSampleIndex > 100)) {
    lastAcCalcMs = now;
    
    if (acSampleIndex > 0) {
      // คำนวณ Mean (DC offset จริง) - ใช้ double
      double mean = acSampleSum / (double)acSampleIndex;
      
      // คำนวณ Variance = E[X²] - E[X]²
      double meanSq = acSampleSumSq / (double)acSampleIndex;
      double variance = meanSq - (mean * mean);
      if (variance < 0) variance = 0;  // ป้องกัน negative
      
      // RMS = sqrt(variance)
      double rmsAdc = sqrt(variance);
      
      // แปลงเป็น Voltage จริง
      float voltage = (float)(rmsAdc * ZMPT_CALIBRATION);
      
      // Noise threshold: ต่ำกว่า 50V ถือว่าไม่มีไฟ (ลดจาก 80V)
      const float NOISE_THRESHOLD = 50.0;
      if (voltage < NOISE_THRESHOLD) {
        voltage = 0.0;
      }
      
      // EMA Filter (Exponential Moving Average)
      // alpha = 0.2 → ตอบสนองช้าลง นิ่งขึ้น
      const float alpha = 0.2;
      if (acVoltageFiltered < 1.0 && voltage > 0) {
        acVoltageFiltered = voltage;  // Initialize
      } else if (voltage > 0) {
        acVoltageFiltered = alpha * voltage + (1.0 - alpha) * acVoltageFiltered;
      }
      
      // ถ้า voltage = 0 ให้ filter ลงเร็วขึ้น (Fast decay)
      if (voltage == 0.0) {
        acVoltageFiltered = acVoltageFiltered * 0.7;  // ลด 30% ต่อรอบ
        if (acVoltageFiltered < 5.0) acVoltageFiltered = 0.0;
      }
      
      // Update global variable
      acVoltage = acVoltageFiltered;
      
      // Update acPowerOk (มี hysteresis)
      // OK เมื่อ >= 180V, FAIL เมื่อ < 80V
      if (acVoltage >= AC_VOLTAGE_NORMAL_MIN) {
        acPowerOk = true;
      } else if (acVoltage < AC_VOLTAGE_POWER_FAIL) {
        acPowerOk = false;
      }
      // ระหว่าง 80-180V คง state เดิม (hysteresis)
      
      #if DEBUG_SENSOR_DETAIL
      static unsigned long lastAcDebug = 0;
      if (now - lastAcDebug > 5000) {
        lastAcDebug = now;
        DBG("[AC] n=%d mean=%.1f var=%.1f rms=%.1f V=%.1f Vfilt=%.1f OK=%d\n", 
            acSampleIndex, (float)mean, (float)variance, (float)rmsAdc, voltage, acVoltageFiltered, acPowerOk);
      }
      #endif
    }
    
    // Reset สำหรับรอบถัดไป
    acSampleIndex = 0;
    acSampleSum = 0;
    acSampleSumSq = 0;
  }
}

// Legacy function สำหรับ compatibility
float readACVoltage() {
  return acVoltage;
}

// ==========================================
//   ตรวจสอบ ZMPT101B Sensor Health
// ==========================================
// วิธีตรวจจับว่าเซ็นเซอร์เสีย:
// 1. ค่า ADC ค้างที่ 0 หรือ 4095 นานเกินไป (สายหลุด/short)
// 2. ค่า ADC ไม่มี AC ripple (ไม่มีความผันผวน = เซ็นเซอร์ตาย)
// 3. ค่า Mean ผิดปกติ (ควรอยู่ประมาณ 1800-2200 เมื่อมีไฟ)
//
bool zmptSensorOk = true;           // สถานะเซ็นเซอร์
bool zmptSensorAlarm = false;       // Alarm เซ็นเซอร์เสีย
unsigned long zmptBadStartMs = 0;   // เวลาเริ่มตรวจพบปัญหา
const unsigned long ZMPT_BAD_ALARM_MS = 30000;  // 30 วินาที ก่อน Alarm

// ค่าสำหรับตรวจสอบ
float lastAcMean = 0;               // ค่า Mean ล่าสุด
float lastAcVariance = 0;           // ค่า Variance ล่าสุด
uint16_t zmptStuckCount = 0;        // นับจำนวนครั้งที่ค่าค้าง

void checkZmptSensorHealth() {
  static unsigned long lastCheckMs = 0;
  unsigned long now = millis();
  
  // เช็คทุก 5 วินาที
  if (now - lastCheckMs < 5000) return;
  lastCheckMs = now;
  
  bool sensorProblem = false;
  String problemMsg = "";
  
  // อ่านค่า ADC หลายครั้งเพื่อเช็ค
  int minAdc = 4095;
  int maxAdc = 0;
  long sumAdc = 0;
  const int CHECK_SAMPLES = 100;
  
  for (int i = 0; i < CHECK_SAMPLES; i++) {
    int adcValue = analogRead(ZMPT101B_PIN);
    if (adcValue < minAdc) minAdc = adcValue;
    if (adcValue > maxAdc) maxAdc = adcValue;
    sumAdc += adcValue;
    delayMicroseconds(200);  // 200µs × 100 = 20ms (1 cycle ของ 50Hz)
  }
  
  float meanAdc = (float)sumAdc / CHECK_SAMPLES;
  int rangeAdc = maxAdc - minAdc;
  
  // บันทึกค่าสำหรับ debug
  lastAcMean = meanAdc;
  lastAcVariance = (float)rangeAdc;
  
  // ===== เงื่อนไขตรวจจับปัญหา =====
  
  // 1. ADC ค้างที่ 0 (สายหลุด/GND short)
  if (maxAdc < 50) {
    sensorProblem = true;
    problemMsg = "ADC stuck at 0 (wire disconnected?)";
  }
  
  // 2. ADC ค้างที่ 4095 (VCC short)
  else if (minAdc > 4000) {
    sensorProblem = true;
    problemMsg = "ADC stuck at 4095 (VCC short?)";
  }
  
  // 3. ไม่มี AC ripple (range < 50 แสดงว่าไม่มี AC signal)
  //    ปกติเมื่อมีไฟ AC จะมี range อย่างน้อย 200-500
  else if (rangeAdc < 30 && acVoltage < 50) {
    // ถ้า range น้อยมากและ voltage ต่ำ = อาจเซ็นเซอร์เสีย
    zmptStuckCount++;
    if (zmptStuckCount > 5) {  // ค้างติดต่อกัน 5 ครั้ง (25 วินาที)
      sensorProblem = true;
      problemMsg = "No AC ripple detected (sensor dead?)";
    }
  } else {
    zmptStuckCount = 0;  // Reset ถ้าปกติ
  }
  
  // 4. Mean ผิดปกติมาก (ควรอยู่ 1500-2500 สำหรับ 3.3V reference)
  if (meanAdc < 100 || meanAdc > 4000) {
    sensorProblem = true;
    problemMsg = "ADC mean abnormal: " + String(meanAdc, 0);
  }
  
  // ===== จัดการ Alarm =====
  if (sensorProblem) {
    if (zmptBadStartMs == 0) {
      zmptBadStartMs = now;
      DBG("[ZMPT] Sensor problem detected: %s\n", problemMsg.c_str());
      DBG("[ZMPT] ADC: min=%d max=%d mean=%.0f range=%d\n", minAdc, maxAdc, meanAdc, rangeAdc);
    }
    
    // Debounce 30 วินาที
    if (!zmptSensorAlarm && (now - zmptBadStartMs >= ZMPT_BAD_ALARM_MS)) {
      zmptSensorAlarm = true;
      zmptSensorOk = false;
      DBG("[ZMPT] *** SENSOR ALARM! *** %s\n", problemMsg.c_str());
      
      mqtt.publish("hf/status/zmpt_sensor", "FAIL");
      mqtt.publish("hf/status/messages", ("ALARM: AC Sensor - " + problemMsg).c_str());
      
      // ไม่หยุดระบบ แต่แจ้งเตือน + เปิดไฟแดง
      relayWrite(STATUS_LAMP_RELAY_PIN, true);
    }
  } else {
    // Sensor OK
    zmptBadStartMs = 0;
    
    if (zmptSensorAlarm) {
      zmptSensorAlarm = false;
      zmptSensorOk = true;
      DBG("[ZMPT] Sensor recovered - OK\n");
      
      mqtt.publish("hf/status/zmpt_sensor", "OK");
      mqtt.publish("hf/status/messages", "AC Sensor OK");
      
      // ปิดไฟแดงถ้าไม่มี error อื่น
      if (!systemError && !powerFailAlarm && !pumpAlarmActive && !soilAlarmActive) {
        relayWrite(STATUS_LAMP_RELAY_PIN, false);
      }
    }
  }
  
  // Log สถานะทุก 1 นาที (debug)
  static unsigned long lastLogMs = 0;
  if (now - lastLogMs > 60000) {
    lastLogMs = now;
    DBG("[ZMPT] Health: OK=%d min=%d max=%d mean=%.0f range=%d V=%.1f\n",
        zmptSensorOk, minAdc, maxAdc, meanAdc, rangeAdc, acVoltage);
  }
}

// ==========================================
//   ตรวจสอบไฟดับ (Power Fail Detection)
// ==========================================
// มี Debounce ทั้งตอนไฟดับและไฟกลับ
// มี Hysteresis: ไฟดับ < 80V, ไฟกลับ >= 180V
//
void checkPowerFail() {
  unsigned long now = millis();
  
  // ===== ตรวจจับไฟดับ =====
  if (!acPowerOk) {
    // Reset restore debounce เมื่อไฟยังไม่ OK
    powerRestoreStartMs = 0;
    
    if (powerFailStartMs == 0) {
      powerFailStartMs = now;
      DBG("[POWER] Low voltage detected: %.1fV (waiting %.1fs)\n", acVoltage, POWER_FAIL_ALARM_MS/1000.0);
    }
    
    // Debounce: รอ 3 วินาทีก่อน Alarm
    if (!powerFailAlarm && (now - powerFailStartMs >= POWER_FAIL_ALARM_MS)) {
      powerFailAlarm = true;
      DBG("[POWER] *** POWER FAIL ALARM! *** Voltage=%.1fV\n", acVoltage);
      
      // ปิด Output ทั้งหมดทันที
      pumpA_On = false;
      pumpB_On = false;
      sol1_On = false;
      sol2_On = false;
      pumpFoggy_On = false;
      fanOn = false;
      lightOn = false;
      tankFillingActive = false;
      
      mqtt.publish("hf/status/messages", "ALARM: Power Fail! All outputs OFF");
      mqtt.publish("hf/status/power_fail", "1");
      mqtt.publish("hf/status/ac_power", "FAIL");
    }
    
    // เปิด R8 ค้างตลอดที่ไฟดับ
    if (powerFailAlarm) {
      digitalWrite(STATUS_LAMP_RELAY_PIN, HIGH);
    }
  }
  // ===== ตรวจจับไฟกลับมา =====
  else {
    // Reset fail debounce เมื่อไฟ OK
    powerFailStartMs = 0;
    
    if (powerFailAlarm) {
      // เริ่ม debounce สำหรับ restore
      if (powerRestoreStartMs == 0) {
        powerRestoreStartMs = now;
        DBG("[POWER] Voltage restored: %.1fV (waiting %.1fs to confirm)\n", acVoltage, POWER_RESTORE_DEBOUNCE_MS/1000.0);
      }
      
      // Debounce: รอ 2 วินาทีก่อนยืนยันไฟกลับ
      if (now - powerRestoreStartMs >= POWER_RESTORE_DEBOUNCE_MS) {
        powerFailAlarm = false;
        powerRestoreStartMs = 0;
        DBG("[POWER] *** POWER RESTORED *** Voltage=%.1fV\n", acVoltage);
        
        // ปิด R8 เมื่อไฟกลับมา
        if (!systemError && !pumpAlarmActive) {
          digitalWrite(STATUS_LAMP_RELAY_PIN, LOW);
        }
        
        mqtt.publish("hf/status/messages", "Power Restored - System Resume");
        mqtt.publish("hf/status/power_fail", "0");
        mqtt.publish("hf/status/ac_power", "OK");
      }
    }
  }
}

// ==========================================
//          13. FUNCTION PROTOTYPES
// ==========================================
void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

void initPins();
void initSensors();
void connectWIFI();
void tryReconnectWiFi();
void initMQTT();

void loadSettings();
void saveSettingInt(const char* key, int value);
void saveSettingFloat(const char* key, float value);
void saveSettingULong(const char* key, unsigned long value);

void readTemperatureC();
void readSensorSoil();
void readWaterLevel();
void sampleACVoltage();      // Non-blocking AC sampling
void calculateACVoltage();   // คำนวณ RMS จาก samples
float readACVoltage();       // Legacy function (returns acVoltage)
void checkPowerFail();      // ตรวจสอบไฟดับ
float checkSensorTemp();
float checkSensorHum();
void beep(int count, int onMs, int offMs);  // Beep function

void controlTemperature();
void controlHumidity();
void controlLightSystem(int hourNow);
void controlWaterTankFill();  // เติมน้ำถังหมอกอัตโนมัติ

// Climate State Machine
void updateClimateState();           // อัพเดท state machine
void executeClimateControl();        // ควบคุมตาม state ปัจจุบัน
void handleStateNormal();
void handleStateOvertemp();
void handleStateRhHigh();
void handleStateRhLow();
const char* getClimateStateName(ClimateState state);

void runSafetyChecks();
void sendToGoogleSheet(float temp, float hum, int soil);
void httpPersistentTask(void* parameter);  // Persistent HTTP Task
void checkHttpWatchdog();                  // HTTP Watchdog
bool checkInternetConnectivity();          // Internet Check
void checkInternetPeriodically();          // Periodic Internet Check
void checkMasterWatchdog();                // Master Watchdog
void updateLCD();

// MQTT & OTA Functions
void initMQTT();
void connectMQTT();
void mqttHeartbeat();
void checkMqttHealth();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void handleOTA();
void performOTA();

void setAlarm(const String& msg);
void clearAlarm(const String& msg);

// Setters สำหรับระบบใหม่
void setPumpA(bool on, const char* reason = nullptr);
void setPumpB(bool on, const char* reason = nullptr);
void setSol1(bool on, const char* reason = nullptr);
void setSol2(bool on, const char* reason = nullptr);
void setPumpFoggy(bool on, const char* reason = nullptr);
void setFan(bool on, const char* reason = nullptr);
void setLight(bool on, const char* reason = nullptr);

// Action functions
void startWatering(const char* reason);   // A + S2 เปิด
void stopWatering(const char* reason);
void startFertilizer(const char* reason); //  B + S2 เปิด
void stopFertilizer(const char* reason);
void startMisting(const char* reason);    // Foggy เปิด
void stopMisting(const char* reason);
void startTankFill(const char* reason);   // A + S1 เปิด (เติมถัง)
void stopTankFill(const char* reason);

void debugTick(bool timeReady, int hourNow, int minuteNow, int secondNow, int dayOfWeek);

// ==========================================
//          14. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // ==========================================
  // Hardware Watchdog Timer (WDT) - ESP32 Arduino Core 3.x
  // ==========================================
  // ถ้า ESP32 ค้างเกิน 60 วินาที จะ reset อัตโนมัติ
  // นี่คือ Hardware level - ทำงานแม้ Software ค้าง
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 60000,           // 60 วินาที
    .idle_core_mask = (1 << 0) | (1 << 1),  // ทั้ง 2 cores
    .trigger_panic = true          // Reset เมื่อ timeout
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);          // เพิ่ม main task เข้า watchdog
  
  DBG("\n\n=== BOOT ===\n");
  DBG("Hardware WDT: 60 sec timeout enabled\n");
  DBG("Pins: PUMP_A=%d PUMP_B=%d SOL1=%d SOL2=%d FOGGY=%d FAN=%d LIGHT=%d LAMP=%d\n",
      PUMP_A_PIN, PUMP_B_PIN, SOL_1_PIN, SOL_2_PIN, PUMP_FOGGY_PIN, FAN_PIN, LIGHT_PIN, STATUS_LAMP_RELAY_PIN);

  initPins();
  moistureSensor.begin();
  loadSettings();

  // เริ่มต้นเป็น Auto Mode เสมอ (ควบคุมผ่าน MQTT เท่านั้น)
  ModeAuto = true;
  DBG("Boot: ModeAuto=%d (MQTT control only)\n", ModeAuto);

  initSensors();
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);

  // LCD I2C ใช้ Wire (I2C Bus #1) ร่วมกับ SHT40 #1
  // Address: 0x27 (or 0x3F) - ถ้าไม่แสดงผลลองเปลี่ยน LCD_I2C_ADDR
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("HOPS FARM v2");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  connectWIFI();
  initMQTT();

  unsigned long now = millis();
  lastTempOkMs = now;
  lastHumOkMs = now;
  tempStuckStartMs = now;
  humStuckStartMs = now;
  soilStuckStartMs = now;
  
  // Initialize WiFi watchdog
  lastWifiConnectedMs = now;
  lastHttpSuccessMs = now;

  // ==========================================
  // สร้าง FreeRTOS Queue และ Persistent HTTP Task
  // ==========================================
  httpQueue = xQueueCreate(HTTP_QUEUE_SIZE, sizeof(HttpPayload));
  if (httpQueue == NULL) {
    DBG("[HTTP] Failed to create queue!\n");
  } else {
    DBG("[HTTP] Queue created (size=%d)\n", HTTP_QUEUE_SIZE);
  }
  
  // สร้าง Persistent Task (ทำงานตลอด ไม่ต้องสร้าง/ลบซ้ำ)
  xTaskCreatePinnedToCore(
    httpPersistentTask,   // Function
    "HTTP_Task",          // Name
    8192,                 // Stack size
    NULL,                 // Parameters
    1,                    // Priority (ต่ำ)
    &httpTaskHandle,      // Handle
    0                     // Core 0 (แยกจาก loop ที่ทำงานใน Core 1)
  );
  DBG("[HTTP] Persistent Task created on Core 0\n");

  mqtt.publish("hf/status/messages", "System Ready");
  mqtt.publish("hf/control/mode_status", "Auto");
  mqtt.publish("hf/status/online", "online");
  DBG("=== SETUP DONE ===\n");
}

// ==========================================
//          15. LOOP
// ==========================================
void loop() {
  // Reset Hardware Watchdog ทุกรอบ loop
  // ถ้า loop ค้าง จะไม่มีการ reset → ESP32 จะ restart อัตโนมัติ
  esp_task_wdt_reset();
  
  connectMQTT();  // เชื่อมต่อ HiveMQ Cloud
  mqtt.loop();
  mqttHeartbeat();  // ส่ง heartbeat ทุก 30 วินาที
  checkMqttHealth();  // ตรวจสอบว่า MQTT ยังทำงานจริง
  tryReconnectWiFi();
  checkHttpWatchdog();
  checkInternetPeriodically();  // เช็คเน็ตจริงสำหรับ SIM Router
  checkMasterWatchdog();  // Master watchdog - restart ถ้าไม่มี activity นานเกินไป
  handleOTA();    // OTA Update

  // Non-blocking AC Voltage sampling (ทำทุก loop)
  sampleACVoltage();
  calculateACVoltage();

  // Non-blocking Time Update (ทุก 1 วินาที แทนที่จะทุก loop)
  unsigned long now = millis();
  if (now - lastTimeUpdateMs >= TIME_UPDATE_INTERVAL) {
    lastTimeUpdateMs = now;
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 0)) {  // timeout = 0 (ไม่รอ)
      cachedHour = timeinfo.tm_hour;
      cachedMinute = timeinfo.tm_min;
      cachedSecond = timeinfo.tm_sec;
      cachedDayOfWeek = timeinfo.tm_wday;
      cachedTimeReady = true;
      
      // Update day/night status
      currentHour = cachedHour;
      isDayTime = (cachedHour >= DAY_START_HOUR && cachedHour < DAY_END_HOUR);
      
      // Reset daily flags at midnight
      if (cachedHour == 0 && cachedMinute == 0 && cachedSecond < 5) {
        hasWateredToday = false;
        hasFertilizedToday = false;
        dailyWaterVolume = 0.0;
      }
    }
  }
  
  // ใช้ cached values
  bool timeReady = cachedTimeReady;
  int hourNow = cachedHour;
  int minuteNow = cachedMinute;
  int secondNow = cachedSecond;
  int dayOfWeek = cachedDayOfWeek;

  // ---------------- AUTO MODE LOGIC ----------------
  if (ModeAuto) {
    // === Smart Watering Cycle State Machine ===
    // States: IDLE -> WATERING -> WAIT_CHECK -> (loop หรือ DONE)
    static enum { WC_IDLE, WC_WATERING, WC_WAIT_CHECK } waterCycleState = WC_IDLE;
    static unsigned long waterCycleStart = 0;
    static unsigned long waitCheckStart = 0;
    static int cycleCount = 0;
    
    // === Scheduled watering ===
    if (timeReady && !systemError && !hasWateredToday && waterCycleState == WC_IDLE) {
      if (hourNow == SET_WATER_HOUR && minuteNow == SET_WATER_MIN) {
        DBG("[AUTO] Time match W:%02d:%02d soil=%d target=%d\n",
            hourNow, minuteNow, (int)moisturePercentage, SOIL_TARGET_PERCENT);
        // เปลี่ยนจาก SET_SOIL_SCHED_MIN เป็น SET_SOIL_LOW (50%)
        if ((int)moisturePercentage < SET_SOIL_LOW) {
          DBG("[AUTO] Scheduled watering TRIGGER\n");
          
          // เช็คว่าวันนี้ต้องใส่ปุ๋ยด้วยไหม
          if ((dayOfWeek == SET_FER_DAY_1 || dayOfWeek == SET_FER_DAY_2) && !hasFertilizedToday) {
            DBG("[AUTO] Fertilizer day TRIGGER\n");
            startFertilizer("Auto: Scheduled Water+Fertilizer");
          } else {
            startWatering("Auto: Scheduled Watering");
            waterCycleState = WC_WATERING;
            waterCycleStart = millis();
            cycleCount = 1;
            DBG("[AUTO] Water cycle #%d START\n", cycleCount);
          }
        } else {
          DBG("[AUTO] Scheduled watering SKIP (soil >= %d%%)\n", SET_SOIL_LOW);
          mqtt.publish("hf/status/messages", "Skip watering: soil ok");
          hasWateredToday = true;  // ข้ามวันนี้ไป
        }
      }
    }

    // === Rescue mode (soil too dry) - เปลี่ยนเป็น SOIL_RESCUE_TRIGGER (45%) ===
    if (timeReady && !systemError && (hourNow >= CHECK_START_HOUR && hourNow <= CHECK_END_HOUR)) {
      if ((int)moisturePercentage < SOIL_RESCUE_TRIGGER && waterCycleState == WC_IDLE && !sol2_On) {
        unsigned long cd = millis() - lastWaterFinishTime;
        if (cd > WATER_COOLDOWN) {
          DBG("[AUTO] Rescue watering TRIGGER soil=%d < %d\n",
              (int)moisturePercentage, SOIL_RESCUE_TRIGGER);
          startWatering("Auto: Soil Dry Rescue");
          waterCycleState = WC_WATERING;
          waterCycleStart = millis();
          cycleCount = 1;
          DBG("[AUTO] Water cycle #%d START (Rescue)\n", cycleCount);
        }
      }
    }

    // === Water Cycle State Machine ===
    if (waterCycleState == WC_WATERING) {
      unsigned long elapsed = millis() - waterCycleStart;
      
      // เช็ค Soil ถึง Target หรือยัง (ระหว่างรดน้ำ)
      if ((int)moisturePercentage >= SOIL_TARGET_PERCENT) {
        DBG("[AUTO] Soil reached %d%% >= %d%% - STOP\n", (int)moisturePercentage, SOIL_TARGET_PERCENT);
        stopWatering("Auto: Soil Target Reached");
        waterCycleState = WC_IDLE;
        hasWateredToday = true;
        lastWaterFinishTime = millis();
        mqtt.publish("hf/status/messages", "Watering DONE: Soil target reached");
      }
      // รดครบ 2 นาที → หยุดและรอเช็ค
      else if (elapsed >= WATER_CYCLE_DURATION) {
        DBG("[AUTO] Water cycle #%d END (2 min) - wait 10s to check\n", cycleCount);
        stopWatering("Auto: Cycle Done, Checking...");
        waterCycleState = WC_WAIT_CHECK;
        waitCheckStart = millis();
      }
    }
    
    // === Wait and Check State ===
    if (waterCycleState == WC_WAIT_CHECK) {
      unsigned long waited = millis() - waitCheckStart;
      
      if (waited >= WATER_WAIT_CHECK) {
        // รอครบ 10 วินาที - เช็ค Soil
        DBG("[AUTO] Check soil after wait: %d%% (target=%d%%)\n", 
            (int)moisturePercentage, SOIL_TARGET_PERCENT);
        
        if ((int)moisturePercentage >= SOIL_TARGET_PERCENT) {
          // ถึง Target แล้ว - หยุด
          DBG("[AUTO] Soil OK! Watering complete.\n");
          waterCycleState = WC_IDLE;
          hasWateredToday = true;
          lastWaterFinishTime = millis();
          mqtt.publish("hf/status/messages", "Watering DONE: Soil target reached");
        } else {
          // ยังไม่ถึง Target - รดต่อ
          cycleCount++;
          DBG("[AUTO] Soil still low (%d%%) - Start cycle #%d\n", 
              (int)moisturePercentage, cycleCount);
          startWatering("Auto: Continue Watering");
          waterCycleState = WC_WATERING;
          waterCycleStart = millis();
          mqtt.publish("hf/status/messages", ("Watering cycle #" + String(cycleCount)).c_str());
        }
      }
    }
    
    // === Reset state if watering stopped externally ===
    if (waterCycleState != WC_IDLE && !sol2_On && !pumpA_On) {
      DBG("[AUTO] Water cycle interrupted externally\n");
      waterCycleState = WC_IDLE;
    }

    // === Fertilizer timer (B + S2) ===
    static unsigned long fertEventStart = 0;
    static bool fertEventActive = false;
    if (pumpB_On && !fertEventActive) {
      fertEventActive = true;
      fertEventStart = millis();
      DBG("[AUTO] Fert event START dur=%ds\n", SET_FER_DUR_SEC);
    }
    if (fertEventActive && (millis() - fertEventStart >= (unsigned long)SET_FER_DUR_SEC * 1000UL)) {
      DBG("[AUTO] Fert pump END, continue watering\n");
      // ปิดแค่ปั๊มปุ๋ย แต่ยังรดน้ำต่อ
      setPumpB(false, "Auto: Fert Timer Done");
      fertEventActive = false;
      hasFertilizedToday = true;
      
      // เริ่ม Water Cycle หลังใส่ปุ๋ย
      waterCycleState = WC_WATERING;
      waterCycleStart = millis();
      cycleCount = 1;
      DBG("[AUTO] Start water cycle after fertilizer\n");
    }
    if (!pumpB_On) fertEventActive = false;

    // === Water Tank Fill Control (A + S1) ===
    controlWaterTankFill();
  }
  // Manual mode: ควบคุมผ่าน MQTT commands เท่านั้น

  // ---------------- OUTPUT UPDATE ----------------
  // Pump และ Solenoid - ปิดเมื่อมี Pump Alarm
  bool allowPumpOutputs = !systemError && !pumpAlarmActive && !powerFailAlarm;
  
  // Fan, Mist, Light - ทำงานต่อแม้มี Pump Alarm
  bool allowClimateOutputs = !systemError && !powerFailAlarm;
  
  // เขียนค่าไปยัง Relay
  relayWrite(PUMP_A_PIN, allowPumpOutputs ? pumpA_On : false);
  relayWrite(PUMP_B_PIN, allowPumpOutputs ? pumpB_On : false);
  relayWrite(SOL_1_PIN, allowPumpOutputs ? sol1_On : false);
  relayWrite(SOL_2_PIN, allowPumpOutputs ? sol2_On : false);
  relayWrite(PUMP_FOGGY_PIN, allowClimateOutputs ? pumpFoggy_On : false);
  relayWrite(FAN_PIN, allowClimateOutputs ? fanOn : false);
  relayWrite(LIGHT_PIN, allowClimateOutputs ? lightOn : false);

  // Track pump A start time for safety
  bool pumpA_actual = allowPumpOutputs ? pumpA_On : false;
  if (!prevPumpA && pumpA_actual) {
    pumpA_StartTime = millis();
    DBG("[OUT] PUMP A start @%lu\n", pumpA_StartTime);
  }

  // Publish status on change - Dashboard compatible topics
  bool pumpB_actual = allowPumpOutputs ? pumpB_On : false;
  bool sol1_actual = allowPumpOutputs ? sol1_On : false;
  bool sol2_actual = allowPumpOutputs ? sol2_On : false;
  bool foggy_actual = allowClimateOutputs ? pumpFoggy_On : false;
  bool fan_actual = allowClimateOutputs ? fanOn : false;
  bool light_actual = allowClimateOutputs ? lightOn : false;

  // Calculate composite states for Dashboard
  bool water_actual = pumpA_actual && sol2_actual && !pumpB_actual;  // Water = A+S2 (no B)
  bool fert_actual = pumpB_actual;                                    // Fert = B active
  bool mist_actual = foggy_actual;                                    // Mist = Foggy
  bool pumpmain_actual = pumpA_actual && sol1_actual;                 // Pump Main = A+S1

  // Publish composite status (for Dashboard 01_Control page)
  static bool prevWater = false, prevFert = false, prevMist = false, prevPumpMain = false;
  
  if (water_actual != prevWater) {
    prevWater = water_actual;
    publishStatus("hf/status/pumpwater", water_actual);
    DBG("[PUB] pumpwater=%s\n", water_actual ? "On" : "Off");
  }
  
  if (fert_actual != prevFert) {
    prevFert = fert_actual;
    publishStatus("hf/status/fertilizer", fert_actual);
    DBG("[PUB] fertilizer=%s\n", fert_actual ? "On" : "Off");
  }
  
  if (mist_actual != prevMist) {
    prevMist = mist_actual;
    publishStatus("hf/status/mist", mist_actual);
    DBG("[PUB] mist=%s\n", mist_actual ? "On" : "Off");
  }
  
  if (pumpmain_actual != prevPumpMain) {
    prevPumpMain = pumpmain_actual;
    publishStatus("hf/status/pumpmain", pumpmain_actual);
    DBG("[PUB] pumpmain=%s\n", pumpmain_actual ? "On" : "Off");
  }

  // Publish individual relay status (for detailed monitoring)
  if (pumpA_actual != prevPumpA) {
    prevPumpA = pumpA_actual;
    publishStatus("hf/status/pump_a", pumpA_actual);
  }
  
  if (pumpB_actual != prevPumpB) {
    prevPumpB = pumpB_actual;
    publishStatus("hf/status/pump_b", pumpB_actual);
  }
  
  if (sol1_actual != prevSol1) {
    prevSol1 = sol1_actual;
    publishStatus("hf/status/sol1", sol1_actual);
  }
  
  if (sol2_actual != prevSol2) {
    prevSol2 = sol2_actual;
    publishStatus("hf/status/sol2", sol2_actual);
  }
  
  if (foggy_actual != prevPumpFoggy) {
    prevPumpFoggy = foggy_actual;
    publishStatus("hf/status/foggy", foggy_actual);
  }
  
  if (fan_actual != prevFan) {
    prevFan = fan_actual;
    publishStatus("hf/status/fan", fan_actual);
  }
  
  if (light_actual != prevLight) {
    prevLight = light_actual;
    publishStatus("hf/status/light", light_actual);
  }

  if (systemError != prevSystemError) {
    prevSystemError = systemError;
    publishStatus("hf/status/alarm", systemError);
  }

  // ---------------- PERIODIC TASKS ----------------

  // Read sensors
  if (now - lastReadMs >= SENSOR_READ_INTERVAL) {
    lastReadMs = now;
    readTemperatureC();
    readSensorSoil();
    readWaterLevel();
    // acVoltage อัพเดทจาก non-blocking sampling แล้ว
    humused = checkSensorHum();
    tempused = checkSensorTemp();

    DBG("[SENSOR] soil=%d%% raw=%lu T=%.1f H=%.0f flow=%.2f WL_Low=%d WL_High=%d AC=%.1fV\n",
        (int)moisturePercentage, moistureValue, tempused, humused, flowRateLpm,
        waterLevelLow, waterLevelHigh, acVoltage);

#if DEBUG_SENSOR_DETAIL
    DBG("[SENSOR-RAW] SHT1 T=%.2f H=%.2f | SHT2 T=%.2f H=%.2f\n", tempmain, hummain, temp2, hum2);
#endif

    // Publish sensor data
    if (!isnan(tempused)) mqtt.publish("hf/sensor/tempuse", String(tempused, 1).c_str());
    if (!isnan(humused)) mqtt.publish("hf/sensor/humuse", String(humused, 0).c_str());
    mqtt.publish("hf/sensor/soil", String(moisturePercentage).c_str());
    mqtt.publish("hf/sensor/soil_raw", String(moistureValue).c_str());
    mqtt.publish("hf/sensor/flow_lpm", String(flowRateLpm, 2).c_str());
    mqtt.publish("hf/sensor/water_vol", String(dailyWaterVolume, 2).c_str());
    mqtt.publish("hf/sensor/tank_low", waterLevelLow ? "1" : "0");
    mqtt.publish("hf/sensor/tank_high", waterLevelHigh ? "1" : "0");
    mqtt.publish("hf/sensor/ac_voltage", String(acVoltage, 1).c_str());
    mqtt.publish("hf/sensor/ac_power", acPowerOk ? "OK" : "FAIL");
    
    // WiFi Signal Strength (เพื่อ debug network issues)
    if (WiFi.status() == WL_CONNECTED) {
      mqtt.publish("hf/sensor/wifi_rssi", String(WiFi.RSSI()).c_str());
    }
  }
  
  // ตรวจสอบไฟดับ (ทำทุก loop เพื่อตอบสนองเร็ว)
  checkPowerFail();

  // Control logic
  if (now - lastControlMs >= CONTROL_INTERVAL) {
    lastControlMs = now;
    if (ModeAuto && !systemError) {
      DBG("[CTRL] Auto control tick - State: %s, Day: %d\n", 
          getClimateStateName(currentClimateState), isDayTime);
      
      // Climate State Machine (ควบคุม Fan, Humidifier, Light)
      updateClimateState();
      executeClimateControl();
      
      // Light control (แยกออกมา แต่ถูก override โดย OVERTEMP)
      if (currentClimateState != STATE_OVERTEMP) {
        controlLightSystem(hourNow);
      }
    } else {
      DBG("[CTRL] Skip (ModeAuto=%d systemError=%d)\n", ModeAuto, systemError);
    }
  }

  // Google sheet logging (ส่งผ่าน FreeRTOS Queue)
  // ใส่ Queue เสมอ ไม่ว่า WiFi จะต่อหรือไม่ → Task จะส่งเมื่อ WiFi กลับมา
  if (now - lastDatalogsMs >= LOG_INTERVAL) {
    lastDatalogsMs = now;
    
    if (httpQueue != NULL) {
      // เตรียมข้อมูล
      HttpPayload payload;
      payload.temp = tempused;
      payload.hum = humused;
      payload.soil = moisturePercentage;
      strncpy(payload.mode, ModeAuto ? "AUTO" : "MANUAL", sizeof(payload.mode));
      strncpy(payload.climateState, getClimateStateName(currentClimateState), sizeof(payload.climateState));
      payload.waterVol = dailyWaterVolume;
      
      // Alarm: รวม Power Fail ด้วย
      if (powerFailAlarm) {
        strncpy(payload.alarm, "POWER FAIL", sizeof(payload.alarm) - 1);
      } else {
        strncpy(payload.alarm, currentErrorMsg.c_str(), sizeof(payload.alarm) - 1);
      }
      payload.alarm[sizeof(payload.alarm) - 1] = '\0';
      
      // ส่งเข้า Queue (ไม่รอ ถ้าเต็มจะ fail ทันที)
      if (xQueueSend(httpQueue, &payload, 0) != pdTRUE) {
        // Queue เต็ม! ดึงตัวเก่าออก 1 ตัวก่อน แล้วส่งใหม่
        HttpPayload discarded;
        if (xQueueReceive(httpQueue, &discarded, 0) == pdTRUE) {
          httpQueueFullCount++;
          DBG("[LOG] Queue full! Dropped oldest (fullCount=%lu)\n", httpQueueFullCount);
          // ส่งข้อมูลใหม่เข้าไปแทน
          xQueueSend(httpQueue, &payload, 0);
        }
      }
      
      int inQueue = uxQueueMessagesWaiting(httpQueue);
      bool wifiOk = (WiFi.status() == WL_CONNECTED);
      DBG("[LOG] Data queued (inQueue=%d, WiFi=%s)\n", 
          inQueue, wifiOk ? "OK" : "OFFLINE");
    }
  }

  runSafetyChecks();
  updateLCD();
  debugTick(timeReady, hourNow, minuteNow, secondNow, dayOfWeek);
}

// ==========================================
//          16. INIT PINS / SENSORS / WIFI
// ==========================================
void initPins() {
  // ⚠️ CRITICAL: Force all relay pins LOW IMMEDIATELY
  // ก่อน pinMode เพื่อป้องกัน relay ติดค้างตอน boot/OTA
  // GPIO 5 มี pull-up ตอน boot ทำให้ MIST อาจติดค้าง
  digitalWrite(PUMP_FOGGY_PIN, LOW);  // R1 MIST - GPIO 5 มีปัญหาตอน boot!
  digitalWrite(SOL_1_PIN, LOW);
  digitalWrite(SOL_2_PIN, LOW);
  digitalWrite(PUMP_A_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(LIGHT_PIN, LOW);
  digitalWrite(PUMP_B_PIN, LOW);
  digitalWrite(STATUS_LAMP_RELAY_PIN, LOW);
  
  // Output pins (Relays)
  pinMode(PUMP_A_PIN, OUTPUT);
  pinMode(PUMP_B_PIN, OUTPUT);
  pinMode(SOL_1_PIN, OUTPUT);
  pinMode(SOL_2_PIN, OUTPUT);
  pinMode(PUMP_FOGGY_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(STATUS_LAMP_RELAY_PIN, OUTPUT);

  // Initialize all relays OFF (อีกครั้งหลัง pinMode)
  relayWrite(PUMP_A_PIN, false);
  relayWrite(PUMP_B_PIN, false);
  relayWrite(SOL_1_PIN, false);
  relayWrite(SOL_2_PIN, false);
  relayWrite(PUMP_FOGGY_PIN, false);
  relayWrite(FAN_PIN, false);
  relayWrite(LIGHT_PIN, false);
  relayWrite(STATUS_LAMP_RELAY_PIN, false);  // Green ON (Normal)

  // Input pins
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  
  // Float Switch - GPIO 16, 17 มี Internal Pull-up (ไม่ conflict กับ WiFi)
  // ✅ ต่อตรงได้เลย: GPIO ---- [Float Switch] ---- GND
  // ⚠️ Flow Sensor ย้ายไป GPIO 27
  pinMode(FLOAT_LOW_PIN, INPUT_PULLUP);   // GPIO 16 - Internal Pull-up
  pinMode(FLOAT_HIGH_PIN, INPUT_PULLUP);  // GPIO 17 - Internal Pull-up
  
  // ZMPT101B Voltage Sensor (Analog Input)
  pinMode(ZMPT101B_PIN, INPUT);
  // ตั้งค่า ADC resolution 12-bit (0-4095)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // อ่านได้ถึง 3.3V
}

void initSensors() {
  // I2C Bus #1: SHT40 #1 + LCD (GPIO 21/22)
  Wire.begin(I2C1_SDA, I2C1_SCL);
  if (!shtMain.begin(&Wire)) DBG("[ERR] SHT #1 begin fail\n");
  else DBG("[OK] SHT #1 on I2C Bus #1 (GPIO %d/%d)\n", I2C1_SDA, I2C1_SCL);

  // I2C Bus #2: SHT40 #2 (GPIO 32/33)
  I2C_1.begin(I2C2_SDA, I2C2_SCL);
  if (!sht2.begin(&I2C_1)) DBG("[ERR] SHT #2 begin fail\n");
  else DBG("[OK] SHT #2 on I2C Bus #2 (GPIO %d/%d)\n", I2C2_SDA, I2C2_SCL);
}

IPAddress local_IP(192, 168, 1, 200);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

void connectWIFI() {
  DBG("Connecting WiFi SSID=%s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    DBG("STA Failed to configure\n");
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    DBG(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    DBG("\nWiFi Connected IP=%s\n", WiFi.localIP().toString().c_str());
    configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Connected!");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    delay(1200);
  } else {
    DBG("\nWiFi NOT Connected (Offline Mode)\n");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Failed!");
    lcd.setCursor(0, 1);
    lcd.print("Offline Mode");
    delay(1200);
  }
}

void tryReconnectWiFi() {
  unsigned long now = millis();
  
  // ถ้า WiFi ต่ออยู่ → อัพเดทเวลาล่าสุด
  if (WiFi.status() == WL_CONNECTED) {
    lastWifiConnectedMs = now;
    return;
  }
  
  // WiFi หลุด
  wifiDisconnectCount++;
  
  // ตรวจสอบว่าหลุดนานแค่ไหน
  unsigned long disconnectDuration = now - lastWifiConnectedMs;
  
  // ถ้าหลุดนานเกิน WIFI_WATCHDOG_TIMEOUT → restart WiFi แบบเต็มรูปแบบ
  if (disconnectDuration > WIFI_WATCHDOG_TIMEOUT) {
    DBG("[WIFI] Watchdog triggered! Disconnected for %lu ms. Full restart...\n", disconnectDuration);
    wifiRestartCount++;
    
    // Full WiFi restart
    WiFi.disconnect(true);  // true = ลบ config เดิม
    delay(1000);
    WiFi.mode(WIFI_OFF);
    delay(1000);
    WiFi.mode(WIFI_STA);
    delay(500);
    
    // ตั้งค่า Static IP ใหม่
    WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    // รอต่อ WiFi (สูงสุด 15 วินาที)
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 30) {
      delay(500);
      DBG(".");
      retry++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      lastWifiConnectedMs = millis();
      lastWifiCheck = millis();
      DBG("\n[WIFI] Reconnected after full restart! IP=%s\n", WiFi.localIP().toString().c_str());
    } else {
      DBG("\n[WIFI] Full restart failed. Will retry...\n");
    }
    return;
  }
  
  // ถ้ายังไม่ถึง watchdog timeout → ลอง reconnect ปกติ
  if (now - lastWifiCheck > WIFI_RECONNECT_INTERVAL) {
    lastWifiCheck = now;
    DBG("[WIFI] Reconnect attempt... (disconnected %lu ms)\n", disconnectDuration);
    WiFi.disconnect();
    delay(100);
    WiFi.reconnect();
  }
}

// ==========================================
// Watchdog 2 ระดับ (Escalation Strategy)
// ==========================================
// ระดับ 1: ส่งไม่ได้ 10 นาที → WiFi.disconnect()
// ระดับ 2: ส่งไม่ได้ 120 นาที → ESP.restart()
// ==========================================

void checkHttpWatchdog() {
  unsigned long now = millis();
  
  // ถ้าไม่เคยส่งสำเร็จเลย (เพิ่งเริ่ม) → ตั้งค่าเริ่มต้น
  if (lastHttpSuccessMs == 0) {
    lastHttpSuccessMs = now;
    return;
  }
  
  unsigned long timeSinceSuccess = now - lastHttpSuccessMs;
  
  // Log สถานะทุก 5 นาที
  static unsigned long lastWatchdogLogMs = 0;
  if (now - lastWatchdogLogMs > 300000) {  // ทุก 5 นาที
    lastWatchdogLogMs = now;
    DBG("[WATCHDOG] Status: HTTP last success %lu min ago, WiFi=%s\n",
        timeSinceSuccess / 60000,
        WiFi.status() == WL_CONNECTED ? "OK" : "DISCONNECTED");
  }
  
  // ==========================================
  // ระดับ 2: ส่งไม่ได้ 30 นาที → ESP.restart() (ลดจาก 120 นาที)
  // ==========================================
  const unsigned long HARD_RESTART_TIMEOUT = 1800000;  // 30 นาที
  if (timeSinceSuccess > HARD_RESTART_TIMEOUT) {
    DBG("\n");
    DBG("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    DBG("[WATCHDOG] CRITICAL! No HTTP for %lu min\n", timeSinceSuccess / 60000);
    DBG("[WATCHDOG] HARD RESTART NOW!\n");
    DBG("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    
    // แสดงบน LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WATCHDOG REBOOT");
    lcd.setCursor(0, 1);
    char buf[17];
    snprintf(buf, sizeof(buf), "No HTTP %lu min", timeSinceSuccess / 60000);
    lcd.print(buf);
    delay(2000);
    
    // Restart ESP32
    ESP.restart();
  }
  
  // ==========================================
  // ระดับ 1: ส่งไม่ได้ 5 นาที → WiFi.disconnect() + reconnect (ลดจาก 10 นาที)
  // ==========================================
  static unsigned long lastLevel1TriggerMs = 0;
  const unsigned long LEVEL1_COOLDOWN = 180000;  // ทำได้ทุก 3 นาที
  const unsigned long SOFT_RESTART_TIMEOUT = 300000;  // 5 นาที
  
  if (timeSinceSuccess > SOFT_RESTART_TIMEOUT) {
    // ป้องกันไม่ให้ trigger ถี่เกินไป
    if (now - lastLevel1TriggerMs > LEVEL1_COOLDOWN) {
      lastLevel1TriggerMs = now;
      
      DBG("\n");
      DBG("[WATCHDOG] WARNING! No HTTP for %lu min\n", timeSinceSuccess / 60000);
      DBG("[WATCHDOG] Force WiFi restart...\n");
      
      // Full WiFi restart
      WiFi.disconnect(true);
      delay(2000);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      wifiRestartCount++;
      
      // รอ WiFi reconnect
      int retry = 0;
      while (WiFi.status() != WL_CONNECTED && retry < 20) {
        delay(500);
        retry++;
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        DBG("[WATCHDOG] WiFi reconnected! IP=%s\n", WiFi.localIP().toString().c_str());
      } else {
        DBG("[WATCHDOG] WiFi reconnect FAILED!\n");
      }
    }
  }
}

// ==========================================
// Internet Connectivity Check (สำหรับ SIM Router)
// ==========================================
bool checkInternetConnectivity() {
  if (WiFi.status() != WL_CONNECTED) return false;
  
  HTTPClient http;
  http.begin("http://clients3.google.com/generate_204");
  http.setTimeout(5000);
  http.setReuse(false);
  
  int httpCode = http.GET();
  http.end();
  
  // Response 204 = Internet OK
  return (httpCode == 204);
}

// เรียกใช้ใน loop ทุก 5 นาที
void checkInternetPeriodically() {
  static unsigned long lastInternetCheckMs = 0;
  const unsigned long INTERNET_CHECK_INTERVAL = 300000;  // 5 นาที
  
  unsigned long now = millis();
  if (now - lastInternetCheckMs < INTERNET_CHECK_INTERVAL) return;
  lastInternetCheckMs = now;
  
  // WiFi connected แต่ internet ไม่ออก (SIM Router ปัญหา)
  if (WiFi.status() == WL_CONNECTED && !checkInternetConnectivity()) {
    DBG("[NET] WiFi OK but no Internet! Forcing reconnect...\n");
    WiFi.disconnect(true);
    delay(1000);
    WiFi.reconnect();
    wifiRestartCount++;
  }
}

// ==========================================
// MASTER WATCHDOG - ตรวจสอบทุกระบบ
// ==========================================
// ถ้าทั้ง HTTP และ MQTT ไม่ทำงานพร้อมกัน = มีปัญหาใหญ่
// จะ restart ESP32 ทันที
// ==========================================
void checkMasterWatchdog() {
  static unsigned long lastMasterCheckMs = 0;
  const unsigned long MASTER_CHECK_INTERVAL = 60000;  // เช็คทุก 1 นาที
  const unsigned long MASTER_TIMEOUT = 900000;        // 15 นาที = restart
  
  unsigned long now = millis();
  if (now - lastMasterCheckMs < MASTER_CHECK_INTERVAL) return;
  lastMasterCheckMs = now;
  
  // คำนวณเวลาที่ไม่มี activity
  unsigned long httpAge = (lastHttpSuccessMs > 0) ? (now - lastHttpSuccessMs) : 0;
  unsigned long mqttAge = (lastMqttActivity > 0) ? (now - lastMqttActivity) : 0;
  
  // Log สถานะ
  DBG("[MASTER] HTTP=%lu min ago, MQTT=%lu min ago, WiFi=%s\n",
      httpAge / 60000, mqttAge / 60000,
      WiFi.status() == WL_CONNECTED ? "OK" : "DOWN");
  
  // ถ้าทั้ง HTTP และ MQTT หยุดทำงานนานเกิน 15 นาที
  bool httpDead = (httpAge > MASTER_TIMEOUT);
  bool mqttDead = (mqttAge > MASTER_TIMEOUT);
  
  if (httpDead && mqttDead) {
    DBG("\n");
    DBG("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    DBG("[MASTER] BOTH HTTP AND MQTT DEAD!\n");
    DBG("[MASTER] HTTP: %lu min, MQTT: %lu min\n", httpAge / 60000, mqttAge / 60000);
    DBG("[MASTER] EMERGENCY RESTART!\n");
    DBG("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    
    // ==========================================
    // EMERGENCY SHUTDOWN SEQUENCE
    // ==========================================
    
    // 1. ปิดอุปกรณ์ทั้งหมดเพื่อความปลอดภัย
    DBG("[EMERGENCY] Shutting down all outputs...\n");
    relayWrite(SOL_1_PIN, false);       // ปิด Solenoid 1
    relayWrite(SOL_2_PIN, false);       // ปิด Solenoid 2
    relayWrite(PUMP_A_PIN, false);      // ปิด Pump A
    relayWrite(PUMP_B_PIN, false);      // ปิด Pump B
    relayWrite(PUMP_FOGGY_PIN, false);  // ปิด Mist
    relayWrite(FAN_PIN, false);         // ปิด Fan
    relayWrite(LIGHT_PIN, false);       // ปิด Light
    
    // 2. อัพเดทสถานะในหน่วยความจำ
    sol1_On = false;
    sol2_On = false;
    pumpA_On = false;
    pumpB_On = false;
    pumpFoggy_On = false;
    fanOn = false;
    lightOn = false;
    
    // 3. บันทึกจำนวน restart ลง Preferences
    preferences.begin("farm_cfg", false);
    unsigned long restartCount = preferences.getULong("restart_cnt", 0);
    restartCount++;
    preferences.putULong("restart_cnt", restartCount);
    preferences.end();
    DBG("[EMERGENCY] Restart count: %lu\n", restartCount);
    
    // 4. แสดงบน LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("EMERGENCY RESTART");
    lcd.setCursor(0, 1);
    char buf[17];
    snprintf(buf, sizeof(buf), "Count: %lu", restartCount);
    lcd.print(buf);
    delay(2000);
    
    // 5. Beep เตือน 3 ครั้ง
    for (int i = 0; i < 3; i++) {
      digitalWrite(STATUS_LAMP_RELAY_PIN, HIGH);
      delay(200);
      digitalWrite(STATUS_LAMP_RELAY_PIN, LOW);
      delay(200);
    }
    
    // 6. Restart ESP32
    DBG("[EMERGENCY] Restarting NOW!\n");
    delay(500);
    ESP.restart();
  }
  
  // ถ้าแค่อันใดอันหนึ่งหยุด แต่ WiFi ยังเชื่อมอยู่ → ลอง reconnect
  if ((httpDead || mqttDead) && WiFi.status() == WL_CONNECTED) {
    DBG("[MASTER] Partial failure detected, forcing MQTT reconnect\n");
    mqtt.disconnect();
    espClient.stop();
  }
}

// ==========================================
//          17. PREFERENCES LOAD/SAVE
// ==========================================
void loadSettings() {
  preferences.begin("farm_cfg", true);

  SET_WATER_HOUR = preferences.getInt("w_hr", 9);
  SET_WATER_MIN = preferences.getInt("w_min", 0);
  SET_WATER_DUR_SEC = preferences.getInt("w_dur", 15);

  SET_LIGHT_START_HOUR = preferences.getInt("l_start", 19);  // Default: เปิด 19:00 น.
  SET_LIGHT_DUR_HOUR = preferences.getInt("l_dur", 9);        // Default: เปิด 9 ชม. = ปิด 04:00 น.

  SET_FER_DAY_1 = preferences.getInt("f_d1", 3);
  SET_FER_DAY_2 = preferences.getInt("f_d2", 0);
  SET_FER_DUR_SEC = preferences.getInt("f_dur", 10);

  // ค่าเก่า (ไม่ได้ใช้แล้ว แต่เก็บไว้ compatibility)
  SET_TEMP_ON = preferences.getFloat("t_on", 30.0);
  SET_TEMP_OFF = preferences.getFloat("t_off", 28.0);
  SET_HUM_ON = preferences.getFloat("h_on", 50.0);
  SET_HUM_OFF = preferences.getFloat("h_off", 80.0);

  SET_SOIL_LOW = preferences.getInt("s_low", 50);
  SET_SOIL_SCHED_MIN = preferences.getInt("s_sch", 70);

  mistOnTimeMs = preferences.getULong("m_on", 20000);
  mistOffTimeMs = preferences.getULong("m_off", 60000);

  // ============ Climate State Thresholds ============
  TEMP_OVERTEMP_ENTER = preferences.getFloat("ot_enter", 29.0);
  TEMP_OVERTEMP_EXIT = preferences.getFloat("ot_exit", 26.0);
  OVERTEMP_EXIT_HOLD_MS = preferences.getULong("ot_hold", 120000);
  
  RH_HIGH_ENTER = preferences.getFloat("rh_hi_en", 75.0);
  RH_HIGH_EXIT = preferences.getFloat("rh_hi_ex", 70.0);
  
  RH_LOW_ENTER = preferences.getFloat("rh_lo_en", 55.0);
  RH_LOW_EXIT = preferences.getFloat("rh_lo_ex", 68.0);
  
  RH_GUARD_MAX = preferences.getFloat("rh_guard", 78.0);
  
  NIGHT_FAN_ON_MS = preferences.getULong("nf_on", 30000);
  NIGHT_FAN_OFF_MS = preferences.getULong("nf_off", 300000);
  
  DAY_START_HOUR = preferences.getInt("day_st", 7);
  DAY_END_HOUR = preferences.getInt("day_en", 19);

  preferences.end();

  // Constrain values
  SET_WATER_HOUR = constrain(SET_WATER_HOUR, 0, 23);
  SET_WATER_MIN = constrain(SET_WATER_MIN, 0, 59);
  SET_WATER_DUR_SEC = constrain(SET_WATER_DUR_SEC, 1, 600);
  SET_LIGHT_START_HOUR = constrain(SET_LIGHT_START_HOUR, 0, 23);
  SET_LIGHT_DUR_HOUR = constrain(SET_LIGHT_DUR_HOUR, 0, 24);
  SET_FER_DAY_1 = constrain(SET_FER_DAY_1, 0, 6);
  SET_FER_DAY_2 = constrain(SET_FER_DAY_2, 0, 6);
  SET_FER_DUR_SEC = constrain(SET_FER_DUR_SEC, 1, 600);
  SET_SOIL_LOW = constrain(SET_SOIL_LOW, 0, 100);
  SET_SOIL_SCHED_MIN = constrain(SET_SOIL_SCHED_MIN, 0, 100);
  if (mistOnTimeMs < 1000) mistOnTimeMs = 1000;
  if (mistOffTimeMs < 1000) mistOffTimeMs = 1000;
  
  // Constrain Climate thresholds
  TEMP_OVERTEMP_ENTER = constrain(TEMP_OVERTEMP_ENTER, 25.0, 40.0);
  TEMP_OVERTEMP_EXIT = constrain(TEMP_OVERTEMP_EXIT, 20.0, 35.0);
  RH_HIGH_ENTER = constrain(RH_HIGH_ENTER, 60.0, 95.0);
  RH_HIGH_EXIT = constrain(RH_HIGH_EXIT, 50.0, 90.0);
  RH_LOW_ENTER = constrain(RH_LOW_ENTER, 30.0, 70.0);
  RH_LOW_EXIT = constrain(RH_LOW_EXIT, 40.0, 80.0);
  RH_GUARD_MAX = constrain(RH_GUARD_MAX, 70.0, 95.0);
  DAY_START_HOUR = constrain(DAY_START_HOUR, 0, 23);
  DAY_END_HOUR = constrain(DAY_END_HOUR, 0, 23);

  DBG("[CFG] Water %02d:%02d dur=%ds | SoilLow=%d SoilSch=%d\n",
      SET_WATER_HOUR, SET_WATER_MIN, SET_WATER_DUR_SEC,
      SET_SOIL_LOW, SET_SOIL_SCHED_MIN);
  DBG("[CFG] Climate: OT_Enter=%.1f OT_Exit=%.1f RH_Hi=%.1f/%.1f RH_Lo=%.1f/%.1f\n",
      TEMP_OVERTEMP_ENTER, TEMP_OVERTEMP_EXIT, 
      RH_HIGH_ENTER, RH_HIGH_EXIT, RH_LOW_ENTER, RH_LOW_EXIT);
}

void saveSettingInt(const char* key, int value) {
  preferences.begin("farm_cfg", false);
  preferences.putInt(key, value);
  preferences.end();
}

void saveSettingFloat(const char* key, float value) {
  preferences.begin("farm_cfg", false);
  preferences.putFloat(key, value);
  preferences.end();
}

void saveSettingULong(const char* key, unsigned long value) {
  preferences.begin("farm_cfg", false);
  preferences.putULong(key, value);
  preferences.end();
}

// ==========================================
//          18. MQTT CALLBACK (HiveMQ Cloud)
// ==========================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // อัพเดท activity time เมื่อรับ message
  lastMqttActivity = millis();
  
  // Convert payload to string
  char payloadStr[length + 1];
  memcpy(payloadStr, payload, length);
  payloadStr[length] = '\0';
  
  String strPayload = String(payloadStr);
  DBG("[MQTT] topic=%s payload=%s\n", topic, payloadStr);

  // -------- OTA UPDATE --------
  if (strcmp(topic, "hf/ota/update") == 0) {
    if (strPayload.startsWith("http://") || strPayload.startsWith("https://")) {
      otaUrl = strPayload;
      otaInProgress = true;
      DBG("[OTA] Received firmware URL: %s\n", otaUrl.c_str());
      mqtt.publish("hf/ota/status", "OTA: Download starting...");
    } else if (strPayload == "version") {
      String versionMsg = "Firmware: " + String(FIRMWARE_VERSION);
      mqtt.publish("hf/ota/status", versionMsg.c_str());
    }
    return;
  }

  // -------- MODE CONTROL --------
  if (strcmp(topic, "hf/mode/cmd") == 0) {
    String s = strPayload;
    s.trim();
    s.toLowerCase();
    if (s == "manual") {
      ModeAuto = false;
      mqtt.publish("hf/control/mode_status", "Manual");
      mqtt.publish("hf/status/messages", "Mode -> MANUAL");
      beep(2, 100, 100);
      DBG("[MODE] set MANUAL\n");
    } else if (s == "auto") {
      ModeAuto = true;
      mqtt.publish("hf/control/mode_status", "Auto");
      mqtt.publish("hf/status/messages", "Mode -> AUTO");
      beep(1, 200, 0);
      DBG("[MODE] set AUTO\n");
    } else {
      mqtt.publish("hf/status/messages", "Mode cmd invalid (Auto/Manual)");
    }
    return;
  }

  // -------- SYNC REQUEST --------
  if (strcmp(topic, "hf/status/online") == 0) {
    if (strPayload == "sync") {
      mqtt.publish("hf/status/online", "online", true);
      delay(10);
      mqtt.publish("hf/control/mode_status", ModeAuto ? "Auto" : "Manual", true);
      delay(10);
      mqtt.publish("hf/status/climate_state", getClimateStateName(currentClimateState));
      delay(10);
      mqtt.publish("hf/status/is_daytime", isDayTime ? "1" : "0");
      delay(10);
      
      publishStatus("hf/status/pumpwater", sol2_On && pumpA_On && !pumpB_On);
      delay(10);
      publishStatus("hf/status/mist", pumpFoggy_On);
      delay(10);
      publishStatus("hf/status/fertilizer", pumpB_On);
      delay(10);
      publishStatus("hf/status/fan", fanOn);
      delay(10);
      publishStatus("hf/status/light", lightOn);
      delay(10);
      publishStatus("hf/status/pumpmain", sol1_On && pumpA_On);
      delay(10);
      publishStatus("hf/status/alarm", systemError);
      delay(10);
      
      publishStatus("hf/status/pump_a", pumpA_On);
      delay(10);
      publishStatus("hf/status/pump_b", pumpB_On);
      delay(10);
      publishStatus("hf/status/sol1", sol1_On);
      delay(10);
      publishStatus("hf/status/sol2", sol2_On);
      delay(10);
      publishStatus("hf/status/foggy", pumpFoggy_On);
      delay(10);
      
      mqtt.publish("hf/sensor/tank_low", waterLevelLow ? "1" : "0");
      delay(10);
      mqtt.publish("hf/sensor/tank_high", waterLevelHigh ? "1" : "0");
      delay(10);
      mqtt.publish("hf/sensor/ac_voltage", String(acVoltage, 1).c_str());
      delay(10);
      mqtt.publish("hf/status/power_fail", powerFailAlarm ? "1" : "0");
      
      DBG("[SYNC] All status published\n");
    }
    return;
  }

  // -------- RESET ALARM --------
  if (strcmp(topic, "hf/system/reset_alarm") == 0) {
    DBG("[ALARM] reset requested\n");
    clearAlarm("System Normal (Remote Reset)");
    return;
  }

  // -------- REBOOT --------
  if (strcmp(topic, "hf/system/reboot") == 0) {
    mqtt.publish("hf/status/messages", "System Rebooting...");
    DBG("[SYS] reboot\n");
    delay(500);
    ESP.restart();
    return;
  }

  // ======== MANUAL CONTROL COMMANDS ========
  
  // --- WATER ---
  if (strcmp(topic, "hf/pumpwater/cmd") == 0 || strcmp(topic, "hf/water/cmd") == 0) {
    bool req = wantOnPayload(payloadStr);
    if (ModeAuto) { mqtt.publish("hf/status/messages", "Blocked: AUTO mode"); return; }
    if (systemError && req) { mqtt.publish("hf/status/messages", "Blocked: systemError"); return; }
    if (req) startWatering("Manual: Water ON");
    else stopWatering("Manual: Water OFF");
    return;
  }

  // --- FERTILIZER ---
  if (strcmp(topic, "hf/fertilizer/cmd") == 0) {
    bool req = wantOnPayload(payloadStr);
    if (ModeAuto) { mqtt.publish("hf/status/messages", "Blocked: AUTO mode"); return; }
    if (systemError && req) { mqtt.publish("hf/status/messages", "Blocked: systemError"); return; }
    if (req) startFertilizer("Manual: Fertilizer ON");
    else stopFertilizer("Manual: Fertilizer OFF");
    return;
  }

  // --- MIST ---
  if (strcmp(topic, "hf/mist/cmd") == 0) {
    bool req = wantOnPayload(payloadStr);
    if (ModeAuto) { mqtt.publish("hf/status/messages", "Blocked: AUTO mode"); return; }
    if (systemError && req) { mqtt.publish("hf/status/messages", "Blocked: systemError"); return; }
    if (req) startMisting("Manual: Mist ON");
    else stopMisting("Manual: Mist OFF");
    return;
  }

  // --- PUMP MAIN (Tank Fill) ---
  if (strcmp(topic, "hf/pumpmain/cmd") == 0 || strcmp(topic, "hf/tankfill/cmd") == 0) {
    bool req = wantOnPayload(payloadStr);
    if (ModeAuto) { mqtt.publish("hf/status/messages", "Blocked: AUTO mode"); return; }
    if (systemError && req) { mqtt.publish("hf/status/messages", "Blocked: systemError"); return; }
    if (req) startTankFill("Manual: Tank Fill ON");
    else stopTankFill("Manual: Tank Fill OFF");
    return;
  }

  // --- FAN ---
  if (strcmp(topic, "hf/fan/cmd") == 0) {
    bool req = wantOnPayload(payloadStr);
    if (ModeAuto) { mqtt.publish("hf/status/messages", "Blocked: AUTO mode"); return; }
    if (systemError && req) { mqtt.publish("hf/status/messages", "Blocked: systemError"); return; }
    setFan(req, req ? "Manual: Fan ON" : "Manual: Fan OFF");
    return;
  }

  // --- LIGHT ---
  if (strcmp(topic, "hf/light/cmd") == 0) {
    bool req = wantOnPayload(payloadStr);
    if (ModeAuto) { mqtt.publish("hf/status/messages", "Blocked: AUTO mode"); return; }
    setLight(req, req ? "Manual: Light ON" : "Manual: Light OFF");
    return;
  }

  // ======== SETTINGS ========
  if (strcmp(topic, "hf/set/water_hour") == 0) {
    SET_WATER_HOUR = constrain(strPayload.toInt(), 0, 23);
    saveSettingInt("w_hr", SET_WATER_HOUR);
    mqtt.publish("hf/status/messages", ("Set Water Hour: " + String(SET_WATER_HOUR)).c_str());
    return;
  }
  if (strcmp(topic, "hf/set/water_min") == 0) {
    SET_WATER_MIN = constrain(strPayload.toInt(), 0, 59);
    saveSettingInt("w_min", SET_WATER_MIN);
    mqtt.publish("hf/status/messages", ("Set Water Min: " + String(SET_WATER_MIN)).c_str());
    return;
  }
  if (strcmp(topic, "hf/set/water_dur") == 0) {
    SET_WATER_DUR_SEC = constrain(strPayload.toInt(), 1, 600);
    saveSettingInt("w_dur", SET_WATER_DUR_SEC);
    mqtt.publish("hf/status/messages", ("Set Water Dur: " + String(SET_WATER_DUR_SEC)).c_str());
    return;
  }
  if (strcmp(topic, "hf/set/light_start") == 0) {
    SET_LIGHT_START_HOUR = constrain(strPayload.toInt(), 0, 23);
    saveSettingInt("l_start", SET_LIGHT_START_HOUR);
    mqtt.publish("hf/status/messages", ("Set Light Start: " + String(SET_LIGHT_START_HOUR)).c_str());
    return;
  }
  if (strcmp(topic, "hf/set/light_dur") == 0) {
    SET_LIGHT_DUR_HOUR = constrain(strPayload.toInt(), 0, 24);
    saveSettingInt("l_dur", SET_LIGHT_DUR_HOUR);
    mqtt.publish("hf/status/messages", ("Set Light Dur: " + String(SET_LIGHT_DUR_HOUR)).c_str());
    return;
  }
  if (strcmp(topic, "hf/set/fer_day1") == 0) {
    SET_FER_DAY_1 = constrain(strPayload.toInt(), 0, 6);
    saveSettingInt("f_d1", SET_FER_DAY_1);
    mqtt.publish("hf/status/messages", ("Set Fer Day1: " + String(SET_FER_DAY_1)).c_str());
    return;
  }
  if (strcmp(topic, "hf/set/fer_day2") == 0) {
    SET_FER_DAY_2 = constrain(strPayload.toInt(), 0, 6);
    saveSettingInt("f_d2", SET_FER_DAY_2);
    mqtt.publish("hf/status/messages", ("Set Fer Day2: " + String(SET_FER_DAY_2)).c_str());
    return;
  }
  if (strcmp(topic, "hf/set/fer_dur") == 0) {
    SET_FER_DUR_SEC = constrain(strPayload.toInt(), 1, 600);
    saveSettingInt("f_dur", SET_FER_DUR_SEC);
    mqtt.publish("hf/status/messages", ("Set Fer Dur: " + String(SET_FER_DUR_SEC)).c_str());
    return;
  }
  if (strcmp(topic, "hf/set/temp_on") == 0) {
    SET_TEMP_ON = strPayload.toFloat();
    saveSettingFloat("t_on", SET_TEMP_ON);
    mqtt.publish("hf/status/messages", ("Set Temp ON: " + String(SET_TEMP_ON)).c_str());
    return;
  }
  if (strcmp(topic, "hf/set/temp_off") == 0) {
    SET_TEMP_OFF = strPayload.toFloat();
    saveSettingFloat("t_off", SET_TEMP_OFF);
    mqtt.publish("hf/status/messages", ("Set Temp OFF: " + String(SET_TEMP_OFF)).c_str());
    return;
  }
  if (strcmp(topic, "hf/set/hum_on") == 0) {
    SET_HUM_ON = strPayload.toFloat();
    saveSettingFloat("h_on", SET_HUM_ON);
    mqtt.publish("hf/status/messages", ("Set Hum ON: " + String(SET_HUM_ON)).c_str());
    return;
  }
  if (strcmp(topic, "hf/set/hum_off") == 0) {
    SET_HUM_OFF = strPayload.toFloat();
    saveSettingFloat("h_off", SET_HUM_OFF);
    mqtt.publish("hf/status/messages", ("Set Hum OFF: " + String(SET_HUM_OFF)).c_str());
    return;
  }
  if (strcmp(topic, "hf/set/soil_low") == 0) {
    SET_SOIL_LOW = constrain(strPayload.toInt(), 0, 100);
    saveSettingInt("s_low", SET_SOIL_LOW);
    mqtt.publish("hf/status/messages", ("Set Soil Low: " + String(SET_SOIL_LOW)).c_str());
    return;
  }
  if (strcmp(topic, "hf/set/soil_sched") == 0) {
    SET_SOIL_SCHED_MIN = constrain(strPayload.toInt(), 0, 100);
    saveSettingInt("s_sch", SET_SOIL_SCHED_MIN);
    mqtt.publish("hf/status/messages", ("Set Soil Sched: " + String(SET_SOIL_SCHED_MIN)).c_str());
    return;
  }
  if (strcmp(topic, "hf/set/mist_on_ms") == 0) {
    long v = strPayload.toInt();
    if (v < 1000) v = 1000;
    mistOnTimeMs = (unsigned long)v;
    saveSettingULong("m_on", mistOnTimeMs);
    mqtt.publish("hf/status/messages", ("Set Mist ON ms: " + String(mistOnTimeMs)).c_str());
    return;
  }
  if (strcmp(topic, "hf/set/mist_off_ms") == 0) {
    long v = strPayload.toInt();
    if (v < 1000) v = 1000;
    mistOffTimeMs = (unsigned long)v;
    saveSettingULong("m_off", mistOffTimeMs);
    mqtt.publish("hf/status/messages", ("Set Mist OFF ms: " + String(mistOffTimeMs)).c_str());
    return;
  }

  // -------- GET SETTINGS --------
  if (strcmp(topic, "hf/get/settings") == 0) {
    String msg = "W:" + String(SET_WATER_HOUR) + ":" + String(SET_WATER_MIN) + 
                 " Dur:" + String(SET_WATER_DUR_SEC) + 
                 " F:" + String(SET_FER_DAY_1) + "," + String(SET_FER_DAY_2) + 
                 " FDur:" + String(SET_FER_DUR_SEC) + 
                 " Ton/Toff:" + String(SET_TEMP_ON) + "/" + String(SET_TEMP_OFF) + 
                 " Hon/Hoff:" + String(SET_HUM_ON) + "/" + String(SET_HUM_OFF) + 
                 " SoilLow:" + String(SET_SOIL_LOW) + 
                 " SoilSch:" + String(SET_SOIL_SCHED_MIN) + 
                 " Mist:" + String(mistOnTimeMs) + "/" + String(mistOffTimeMs) + 
                 " LStart:" + String(SET_LIGHT_START_HOUR) + 
                 " LDur:" + String(SET_LIGHT_DUR_HOUR);
    mqtt.publish("hf/status/current_settings", msg.c_str());
    mqtt.publish("hf/status/messages", "Settings sent");
    return;
  }

  // ======== RESET PUMP ALARM ========
  if (strcmp(topic, "hf/system/reset_pump_alarm") == 0) {
    if (pumpAlarmActive && wantOnPayload(payloadStr)) {
      pumpAlarmActive = false;
      currentErrorMsg = "Normal";
      flowLowStart = 0;
      currentPumpATask = PUMP_A_IDLE;
      if (!systemError && !powerFailAlarm) {
        relayWrite(STATUS_LAMP_RELAY_PIN, false);
      }
      mqtt.publish("hf/status/messages", "Pump Alarm Cleared");
      mqtt.publish("hf/status/pump_alarm", "0");
      DBG("[ALARM] Pump alarm cleared via MQTT\n");
    }
    return;
  }

  // ======== RESET SETTINGS ========
  if (strcmp(topic, "hf/system/reset_settings") == 0) {
    if (wantOnPayload(payloadStr)) {
      DBG("[SETTINGS] Resetting all settings to default...\n");
      preferences.begin("farm_cfg", false);
      preferences.clear();
      preferences.end();
      
      SET_WATER_HOUR = 9;
      SET_WATER_MIN = 0;
      SET_WATER_DUR_SEC = 15;
      SET_LIGHT_START_HOUR = 19;
      SET_LIGHT_DUR_HOUR = 9;
      SET_FER_DAY_1 = 3;
      SET_FER_DAY_2 = 0;
      SET_FER_DUR_SEC = 10;
      SET_SOIL_LOW = 50;
      SET_SOIL_SCHED_MIN = 70;
      mistOnTimeMs = 20000;
      mistOffTimeMs = 60000;
      TEMP_OVERTEMP_ENTER = 29.0;
      TEMP_OVERTEMP_EXIT = 26.0;
      OVERTEMP_EXIT_HOLD_MS = 120000;
      RH_HIGH_ENTER = 75.0;
      RH_HIGH_EXIT = 70.0;
      RH_LOW_ENTER = 55.0;
      RH_LOW_EXIT = 68.0;
      RH_GUARD_MAX = 78.0;
      RH_EMERGENCY_OFF = 75.0;
      NIGHT_FAN_ON_MS = 30000;
      NIGHT_FAN_OFF_MS = 300000;
      DAY_START_HOUR = 7;
      DAY_END_HOUR = 19;
      
      mqtt.publish("hf/status/messages", "Settings Reset to Default!");
      DBG("[SETTINGS] Reset complete!\n");
    }
    return;
  }

  // ======== CLIMATE THRESHOLDS ========
  if (strcmp(topic, "hf/set/ot_enter") == 0) {
    float val = strPayload.toFloat();
    if (val >= 25.0 && val <= 40.0) {
      TEMP_OVERTEMP_ENTER = val;
      saveSettingFloat("ot_enter", val);
      mqtt.publish("hf/settings/ot_enter", payloadStr);
    }
    return;
  }
  if (strcmp(topic, "hf/set/ot_exit") == 0) {
    float val = strPayload.toFloat();
    if (val >= 20.0 && val <= 35.0) {
      TEMP_OVERTEMP_EXIT = val;
      saveSettingFloat("ot_exit", val);
      mqtt.publish("hf/settings/ot_exit", payloadStr);
    }
    return;
  }
  if (strcmp(topic, "hf/set/ot_hold") == 0) {
    unsigned long val = strPayload.toInt();
    if (val <= 600000) {
      OVERTEMP_EXIT_HOLD_MS = val;
      saveSettingULong("ot_hold", val);
    }
    return;
  }
  if (strcmp(topic, "hf/set/rh_hi_enter") == 0) {
    float val = strPayload.toFloat();
    if (val >= 60.0 && val <= 95.0) {
      RH_HIGH_ENTER = val;
      saveSettingFloat("rh_hi_en", val);
      mqtt.publish("hf/settings/rh_hi_enter", payloadStr);
    }
    return;
  }
  if (strcmp(topic, "hf/set/rh_hi_exit") == 0) {
    float val = strPayload.toFloat();
    if (val >= 50.0 && val <= 90.0) {
      RH_HIGH_EXIT = val;
      saveSettingFloat("rh_hi_ex", val);
      mqtt.publish("hf/settings/rh_hi_exit", payloadStr);
    }
    return;
  }
  if (strcmp(topic, "hf/set/rh_lo_enter") == 0) {
    float val = strPayload.toFloat();
    if (val >= 30.0 && val <= 70.0) {
      RH_LOW_ENTER = val;
      saveSettingFloat("rh_lo_en", val);
      mqtt.publish("hf/settings/rh_lo_enter", payloadStr);
    }
    return;
  }
  if (strcmp(topic, "hf/set/rh_lo_exit") == 0) {
    float val = strPayload.toFloat();
    if (val >= 40.0 && val <= 80.0) {
      RH_LOW_EXIT = val;
      saveSettingFloat("rh_lo_ex", val);
      mqtt.publish("hf/settings/rh_lo_exit", payloadStr);
    }
    return;
  }
  if (strcmp(topic, "hf/set/rh_guard") == 0) {
    float val = strPayload.toFloat();
    if (val >= 70.0 && val <= 95.0) {
      RH_GUARD_MAX = val;
      RH_EMERGENCY_OFF = val - 3.0;
      saveSettingFloat("rh_guard", val);
      mqtt.publish("hf/settings/rh_guard", payloadStr);
    }
    return;
  }
  if (strcmp(topic, "hf/set/nf_on") == 0) {
    unsigned long val = strPayload.toInt();
    if (val >= 5000 && val <= 300000) {
      NIGHT_FAN_ON_MS = val;
      saveSettingULong("nf_on", val);
    }
    return;
  }
  if (strcmp(topic, "hf/set/nf_off") == 0) {
    unsigned long val = strPayload.toInt();
    if (val >= 30000 && val <= 1800000) {
      NIGHT_FAN_OFF_MS = val;
      saveSettingULong("nf_off", val);
    }
    return;
  }
  if (strcmp(topic, "hf/set/day_start") == 0) {
    int val = strPayload.toInt();
    if (val >= 0 && val <= 23) {
      DAY_START_HOUR = val;
      saveSettingInt("day_st", val);
      mqtt.publish("hf/settings/day_start", payloadStr);
    }
    return;
  }
  if (strcmp(topic, "hf/set/day_end") == 0) {
    int val = strPayload.toInt();
    if (val >= 0 && val <= 23) {
      DAY_END_HOUR = val;
      saveSettingInt("day_en", val);
      mqtt.publish("hf/settings/day_end", payloadStr);
    }
    return;
  }
}

// ==========================================
//          MQTT CONNECT & SUBSCRIBE
// ==========================================
void connectMQTT() {
  if (mqtt.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;
  
  static unsigned long lastMqttAttempt = 0;
  if (millis() - lastMqttAttempt < 5000) return;
  lastMqttAttempt = millis();
  
  DBG("[MQTT] Connecting to %s:%d...\n", MQTT_HOST, MQTT_PORT);
  
  // ใช้ Random Client ID ป้องกันการโดนเตะหลุด (Clean Session conflict)
  char clientId[32];
  snprintf(clientId, sizeof(clientId), "HopsFarm_%08X", (unsigned int)esp_random());
  DBG("[MQTT] Client ID: %s\n", clientId);
  
  // LWT (Last Will and Testament) - จะ publish "offline" เมื่อหลุด
  const char* willTopic = "hf/status/online";
  const char* willMessage = "offline";
  bool willRetain = true;
  int willQoS = 1;
  
  if (mqtt.connect(clientId, MQTT_USER, MQTT_PASS, 
                   willTopic, willQoS, willRetain, willMessage)) {
    DBG("[MQTT] Connected with LWT!\n");
    
    // ตั้ง flag และ activity time
    mqttEverConnected = true;
    lastMqttActivity = millis();
    
    // Subscribe
    mqtt.subscribe("hf/#");
    delay(10);
    
    // Publish online status (retained)
    mqtt.publish("hf/status/online", "online", true);
    delay(10);
    
    // Publish firmware version
    char fwMsg[80];
    snprintf(fwMsg, sizeof(fwMsg), "Online - FW:%s", FIRMWARE_VERSION.c_str());
    mqtt.publish("hf/ota/status", fwMsg, true);
    delay(10);
    
    // Publish mode status
    mqtt.publish("hf/control/mode_status", ModeAuto ? "Auto" : "Manual", true);
    delay(10);
    
    // Publish restart counter
    preferences.begin("farm_cfg", true);
    unsigned long restartCount = preferences.getULong("restart_cnt", 0);
    preferences.end();
    char restartMsg[20];
    snprintf(restartMsg, sizeof(restartMsg), "%lu", restartCount);
    mqtt.publish("hf/status/restart_count", restartMsg, true);
    delay(10);
    
    // Publish WiFi restart counter
    char wifiRestartMsg[20];
    snprintf(wifiRestartMsg, sizeof(wifiRestartMsg), "%lu", wifiRestartCount);
    mqtt.publish("hf/status/wifi_restart_count", wifiRestartMsg, true);
    delay(10);
    
    // Publish MQTT reconnect counter
    char mqttReconnectMsg[20];
    snprintf(mqttReconnectMsg, sizeof(mqttReconnectMsg), "%lu", mqttReconnectCount);
    mqtt.publish("hf/status/mqtt_reconnect_count", mqttReconnectMsg, true);
    delay(10);
    
    // Publish free heap memory
    char heapMsg[20];
    snprintf(heapMsg, sizeof(heapMsg), "%lu", (unsigned long)ESP.getFreeHeap());
    mqtt.publish("hf/status/free_heap", heapMsg, true);
    
    DBG("[MQTT] Subscribed to hf/#\n");
    DBG("[MQTT] Restart=%lu, WiFi_Restart=%lu, MQTT_Reconnect=%lu, Heap=%lu\n",
        restartCount, wifiRestartCount, mqttReconnectCount, (unsigned long)ESP.getFreeHeap());
  } else {
    DBG("[MQTT] Connection failed, rc=%d\n", mqtt.state());
  }
}

// ==========================================
// MQTT Health Check - ตรวจสอบว่า MQTT ยังทำงานจริง
// ==========================================
void checkMqttHealth() {
  static unsigned long lastHealthCheckMs = 0;
  const unsigned long HEALTH_CHECK_INTERVAL = 30000;  // เช็คทุก 30 วินาที (เร็วขึ้น)
  const unsigned long MQTT_TIMEOUT = 90000;           // ถ้าไม่มี activity 90 วินาที = มีปัญหา
  
  unsigned long now = millis();
  if (now - lastHealthCheckMs < HEALTH_CHECK_INTERVAL) return;
  lastHealthCheckMs = now;
  
  // ถ้ายังไม่เคย connect สำเร็จ ข้าม health check
  if (!mqttEverConnected) {
    DBG("[MQTT] Health: waiting for first connection...\n");
    return;
  }
  
  // ถ้า MQTT บอกว่า connected แต่ไม่มี activity นานเกินไป
  if (mqtt.connected()) {
    unsigned long timeSinceActivity = now - lastMqttActivity;
    
    if (timeSinceActivity > MQTT_TIMEOUT) {
      DBG("\n!!! [MQTT] STALE CONNECTION DETECTED !!!\n");
      DBG("[MQTT] No activity for %lu sec - forcing reconnect\n", timeSinceActivity / 1000);
      
      mqtt.disconnect();
      espClient.stop();  // ปิด TCP connection ด้วย
      mqttReconnectCount++;
      
      delay(2000);  // รอ 2 วินาที
      // connectMQTT() จะถูกเรียกใน loop ถัดไป
    } else {
      DBG("[MQTT] Health OK: activity %lu sec ago, reconnects=%lu\n",
          timeSinceActivity / 1000, mqttReconnectCount);
    }
  } else {
    DBG("[MQTT] Health: disconnected, will reconnect...\n");
  }
}

// Heartbeat - ส่ง online ทุก 30 วินาที กัน NAT timeout
void mqttHeartbeat() {
  static unsigned long lastHeartbeatMs = 0;
  const unsigned long HEARTBEAT_INTERVAL = 30000;  // 30 วินาที
  
  if (!mqtt.connected()) {
    DBG("[MQTT] Heartbeat skipped - not connected (state=%d)\n", mqtt.state());
    return;
  }
  
  unsigned long now = millis();
  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL) {
    lastHeartbeatMs = now;
    
    DBG("[MQTT] Starting full status sync...\n");
    
    // ========== FULL STATUS SYNC ทุก 30 วินาที ==========
    bool publishOk = true;
    
    // Online status
    bool ok1 = mqtt.publish("hf/status/online", "online", true);
    publishOk &= ok1;
    DBG("[MQTT] publish online: %s\n", ok1 ? "OK" : "FAIL");
    delay(10);
    
    // Mode & State
    publishOk &= mqtt.publish("hf/control/mode_status", ModeAuto ? "Auto" : "Manual", true);
    delay(10);
    publishOk &= mqtt.publish("hf/status/climate_state", getClimateStateName(currentClimateState), true);
    delay(10);
    mqtt.publish("hf/status/is_daytime", isDayTime ? "1" : "0");
    delay(10);
    
    // Output Status (ใช้ retained เพื่อให้ App เห็นทันที)
    mqtt.publish("hf/status/fan", fanOn ? "On" : "Off", true);
    delay(10);
    mqtt.publish("hf/status/light", lightOn ? "On" : "Off", true);
    delay(10);
    mqtt.publish("hf/status/mist", pumpFoggy_On ? "On" : "Off", true);
    delay(10);
    mqtt.publish("hf/status/pumpwater", (sol2_On && pumpA_On && !pumpB_On) ? "On" : "Off", true);
    delay(10);
    mqtt.publish("hf/status/fertilizer", pumpB_On ? "On" : "Off", true);
    delay(10);
    mqtt.publish("hf/status/pumpmain", (sol1_On && pumpA_On) ? "On" : "Off", true);
    delay(10);
    
    // Sensor values (ใช้ char buffer แทน String เพื่อลด heap fragmentation)
    char buf[16];
    if (!isnan(tempused)) {
      snprintf(buf, sizeof(buf), "%.1f", tempused);
      mqtt.publish("hf/sensor/tempuse", buf);
      delay(10);
    }
    if (!isnan(humused)) {
      snprintf(buf, sizeof(buf), "%.0f", humused);
      mqtt.publish("hf/sensor/humuse", buf);
      delay(10);
    }
    snprintf(buf, sizeof(buf), "%lu", moisturePercentage);
    mqtt.publish("hf/sensor/soil", buf);
    delay(10);
    snprintf(buf, sizeof(buf), "%.2f", flowRateLpm);
    mqtt.publish("hf/sensor/flow", buf);
    delay(10);
    
    // Alarm & Tank
    mqtt.publish("hf/status/alarm", systemError ? "On" : "Off", true);
    delay(10);
    mqtt.publish("hf/status/pump_alarm", pumpAlarmActive ? "1" : "0");
    delay(10);
    mqtt.publish("hf/status/soil_alarm", soilAlarmActive ? "1" : "0");
    delay(10);
    mqtt.publish("hf/sensor/tank_low", waterLevelLow ? "1" : "0");
    delay(10);
    mqtt.publish("hf/sensor/tank_high", waterLevelHigh ? "1" : "0");
    delay(10);
    
    // Daily Water Volume
    snprintf(buf, sizeof(buf), "%.2f", dailyWaterVolume);
    mqtt.publish("hf/sensor/daily_water", buf);
    
    // Track publish success และ activity time
    if (publishOk) {
      lastMqttPublishSuccess = now;
      lastMqttActivity = now;  // อัพเดท activity time
    }
    
    DBG("[MQTT] Full status sync sent (publishOk=%d)\n", publishOk);
  }
}

void initMQTT() {
  espClient.setInsecure();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(1024);
  mqtt.setKeepAlive(60);  // Keep-alive 60 วินาที
  
  DBG("[MQTT] Initialized for HiveMQ Cloud\n");
}

// ==========================================
//          19. SETTERS (Individual Components)
// ==========================================
void setPumpA(bool on, const char* reason) {
  if (pumpA_On == on) return;
  DBG("[SET] PumpA %d -> %d (%s)\n", pumpA_On, on, reason ? reason : "-");
  pumpA_On = on;
  if (on) pumpA_StartTime = millis();
  if (reason) mqtt.publish("hf/status/messages", reason);
}

void setPumpB(bool on, const char* reason) {
  if (pumpB_On == on) return;
  DBG("[SET] PumpB %d -> %d (%s)\n", pumpB_On, on, reason ? reason : "-");
  pumpB_On = on;
  if (on) pumpB_StartTime = millis();
  if (reason) mqtt.publish("hf/status/messages", reason);
}

void setSol1(bool on, const char* reason) {
  if (sol1_On == on) return;
  DBG("[SET] Sol1 %d -> %d (%s)\n", sol1_On, on, reason ? reason : "-");
  sol1_On = on;
  if (reason) mqtt.publish("hf/status/messages", reason);
}

void setSol2(bool on, const char* reason) {
  if (sol2_On == on) return;
  DBG("[SET] Sol2 %d -> %d (%s)\n", sol2_On, on, reason ? reason : "-");
  sol2_On = on;
  if (reason) mqtt.publish("hf/status/messages", reason);
}

void setPumpFoggy(bool on, const char* reason) {
  if (pumpFoggy_On == on) return;
  DBG("[SET] PumpFoggy %d -> %d (%s)\n", pumpFoggy_On, on, reason ? reason : "-");
  pumpFoggy_On = on;
  if (reason) mqtt.publish("hf/status/messages", reason);
}

void setFan(bool on, const char* reason) {
  if (fanOn == on) return;
  DBG("[SET] Fan %d -> %d (%s)\n", fanOn, on, reason ? reason : "-");
  fanOn = on;
  if (reason) mqtt.publish("hf/status/messages", reason);
}

void setLight(bool on, const char* reason) {
  if (lightOn == on) return;
  DBG("[SET] Light %d -> %d (%s)\n", lightOn, on, reason ? reason : "-");
  lightOn = on;
  if (reason) mqtt.publish("hf/status/messages", reason);
}

// ==========================================
//          20. ACTION FUNCTIONS (Composite)
// ==========================================

// รดน้ำ: A + S2 เปิด
void startWatering(const char* reason) {
  DBG("[ACTION] startWatering: %s\n", reason ? reason : "-");
  setPumpA(true, nullptr);
  setSol2(true, nullptr);
  currentPumpATask = PUMP_A_WATERING;  // Track task type
  if (reason) mqtt.publish("hf/status/messages", reason);
}

void stopWatering(const char* reason) {
  DBG("[ACTION] stopWatering: %s\n", reason ? reason : "-");
  setSol2(false, nullptr);
  // ปิด A เฉพาะเมื่อไม่มีใครใช้
  if (!sol1_On && !sol2_On) {
    setPumpA(false, nullptr);
    currentPumpATask = PUMP_A_IDLE;
  }
  if (reason) mqtt.publish("hf/status/messages", reason);
}

// ปุ๋ย: B + S2 เปิด
void startFertilizer(const char* reason) {
  DBG("[ACTION] startFertilizer: %s\n", reason ? reason : "-");
  // setPumpA(true, nullptr);        // คอมเมนต์ออกหรือลบทิ้ง
  setPumpB(true, nullptr);           // เปิดเฉพาะปั๊มปุ๋ย (GPIO 12 / R7)
  setSol2(true, nullptr);            // เปิดวาล์วรดน้ำ/ปุ๋ย (GPIO 4 / R3)
  
  // currentPumpATask = PUMP_A_WATERING; // คอมเมนต์ออก เพราะไม่ได้เปิดปั๊ม A
  
  fertilizerStartTime = millis();
  if (reason) mqtt.publish("hf/status/messages", reason);
}

void stopFertilizer(const char* reason) {
  DBG("[ACTION] stopFertilizer: %s\n", reason ? reason : "-");
  setPumpB(false, nullptr);
  setSol2(false, nullptr);
  // ปิด A เฉพาะเมื่อไม่มีใครใช้
  if (!sol1_On && !sol2_On) {
    setPumpA(false, nullptr);
    currentPumpATask = PUMP_A_IDLE;
  }
  if (reason) mqtt.publish("hf/status/messages", reason);
}

// หมอก: Foggy เปิด (ใช้น้ำจากถังหมอก)
void startMisting(const char* reason) {
  DBG("[ACTION] startMisting: %s\n", reason ? reason : "-");
  setPumpFoggy(true, nullptr);
  if (reason) mqtt.publish("hf/status/messages", reason);
}

void stopMisting(const char* reason) {
  DBG("[ACTION] stopMisting: %s\n", reason ? reason : "-");
  setPumpFoggy(false, nullptr);
  mistPulsing = false;
  mistWaiting = false;
  if (reason) mqtt.publish("hf/status/messages", reason);
}

// เติมน้ำถัง: A + S1 เปิด
void startTankFill(const char* reason) {
  DBG("[ACTION] startTankFill: %s\n", reason ? reason : "-");
  setPumpA(true, nullptr);
  setSol1(true, nullptr);
  currentPumpATask = PUMP_A_TANKFILL;  // Track task type
  tankFillingActive = true;
  if (reason) mqtt.publish("hf/status/messages", reason);
}

void stopTankFill(const char* reason) {
  DBG("[ACTION] stopTankFill: %s\n", reason ? reason : "-");
  setSol1(false, nullptr);
  tankFillingActive = false;
  // ปิด A เฉพาะเมื่อไม่มีใครใช้
  if (!sol1_On && !sol2_On) {
    setPumpA(false, nullptr);
    currentPumpATask = PUMP_A_IDLE;
  }
  if (reason) mqtt.publish("hf/status/messages", reason);
}

// ==========================================
//          21. READ SENSORS
// ==========================================
void readSensorSoil() {
  moistureValue = moistureSensor.readMoisture();
  moisturePercentage = constrain(map(moistureValue, MOISTURE_MIN_VALUE, MOISTURE_MAX_VALUE, 100, 0), 0, 100);
}

void readTemperatureC() {
  sensors_event_t h1, t1, h2, t2;
  bool ok1 = shtMain.getEvent(&h1, &t1);
  bool ok2 = sht2.getEvent(&h2, &t2);

  if (ok1) {
    tempmain = t1.temperature;
    hummain = h1.relative_humidity;
  } else {
    tempmain = NAN;
    hummain = NAN;
  }

  if (ok2) {
    temp2 = t2.temperature;
    hum2 = h2.relative_humidity;
  } else {
    temp2 = NAN;
    hum2 = NAN;
  }

  unsigned long now = millis();
  if (!isnan(tempmain) || !isnan(temp2)) lastTempOkMs = now;
  if (!isnan(hummain) || !isnan(hum2)) lastHumOkMs = now;
}

float checkSensorTemp() {
  if (!isnan(tempmain) && !isnan(temp2))
    return (fabs(tempmain - temp2) <= 5.0) ? (tempmain + temp2) / 2.0 : tempmain;
  if (!isnan(tempmain)) return tempmain;
  if (!isnan(temp2)) return temp2;
  return NAN;
}

float checkSensorHum() {
  if (!isnan(hummain) && !isnan(hum2))
    return (fabs(hummain - hum2) <= 5.0) ? (hummain + hum2) / 2.0 : hummain;
  if (!isnan(hummain)) return hummain;
  if (!isnan(hum2)) return hum2;
  return NAN;
}

// ==========================================
//          22. CLIMATE STATE MACHINE
// ==========================================

const char* getClimateStateName(ClimateState state) {
  switch (state) {
    case STATE_NORMAL:   return "NORMAL";
    case STATE_OVERTEMP: return "OVERTEMP";
    case STATE_RH_HIGH:  return "RH_HIGH";
    case STATE_RH_LOW:   return "RH_LOW";
    default:             return "UNKNOWN";
  }
}

// ==========================================
//   UPDATE CLIMATE STATE (State Transitions)
// ==========================================
void updateClimateState() {
  if (isnan(tempused) || isnan(humused)) {
    DBG("[CLIMATE] Sensor NaN - staying in current state\n");
    return;
  }
  
  unsigned long now = millis();
  ClimateState newState = currentClimateState;
  
  // ============ OVERTEMP มีความสำคัญสูงสุด ============
  // เข้า OVERTEMP: Temp >= 29°C (จากทุก state)
  if (tempused >= TEMP_OVERTEMP_ENTER && currentClimateState != STATE_OVERTEMP) {
    newState = STATE_OVERTEMP;
    overtempExitPending = false;
    overtempExitConditionMs = 0;
    DBG("[CLIMATE] ENTER OVERTEMP: Temp=%.1f >= %.1f\n", tempused, TEMP_OVERTEMP_ENTER);
  }
  
  // ออก OVERTEMP: Temp <= 26°C ค้าง 2 นาที
  if (currentClimateState == STATE_OVERTEMP) {
    if (tempused <= TEMP_OVERTEMP_EXIT) {
      if (!overtempExitPending) {
        overtempExitPending = true;
        overtempExitConditionMs = now;
        DBG("[CLIMATE] OVERTEMP exit condition met, holding 2 min...\n");
      } else if (now - overtempExitConditionMs >= OVERTEMP_EXIT_HOLD_MS) {
        // ค้างครบ 2 นาที -> ออกได้
        newState = STATE_NORMAL;
        overtempExitPending = false;
        DBG("[CLIMATE] EXIT OVERTEMP after hold\n");
      }
    } else {
      // อุณหภูมิขึ้นมาอีก -> reset
      if (overtempExitPending) {
        overtempExitPending = false;
        DBG("[CLIMATE] OVERTEMP exit canceled - temp rose again\n");
      }
    }
  }
  
  // ============ ถ้าไม่ใช่ OVERTEMP ให้เช็ค RH ============
  if (newState != STATE_OVERTEMP && currentClimateState != STATE_OVERTEMP) {
    
    // เข้า RH_HIGH: RH >= 75%
    if (humused >= RH_HIGH_ENTER && currentClimateState != STATE_RH_HIGH) {
      newState = STATE_RH_HIGH;
      DBG("[CLIMATE] ENTER RH_HIGH: Hum=%.1f >= %.1f\n", humused, RH_HIGH_ENTER);
    }
    // ออก RH_HIGH: RH <= 70%
    else if (currentClimateState == STATE_RH_HIGH && humused <= RH_HIGH_EXIT) {
      newState = STATE_NORMAL;
      DBG("[CLIMATE] EXIT RH_HIGH: Hum=%.1f <= %.1f\n", humused, RH_HIGH_EXIT);
    }
    
    // เข้า RH_LOW: RH <= 55% (ถ้าไม่ได้อยู่ใน RH_HIGH)
    else if (humused <= RH_LOW_ENTER && currentClimateState != STATE_RH_HIGH && currentClimateState != STATE_RH_LOW) {
      newState = STATE_RH_LOW;
      DBG("[CLIMATE] ENTER RH_LOW: Hum=%.1f <= %.1f\n", humused, RH_LOW_ENTER);
    }
    // ออก RH_LOW: RH >= 68%
    else if (currentClimateState == STATE_RH_LOW && humused >= RH_LOW_EXIT) {
      newState = STATE_NORMAL;
      DBG("[CLIMATE] EXIT RH_LOW: Hum=%.1f >= %.1f\n", humused, RH_LOW_EXIT);
    }
  }
  
  // ============ เปลี่ยน State ============
  if (newState != currentClimateState) {
    prevClimateState = currentClimateState;
    currentClimateState = newState;
    climateStateEnterMs = now;
    
    // Reset pulse timers เมื่อเปลี่ยน state
    if (newState == STATE_OVERTEMP) {
      // เข้า OVERTEMP → เริ่ม Mist ทันที (เริ่มที่ช่วง ON)
      overtempHumidPulseOn = true;
      overtempHumidTimer = now;
      startMisting("OVERTEMP: Mist START immediately");
    } else if (newState == STATE_RH_LOW) {
      // เข้า RH_LOW → เปิด Mist ทันที, reset hold timer
      rhLowMistHoldActive = false;
      startMisting("RH_LOW: Mist START immediately");
    } else {
      overtempHumidPulseOn = false;
      overtempHumidTimer = now;
    }
    nightFanPulseOn = false;
    nightFanTimer = now;
    rhLowMistHoldActive = false;  // Reset hold timer เมื่อเปลี่ยน state
    
    // Publish state change
    mqtt.publish("hf/status/climate_state", getClimateStateName(currentClimateState));
    mqtt.publish("hf/status/messages", (String("Climate: ") + getClimateStateName(currentClimateState)).c_str());
    
    DBG("[CLIMATE] State changed: %s -> %s\n", 
        getClimateStateName(prevClimateState), getClimateStateName(currentClimateState));
  }
}

// ==========================================
//   EXECUTE CLIMATE CONTROL (ตาม State)
// ==========================================
void executeClimateControl() {
  switch (currentClimateState) {
    case STATE_OVERTEMP:
      handleStateOvertemp();
      break;
    case STATE_RH_HIGH:
      handleStateRhHigh();
      break;
    case STATE_RH_LOW:
      handleStateRhLow();
      break;
    case STATE_NORMAL:
    default:
      handleStateNormal();
      break;
  }
}

// ==========================================
//   STATE: OVERTEMP (สำคัญสุด)
// ==========================================
void handleStateOvertemp() {
  unsigned long now = millis();
  
  // Fan = ON ค้างตลอด
  if (!fanOn) {
    setFan(true, "OVERTEMP: Fan ON");
  }
  
  // Light = OFF ทันที
  if (lightOn) {
    setLight(false, "OVERTEMP: Light OFF");
  }
  
  // ============ Humidifier Logic (กลางวัน vs กลางคืน) ============
  // กลางวัน: ถ้า RH < 55% → Mist ON ต่อเนื่องจนถึง 60%
  //          ถ้า RH >= 60% → Pulse 3 นาที ON / 3 นาที OFF
  // กลางคืน: Pulse เหมือนเดิม
  // Safety: ถ้า RH >= 78% → Mist OFF ทันที
  
  // Threshold สำหรับ RH ต่ำตอน OVERTEMP กลางวัน
  const float OVERTEMP_RH_LOW_ENTER = 55.0;  // เริ่ม continuous mist
  const float OVERTEMP_RH_LOW_EXIT = 60.0;   // กลับไป pulse mode
  
  // Static variable เก็บสถานะ continuous mist
  static bool overtempContinuousMist = false;
  
  if (humused >= RH_GUARD_MAX) {
    // RH สูงเกินไป -> ปิด humidifier ทันที
    if (pumpFoggy_On) {
      stopMisting("OVERTEMP: RH Guard - Mist OFF");
    }
    overtempHumidPulseOn = false;
    overtempContinuousMist = false;
    
  } else if (isDayTime && humused < OVERTEMP_RH_LOW_ENTER && !overtempContinuousMist) {
    // กลางวัน + RH ต่ำมาก (<55%) → เข้าโหมด continuous mist
    overtempContinuousMist = true;
    if (!pumpFoggy_On) {
      startMisting("OVERTEMP DAY: RH<55 Continuous Mist ON");
    }
    DBG("[OVERTEMP] DAY: RH=%.1f < 55%% -> Continuous Mist\n", humused);
    
  } else if (isDayTime && overtempContinuousMist) {
    // อยู่ในโหมด continuous mist
    if (humused >= OVERTEMP_RH_LOW_EXIT) {
      // RH ถึง 60% แล้ว → กลับไป pulse mode
      overtempContinuousMist = false;
      stopMisting("OVERTEMP DAY: RH>=60 Switch to Pulse");
      overtempHumidTimer = now;  // Reset timer
      overtempHumidPulseOn = false;
      DBG("[OVERTEMP] DAY: RH=%.1f >= 60%% -> Pulse Mode\n", humused);
    } else {
      // RH ยังไม่ถึง 60% → Mist ON ต่อเนื่อง
      if (!pumpFoggy_On) {
        startMisting("OVERTEMP DAY: Continuous Mist");
      }
    }
    
  } else {
    // Pulse mode ปกติ (กลางคืน หรือ กลางวันที่ RH >= 60%)
    overtempContinuousMist = false;  // Reset flag
    
    if (!overtempHumidPulseOn) {
      // ช่วง OFF
      if (now - overtempHumidTimer >= OVERTEMP_HUMID_OFF_MS) {
        overtempHumidPulseOn = true;
        overtempHumidTimer = now;
        startMisting("OVERTEMP: Mist Pulse ON");
        DBG("[OVERTEMP] Humid pulse ON\n");
      }
    } else {
      // ช่วง ON
      if (now - overtempHumidTimer >= OVERTEMP_HUMID_ON_MS) {
        overtempHumidPulseOn = false;
        overtempHumidTimer = now;
        stopMisting("OVERTEMP: Mist Pulse OFF");
        DBG("[OVERTEMP] Humid pulse OFF\n");
      }
    }
  }
}

// ==========================================
//   STATE: RH_HIGH (ความชื้นสูง)
// ==========================================
void handleStateRhHigh() {
  unsigned long now = millis();
  
  // Humidifier = OFF เสมอ
  if (pumpFoggy_On) {
    stopMisting("RH_HIGH: Mist OFF");
  }
  
  // Fan = ON ตลอด (ทั้งกลางวันและกลางคืน)
  if (!fanOn) {
    setFan(true, "RH_HIGH: Fan ON");
  }
}

// ==========================================
//   STATE: RH_LOW (ความชื้นต่ำ)
// ==========================================
void handleStateRhLow() {
  unsigned long now = millis();
  
  // Humidifier Logic:
  // 1. ON จนกว่า RH >= 68%
  // 2. เมื่อ RH >= 68% → เปิดต่ออีก 1 นาที แล้วค่อยปิด
  // 3. Safety: ถ้า RH >= 75% → ปิดทันที
  
  if (humused >= RH_EMERGENCY_OFF) {
    // ความชื้นพุ่งสูงเกินไป -> ปิดทันที (ไม่สน hold)
    if (pumpFoggy_On) {
      stopMisting("RH_LOW: Emergency OFF (RH high)");
    }
    rhLowMistHoldActive = false;
    
  } else if (humused >= RH_LOW_EXIT) {
    // RH >= 68% → เริ่ม/ดำเนิน hold period
    if (!rhLowMistHoldActive) {
      // เพิ่งถึง 68% → เริ่มนับ 1 นาที
      rhLowMistHoldActive = true;
      rhLowMistHoldStartMs = now;
      DBG("[RH_LOW] RH>=68%%, starting 1 min hold, Mist continues\n");
      // ให้ Mist เปิดต่อ
      if (!pumpFoggy_On) {
        startMisting("RH_LOW: Mist HOLD 1 min");
      }
    } else {
      // อยู่ใน hold period
      if (now - rhLowMistHoldStartMs >= RH_LOW_MIST_HOLD_MS) {
        // ครบ 1 นาที → ปิด Mist
        if (pumpFoggy_On) {
          stopMisting("RH_LOW: Mist OFF after 1 min hold");
        }
        DBG("[RH_LOW] 1 min hold complete, Mist OFF\n");
      } else {
        // ยังไม่ครบ → เปิดต่อ
        if (!pumpFoggy_On) {
          startMisting("RH_LOW: Mist HOLD continuing");
        }
      }
    }
    
  } else {
    // RH < 68% → เปิด Mist ปกติ, reset hold
    rhLowMistHoldActive = false;
    if (!pumpFoggy_On) {
      startMisting("RH_LOW: Mist ON");
    }
  }
  
  // Fan = ON ตลอด (ทั้งกลางวันและกลางคืน)
  if (!fanOn) {
    setFan(true, "RH_LOW: Fan ON");
  }
}

// ==========================================
//   STATE: NORMAL
// ==========================================
void handleStateNormal() {
  unsigned long now = millis();
  
  // ============ Humidifier Control ============
  // ON เมื่อ RH <= 55%, OFF เมื่อ RH >= 70%
  // ห้ามเปิดถ้า RH >= 75-78%
  
  if (humused >= RH_GUARD_MAX) {
    // ความชื้นสูงมาก -> ปิด humidifier
    if (pumpFoggy_On) {
      stopMisting("NORMAL: Mist OFF (RH high)");
    }
  } else if (humused <= RH_LOW_ENTER) {
    // ความชื้นต่ำ -> เปิด humidifier
    if (!pumpFoggy_On) {
      startMisting("NORMAL: Mist ON (RH low)");
    }
  } else if (humused >= RH_HIGH_EXIT) {
    // ความชื้นถึง 70% -> ปิด humidifier
    if (pumpFoggy_On) {
      stopMisting("NORMAL: Mist OFF (RH ok)");
    }
  }
  
  // ============ Fan Control ============
  // Fan ON ตลอด (ทั้งกลางวันและกลางคืน)
  if (!fanOn) {
    setFan(true, "NORMAL: Fan ON");
  }
  
  // Light: ควบคุมโดย controlLightSystem() ที่เรียกแยก
}

// ==========================================
//   LIGHT CONTROL (แยกออกมา)
// ==========================================
void controlLightSystem(int hourNow) {
  if (hourNow < 0) return;  // Time not ready
  
  // ถ้าอยู่ใน OVERTEMP -> Light ถูกปิดโดย handleStateOvertemp() แล้ว
  if (currentClimateState == STATE_OVERTEMP) {
    return;
  }
  
  if (SET_LIGHT_DUR_HOUR == 0) {
    if (lightOn) setLight(false, "Auto: Light Dur=0 -> OFF");
    return;
  }

  int endHour = (SET_LIGHT_START_HOUR + SET_LIGHT_DUR_HOUR) % 24;
  bool shouldOn = false;

  if (SET_LIGHT_START_HOUR < endHour) {
    if (hourNow >= SET_LIGHT_START_HOUR && hourNow < endHour) shouldOn = true;
  } else {
    if (hourNow >= SET_LIGHT_START_HOUR || hourNow < endHour) shouldOn = true;
  }

  if (shouldOn && !lightOn) {
    setLight(true, "Auto: Light Timer Start");
  } else if (!shouldOn && lightOn) {
    setLight(false, "Auto: Light Timer End");
  }
}

// ควบคุมการเติมน้ำถังหมอกอัตโนมัติ (A + S1)
// เงื่อนไข: ระดับน้ำต่ำ -> เปิด, ระดับน้ำสูง -> ปิด
void controlWaterTankFill() {
  // ถ้ากำลังรดน้ำหรือใส่ปุ๋ยอยู่ (sol2_On) ไม่เติมถัง
  if (sol2_On) {
    if (tankFillingActive) {
      stopTankFill("Auto: Pause tank fill (watering)");
    }
    return;
  }

  // เช็คระดับน้ำ
  if (waterLevelLow && !waterLevelHigh && !tankFillingActive) {
    // น้ำต่ำ -> เริ่มเติม
    DBG("[AUTO] Tank water LOW -> Start filling\n");
    startTankFill("Auto: Tank Fill Start (Low)");
  } 
  else if (waterLevelHigh && tankFillingActive) {
    // น้ำเต็ม -> หยุดเติม
    DBG("[AUTO] Tank water HIGH -> Stop filling\n");
    stopTankFill("Auto: Tank Fill Done (Full)");
  }
}

// ==========================================
//          23. ALARM / CLEAR
// ==========================================

// Alarm เฉพาะ Pump (ปิดแค่ Pump/Solenoid, Fan/Mist/Light ทำงานต่อ)
void setPumpAlarm(const String& msg) {
  DBG("[PUMP ALARM] SET: %s\n", msg.c_str());
  
  pumpAlarmActive = true;
  currentErrorMsg = msg;
  
  // ปิดเฉพาะ Pump และ Solenoid
  pumpA_On = false;
  pumpB_On = false;
  sol1_On = false;
  sol2_On = false;
  tankFillingActive = false;
  currentPumpATask = PUMP_A_IDLE;
  
  // Reset flow tracking
  flowLowStart = 0;
  
  // เปิด R8 แจ้งเตือน
  digitalWrite(STATUS_LAMP_RELAY_PIN, HIGH);
  
  mqtt.publish("hf/status/messages", msg.c_str());
  mqtt.publish("hf/status/pump_alarm", "1");
}

// Alarm ทั้งระบบ (ปิดทุกอย่าง)
void setAlarm(const String& msg) {
  DBG("[ALARM] SET: %s\n", msg.c_str());
  systemError = true;
  currentErrorMsg = msg;
  
  // Force all outputs OFF
  pumpA_On = false;
  pumpB_On = false;
  sol1_On = false;
  sol2_On = false;
  pumpFoggy_On = false;
  fanOn = false;
  lightOn = false;
  tankFillingActive = false;
  mistPulsing = false;
  mistWaiting = false;

  relayWrite(STATUS_LAMP_RELAY_PIN, true);  // Red ON
  mqtt.publish("hf/status/messages", msg.c_str());
  mqtt.publish("hf/status/alarm", "1");
}

void clearAlarm(const String& msg) {
  DBG("[ALARM] CLEAR: %s\n", msg.c_str());
  systemError = false;
  pumpAlarmActive = false;  // Clear pump alarm ด้วย
  soilAlarmActive = false;  // Clear soil alarm ด้วย
  currentErrorMsg = "Normal";
  
  if (!powerFailAlarm) {
    relayWrite(STATUS_LAMP_RELAY_PIN, false);  // Green ON
  }

  // Reset tracking variables
  flowLowStart = 0;
  flowNoiseStartMs = 0;
  tempMismatchStartMs = 0;
  humMismatchStartMs = 0;
  soilBadStartMs = 0;
  tempStuckStartMs = millis();
  humStuckStartMs = millis();
  soilStuckStartMs = millis();
  lastSoilPctForStuck = -1;
  currentPumpATask = PUMP_A_IDLE;

  mqtt.publish("hf/status/messages", msg.c_str());
  mqtt.publish("hf/status/alarm", "0");
  mqtt.publish("hf/status/pump_alarm", "0");
  mqtt.publish("hf/status/soil_alarm", "0");
}

// ==========================================
//          BEEP FUNCTION
// ==========================================
// Beep สั้นๆ สำหรับแจ้งเตือนเหตุการณ์ (ไม่ใช่ Alarm)
void beep(int count, int onMs, int offMs) {
  for (int i = 0; i < count; i++) {
    digitalWrite(STATUS_LAMP_RELAY_PIN, HIGH);  // Buzzer ON
    delay(onMs);
    digitalWrite(STATUS_LAMP_RELAY_PIN, LOW);   // Buzzer OFF
    if (i < count - 1) {
      delay(offMs);
    }
  }
}

// ==========================================
//          24. SAFETY CHECKS
// ==========================================
void runSafetyChecks() {
  unsigned long now = millis();

  // Flow calculation every 1s
  if (now - oldTimeFlow > 1000) {
    detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));
    flowRateLpm = ((1000.0f / (now - oldTimeFlow)) * (float)pulseCount) / FLOW_CAL_K;
    float litersThisSec = (float)pulseCount / (FLOW_CAL_K * 60.0f);
    dailyWaterVolume += litersThisSec;
    oldTimeFlow = now;
    pulseCount = 0;
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);
  }

  // ปั๊ม A ทำงานเมื่อ sol1 หรือ sol2 เปิด (และไม่มี error)
  bool pumpA_actual = (!systemError && !pumpAlarmActive) && pumpA_On;

  // ============ Pump A - เช็คจาก Flow (ไม่ใช่ Timeout) ============
  // ถ้าปั๊มเปิดแต่น้ำไม่ไหล → Pump Alarm ทันที (หลัง grace period)
  // ใช้แทน Pump Timeout เดิม

  // ============ Pump B Timeout ============
  if (!systemError && !pumpAlarmActive && pumpB_On) {
    if (now - pumpB_StartTime > PUMP_B_TIMEOUT) {
      DBG("[SAFETY] Pump B timeout now=%lu start=%lu\n", now, pumpB_StartTime);
      setPumpAlarm("ALARM: Pump B (Fertilizer) Timeout");
    }
  }

  // ============ Dry run detection (only when sol2 is on for watering) ============
  if (!systemError && !pumpAlarmActive && pumpA_On && sol2_On) {
    unsigned long sinceStart = now - pumpA_StartTime;
    if (sinceStart > FLOW_GRACE_MS) {
      if (flowRateLpm < FLOW_MIN_LPM) {
        if (flowLowStart == 0) flowLowStart = now;
        if (now - flowLowStart > FLOW_FAIL_MS) {
          DBG("[SAFETY] Dry run flow=%.2f < %.2f\n", flowRateLpm, FLOW_MIN_LPM);
          setPumpAlarm("ALARM: No Flow (Dry Run)");
        }
      } else {
        flowLowStart = 0;
      }
    }
  } 
  // ============ Dry run detection for Tank Fill (sol1) ============
  else if (!systemError && !pumpAlarmActive && pumpA_On && sol1_On && tankFillingActive) {
    unsigned long sinceStart = now - pumpA_StartTime;
    if (sinceStart > FLOW_GRACE_MS) {
      if (flowRateLpm < FLOW_MIN_LPM) {
        if (flowLowStart == 0) flowLowStart = now;
        if (now - flowLowStart > FLOW_FAIL_MS) {
          DBG("[SAFETY] Tank Fill - No flow! flow=%.2f < %.2f\n", flowRateLpm, FLOW_MIN_LPM);
          setPumpAlarm("ALARM: No Flow (Tank Fill)");
        }
      } else {
        flowLowStart = 0;
      }
    }
  }
  else if (!pumpA_On) {
    flowLowStart = 0;
  }

  // ============ Flow noise detection (flow when pump A is off) ============
  if (!systemError && !pumpAlarmActive) {
    if (!pumpA_actual && flowRateLpm > FLOW_NOISE_LPM) {
      if (flowNoiseStartMs == 0) flowNoiseStartMs = now;
      if (now - flowNoiseStartMs > FLOW_NOISE_ALARM_MS) {
        DBG("[SAFETY] Flow noise flow=%.2f while pump off\n", flowRateLpm);
        setPumpAlarm("ALARM: FLOW noise (pump OFF)");
      }
    } else {
      flowNoiseStartMs = 0;
    }
  }

  // ============ Sensor checks (only in Auto mode) - ใช้ setAlarm ============
  // เพราะถ้า Sensor เสีย ควรหยุดทุกอย่าง
  // แต่ถ้า Sensor กลับมาปกติ → Auto Recovery!
  // ถ้าไม่กลับมานานเกิน 5 นาที → Restart ESP32!
  
  // ============ AUTO RECOVERY: ถ้า Sensor กลับมาปกติ ============
  if (systemError && ModeAuto) {
    // เช็คว่า Sensor กลับมาปกติหรือยัง
    bool tempOk = !isnan(tempmain) || !isnan(temp2);
    bool humOk = !isnan(hummain) || !isnan(hum2);
    
    // ถ้า Error เป็น Sensor NaN และตอนนี้ Sensor กลับมาแล้ว
    if (currentErrorMsg.indexOf("TEMP sensor NaN") >= 0 && tempOk) {
      DBG("[RECOVERY] TEMP sensor recovered! Clearing alarm.\n");
      clearAlarm("Auto Recovery: TEMP sensor OK");
      lastTempOkMs = now;  // Reset timer
      sensorErrorStartMs = 0;  // Reset restart timer
    }
    else if (currentErrorMsg.indexOf("HUM sensor NaN") >= 0 && humOk) {
      DBG("[RECOVERY] HUM sensor recovered! Clearing alarm.\n");
      clearAlarm("Auto Recovery: HUM sensor OK");
      lastHumOkMs = now;  // Reset timer
      sensorErrorStartMs = 0;  // Reset restart timer
    }
    // ถ้า Error เป็น Mismatch และตอนนี้ค่าใกล้กัน
    else if (currentErrorMsg.indexOf("TEMP mismatch") >= 0) {
      if (!isnan(tempmain) && !isnan(temp2)) {
        float dT = fabs(tempmain - temp2);
        if (dT <= TEMP_DIFF_MAX) {
          DBG("[RECOVERY] TEMP mismatch resolved! dT=%.1f\n", dT);
          clearAlarm("Auto Recovery: TEMP sensors match");
          tempMismatchStartMs = 0;
          sensorErrorStartMs = 0;  // Reset restart timer
        }
      }
    }
    else if (currentErrorMsg.indexOf("HUM mismatch") >= 0) {
      if (!isnan(hummain) && !isnan(hum2)) {
        float dH = fabs(hummain - hum2);
        if (dH <= HUM_DIFF_MAX) {
          DBG("[RECOVERY] HUM mismatch resolved! dH=%.1f\n", dH);
          clearAlarm("Auto Recovery: HUM sensors match");
          humMismatchStartMs = 0;
          sensorErrorStartMs = 0;  // Reset restart timer
        }
      }
    }
    
    // ============ RESTART ESP32: ถ้า Sensor Error นานเกิน 5 นาที ============
    if (currentErrorMsg.indexOf("sensor") >= 0 || currentErrorMsg.indexOf("mismatch") >= 0) {
      if (sensorErrorStartMs == 0) {
        sensorErrorStartMs = now;  // เริ่มจับเวลา
      }
      
      unsigned long errorDuration = now - sensorErrorStartMs;
      if (errorDuration > SENSOR_ERROR_RESTART_MS) {
        DBG("[RECOVERY] Sensor error > 5 min! Restarting ESP32...\n");
        mqtt.publish("hf/status/messages", "RESTART: Sensor error timeout");
        delay(1000);  // รอให้ MQTT ส่งเสร็จ
        ESP.restart();
      }
      
      // แจ้งเตือนทุก 1 นาที
      static unsigned long lastWarnMs = 0;
      if (now - lastWarnMs > 60000) {
        lastWarnMs = now;
        unsigned long remaining = (SENSOR_ERROR_RESTART_MS - errorDuration) / 1000;
        DBG("[RECOVERY] Sensor error for %lu sec, restart in %lu sec\n", 
            errorDuration / 1000, remaining);
      }
    }
  } else {
    // ไม่มี error → reset timer
    sensorErrorStartMs = 0;
  }
  
  // ============ Sensor Error Detection ============
  if (ModeAuto && !systemError) {
    // Temperature NaN check
    if ((now - lastTempOkMs) > SENSOR_NAN_ALARM_MS) {
      DBG("[SAFETY] TEMP NaN too long\n");
      setAlarm("ALARM: TEMP sensor NaN");
      return;
    }
    
    // Humidity NaN check
    if ((now - lastHumOkMs) > SENSOR_NAN_ALARM_MS) {
      DBG("[SAFETY] HUM NaN too long\n");
      setAlarm("ALARM: HUM sensor NaN");
      return;
    }

    // Temperature mismatch check
    if (!isnan(tempmain) && !isnan(temp2)) {
      float dT = fabs(tempmain - temp2);
      if (dT > TEMP_DIFF_MAX) {
        if (tempMismatchStartMs == 0) tempMismatchStartMs = now;
        if (now - tempMismatchStartMs > SENSOR_MISMATCH_MS) {
          DBG("[SAFETY] TEMP mismatch dT=%.2f\n", dT);
          setAlarm("ALARM: TEMP mismatch SHT1/SHT2");
          return;
        }
      } else {
        tempMismatchStartMs = 0;
      }
    } else {
      tempMismatchStartMs = 0;
    }

    // Humidity mismatch check
    if (!isnan(hummain) && !isnan(hum2)) {
      float dH = fabs(hummain - hum2);
      if (dH > HUM_DIFF_MAX) {
        if (humMismatchStartMs == 0) humMismatchStartMs = now;
        if (now - humMismatchStartMs > SENSOR_MISMATCH_MS) {
          DBG("[SAFETY] HUM mismatch dH=%.2f\n", dH);
          setAlarm("ALARM: HUM mismatch SHT1/SHT2");
          return;
        }
      } else {
        humMismatchStartMs = 0;
      }
    } else {
      humMismatchStartMs = 0;
    }

    // Soil sensor abnormal check
    if (moistureValue < SOIL_RAW_TOO_LOW || moistureValue > SOIL_RAW_TOO_HIGH) {
      if (soilBadStartMs == 0) soilBadStartMs = now;
      if (now - soilBadStartMs > SOIL_BAD_ALARM_MS) {
        DBG("[SAFETY] SOIL raw abnormal raw=%lu\n", moistureValue);
        setAlarm("ALARM: SOIL sensor abnormal (raw)");
        return;
      }
    } else {
      soilBadStartMs = 0;
    }

    // Soil stuck check
    if (lastSoilPctForStuck < 0) {
      lastSoilPctForStuck = (int)moisturePercentage;
      soilStuckStartMs = now;
    } else {
      if ((int)moisturePercentage != lastSoilPctForStuck) {
        lastSoilPctForStuck = (int)moisturePercentage;
        soilStuckStartMs = now;
        // Clear soil alarm ถ้าค่าเปลี่ยน
        if (soilAlarmActive) {
          soilAlarmActive = false;
          relayWrite(STATUS_LAMP_RELAY_PIN, false);  // ปิดไฟแดง
          mqtt.publish("hf/status/soil_alarm", "0");
          mqtt.publish("hf/status/messages", "Soil sensor OK");
          DBG("[SAFETY] Soil alarm cleared - sensor working\n");
        }
      } else if (now - soilStuckStartMs > SOIL_STUCK_ALARM_MS) {
        // Soil stuck แต่ไม่หยุดระบบทั้งหมด
        if (!soilAlarmActive) {
          soilAlarmActive = true;
          relayWrite(STATUS_LAMP_RELAY_PIN, true);  // เปิดไฟแดง R8
          mqtt.publish("hf/status/soil_alarm", "1");
          mqtt.publish("hf/status/messages", "ALARM: Soil sensor stuck!");
          beep(5, 100, 100);  // Beep 5 ครั้ง
          DBG("[SAFETY] SOIL stuck pct=%d - ALARM ACTIVE (other systems still running)\n", (int)moisturePercentage);
        }
        // ไม่ return! ให้ระบบอื่นทำงานต่อ
      }
    }
  }

  // ============ Float Switch Mismatch Check (Serial warning only) ============
  // สถานะที่เป็นไปไม่ได้: น้ำต่ำ และ น้ำเต็ม พร้อมกัน
  if (waterLevelLow && waterLevelHigh) {
    DBG("[SAFETY] WARNING: Tank sensor mismatch! LOW=%d HIGH=%d (impossible state)\n", 
        waterLevelLow, waterLevelHigh);
  }

  // Normal status - green lamp (ไม่ปิดถ้า Power Fail, Pump Alarm หรือ Soil Alarm)
  if (!systemError && !powerFailAlarm && !pumpAlarmActive && !soilAlarmActive) {
    relayWrite(STATUS_LAMP_RELAY_PIN, false);
  }
}

// ==========================================
//          25. GOOGLE SHEET LOG (Persistent Task + FreeRTOS Queue)
// ==========================================
// วิธีนี้เหมาะสำหรับ IoT 24/7:
// - Task ทำงานตลอด ไม่ต้องสร้าง/ลบซ้ำ
// - ใช้ FreeRTOS Queue รับข้อมูล (thread-safe)
// - ไม่มี Memory Leak
// - ไม่ต้องใช้ vTaskDelete
// ==========================================

// ฟังก์ชันส่ง HTTP request (พร้อม retry)
bool sendHttpRequest(HttpPayload* p) {
  char payload[350];
  snprintf(payload, sizeof(payload),
    "{\"temp\":%s,\"hum\":%s,\"mode\":\"%s\","
    "\"climate_state\":\"%s\",\"water_vol\":%.2f,\"alarm\":\"%s\",\"soil\":%d}",
    isnan(p->temp) ? "null" : String(p->temp, 1).c_str(),
    isnan(p->hum) ? "null" : String(p->hum, 0).c_str(),
    p->mode,
    p->climateState,
    p->waterVol,
    p->alarm,
    p->soil
  );

  for (int retry = 0; retry < HTTP_MAX_RETRY; retry++) {
    if (WiFi.status() != WL_CONNECTED) {
      DBG("[HTTP] WiFi lost during retry %d\n", retry + 1);
      return false;
    }

    HTTPClient http;
    http.begin(GOOGLE_SHEET_URL);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(10000);
    http.setConnectTimeout(5000);
    http.setReuse(false);  // ปิด connection reuse กัน hang

    DBG("[HTTP] Attempt %d/%d: %s\n", retry + 1, HTTP_MAX_RETRY, payload);
    
    int httpCode = http.POST(payload);
    http.end();  // ปิดทันทีหลัง POST

    if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY || httpCode == 302) {
      httpSuccessCount++;
      lastHttpSuccessMs = millis();
      DBG("[HTTP] Success! (attempt %d, total=%lu)\n", retry + 1, httpSuccessCount);
      return true;
    }

    httpFailCount++;
    DBG("[HTTP] Attempt %d failed: code=%d %s\n", retry + 1, httpCode, 
        httpCode < 0 ? HTTPClient::errorToString(httpCode).c_str() : "");
    
    if (retry < HTTP_MAX_RETRY - 1) {
      vTaskDelay(2000 / portTICK_PERIOD_MS);  // รอ 2 วินาที ก่อน retry
    }
  }

  return false;
}

// Persistent HTTP Task - ทำงานตลอด 24/7 ไม่ต้องสร้าง/ลบ
void httpPersistentTask(void* parameter) {
  HttpPayload payload;
  bool holdingPayload = false;              // flag ว่ากำลังถือข้อมูลรอส่งอยู่
  const int MAX_BURST_SEND = 3;             // ส่งต่อเนื่องสูงสุด 3 ชุด/รอบ
  const int THROTTLE_DELAY_MS = 5000;       // รอ 5 วินาที ระหว่างชุด (ป้องกัน rate limit)
  const int WIFI_WAIT_DELAY_MS = 5000;      // รอ WiFi ทุก 5 วินาที
  
  DBG("[HTTP] Persistent Task started, waiting for data...\n");
  
  // วนลูปตลอด (ไม่มี vTaskDelete)
  for (;;) {
    
    // ถ้าไม่มีข้อมูลในมือ → ไปหยิบจาก Queue
    if (!holdingPayload) {
      if (xQueueReceive(httpQueue, &payload, pdMS_TO_TICKS(30000)) == pdTRUE) {
        holdingPayload = true;
        DBG("[HTTP] Got data from queue, ready to send\n");
      } else {
        // Timeout - ไม่มีข้อมูลใหม่
        DBG("[HTTP] Task heartbeat (waiting for data...)\n");
        continue;
      }
    }
    
    // ตอนนี้มีข้อมูลในมือแล้ว (holdingPayload = true)
    
    // ถ้า WiFi ยังไม่ต่อ → ถือข้อมูลรอ (ไม่ทิ้ง!)
    if (WiFi.status() != WL_CONNECTED) {
      DBG("[HTTP] WiFi not connected, holding data... (queue=%d)\n", 
          uxQueueMessagesWaiting(httpQueue));
      vTaskDelay(WIFI_WAIT_DELAY_MS / portTICK_PERIOD_MS);
      continue;  // วนกลับไปเช็ค WiFi ใหม่
    }
    
    // WiFi ต่อแล้ว → ส่งข้อมูลที่ถืออยู่
    if (sendHttpRequest(&payload)) {
      holdingPayload = false;  // ส่งสำเร็จ → ปล่อยมือ
      DBG("[HTTP] Sent successfully, hand is free\n");
    } else {
      // ส่งไม่สำเร็จ → ยังถือไว้ รอส่งใหม่
      DBG("[HTTP] Send failed, still holding data...\n");
      vTaskDelay(WIFI_WAIT_DELAY_MS / portTICK_PERIOD_MS);
      continue;
    }
    
    // ส่งสำเร็จแล้ว → ถ้ามีข้อมูลค้างใน Queue → ส่งต่อ (Throttle)
    int burstCount = 0;
    while (burstCount < MAX_BURST_SEND && 
           uxQueueMessagesWaiting(httpQueue) > 0 && 
           WiFi.status() == WL_CONNECTED) {
      
      // รอ Throttle delay ก่อนส่งชุดถัดไป
      DBG("[HTTP] Throttle: waiting %dms before next send...\n", THROTTLE_DELAY_MS);
      vTaskDelay(THROTTLE_DELAY_MS / portTICK_PERIOD_MS);
      
      // เช็ค WiFi อีกครั้งหลังรอ
      if (WiFi.status() != WL_CONNECTED) {
        DBG("[HTTP] WiFi lost during burst, pausing...\n");
        break;
      }
      
      if (xQueueReceive(httpQueue, &payload, 0) == pdTRUE) {
        if (sendHttpRequest(&payload)) {
          burstCount++;
          DBG("[HTTP] Burst sent %d/%d\n", burstCount, MAX_BURST_SEND);
        } else {
          // ส่งไม่ได้ → ถือไว้รอส่งรอบหน้า
          holdingPayload = true;
          DBG("[HTTP] Burst failed, holding data for next cycle\n");
          break;
        }
      }
    }
    
    // แจ้งถ้ายังมี Queue ค้างอยู่
    int remaining = uxQueueMessagesWaiting(httpQueue);
    if (remaining > 0) {
      DBG("[HTTP] Burst limit reached, %d items still in queue\n", remaining);
    }
    
    // ให้ Task อื่นทำงานบ้าง
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Legacy function (ไม่ใช้แล้ว แต่เก็บไว้ compatibility)
void sendToGoogleSheet(float temp, float hum, int soil) {
  // ฟังก์ชันนี้ถูกแทนที่ด้วย httpPersistentTask แล้ว
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(GOOGLE_SHEET_URL);
  http.addHeader("Content-Type", "application/json");

  char payload[300];
  snprintf(payload, sizeof(payload),
    "{\"temp\":%s,\"hum\":%s,\"mode\":\"%s\","
    "\"climate_state\":\"%s\",\"water_vol\":%.2f,\"alarm\":\"%s\"}",
    isnan(temp) ? "null" : String(temp, 1).c_str(),
    isnan(hum) ? "null" : String(hum, 0).c_str(),
    ModeAuto ? "AUTO" : "MANUAL",
    getClimateStateName(currentClimateState),
    dailyWaterVolume,
    currentErrorMsg.c_str()
  );

  DBG("[LOG] Sending: %s\n", payload);
  http.POST(payload);
  http.end();
}

// ==========================================
//          26. LCD UPDATE
// ==========================================
void updateLCD() {
  static unsigned long lastLcdUpdate = 0;
  static int page = 0;
  
  if (millis() - lastLcdUpdate < LCD_UPDATE_INTERVAL) return;
  lastLcdUpdate = millis();

  lcd.clear();

  switch (page) {
    case 0:
      lcd.setCursor(0, 0);
      if (isnan(tempused) || isnan(humused)) {
        lcd.print("T:--.- H:--%");
      } else {
        lcd.printf("T:%.1f H:%.0f%%", tempused, humused);
      }
      lcd.setCursor(0, 1);
      lcd.printf("Soil:%d%% %s", (int)moisturePercentage, ModeAuto ? "AUT" : "MAN");
      break;
      
    case 1:
      // Climate State
      lcd.setCursor(0, 0);
      lcd.printf("State:%s", getClimateStateName(currentClimateState));
      lcd.setCursor(0, 1);
      lcd.printf("%s Fan:%s", isDayTime ? "DAY " : "NITE", fanOn ? "ON" : "OF");
      break;
      
    case 2:
      lcd.setCursor(0, 0);
       lcd.printf("Fan:%s Mist:%s", fanOn ? "ON" : "OF", pumpFoggy_On ? "ON" : "OF");
       lcd.setCursor(0, 1);
       lcd.printf("Light:%s", lightOn ? "ON" : "OFF");
        break;
      
    case 3:
      lcd.setCursor(0, 0);
      lcd.printf("F:%.1f V:%.1fL", flowRateLpm, dailyWaterVolume);
      lcd.setCursor(0, 1);
      lcd.printf("Tank L:%d H:%d", waterLevelLow ? 1 : 0, waterLevelHigh ? 1 : 0);
      break;
      
    case 4:
      lcd.setCursor(0, 0);
      lcd.printf("AC: %.0fV", acVoltage);
      if (powerFailAlarm) lcd.print(" FAIL!");
      lcd.setCursor(0, 1);
      lcd.print(systemError ? "!! ALARM !!" : "System Normal");
      break;
      
    case 5:
      lcd.setCursor(0, 0);
      lcd.print("IP:");
      lcd.setCursor(0, 1);
      if (WiFi.status() == WL_CONNECTED) {
        lcd.print(WiFi.localIP());
      } else {
        lcd.print("Offline");
      }
      break;
  }
  
  page = (page + 1) % 6;
}

// ==========================================
//          27. DEBUG TICK
// ==========================================
void debugTick(bool timeReady, int hourNow, int minuteNow, int secondNow, int dayOfWeek) {
#if DEBUG_SERIAL
  unsigned long now = millis();
  if (now - lastDebugMs < DEBUG_EVERY_MS) return;
  lastDebugMs = now;

  DBG("\n==== TICK @%lu ====\n", now);
  
  // WiFi Status & Signal Strength
  if (WiFi.status() == WL_CONNECTED) {
    int rssi = WiFi.RSSI();
    const char* quality = (rssi > -50) ? "Excellent" : (rssi > -60) ? "Good" : (rssi > -70) ? "Fair" : "Weak";
    DBG("WiFi: Connected RSSI=%ddBm (%s)\n", rssi, quality);
  } else {
    DBG("WiFi: DISCONNECTED!\n");
  }
  
  DBG("ModeAuto=%d systemError=%d hasWatered=%d hasFert=%d\n", 
      ModeAuto, systemError, hasWateredToday, hasFertilizedToday);

  if (timeReady) {
    DBG("Time=%02d:%02d:%02d DOW=%d isDayTime=%d\n", hourNow, minuteNow, secondNow, dayOfWeek, isDayTime);
  } else {
    DBG("Time=NOT READY\n");
  }

  DBG("Sensors: soil=%d%% raw=%lu T=%.1f H=%.0f flow=%.2fLPM\n",
      (int)moisturePercentage, moistureValue, tempused, humused, flowRateLpm);
  
  DBG("Climate State: %s (prev: %s)\n", 
      getClimateStateName(currentClimateState), getClimateStateName(prevClimateState));
  
  DBG("Tank: Low=%d High=%d Filling=%d\n", waterLevelLow, waterLevelHigh, tankFillingActive);
  
  DBG("AC Voltage: %.1fV PowerFail=%d\n", acVoltage, powerFailAlarm);

  DBG("Outputs: A=%d B=%d S1=%d S2=%d Foggy=%d Fan=%d Light=%d\n",
      pumpA_On, pumpB_On, sol1_On, sol2_On, pumpFoggy_On, fanOn, lightOn);
  
  // HTTP Queue stats
  int queueCount = (httpQueue != NULL) ? uxQueueMessagesWaiting(httpQueue) : 0;
  DBG("HTTP: success=%lu fail=%lu queueFull=%lu inQueue=%d\n", 
      httpSuccessCount, httpFailCount, httpQueueFullCount, queueCount);
  
  // Memory stats (สำคัญสำหรับ 24/7 operation)
  DBG("Memory: freeHeap=%lu minFreeHeap=%lu\n", 
      ESP.getFreeHeap(), ESP.getMinFreeHeap());
  
  // Watchdog status
  unsigned long minSinceSuccess = (millis() - lastHttpSuccessMs) / 60000;
  DBG("Watchdog: wifiRestarts=%lu noSuccess=%lumin (L1@10min, L2@120min)\n", 
      wifiRestartCount, minSinceSuccess);
#endif
}

// ==========================================
//          OTA UPDATE FUNCTIONS
// ==========================================
void performOTA() {
  if (!otaInProgress || otaUrl.length() == 0) return;
  
  DBG("[OTA] Starting update from: %s\n", otaUrl.c_str());
  mqtt.publish("hf/ota/status", "OTA: Downloading firmware...");
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("OTA UPDATE...");
  lcd.setCursor(0, 1);
  lcd.print("Please wait");
  
  HTTPClient http;
  http.begin(otaUrl);
  http.setTimeout(30000);
  
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    
    if (contentLength > 0) {
      DBG("[OTA] Firmware size: %d bytes\n", contentLength);
      mqtt.publish("hf/ota/status", ("OTA: Size=" + String(contentLength) + " bytes").c_str());
      
      if (Update.begin(contentLength)) {
        WiFiClient* stream = http.getStreamPtr();
        size_t written = Update.writeStream(*stream);
        
        if (written == contentLength) {
          DBG("[OTA] Written: %d bytes\n", written);
        }
        
        if (Update.end()) {
          if (Update.isFinished()) {
            DBG("[OTA] Update SUCCESS! Rebooting...\n");
            mqtt.publish("hf/ota/status", "OTA: SUCCESS! Rebooting...");
            delay(1000);
            ESP.restart();
          }
        } else {
          mqtt.publish("hf/ota/status", ("OTA: ERROR - " + String(Update.errorString())).c_str());
        }
      } else {
        mqtt.publish("hf/ota/status", "OTA: ERROR - Not enough space");
      }
    }
  } else {
    mqtt.publish("hf/ota/status", ("OTA: ERROR - HTTP " + String(httpCode)).c_str());
  }
  
  http.end();
  otaInProgress = false;
  otaUrl = "";
  lcd.clear();
}

void handleOTA() {
  if (otaInProgress) {
    performOTA();
  }
}
