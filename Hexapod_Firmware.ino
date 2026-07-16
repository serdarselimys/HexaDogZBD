#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>

// ############################################################
// #  ALL TUNABLE SETTINGS LIVE IN THIS FILE.                 #
// #  Change a value  -> here (Hexapod_Main.ino)              #
// #  Change how it moves/balances -> Robot_Gait_Mechanism.h  #
// #  Change / add emotes -> Robot_Emotes.h                   #
// #  Change what's on screen       -> Screen_Settings.h      #
// ############################################################

// ============================================================
// HARDWARE INSTANCES
// ============================================================
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm3 = Adafruit_PWMServoDriver(0x41);
Adafruit_MPU6050 mpu;
Preferences preferences;
TFT_eSPI tft = TFT_eSPI();
WiFiUDP udp;

// ============================================================
// I2C / PIN CONFIG
// ============================================================
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;
const uint32_t I2C_CLOCK_HZ = 400000;
const int VOLTAGE_PIN = 32;
const float DIVIDER_RATIO = 5.0;
const float ADC_CAL_FACTOR = 1.08; // fine-trim multiplier for the voltage read

// ============================================================
// SERVO CONFIG
// ============================================================
#define USMIN 500.0
#define USMAX 2500.0
const uint8_t PWM_FREQ_HZ = 50;

// Servo Offsets (persisted to flash; edited in calibration mode)
float servoOffsets[18] = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};

// ============================================================
// ROBOT GEOMETRY
// ============================================================
const float L1 = 0.1000;
const float L2 = 0.1000;
const float BODY_OFFSET = 0.0544;
const float KNEE_ASSEMBLY_OFFSET = PI / 2.0;

// ============================================================
// GAIT / HEIGHT TUNING
// ============================================================
const float DT_SEC = 0.008333; // 120Hz loop period

const float MIN_HEIGHT = 0.14;
const float MAX_HEIGHT = 0.23;
const float HEIGHT_SPEED = 0.06; // m/s, manual trim rate + normal height ramp rate

const float EMOTE_HEIGHT_SPEED = 0.03;
const float STOP_BLEND_SPEED = 0.08;

const float STARTUP_DURATION = 2.0;          // stand/sit ramp seconds
const float REQUIRED_HOLD_DURATION = 2.0;    // L1+R1 hold-to-toggle seconds

// Directional shift low-pass filter (anti-shake, from the PyBullet script)
const float URDF_X_ALPHA = 0.04; // 0.02 = smoother, 0.08 = faster

// ============================================================
// BALANCE / IMU TUNING
// ============================================================
// High COMP_ALPHA = trust gyro for fast motion, slow accel correction.
// gyro drift over seconds without letting servo vibration noise through.
const float COMP_ALPHA = 0.97f;

// Deadband: angles smaller than this are clamped to zero before reaching
// vibration and small IMU noise don't register as a tilt needing correction.
const float BALANCE_DEADBAND_RAD = 0.10f;

// Balance IK gain -- how far the legs shift per radian of tilt.
// Lower = slower, less aggressive corrections. Do not raise above 1.6.
const float BALANCE_GAIN = 1.0f;

// cannot overshoot and chase its own vibration. Raise toward 0.99 if it still oscillates; lower toward 0.94 if too sluggish.
const float COMP_OUTPUT_ALPHA = 0.955f;

// Seconds for compensation to fade 0->full once height has settled.
const float BALANCE_BLEND_RAMP_SEC = 1.0f;

// Target body height forced while balancing (m).
const float BALANCE_TARGET_HEIGHT = 0.18f;

// Boot sit-still window used to measure gyro bias.
const unsigned long BOOT_DURATION_MS = 2500;
const int   GYRO_CALIB_SAMPLES  = 200;
const float GYRO_CALIB_SAMPLE_MS = 10.0f;

// ============================================================
// CONTROL / INPUT CONFIG
// ============================================================
const float JOYSTICK_DEADZONE = 0.12;
const float TRIGGER_THRESHOLD = 0.05;

// ============================================================
// NETWORK CONFIG
// ============================================================

const char* AP_SSID = "HEXAPOD_ESP32";
const char* AP_PASSWORD = "12345678";
const uint16_t UDP_PORT = 5000;
const int RX_PACKET_MIN_SIZE = 22;
uint8_t networkBuffer[22];

// ============================================================
// DISPLAY CONFIG
// ============================================================
const uint8_t SCREEN_ROTATION = 3;   // orientation used by the UI
const bool    SCREEN_INVERT   = true;
uint16_t MY_BLACK, MY_CYAN;          // assigned in setup() via color565()
const int eyeWidth = 90, eyeHeight = 110, eyeSpacing = 50;

// ============================================================
// RUNTIME STATE (not tunable -- do not edit to change behavior)
// ============================================================

bool target_standing_state = false;
float current_transition_progress = 0.0;
bool servosArePowered = false;
bool in_calibration_mode = false;
unsigned long bootStartTime = 0;
bool bootSequenceComplete = false;
float button_hold_time = 0.0;

// Height ramp state
float user_selected_height = 0.20;
float current_body_height = 0.20;

// Mode flags
bool balance_enabled = false;
bool dirt_mode_enabled = false;


bool          emote_mode_enabled = false;
bool          emote_playing      = false;
uint8_t       emote_playing_id   = 255;
uint8_t       selected_emote_id  = 0;
unsigned long emote_start_ms     = 0;

bool          exiting_emote_ramp = false;

// IMU filtered outputs + measured gyro bias
float accelRestMag = 9.81f;
float body_roll_filtered = 0.0;
float body_pitch_filtered = 0.0;
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
bool  imuCalibrationDone = false;

// Gait runtime
float urdf_x_filtered = 0.0;
float phase_accumulator = 0.0;
float motion_blend = 0.0;

// Controller input (written by gait task, read everywhere)
volatile float joy_fwd = 0.0, joy_side = 0.0, joy_spin = 0.0;
volatile float norm_lt = 0.0, norm_rt = 0.0;

// Telemetry values
volatile float batteryVoltage = 0.0;
volatile float imu_ax = 0.0, imu_ay = 0.0, imu_az = 0.0;

// Cross-core sync
SemaphoreHandle_t i2cMutex = NULL;
TaskHandle_t KinematicsTaskHandle;
TaskHandle_t TelemetryTaskHandle;

// Calibration UI state + edge-detect latches
volatile int selectedJointID = 0;
bool lastDpadLeftPressed = false, lastDpadRightPressed = false;
bool lastDpadUpPressed = false, lastDpadDownPressed = false;
bool lastAPressed = false, lastBPressed = false;
bool lastL1Pressed = false, lastR1Pressed = false;
// Edge-detect latches for the new emote-control bits.
bool lastEmoteModeTogglePressed = false;
bool lastEmotePlayPressed       = false;
bool lastEmoteStopPressed       = false;
bool justSavedFeedback = false;
unsigned long saveFeedbackTimer = 0;

// Display eye-engine state
uint8_t lastRenderedState = 255;
unsigned long blinkTimer = 0, blinkDuration = 0, lastMovementTime = 0, eyeLookTimer = 0;
bool isBlinking = false, screenDrawnForMotion = false;
int horizontalEyeOffset = 0;

// indices 0-2 FR, 3-5 FL, 6-8 MR, 9-11 ML, 12-14 RR, 15-17 RL
const char* JOINT_NAMES[18] = {
    "FR KNEE", "FR THIGH", "FR HIP", "FL KNEE", "FL THIGH", "FL HIP",
    "MR KNEE", "MR THIGH", "MR HIP", "ML KNEE", "ML THIGH", "ML HIP",
    "RR KNEE", "RR THIGH", "RR HIP", "RL KNEE", "RL THIGH", "RL HIP"
};

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void loadOffsetsFromFlash();
void shutDownServosHardware();
void calibrateGyroBias();        // gait file -- balance sensing (boot)
void updateIMUAndBalance();      // gait file -- balance sensing (per tick)
void setupDiagnosticUI();        // screen file
void runEmoteTick();             // emotes file -- called from KinematicsTask
void KinematicsTask(void * pvParameters);
void TelemetryTask(void * pvParameters);


#include "Robot_Emotes.h"          // emote catalog + runEmoteTick()
#include "Robot_Gait_Mechanism.h"  // motion + balance + servo I/O
#include "Screen_Settings.h"       // display + telemetry TX

void setup() {
    Serial.begin(921600);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);

    i2cMutex = xSemaphoreCreateMutex();

    tft.init();
    tft.setRotation(SCREEN_ROTATION);
    tft.invertDisplay(SCREEN_INVERT);
    MY_BLACK = tft.color565(0, 0, 0);
    MY_CYAN  = tft.color565(0, 255, 255);
    setupDiagnosticUI();

    loadOffsetsFromFlash();

    if (!mpu.begin(0x68, &Wire)) {
        Serial.println("Failed to find MPU6050 chip");
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    pinMode(VOLTAGE_PIN, INPUT);

    pwm1.begin(); pwm3.begin();
    pwm1.setPWMFreq(PWM_FREQ_HZ); pwm3.setPWMFreq(PWM_FREQ_HZ);
    shutDownServosHardware();

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    udp.begin(UDP_PORT);

    bootStartTime = millis();
    current_body_height = user_selected_height;

    xTaskCreatePinnedToCore(KinematicsTask, "GaitEngine", 8192, NULL, 3, &KinematicsTaskHandle, 1);
    xTaskCreatePinnedToCore(TelemetryTask, "TelemetryEngine", 4096, NULL, 1, &TelemetryTaskHandle, 0);
}

void loop() {
    vTaskDelete(NULL);
}
