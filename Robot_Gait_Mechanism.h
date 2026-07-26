#ifndef ROBOT_GAIT_MECHANISM_H
#define ROBOT_GAIT_MECHANISM_H



// ============================================================
// GEOMETRY LAYOUT ARRAYS
// ============================================================
const char* LEG_ORDER[6] = {"FL", "ML", "RL", "FR", "MR", "RR"};
const float LEG_PHASES[6] = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5};
const float LEG_ROOTS[6][2] = {
    {0.12, 0.10}, {0.00, 0.10}, {-0.12, 0.10},
    {0.12, -0.10}, {0.00, -0.10}, {-0.12, -0.10}
};

struct CSVGaitRow {
    char direction[12];
    float speed;
    float frequency;
    float step_amplitude;
};

#define GAIT_MATRIX_SIZE 17
const CSVGaitRow gaitMatrix[GAIT_MATRIX_SIZE] = {
    { "straight", 0.150, 1.14296397932823, 0.0344655160996815 },
    { "straight", 0.125, 1.17823074629728, 0.0299912941411941 },
    { "straight", 0.100, 1.18448955665322, 0.0248265321935404 },
    { "straight", 0.075, 1.08529492094338, 0.0197727859164272 },
    { "straight", 0.200, 1.24891843728639, 0.0380766109815619 },
    { "sideways", 0.150, 0.97968420390789, 0.0243236670766971 },
    { "sideways", 0.125, 0.88235391176811, 0.021911139968899  },
    { "sideways", 0.100, 0.69258886449631, 0.0242552794207171 },
    { "sideways", 0.075, 0.50289089214640, 0.0227976638015279 },
    { "sideways", 0.050, 0.41802299197653, 0.0186529566991328 },
    { "spin",     0.700, 1.29902461359888, 0.1198297299757320 },
    { "spin",     0.600, 1.29982704068771, 0.0982409866415720 },
    { "spin",     0.400, 1.05657433494582, 0.0839461832221573 },
    { "spin",     0.200, 0.61414380342244, 0.0823087528780208 },
    { "diagonal", 0.125, 1.22855595893970, 0.0476998798954310 },
    { "diagonal", 0.100, 1.06749130398247, 0.0442829644974364 },
    { "diagonal", 0.075, 0.88051134658116, 0.0405915399991293 }
};

// ============================================================
// DIRECTION-SPECIFIC CONFIGURATION
// ============================================================
struct DirectionParams {
    float step_height;
    float urdf_x_offset;
};

const DirectionParams DIR_FORWARD  = { 0.0125, -0.025 };
const DirectionParams DIR_BACKWARD = { 0.0125,  0.010 };
const DirectionParams DIR_SIDEWAYS = { 0.0150,  0.005 };
const DirectionParams DIR_DIAGONAL = { 0.0100, -0.015 };
const DirectionParams DIR_SPIN     = { 0.0125,  0.020 };

struct CalculatedGait {
    float freq;
    float step_amplitude;
    float step_h;
    float urdf_x;
};

// ============================================================
// SERVO I/O + FLASH PERSISTENCE
// ============================================================
void saveOffsetsToFlash() {
    preferences.begin("hexapod-cal", false);
    for (int i = 0; i < 18; i++) {
        String key = "off_" + String(i);
        preferences.putFloat(key.c_str(), servoOffsets[i]);
    }
    preferences.end();
    Serial.println("Calibration metrics permanently stored to Flash NVS.");
}

void loadOffsetsFromFlash() {
    preferences.begin("hexapod-cal", true);
    for (int i = 0; i < 18; i++) {
        String key = "off_" + String(i);
        servoOffsets[i] = preferences.getFloat(key.c_str(), servoOffsets[i]);
    }
    preferences.end();
    Serial.println("Saved system offsets successfully deployed from Flash.");
}

void shutDownServosHardware() {
    for (int pin = 0; pin < 16; pin++) {
        pwm1.setPWM(pin, 0, 4096);
        pwm3.setPWM(pin, 0, 4096);
    }
    servosArePowered = false;
    Serial.println("Servos safe: PWM outputs completely disabled.");
}

void setServo(int id, float angle) {
    if (id < 0 || id >= 18) return;
    Adafruit_PWMServoDriver* board = (id < 9) ? &pwm1 : &pwm3;
    int relativeId = id % 9;
    int physicalPin = relativeId + (relativeId / 3);
    float calibratedAngle = constrain(angle + servoOffsets[id], 0.0, 180.0);
    float preciseMicroseconds = USMIN + (calibratedAngle * (2000.0 / 180.0));
    board->writeMicroseconds(physicalPin, (int)preciseMicroseconds);
}

// ============================================================
// BALANCE SENSING -- GYRO BIAS CALIBRATION (called once at boot)
// ============================================================
void calibrateGyroBias() {
    sensors_event_t a, g_imu, temp;
    double calibSumX = 0.0, calibSumY = 0.0, calibAccelSum = 0.0;
    int calibCount = 0;
    while (calibCount < GYRO_CALIB_SAMPLES) {
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            mpu.getEvent(&a, &g_imu, &temp);
            calibSumX += g_imu.gyro.x;
            calibSumY += g_imu.gyro.y;
            calibAccelSum += sqrt(a.acceleration.x * a.acceleration.x +
                                  a.acceleration.y * a.acceleration.y +
                                  a.acceleration.z * a.acceleration.z);
            xSemaphoreGive(i2cMutex);
            calibCount++;
        }
        vTaskDelay(pdMS_TO_TICKS((int)GYRO_CALIB_SAMPLE_MS));
    }
    gyroBiasX = (float)(calibSumX / calibCount);
    gyroBiasY = (float)(calibSumY / calibCount);
    accelRestMag = (float)(calibAccelSum / calibCount);
    imuCalibrationDone = true;
    Serial.printf("[IMU] Gyro bias X:%.5f Y:%.5f | Accel rest mag:%.3f (m/s2)\n",
                  gyroBiasX, gyroBiasY, accelRestMag);
}

// ============================================================
// BALANCE SENSING -- COMPLEMENTARY FILTER (called once per telemetry tick)
// ============================================================
void updateIMUAndBalance() {
    static unsigned long lastImuTime  = millis();
    static unsigned long lastDiagTime = 0;
    sensors_event_t a, g_imu, temp;

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        mpu.getEvent(&a, &g_imu, &temp);
        imu_ax = a.acceleration.x;
        imu_ay = a.acceleration.y;
        imu_az = a.acceleration.z;

        unsigned long now = millis();
        float imu_dt = constrain((now - lastImuTime) / 1000.0f, 0.001f, 0.05f);
        lastImuTime = now;

        float pitch_accel_raw = -atan2(imu_ay, imu_az);
        float roll_accel_raw  =  atan2(imu_ax, sqrt(imu_ay * imu_ay + imu_az * imu_az));

        float gyro_pitch_rate = -(g_imu.gyro.y - gyroBiasY);
        float gyro_roll_rate  = -(g_imu.gyro.x - gyroBiasX);

        float a_mag = sqrt(imu_ax * imu_ax + imu_ay * imu_ay + imu_az * imu_az);
        float excess = fabs(a_mag - accelRestMag);
        float gate   = constrain(1.0f - (excess - 0.5f) / 3.0f, 0.0f, 1.0f);
        float alpha  = COMP_ALPHA + (0.999f - COMP_ALPHA) * (1.0f - gate);

        body_pitch_filtered = alpha * (body_pitch_filtered + gyro_pitch_rate * imu_dt)
                              + (1.0f - alpha) * pitch_accel_raw;
        body_roll_filtered  = alpha * (body_roll_filtered  + gyro_roll_rate  * imu_dt)
                              + (1.0f - alpha) * roll_accel_raw;

        if (millis() - lastDiagTime >= 200) {
            lastDiagTime = millis();
            Serial.printf(
                "[IMU] ax:%6.3f ay:%6.3f az:%6.3f | "
                "roll_accel:%6.2f deg pitch_accel:%6.2f deg | "
                "ROLL_FILT:%6.2f deg PITCH_FILT:%6.2f deg | "
                "gyro_roll:%6.3f gyro_pitch:%6.3f\n",
                imu_ax, imu_ay, imu_az,
                degrees(roll_accel_raw),
                degrees(pitch_accel_raw),
                degrees(body_roll_filtered),
                degrees(body_pitch_filtered),
                gyro_roll_rate,
                gyro_pitch_rate
            );
        }

        xSemaphoreGive(i2cMutex);
    }
}

// ============================================================
// INVERSE KINEMATICS
// ============================================================
bool solve_leg_ik_3dof(float tx, float ty, float tz, float urdf_x_offset, float &hip, float &thigh, float &knee) {
    float x = tx + urdf_x_offset;
    float y = ty;
    float z_from_thigh = -(tz - BODY_OFFSET);

    hip = atan2(y, -z_from_thigh);
    float z_sag = -sqrt(y*y + z_from_thigh*z_from_thigh);

    float dist_sq = x*x + z_sag*z_sag;
    float dist = sqrt(dist_sq);

    if (dist > (L1 + L2) * 0.99 || dist < abs(L1 - L2)) return false;

    float cos_phi = (L1*L1 + L2*L2 - dist_sq) / (2.0 * L1 * L2);
    knee = PI - acos(constrain(cos_phi, -1.0, 1.0));

    float alpha = atan2(z_sag, x);
    float cos_beta = (L1*L1 + dist_sq - L2*L2) / (2.0 * L1 * dist);

    float beta = acos(constrain(cos_beta, -1.0, 1.0));
    thigh = alpha - beta + PI/2.0;
    return true;
}

// ============================================================
// GAIT BLENDING
// ============================================================
CSVGaitRow safePickRow(const char* mode, float normalized_mag) {
    float min_csv_speed = 999.0;
    float max_csv_speed = -999.0;
    int matches[GAIT_MATRIX_SIZE];
    int match_count = 0;

    for (int i = 0; i < GAIT_MATRIX_SIZE; i++) {
        if (strcmp(gaitMatrix[i].direction, mode) == 0) {
            matches[match_count++] = i;
            if (gaitMatrix[i].speed < min_csv_speed) min_csv_speed = gaitMatrix[i].speed;
            if (gaitMatrix[i].speed > max_csv_speed) max_csv_speed = gaitMatrix[i].speed;
        }
    }

    if (match_count == 0) return gaitMatrix[0];

    float target = min_csv_speed + (normalized_mag * (max_csv_speed - min_csv_speed));
    int best_idx = matches[0];
    float min_err = abs(gaitMatrix[best_idx].speed - target);

    for (int i = 1; i < match_count; i++) {
        int idx = matches[i];
        float error = abs(gaitMatrix[idx].speed - target);
        if (error < min_err) {
            min_err = error;
            best_idx = idx;
        }
    }
    return gaitMatrix[best_idx];
}

CalculatedGait getBlendedGaitParams(float fwd, float side, float spin, float norm_mag, bool dirtMode) {
    float f = abs(fwd);
    float s = abs(side);
    float r = abs(spin);

    const DirectionParams &straightParams = (fwd >= 0.0) ? DIR_FORWARD : DIR_BACKWARD;

    float diag_w = min(f, s) * 2.0;
    float straight_w = max(0.0f, f - diag_w / 2.0f);
    float sideways_w = max(0.0f, s - diag_w / 2.0f);
    float spin_w = r;
    float total_w = straight_w + sideways_w + diag_w + spin_w;

    CSVGaitRow row_str = safePickRow("straight", norm_mag);
    CSVGaitRow row_sid = safePickRow("sideways", norm_mag);
    CSVGaitRow row_dia = safePickRow("diagonal", norm_mag);
    CSVGaitRow row_spi = safePickRow("spin", r);

    CalculatedGait g;

    if (total_w > 0.001) {
        g.freq = (straight_w * row_str.frequency + sideways_w * row_sid.frequency +
                  diag_w * row_dia.frequency + spin_w * row_spi.frequency) / total_w;

        g.step_amplitude = (straight_w * row_str.step_amplitude + sideways_w * row_sid.step_amplitude +
                            diag_w * row_dia.step_amplitude + spin_w * row_spi.step_amplitude) / total_w;

        if (dirtMode) {
            g.step_h = 0.020;
        } else {
            g.step_h = (straight_w * straightParams.step_height +
                        sideways_w * DIR_SIDEWAYS.step_height +
                        diag_w     * DIR_DIAGONAL.step_height +
                        spin_w     * DIR_SPIN.step_height) / total_w;
        }

        g.urdf_x = (straight_w * straightParams.urdf_x_offset +
                    sideways_w * DIR_SIDEWAYS.urdf_x_offset +
                    diag_w     * DIR_DIAGONAL.urdf_x_offset +
                    spin_w     * DIR_SPIN.urdf_x_offset) / total_w;
    } else {
        g.freq = row_str.frequency;
        g.step_amplitude = row_str.step_amplitude;
        g.step_h = dirtMode ? 0.020 : straightParams.step_height;
        g.urdf_x = straightParams.urdf_x_offset;
    }

    return g;
}

// ============================================================
// WALKING PROCESS ENGINE LOOP (Core 1)
// ============================================================
void KinematicsTask(void * pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(8.333);

    // Balance compensation blend state (walking-mode only).
    float balance_comp_blend = 0.0f;
    float smooth_comp_pitch  = 0.0f;
    float smooth_comp_roll   = 0.0f;

    for(;;) {
        int packetSize = udp.parsePacket();
        if (packetSize >= RX_PACKET_MIN_SIZE) {
            udp.read(networkBuffer, packetSize);
            if (bootSequenceComplete) {
                float fwd, side, spin, lt, rt;
                memcpy(&fwd,  &networkBuffer[0],  4);
                memcpy(&side, &networkBuffer[4],  4);
                memcpy(&spin, &networkBuffer[8],  4);
                memcpy(&lt,   &networkBuffer[12], 4);
                memcpy(&rt,   &networkBuffer[16], 4);
                uint8_t buttons        = networkBuffer[20];
                uint8_t incoming_emote = networkBuffer[21];

                bool buttonAPressed          = (buttons & 0x01) != 0;
                bool buttonBPressed          = (buttons & 0x02) != 0;
                bool buttonL1Pressed         = (buttons & 0x04) != 0;
                bool buttonR1Pressed         = (buttons & 0x08) != 0;
                bool emoteModeTogglePressed  = (buttons & 0x10) != 0;
                bool emotePlayPressed        = (buttons & 0x20) != 0;
                bool emoteStopPressed        = (buttons & 0x40) != 0;


                if (incoming_emote < NUM_EMOTES) selected_emote_id = incoming_emote;

                joy_fwd  = (abs(fwd)  > JOYSTICK_DEADZONE) ? fwd  : 0.0;
                joy_side = (abs(side) > JOYSTICK_DEADZONE) ? side : 0.0;
                joy_spin = (abs(spin) > JOYSTICK_DEADZONE) ? spin : 0.0;
                norm_lt  = lt;
                norm_rt  = rt;

                // ----- HOLD-TO-CONFIRM L1+R1 STAND/SIT TOGGLE -----

                bool is_mid_transition = (target_standing_state && current_transition_progress < 1.0) ||
                                         (!target_standing_state && current_transition_progress > 0.0);

                if (buttonL1Pressed && buttonR1Pressed && !is_mid_transition
                    && !in_calibration_mode && !emote_mode_enabled) {
                    button_hold_time += DT_SEC;
                    if (button_hold_time >= REQUIRED_HOLD_DURATION) {
                        target_standing_state = !target_standing_state;
                        button_hold_time = 0.0;
                    }
                } else {
                    button_hold_time = 0.0;
                }

                // ----- CALIBRATION MODE (only reachable while fully sitting) -----
                bool fully_sitting = (!target_standing_state) && (current_transition_progress <= 0.0);

                if (!in_calibration_mode && fully_sitting && buttonAPressed && !lastAPressed) {
                    in_calibration_mode = true;
                }
                else if (in_calibration_mode && buttonAPressed && !lastAPressed) {
                    in_calibration_mode = false;
                }
                else if (in_calibration_mode && buttonBPressed && !lastBPressed) {
                    saveOffsetsToFlash();
                    justSavedFeedback = true;
                    saveFeedbackTimer = millis();
                    in_calibration_mode = false;
                }

                if (in_calibration_mode) {
                    bool dpadLeft  = (joy_spin < -0.7);
                    bool dpadRight = (joy_spin > 0.7);
                    bool dpadUp    = (joy_side > 0.7);
                    bool dpadDown  = (joy_side < -0.7);

                    if (dpadRight && !lastDpadRightPressed) selectedJointID = (selectedJointID + 1) % 18;
                    else if (dpadLeft && !lastDpadLeftPressed) selectedJointID = (selectedJointID - 1 + 18) % 18;
                    if (dpadUp && !lastDpadUpPressed) servoOffsets[selectedJointID] += 0.5;
                    else if (dpadDown && !lastDpadDownPressed) servoOffsets[selectedJointID] -= 0.5;

                    lastDpadLeftPressed  = dpadLeft;
                    lastDpadRightPressed = dpadRight;
                    lastDpadUpPressed    = dpadUp;
                    lastDpadDownPressed  = dpadDown;
                }

                // ----- BALANCE / DIRT / EMOTE TOGGLES + HEIGHT TRIM -----

                bool is_fully_active = (current_transition_progress >= 1.0) && !in_calibration_mode;

                if (is_fully_active && !emote_mode_enabled) {
                    if (norm_rt > TRIGGER_THRESHOLD) user_selected_height += HEIGHT_SPEED * norm_rt * DT_SEC;
                    if (norm_lt > TRIGGER_THRESHOLD) user_selected_height -= HEIGHT_SPEED * norm_lt * DT_SEC;
                    user_selected_height = constrain(user_selected_height, MIN_HEIGHT, MAX_HEIGHT);

                    if (buttonAPressed && !lastAPressed) balance_enabled   = !balance_enabled;
                    if (buttonBPressed && !lastBPressed) dirt_mode_enabled = !dirt_mode_enabled;
                } else if (!in_calibration_mode) {
                    joy_fwd = joy_side = joy_spin = 0.0;
                }

                // ----- EMOTE MODE CONTROL EDGES -----

                if (emoteModeTogglePressed && !lastEmoteModeTogglePressed) {
                    if (is_fully_active && !emote_mode_enabled) {
                        emote_mode_enabled = true;
                        emote_playing      = false;
                        emote_playing_id   = EMOTE_NONE;

                        balance_enabled    = false;
                        dirt_mode_enabled  = false;

                        exiting_emote_ramp = false;
                    } else if (emote_mode_enabled) {
                        emote_mode_enabled = false;
                        emote_playing      = false;
                        emote_playing_id   = EMOTE_NONE;

                        exiting_emote_ramp = true;
                    }
                }


                static bool pending_play = false;
                if (emotePlayPressed && !lastEmotePlayPressed && emote_mode_enabled && !emote_playing) {
                    pending_play = true;
                }


                if (emoteStopPressed && !lastEmoteStopPressed && emote_mode_enabled) {
                    emote_playing    = false;
                    emote_playing_id = EMOTE_NONE;
                    pending_play     = false;
                }


                if (pending_play && emote_mode_enabled && !emote_playing
                    && abs(current_body_height - EMOTE_BODY_HEIGHT) < EMOTE_HEIGHT_SETTLED_M
                    && selected_emote_id < NUM_EMOTES) {
                    emote_playing_id = selected_emote_id;
                    emote_start_ms   = millis();
                    emote_playing    = true;
                    pending_play     = false;
                }

                lastAPressed                = buttonAPressed;
                lastBPressed                = buttonBPressed;
                lastL1Pressed               = buttonL1Pressed;
                lastR1Pressed               = buttonR1Pressed;
                lastEmoteModeTogglePressed  = emoteModeTogglePressed;
                lastEmotePlayPressed        = emotePlayPressed;
                lastEmoteStopPressed        = emoteStopPressed;
            }
        }

        if (!bootSequenceComplete) {
            if (millis() - bootStartTime >= BOOT_DURATION_MS) bootSequenceComplete = true;
        }

        // ----- STAND/SIT TRANSITION RAMP -----
        if (bootSequenceComplete) {
            if (in_calibration_mode) {
                servosArePowered = true;
                current_transition_progress = 0.0;
            } else if (target_standing_state || emote_mode_enabled) {

                servosArePowered = true;
                current_transition_progress = min(1.0f, current_transition_progress + (DT_SEC / STARTUP_DURATION));
            } else {
                current_transition_progress = max(0.0f, current_transition_progress - (DT_SEC / STARTUP_DURATION));
                if (current_transition_progress <= 0.0 && servosArePowered) {
                    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        shutDownServosHardware();
                        xSemaphoreGive(i2cMutex);
                    }
                }
            }
        }

        float smooth_progress = 0.5 - 0.5 * cos(PI * current_transition_progress);
        bool is_fully_active = (current_transition_progress >= 1.0) && !in_calibration_mode;

        if (!target_standing_state && !servosArePowered && !in_calibration_mode && !emote_mode_enabled) {
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // ----- CALIBRATION NEUTRAL POSE -----
        if (in_calibration_mode) {
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                for (int idx = 0; idx < 6; idx++) {
                    String legName = LEG_ORDER[idx];
                    bool is_left = legName.endsWith("L");
                    int hipHWID, thighHWID, kneeHWID;

                    if (legName == "FR")      { kneeHWID = 0;  thighHWID = 1;  hipHWID = 2;  }
                    else if (legName == "FL") { kneeHWID = 3;  thighHWID = 4;  hipHWID = 5;  }
                    else if (legName == "MR") { kneeHWID = 6;  thighHWID = 7;  hipHWID = 8;  }
                    else if (legName == "ML") { kneeHWID = 9;  thighHWID = 10; hipHWID = 11; }
                    else if (legName == "RR") { kneeHWID = 12; thighHWID = 13; hipHWID = 14; }
                    else if (legName == "RL") { kneeHWID = 15; thighHWID = 16; hipHWID = 17; }

                    if (is_left) {
                        setServo(hipHWID,   90.0);
                        setServo(thighHWID, 90.0);
                        setServo(kneeHWID,  90.0);
                    } else {
                        setServo(hipHWID,   90.0);
                        setServo(thighHWID, 90.0);
                        setServo(kneeHWID,  90.0);
                    }
                }
                xSemaphoreGive(i2cMutex);
            }
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // ----- EMOTE MODE BRANCH -----

        if (emote_mode_enabled && bootSequenceComplete) {
            joy_fwd = joy_side = joy_spin = 0.0f;

            float target_height = EMOTE_BODY_HEIGHT;
            float height_error  = target_height - current_body_height;
            float max_step      = EMOTE_HEIGHT_SPEED * DT_SEC;
            if (abs(height_error) > max_step) {
                current_body_height += (height_error > 0 ? 1.0f : -1.0f) * max_step;
            } else {
                current_body_height = target_height;
            }

            runEmoteTick();

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // ----- ACTIVE GAIT ENGINE -----
        float local_fwd  = is_fully_active ? joy_fwd  : 0.0f;
        float local_side = is_fully_active ? joy_side : 0.0f;
        float local_spin = is_fully_active ? joy_spin : 0.0f;

        float joy_mag  = sqrt(local_fwd * local_fwd + local_side * local_side);
        float norm_mag = constrain(joy_mag, 0.0f, 1.0f);

        bool active = is_fully_active && (norm_mag > 0.02 || abs(local_spin) > 0.02);
        float direction_rad = (norm_mag > 0.001) ? atan2(local_side, local_fwd) : 0.0;

        CalculatedGait g = getBlendedGaitParams(local_fwd, local_side, local_spin, norm_mag, dirt_mode_enabled);

        urdf_x_filtered = (1.0f - URDF_X_ALPHA) * urdf_x_filtered + URDF_X_ALPHA * g.urdf_x;

        if (balance_enabled)   { g.freq *= 0.75;  g.step_amplitude *= 0.75;  }
        if (dirt_mode_enabled) { g.freq *= 0.75; g.step_amplitude *= 0.75; }

        if (active) {
            phase_accumulator = fmod(phase_accumulator + g.freq * DT_SEC, 1.0);
        } else {
            phase_accumulator = 0.0;
        }
        motion_blend += (((active) ? 1.0f : 0.0f) - motion_blend) * STOP_BLEND_SPEED;

        // ----- HEIGHT RAMP + BALANCE COMPENSATION BLEND -----
        float target_height = balance_enabled ? BALANCE_TARGET_HEIGHT : user_selected_height;

        float ramp_speed = exiting_emote_ramp ? EMOTE_HEIGHT_SPEED : HEIGHT_SPEED;
        float height_error  = target_height - current_body_height;
        float max_step      = ramp_speed * DT_SEC;
        if (abs(height_error) > max_step) {
            current_body_height += (height_error > 0 ? 1.0f : -1.0f) * max_step;
        } else {
            current_body_height = target_height;

            exiting_emote_ramp = false;
        }

        bool balance_height_settled = (abs(current_body_height - target_height) < 0.002f);

        if (balance_enabled && balance_height_settled) {
            balance_comp_blend = min(1.0f, balance_comp_blend + DT_SEC / BALANCE_BLEND_RAMP_SEC);
        } else {
            balance_comp_blend = 0.0f;
        }

        if (balance_enabled && balance_height_settled) {
            float target_pitch = (abs(body_pitch_filtered) > BALANCE_DEADBAND_RAD)
                                 ? body_pitch_filtered : 0.0f;
            float target_roll  = (abs(body_roll_filtered)  > BALANCE_DEADBAND_RAD)
                                 ? body_roll_filtered : 0.0f;
            smooth_comp_pitch = COMP_OUTPUT_ALPHA * smooth_comp_pitch + (1.0f - COMP_OUTPUT_ALPHA) * target_pitch;
            smooth_comp_roll  = COMP_OUTPUT_ALPHA * smooth_comp_roll  + (1.0f - COMP_OUTPUT_ALPHA) * target_roll;
        } else {
            smooth_comp_pitch = 0.0f;
            smooth_comp_roll  = 0.0f;
        }

        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            for (int idx = 0; idx < 6; idx++) {
                String legName = LEG_ORDER[idx];
                bool is_left  = legName.endsWith("L");
                bool is_right = legName.endsWith("R");
                float lx_root = LEG_ROOTS[idx][0];
                float ly_root = LEG_ROOTS[idx][1];

                float phase = fmod(phase_accumulator + LEG_PHASES[idx], 1.0);
                float s_phase = (phase < 0.5 ? phase : phase - 0.5) * 2.0;
                float cycloid_factor = s_phase - (sin(2.0 * PI * s_phase) / (2.0 * PI));

                float phase_multiplier, gait_z_offset;
                if (phase < 0.5) {
                    phase_multiplier = (1.0 - 2.0 * cycloid_factor);
                    gait_z_offset = g.step_h * 0.5 * (1.0 - cos(2.0 * PI * s_phase));
                } else {
                    phase_multiplier = (-1.0 + 2.0 * cycloid_factor);
                    gait_z_offset = 0.0;
                }

                float sign_x = (cos(direction_rad) >= 0) ? 1.0 : -1.0;
                float sign_y = (sin(direction_rad) >= 0) ? 1.0 : -1.0;

                float tx_trans = g.step_amplitude * phase_multiplier * abs(cos(direction_rad)) * sign_x * norm_mag;
                float ty_trans = g.step_amplitude * phase_multiplier * abs(sin(direction_rad)) * sign_y * norm_mag;

                float omega   = -local_spin * g.step_amplitude * 2.0;
                float tx_spin = -ly_root * omega * phase_multiplier;
                float ty_spin =  lx_root * omega * phase_multiplier;

                float tx = (tx_trans + tx_spin) * motion_blend;
                float ty = (ty_trans + ty_spin) * motion_blend;

                float tx_compensation = 0.0, ty_compensation = 0.0, delta_z = 0.0;

                if (balance_enabled && balance_height_settled) {
                    tx_compensation = -lx_root * (1.0f - cos(smooth_comp_pitch)) * balance_comp_blend;
                    ty_compensation = -ly_root * (1.0f - cos(smooth_comp_roll))  * balance_comp_blend;
                    delta_z = ((-lx_root * sin(smooth_comp_pitch)) - (ly_root * sin(smooth_comp_roll)))
                              * BALANCE_GAIN * balance_comp_blend;
                }

                tx += tx_compensation;
                ty += ty_compensation;
                float tz = current_body_height + delta_z + gait_z_offset;

                float current_h = 0.12 + (tz - 0.12) * smooth_progress;

                float hip, thigh, knee;
                if (solve_leg_ik_3dof(tx, ty, current_h, urdf_x_filtered, hip, thigh, knee)) {
                    float actual_th = is_left ? -thigh : thigh;
                    float actual_kn = is_left ? -knee  : knee;

                    if (is_right) actual_th = -thigh;

                    float i_kn = is_left ? -KNEE_ASSEMBLY_OFFSET : KNEE_ASSEMBLY_OFFSET;

                    float f_h_real  = hip      * smooth_progress;
                    float f_th_real = actual_th * smooth_progress;
                    float f_kn_real = i_kn + (actual_kn - i_kn) * smooth_progress;

                    float h_phys_deg  = f_h_real  * 180.0 / PI;
                    float th_phys_deg = f_th_real * 180.0 / PI;
                    float kn_phys_deg = (f_kn_real - i_kn) * 180.0 / PI;

                    int hipHWID, thighHWID, kneeHWID;
                    if (legName == "FR")      { kneeHWID = 0;  thighHWID = 1;  hipHWID = 2;  }
                    else if (legName == "FL") { kneeHWID = 3;  thighHWID = 4;  hipHWID = 5;  }
                    else if (legName == "MR") { kneeHWID = 6;  thighHWID = 7;  hipHWID = 8;  }
                    else if (legName == "ML") { kneeHWID = 9;  thighHWID = 10; hipHWID = 11; }
                    else if (legName == "RR") { kneeHWID = 12; thighHWID = 13; hipHWID = 14; }
                    else if (legName == "RL") { kneeHWID = 15; thighHWID = 16; hipHWID = 17; }

                    if (is_left) {
                        setServo(hipHWID,   90.0 - h_phys_deg);
                        setServo(thighHWID, 90.0 - th_phys_deg);
                        setServo(kneeHWID,  90.0 + kn_phys_deg);
                    } else {
                        setServo(hipHWID,   90.0 - h_phys_deg);
                        setServo(thighHWID, 90.0 + th_phys_deg);
                        setServo(kneeHWID,  90.0 + kn_phys_deg);
                    }
                }
            }
            xSemaphoreGive(i2cMutex);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

#endif // ROBOT_GAIT_MECHANISM_H
