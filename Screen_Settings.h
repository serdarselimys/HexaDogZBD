#ifndef SCREEN_SETTINGS_H
#define SCREEN_SETTINGS_H


// Shell for the Telemetry Layout
void setupDiagnosticUI() {
    tft.setRotation(SCREEN_ROTATION);
    tft.fillScreen(MY_BLACK); tft.setTextColor(TFT_WHITE, MY_BLACK); tft.setTextSize(2);
    tft.setCursor(10, 10); tft.println("HEXAPOD CORE TELEMETRY"); tft.drawFastHLine(0, 32, tft.width(), TFT_DARKGREY);
}

// Battery ADC read -- small, only feeds the display, so it stays here.
void readBattery() {
    int rawADC = analogRead(VOLTAGE_PIN);
    float measuredPinVoltage = (rawADC / 4095.0) * 3.3 * ADC_CAL_FACTOR;
    batteryVoltage = measuredPinVoltage * DIVIDER_RATIO;
}

// Telemetry Rendering, Outbound Transmitter, & Animated Eye Engine (Core 0)
void TelemetryTask(void * pvParameters) {
    unsigned long lastBatteryTime = 0;
    unsigned long lastUdpTxTime   = 0;

    // ----- BOOT: gyro bias calibration (logic lives in the gait file) -----
    tft.setTextSize(1);
    tft.setTextColor(TFT_YELLOW, MY_BLACK);
    tft.setCursor(10, 50);
    tft.print("Calibrating IMU gyro...");

    calibrateGyroBias();

    tft.setCursor(10, 62);
    tft.setTextColor(TFT_GREEN, MY_BLACK);
    tft.printf("Done. Bias X:%.4f Y:%.4f", gyroBiasX, gyroBiasY);
    vTaskDelay(pdMS_TO_TICKS(600));


    uint8_t txTelemetryBuffer[28];

    for(;;) {
        updateIMUAndBalance();

        if (millis() - lastBatteryTime >= 2000) {
            lastBatteryTime = millis();
            readBattery();
        }


        uint8_t displayState;
        if (in_calibration_mode) {
            displayState = 2;
        } else if (emote_mode_enabled) {
            displayState = 3;
        } else if (target_standing_state) {
            displayState = 1;
        } else {
            displayState = 0;
        }

        // ----- UDP TELEMETRY OUTBOUND TRANSMITTER (28-BYTE PACKET) -----
        if (millis() - lastUdpTxTime >= 50) {
            lastUdpTxTime = millis();

            IPAddress remoteIp = udp.remoteIP();
            uint16_t remotePort = udp.remotePort();

            if (remoteIp[0] != 0) {
                float currentActiveOffset = servoOffsets[selectedJointID];
                int32_t activeJointInt = (int32_t)selectedJointID;
                uint8_t playing_id_out = emote_playing ? emote_playing_id : EMOTE_NONE;

                memcpy(&txTelemetryBuffer[0],  (const void*)&imu_ax, 4);
                memcpy(&txTelemetryBuffer[4],  (const void*)&imu_ay, 4);
                memcpy(&txTelemetryBuffer[8],  (const void*)&imu_az, 4);
                memcpy(&txTelemetryBuffer[12], (const void*)&batteryVoltage, 4);
                memcpy(&txTelemetryBuffer[16], &activeJointInt, 4);
                memcpy(&txTelemetryBuffer[20], (const void*)&currentActiveOffset, 4);
                txTelemetryBuffer[24] = displayState;
                txTelemetryBuffer[25] = balance_enabled ? 1 : 0;
                txTelemetryBuffer[26] = dirt_mode_enabled ? 1 : 0;
                txTelemetryBuffer[27] = playing_id_out;

                udp.beginPacket(remoteIp, remotePort);
                udp.write(txTelemetryBuffer, 28);
                udp.endPacket();
            }
        }

        if (displayState != lastRenderedState) {
            tft.fillScreen(MY_BLACK);
            if (displayState != 1) {
                setupDiagnosticUI();
            } else {
                tft.setRotation(SCREEN_ROTATION);
            }
            lastRenderedState = displayState;
            screenDrawnForMotion = false;
        }

        if (displayState == 1 && current_transition_progress >= 0.5) {
            // Eye engine -- only in normal standing/active mode.
            int midX = tft.width() / 2;
            int midY = tft.height() / 2;
            bool currentlyMoving = (sqrt(joy_fwd*joy_fwd + joy_side*joy_side) > 0.05 || abs(joy_spin) > 0.05);

            if (currentlyMoving) {
                lastMovementTime = millis();
                if (!screenDrawnForMotion) {
                    tft.fillScreen(MY_BLACK);
                    tft.fillRoundRect(midX - eyeSpacing - eyeWidth, midY - (eyeHeight / 2), eyeWidth, eyeHeight, 15, MY_CYAN);
                    tft.fillRoundRect(midX + eyeSpacing,            midY - (eyeHeight / 2), eyeWidth, eyeHeight, 15, MY_CYAN);
                    screenDrawnForMotion = true;
                    isBlinking = false;
                    horizontalEyeOffset = 0;
                }
            }
            else {
                unsigned long idleDuration = millis() - lastMovementTime;
                screenDrawnForMotion = false;

                if (idleDuration < 1000) {
                    if (isBlinking || horizontalEyeOffset != 0) {
                        tft.fillScreen(MY_BLACK);
                        isBlinking = false;
                        horizontalEyeOffset = 0;
                    }
                    tft.fillRoundRect(midX - eyeSpacing - eyeWidth, midY - (eyeHeight / 2), eyeWidth, eyeHeight, 15, MY_CYAN);
                    tft.fillRoundRect(midX + eyeSpacing,            midY - (eyeHeight / 2), eyeWidth, eyeHeight, 15, MY_CYAN);
                }
                else {
                    if (millis() > eyeLookTimer) {
                        eyeLookTimer = millis() + random(3000, 6000);
                        int roll = random(0, 3);
                        int oldOffset = horizontalEyeOffset;
                        if (roll == 0) horizontalEyeOffset = -25;
                        else if (roll == 1) horizontalEyeOffset = 25;
                        else horizontalEyeOffset = 0;

                        if(oldOffset != horizontalEyeOffset) {
                            tft.fillScreen(MY_BLACK);
                        }
                    }

                    if (!isBlinking && millis() > blinkTimer) {
                        isBlinking = true;
                        blinkDuration = millis() + random(150, 250);
                    } else if (isBlinking && millis() > blinkDuration) {
                        isBlinking = false;
                        blinkTimer = millis() + random(5000, 10000);
                        tft.fillScreen(MY_BLACK);
                    }

                    if (isBlinking) {
                        tft.fillRoundRect(midX - eyeSpacing - eyeWidth + horizontalEyeOffset, midY - 6, eyeWidth, 12, 6, MY_CYAN);
                        tft.fillRoundRect(midX + eyeSpacing + horizontalEyeOffset,            midY - 6, eyeWidth, 12, 6, MY_CYAN);
                    } else {
                        tft.fillRoundRect(midX - eyeSpacing - eyeWidth + horizontalEyeOffset, midY - (eyeHeight / 2), eyeWidth, eyeHeight, 15, MY_CYAN);
                        tft.fillRoundRect(midX + eyeSpacing + horizontalEyeOffset,            midY - (eyeHeight / 2), eyeWidth, eyeHeight, 15, MY_CYAN);
                    }
                }
            }
        }
        else {
            // ----- TELEMETRY-STYLE DISPLAY (SIT / CALIBRATE / EMOTE) -----
            screenDrawnForMotion = false;
            tft.setTextSize(2);
            tft.setCursor(10, 35);

            if (displayState == 2) {
                tft.setTextColor(TFT_ORANGE, MY_BLACK);
                tft.printf("MODE: CALIBRATE ");
            } else if (displayState == 3) {
                tft.setTextColor(MY_CYAN, MY_BLACK);
                tft.printf("MODE: EMOTE     ");
            } else if (displayState == 0 && current_transition_progress <= 0.0) {
                if (justSavedFeedback && (millis() - saveFeedbackTimer < 3000)) {
                    tft.setTextColor(TFT_CYAN, MY_BLACK);
                    tft.print("OFFSETS SAVED!  ");
                } else {
                    justSavedFeedback = false;
                    tft.setTextColor(TFT_DARKGREY, MY_BLACK);
                    tft.print("MODE: ASLEEP    ");
                }
            } else {
                tft.setTextColor(TFT_YELLOW, MY_BLACK);
                tft.print("MODE: SHIFTING  ");
            }

            tft.setTextSize(2);
            tft.setCursor(10, 65);
            if (displayState == 2) {
                tft.setTextColor(TFT_GREEN, MY_BLACK);
                tft.printf("JOINT: %s [ID:%02d]      \n", JOINT_NAMES[selectedJointID], selectedJointID);
                tft.setCursor(10, 90);
                tft.printf("OFFSET: %2.1f Deg      ", servoOffsets[selectedJointID]);
            } else if (displayState == 3) {
 
                tft.setTextSize(1);
                if (emote_playing && emote_playing_id < NUM_EMOTES) {
                    tft.setTextColor(TFT_GREEN, MY_BLACK);
                    tft.printf("PLAYING: %s                    ", EMOTES[emote_playing_id].name);
                    tft.setCursor(10, 78);
                    float t_e = (millis() - emote_start_ms) / 1000.0f;
                    float dur = EMOTES[emote_playing_id].duration_sec;
                    tft.setTextColor(TFT_YELLOW, MY_BLACK);
                    tft.printf("T: %.1fs / %.1fs               ", t_e, dur);
                } else {
                    tft.setTextColor(TFT_DARKGREY, MY_BLACK);
                    tft.print("READY                                    ");
                    tft.setCursor(10, 78);
                    tft.setTextColor(TFT_GREEN, MY_BLACK);
                    if (selected_emote_id < NUM_EMOTES) {
                        tft.printf("SELECTED: %s                    ", EMOTES[selected_emote_id].name);
                    } else {
                        tft.print("SELECTED: --                              ");
                    }
                }
            } else {
                tft.fillRect(10, 65, 230, 50, MY_BLACK);
                float volt_copy = batteryVoltage;
                if (volt_copy < 10.2) tft.setTextColor(TFT_RED, MY_BLACK);
                else if (volt_copy < 11.1) tft.setTextColor(TFT_YELLOW, MY_BLACK);
                else tft.setTextColor(MY_CYAN, MY_BLACK);
                tft.printf("BAT: %2.2f V  ", volt_copy);
            }


            if (displayState != 2 && displayState != 3) {
                tft.setTextSize(1);
                tft.setCursor(10, 100);
                tft.setTextColor(balance_enabled ? MY_CYAN : TFT_DARKGREY, MY_BLACK);
                tft.printf("BALANCE: %s   ", balance_enabled ? "ON " : "OFF");
                tft.setCursor(140, 100);
                tft.setTextColor(dirt_mode_enabled ? MY_CYAN : TFT_DARKGREY, MY_BLACK);
                tft.printf("DIRT: %s   ", dirt_mode_enabled ? "ON " : "OFF");
            }

            tft.setTextSize(2);
            tft.setTextColor(TFT_GREEN, MY_BLACK);
            tft.setCursor(10, 120);
            tft.printf("IMU X: %1.2f m/s2  ", imu_ax);
            tft.setCursor(10, 145);
            tft.printf("IMU Y: %1.2f m/s2  ", imu_ay);
            tft.setCursor(10, 170);
            tft.printf("IMU Z: %1.2f m/s2  ", imu_az);
            tft.setTextSize(1);
            tft.setTextColor(TFT_YELLOW, MY_BLACK);
            tft.setCursor(10, 195);
            tft.printf("ROLL: %+6.1f deg   ", degrees(body_roll_filtered));
            tft.setCursor(10, 207);
            tft.printf("PITCH:%+6.1f deg   ", degrees(body_pitch_filtered));
        }

        // DYNAMIC HIGH-SPEED OVERRIDE FOR TELEMETRY RESPONSIVENESS
        if (displayState == 2) {
            vTaskDelay(pdMS_TO_TICKS(15));
        } else if (displayState == 3) {
            // Emote text updates ~10 Hz so the playback timer looks live.
            vTaskDelay(pdMS_TO_TICKS(100));
        } else if (displayState == 1 && (sqrt(joy_fwd*joy_fwd + joy_side*joy_side) > 0.05 || abs(joy_spin) > 0.05)) {
            vTaskDelay(pdMS_TO_TICKS(100));
        } else if (displayState == 1) {
            vTaskDelay(pdMS_TO_TICKS(40));
        } else {
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }
}

#endif // SCREEN_SETTINGS_H
