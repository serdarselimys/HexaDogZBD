# Hexapod ESP32 Firmware

Firmware for an 18-servo hexapod robot running on an ESP32. It handles walking, balancing, scripted "emote" animations, servo calibration, live telemetry over UDP, and an animated eye display on a TFT screen.

The code is designed to be edited in one place per concern — you shouldn't have to hunt through the whole project to change how the robot moves or what its eyes look like.

---

## Hardware

- **MCU:** ESP32 (dual-core, uses both cores)
- **Servo drivers:** 2× Adafruit PCA9685 PWM boards (I²C addresses `0x40` and `0x41`)
- **IMU:** MPU6050 (I²C `0x68`)
- **Display:** TFT panel driven via `TFT_eSPI`
- **Servos:** 18 total (6 legs × 3 joints: hip, thigh, knee)
- **Power sensing:** analog voltage divider on GPIO 32
- **I²C pins:** SDA = 21, SCL = 22, clock 400 kHz

The robot exposes itself as a Wi-Fi access point (`HEXAPOD_ESP32`, password `12345678`) and communicates with a companion Android controller app over UDP on port `5000`.

---

## File layout

Everything lives in the main sketch folder. Each file has a single job.

| File | What's in it |
|---|---|
| `Hexapod_Main.ino` | All tunable constants (geometry, gait timing, balance gains, network config, pins), hardware instances, runtime state, and `setup()` / `loop()`. **Start here to tune anything.** |
| `Robot_Gait_Mechanism.h` | Inverse kinematics, gait blending from the CSV-derived motion matrix, the main `KinematicsTask` running on Core 1, IMU complementary filter, and low-level servo I/O with flash-persisted offsets. |
| `Robot_Emotes.h` | The emote catalog and all scripted animations (wiggle, play bow, victory wave, push-ups, etc.), plus the per-tick emote runner. |
| `Screen_Settings.h` | TFT UI, animated eye engine, battery reading, and the `TelemetryTask` running on Core 0 (also sends UDP telemetry back to the controller). |

The three headers are included from `Hexapod_Main.ino` in a specific order (Emotes → Gait → Screen) because of cross-references between the modules — don't reorder them.

---

## Where to edit what

- **Change a tuning value** (heights, speeds, deadzones, gains, pins, Wi-Fi credentials): `Hexapod_Main.ino`
- **Change how walking or balancing works**: `Robot_Gait_Mechanism.h`
- **Add or change an emote**: `Robot_Emotes.h` — write a new `emote_yourthing(float t, EmoteTargets &out)` function, then add an entry to the `EMOTES[]` catalog at the bottom of the file.
- **Change what's on screen**: `Screen_Settings.h`

---

## Runtime behavior

### Boot
- ~2.5 s sit-still window while gyro bias is measured (200 samples).
- Screen shows "Calibrating IMU gyro…" then the measured bias.

### Modes
- **Asleep** — servos unpowered, screen shows telemetry.
- **Standing / walking** — normal mode. Animated eyes appear on screen.
- **Balance mode** — active leg-shift compensation for pitch/roll using the IMU.
- **Dirt mode** — reduced gait frequency and amplitude for rough terrain.
- **Emote mode** — walking disabled, robot rises to a fixed height and plays scripted animations.
- **Calibration mode** — only reachable while fully sitting; lets you nudge per-servo offsets with the D-pad and save them to flash.

### Input packet (Android app → ESP32, 22 bytes, little-endian)
```
[0:4]   joy_fwd    float
[4:8]   joy_side   float
[8:12]  joy_spin   float
[12:16] norm_lt    float   (left trigger 0..1)
[16:20] norm_rt    float   (right trigger 0..1)
[20]    buttons    uint8   bit0=A bit1=B bit2=L1 bit3=R1
                           bit4=emote_mode_toggle
                           bit5=emote_play
                           bit6=emote_stop
[21]    selected_emote_id  uint8 (0..NUM_EMOTES-1)
```

### Telemetry packet (ESP32 → Android app, 28 bytes, ~20 Hz)
IMU X/Y/Z (float ×3), battery voltage (float), selected joint (int32), current joint offset (float), display state (u8), balance on/off (u8), dirt on/off (u8), currently playing emote id (u8, `255` = none).

### Key controls
- **Hold L1 + R1 for 2 s** — toggle stand / sit.
- **A while sitting** — enter calibration mode; D-pad selects joint and nudges offset; **B** saves to flash.
- **A while standing** — toggle balance mode.
- **B while standing** — toggle dirt mode.
- **Triggers (LT / RT)** — trim body height down / up.
- **Emote mode toggle** — enter/exit emote mode (stand mode only).
- **Emote play / stop** — start the currently selected emote / cancel it.

---

## Emotes included

Curious Head Tilt, Shy Peek-a-boo, Cautious Object Tap, The Wiggle, Play Bow, Happy Dance into Sneak, Stadium Ripple Wave, Breathing into Foot Stomp, Itch Scratch, Matrix Gyro Roll, Push-Ups, Sit & Wave Hello, Victory Wave, Battle Mode / Intimidate.

Each entry in `EMOTES[]` has a name, duration, intro fade-in seconds, outro fade-out seconds, and a pointer to its animation function. Emotes are ported from a PyBullet reference simulation, which is why the poses/timing look consistent across the catalog.

---

## Dependencies

Install these libraries in the Arduino IDE (Library Manager):

- `Adafruit PWM Servo Driver Library`
- `Adafruit MPU6050`
- `Adafruit Unified Sensor`
- `TFT_eSPI` (needs `User_Setup.h` configured for your specific TFT wiring)
- `Preferences`, `WiFi`, `WiFiUdp`, `Wire`, `SPI` (bundled with the ESP32 core)

Board: **ESP32 Dev Module** (or your specific board) via the Espressif ESP32 Arduino core.

---

## Building & flashing

1. Open `Hexapod_Main.ino` in the Arduino IDE. The three `.h` files must sit next to it in the same folder.
2. Configure `TFT_eSPI`'s `User_Setup.h` to match your display's pins.
3. Select your ESP32 board and the correct serial port.
4. Upload.
5. Connect to the `HEXAPOD_ESP32` Wi-Fi network from the companion controller app and start sending 22-byte packets to `192.168.4.1:5000`.

---

## Calibration workflow

1. With the robot sitting, press **A** to enter calibration mode.
2. Use the D-pad (mapped from the right stick) to select a joint and nudge its offset in 0.5° steps.
3. Press **B** to save offsets to flash NVS. They persist across reboots.
4. Press **A** again to exit without saving.

---

## Safety notes

- Servos are actively disabled (`shutDownServosHardware()`) whenever the robot is fully sitting and not in calibration mode — no holding torque, no buzz.
- Height is clamped between `MIN_HEIGHT` and `MAX_HEIGHT` (0.14 m – 0.23 m).
- Balance compensation only engages once the body height has settled, ramps in over 1 s, and has a deadband so it ignores small IMU noise.
- Emote playback won't start until the body has actually reached emote height, so animations never jerk mid-ramp.

---

## Licensing & Commercial Use

These files are licensed under the Creative Commons Attribution–NonCommercial 4.0 International (CC BY-NC 4.0) license.

You are free to remix, adapt, and build upon this design for non-commercial purposes, as long as you give appropriate credit.

You may not use the material for commercial purposes of any kind.

Link: https://creativecommons.org/licenses/by-nc/4.0/
