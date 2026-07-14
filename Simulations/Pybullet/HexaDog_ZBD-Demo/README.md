# HexaDog ZBD — Hexapod Robot Simulation

A PyBullet-based simulation of the **HexaDog ZBD** hexapod robot, driven by a tripod gait with real-time joystick or keyboard control. Gait parameters (frequency, step amplitude) are loaded from a CSV lookup table and smoothly blended across direction axes (forward, sideways, diagonal, spin).

---

## Features

- Real-time 3-DOF inverse kinematics for all 6 legs
- Tripod gait with cycloidal foot trajectory
- Smooth parameter blending between straight / sideways / diagonal / spin motion
- Xbox joystick support, with keyboard fallback
- Adjustable body height via triggers (or default)
- Per-cycle speed reporting in the console

---

## Repository Structure

```
HexaDog_ZBD-Demo/
├── gait_params.csv           # Gait parameter database (speed, frequency, step_amplitude, direction)
├── HexaDog_ZBD.urdf          # URDF model of the robot
├── Meshes/                   # Mesh files referenced by the URDF
├── HexaDog_ZBD_Pybullet.py   # Main simulation script
├── requirements.txt          # Python dependencies
└── README.md
```

---

## 1. Prerequisites

- **Operating System:** Windows 10/11, macOS, or Linux
- **Python:** 3.9 – 3.11 (PyBullet wheels are most reliable in this range)
- **Git:** to clone the repository
- **(Optional)** Logitech F710 gamepad in **X-mode** (switch on the front of the controller set to `X`)

### Install Python

- **Windows:** Download from [python.org/downloads](https://www.python.org/downloads/). During installation, **check "Add Python to PATH"**.
- **macOS:** `brew install python@3.11` (with [Homebrew](https://brew.sh/)) or download from python.org.
- **Linux (Debian/Ubuntu):**
  ```bash
  sudo apt update
  sudo apt install python3 python3-venv python3-pip git
  ```

Verify:
```bash
python --version
pip --version
git --version
```

---

## 2. Clone the Repository

```bash
git clone https://github.com/<your-username>/hexapod-simulation.git
cd hexapod-simulation
```

Replace `<your-username>` with the actual GitHub account/organization hosting the repo.

---

## 3. Create a Virtual Environment

Keeping dependencies isolated is strongly recommended.

**Windows (PowerShell):**
```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
```

If PowerShell blocks the activation script, run once:
```powershell
Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
```

**Windows (Command Prompt):**
```cmd
python -m venv venv
venv\Scripts\activate.bat
```

**macOS / Linux:**
```bash
python3 -m venv venv
source venv/bin/activate
```

Your prompt should now be prefixed with `(venv)`.

---

## 4. Install Dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

If you don't yet have a `requirements.txt`, use:

```bash
pip install pybullet pandas numpy pygame
```

Then generate the file:
```bash
pip freeze > requirements.txt
```

---

## 5. Run the Simulation

Make sure you are in the project root (where `HexaDog_ZBD_Pybullet.py`, `HexaDog_ZBD.urdf`, and `gait_params.csv` all live), then:

```bash
python HexaDog_ZBD_Pybullet.py
```

A PyBullet GUI window will open with the robot standing on a plane.

---

## 6. Controls

### Logitech F710 (X-mode) — Tested.

| Input | Action |
|---|---|
| Left stick Y | Walk forward / backward |
| Left stick X | Strafe left / right |
| Right stick X | Spin (yaw) left / right |
| Right trigger (RT) | Raise body height |
| Left trigger (LT) | Lower body height |

Diagonal walking works automatically — push the left stick at any angle and the gait parameters blend between the straight and sideways lookup tables.

### Keyboard fallback (no joystick detected)

Click the PyBullet window first so it captures key events.

| Key | Action |
|---|---|
| ↑ / ↓ | Forward / backward |
| ← / → | Strafe left / right |
| Q / E | Spin CCW / CW |

Body-height adjustment is joystick-only in the current script.

---

## 7. Console Output

Each completed gait cycle prints a line like:

```
[Cycle 012] Speed:  14.32 cm/s | Freq: 1.85Hz | Target Body H: 0.200m
```

Use these to sanity-check that the gait parameters in `gait_params.csv` produce the speeds you expect.

---

## 8. Troubleshooting

- **`FileNotFoundError: HexaDog_ZBD.urdf`** — run the script from the project root, not from another folder.
- **URDF loads but meshes are missing/invisible** — the `Meshes/` folder must sit next to the URDF, and mesh paths inside the URDF must match (case-sensitive on Linux/macOS).
- **`pygame.error: No available video device` / joystick not detected** — plug in the controller before launching, switch it to **X-mode** for Logitech F710, then re-run. The script automatically falls back to keyboard if no joystick is found.
- **PyBullet fails to install on Python 3.12+** — downgrade to Python 3.11.
- **Robot collapses or twitches** — check that `gait_params.csv` has non-zero `frequency` and `step_amplitude` entries for each `direction` (`straight`, `sideways`, `diagonal`, `spin`).

---

## 9. Deactivating the Environment

When you're done:
```bash
deactivate
```

---

## License

Add your preferred license here (e.g., MIT).
