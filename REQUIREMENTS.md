# Requirements

This file lists the main software and hardware requirements for running the Tic-Tac-Toe Dobot project.

---

## 1. Hardware

- Dobot Magician Lite robotic arm.
- USB camera (overhead view).
- Computer (Windows / Linux / macOS) with:
  - At least 4 GB RAM.
  - USB ports for Dobot and camera.
- Pen or pencil compatible with the Dobot pen holder.
- A4 paper taped on the workspace.

---

## 2. Software

### 2.1. Operating System

- Windows 10/11, or
- Ubuntu 20.04+ (or similar Linux distro), or
- macOS (if Dobot drivers and pyserial work properly).

### 2.2. Python

- Python 3.8 or newer is recommended.

---

## 3. Python Dependencies

You can install dependencies using:

```bash
pip install -r REQUIREMENTS.txt
```

Suggested contents of `REQUIREMENTS.txt`:

```txt
opencv-python
numpy
pyserial
pydobot
```

If you use extra libraries (for GUI, logging, or configuration), add them to this list as well.

---

## 4. Dobot and Drivers

- Install Dobot drivers/software as required by your OS.
- Confirm the robot appears as a serial device:
  - On Linux: `/dev/ttyACM*` or `/dev/ttyUSB*`.
  - On Windows: `COMx` (for example, `COM3`).

Update the `DOBOT_PORT` environment variable or configuration in the script if needed.

---

## 5. Camera

- A UVC-compatible USB camera.
- Ensure the camera index in code (or `CAM_INDEX` environment variable) matches your system.
- The camera should clearly see the entire 3×3 grid and some border around it.

---

## 6. Optional Tools

- Git (for version control).
- Virtual environment tools:
  - `venv` (built into Python), or
  - `conda` if you prefer Anaconda/Miniconda.

---

## 7. Quick Installation Summary

```bash
# Create and activate a virtual environment (optional)
python -m venv venv
source venv/bin/activate   # Linux/macOS
# or
venv\Scripts\activate    # Windows

# Install dependencies
pip install opencv-python numpy pyserial pydobot

# Run the main script
python Tic_tak_toe.py
```
