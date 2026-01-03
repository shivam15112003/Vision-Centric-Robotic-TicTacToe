# How to Use – Vision-Centric Tic-Tac-Toe Robot

This guide explains how to set up and run the Tic-Tac-Toe robot that uses an overhead camera and the Dobot Magician Lite.

---

## 1. Hardware Requirements

- Dobot Magician Lite robotic arm
- USB camera mounted above the playing area (overhead view)
- Computer with Python 3 installed
- A4 paper taped on the table within Dobot’s reachable area
- Pen/pencil mounted in the Dobot pen holder

---

## 2. Software Requirements

Make sure Python 3 is installed, then install the required packages:

```bash
pip install -r REQUIREMENTS.txt
```

or, if you prefer manually:

```bash
pip install opencv-python numpy pyserial pydobot
```

(If you add more dependencies, list them in `REQUIREMENTS.md` / `REQUIREMENTS.txt`.)

---

## 3. Environment Variables

Optional environment variables (with typical examples):

- `DOBOT_PORT` – Serial port of the Dobot (e.g. `/dev/ttyACM0` on Linux, `COM3` on Windows)
- `CAM_INDEX` – Camera index (default might be `0`, `1`, or `2`)
- `Z_DRAW` – Z height for drawing on paper (e.g. `-35`)
- `Z_SAFE` – Z height for safe travel (e.g. `25`)
- `GRID_W`, `GRID_H` – Width and height of the Tic-Tac-Toe grid in mm (e.g. `150 × 150`)

You can set them like:

```bash
export DOBOT_PORT=/dev/ttyACM0
export CAM_INDEX=0
```

(on Windows PowerShell, use `$env:DOBOT_PORT="COM3"`).

---

## 4. Running the Program

1. Place all project files (including `Tic_tak_toe.py` and `camera.py`) in one folder.
2. (Optional) Create and activate a virtual environment.
3. Install dependencies:

   ```bash
   pip install -r REQUIREMENTS.txt
   ```

4. Run the main script:

   ```bash
   python Tic_tak_toe.py
   ```

5. Follow terminal prompts:
   - The robot will home (if implemented), then draw the grid.
   - You’ll be asked who goes first (`r` for robot, `h` for human).
   - After that, the camera continuously watches the board.

---

## 5. How Gameplay Works

1. **Robot draws grid**  
   The Dobot draws a 3×3 grid on the paper.

2. **Camera detects board**  
   The camera feed is processed to detect grid lines and warp the board into a fixed 3×3 representation.

3. **Human makes move**  
   - Draw an X or O in one cell.
   - The system waits until the board is visually stable (same reading for N frames).
   - It checks that exactly one new symbol has appeared.

4. **Move validation**  
   - If the move is valid, the internal board state is updated.
   - If invalid (multiple new marks, erasures, or tampering), the robot writes an “E” and logs the error in the console.

5. **Robot move**  
   - The robot runs Minimax on the current board.
   - It chooses the best move (X or O depending on who it is).
   - The Dobot draws its symbol in the selected cell.

6. **Game end**  
   - If the robot or human wins, the robot:
     - Draws a line through the winning three cells.
     - Writes “W” or “L”.
   - If it’s a draw, the robot writes “D”.

---

## 6. Troubleshooting

- Robot doesn’t draw on paper: adjust `Z_DRAW` so the pen just touches the paper.
- Camera can’t see full grid: raise the camera or zoom out so all 9 cells are visible.
- Misclassification of symbols: draw clearer X/O, avoid glare, and ensure good contrast.
- Port errors: confirm `DOBOT_PORT` matches your system’s port name for the Dobot.

---

## 7. Safety

- Keep your hands away from the robot while in motion.
- Use conservative speeds and safe Z heights.
- Stop the script with `Ctrl + C` if anything unexpected happens.
