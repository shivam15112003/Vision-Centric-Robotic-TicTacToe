# Vision-Centric Tic-Tac-Toe Robot on Dobot Magician Lite

This repository contains a fully autonomous, vision-driven Tic-Tac-Toe robot built on the **Dobot Magician Lite**.

An overhead camera is the only sensor:
- The robot draws the 3×3 grid on paper.
- Uses computer vision to “see” the board.
- Validates human moves strictly from vision.
- Plays optimally using a depth-aware Minimax policy.
- Writes the game result (W/L/D) or error code (E) directly on the paper.

---

## 🎥 Demonstration

- ▶️ **YouTube Demo (Shorts)**:  
  https://youtube.com/shorts/80Viijxftmc?si=0B3xEiCY_VNAyPee

---

## 🚀 Quick Start

1. Install Python 3 and required packages (see `REQUIREMENTS.md`).
2. Connect the **Dobot Magician Lite** and **overhead camera**.
3. Adjust environment variables if needed (e.g., `DOBOT_PORT`, `CAM_INDEX`).
4. Run:

   ```bash
   python Tic_tak_toe.py
   ```

5. Follow the on-terminal prompt for who goes first, then play by drawing X/O on the paper while the camera watches.

For full details, see:
- `HOW_TO_USE.md` – setup & usage
- `METHODOLOGY.md` – technical details and approach

---

## 🧠 Key Features

- **Self-anchored perception**: the robot draws the grid first, then detects those lines to calibrate the board geometry.
- **Symbol classification**: OpenCV pipeline detects X/O vs empty using edges, lines, circles and ink fraction.
- **Strict move validation**: only accepts states where exactly one valid new human mark appears (no erasing or modifying older marks).
- **Depth-aware Minimax agent**: ensures optimal play while preferring faster wins and delaying losses.
- **On-paper feedback**: the robot writes **W, L, D, or E** on the paper to indicate the outcome or rule violations.

---

## 📁 Repo Structure

```text
.
├── Tic_tak_toe.py            # Main robot + vision + Minimax script
├── HOW_TO_USE.md             # Setup and usage instructions
├── METHODOLOGY.md            # Approach, algorithms, and system design
├── REQUIREMENTS.md           # Dependencies and environment
├── Tic_tak_toe.docx          # Project report / paper
└── README.md                 # Project overview
```
