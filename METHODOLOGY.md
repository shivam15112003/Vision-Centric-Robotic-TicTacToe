# Methodology – Vision-Centric Tic-Tac-Toe on Dobot

This document explains the core methods used to implement the Tic-Tac-Toe robot: from perception to decision-making and actuation.

---

## 1. System Overview

The system follows the classic perception–decision–action loop:

1. Perception – Overhead camera observes the game board.
2. Decision – A Minimax-based agent chooses optimal moves.
3. Action – Dobot Magician Lite draws the grid, its moves, and outcome.

The human interacts only through the paper (drawing X/O), and the robot understands moves purely from vision.

---

## 2. Grid Drawing and Coordinate System

- The robot starts from a known pose (center of the future grid).
- Using simple geometry, it:
  - Computes the coordinates of grid lines in robot space (mm).
  - Moves at safe Z for travel and at draw Z for plotting.
- The grid is axis-aligned to the robot’s base, simplifying planning.

---

## 3. Board Detection and Warping

Once the grid is drawn, the camera takes a frame:

1. Preprocessing
   - Convert to grayscale.
   - Apply blur to reduce noise.
   - Run Canny edge detection.

2. Line Detection
   - Use Hough line transform to detect the grid lines.
   - Cluster lines into vertical and horizontal groups.
   - Intersect them to infer the four outer corners of the grid.

3. Perspective Warp
   - Define a fixed square output size (for example, 540×540 pixels).
   - Compute homography from the detected corner quadrilateral to this square.
   - Warp the original image to get a clean top-down board image.

This gives a stable, normalized 3×3 board image.

---

## 4. Cell Segmentation and Symbol Classification

The warped board is divided into 9 equal cells (3×3):

1. Cell extraction
   - Split the warped board into 3×3 sub-images.

2. Symbol detection in each cell
   - Check for O:
     - Run circle detection (for example, HoughCircles) or analyze round contours.
   - Check for X:
     - Look for two strong, crossing line segments at roughly ±45 degrees.
   - If neither strong circle nor clear crossing lines are detected, treat as empty.

3. Fallback rules
   - Optionally check ink coverage (fraction of dark pixels).
   - Filter noise with morphological operations.

Each cell is thus classified as X, O, or empty.

---

## 5. Board Stabilization and Move Validation

To avoid noise:

1. Stabilization
   - Maintain a short history of board states.
   - If the last N states are identical, consider the board stable.

2. Move validation
   - Compare the previous stable state and the new stable state:
     - Count how many cells changed.
     - Ensure:
       - Exactly one new symbol appears.
       - No existing X/O is erased or modified.
       - Robot’s own marks are untouched.
   - If validation fails:
     - Reject the move.
     - Robot writes “E” near the board or in a reserved area.

This enforces fair play and prevents cheating or accidental scribbles.

---

## 6. Minimax-Based Game Agent

The game agent is a standard Minimax solver with tweaks:

- State: 3×3 grid with {X, O, empty}.
- Actions: place current player’s mark in any empty cell.
- Terminal states:
  - Win for X.
  - Win for O.
  - Draw (no empty cells).

### Scoring

- Robot win: high positive score (for example, +10 - depth).
- Human win: high negative score (for example, -10 + depth).
- Draw: 0.

The depth term encourages:

- Faster wins (higher score).
- Delayed losses (less negative).

The agent always chooses the action with the best Minimax score for the robot symbol.

---

## 7. Robot Motion Planning

Given the chosen cell:

1. Convert cell index (0–8) to:
   - Local board coordinates.
   - Then to world coordinates using known board center and cell size.

2. Plotting primitives:
   - For X:
     - Draw two diagonal strokes inside the cell.
   - For O:
     - Approximate a circle using multiple short segments.

All drawing is done at draw height, moving at safe height in between strokes.

---

## 8. Outcome Annotation

At game end:

- If robot wins:
  - Robot draws a line across the winning three cells.
  - Writes “W” near the board.
- If human wins:
  - Draws the strike line.
  - Writes “L”.
- If draw:
  - Writes “D”.

If the game is aborted or invalid:

- Writes “E” to signal error or invalid play.

---

## 9. Possible Extensions

- Replace rule-based symbol detection with a small CNN.
- Add multi-game sessions on the same sheet.
- Use reinforcement learning-based policies instead of Minimax.
- Introduce speech or gesture input as an additional modality.
