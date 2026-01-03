#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tic‑Tac‑Toe — Dobot + Overhead Camera (Prompt + Hands‑free)

- Uses your `camera.py` for auto‑calibration and symbol detection:
  detect_grid_by_lines(frame) → warp_board(frame, quad) → classify_cell(cell).
- Draws a straight, axis‑aligned grid.
- PROMPT: "Who goes first? (r/h)".
- Hands‑free after that (no s/g/p): watch camera, wait for a STABLE board to accept moves.
- Strict validation:
  * Move valid only if exactly one human mark added and no robot symbol changed.
  * On invalid → draw 'E' at the side AND print explicit reasons.
- Exception: if human starts, the very first human move does not draw 'E'; we just wait until a single clear mark appears to infer the symbol.
- On draw → draw 'D' at the side (mandatory).
- On win → draw a strike‑through across the winning 3‑in‑a‑row.
"""

import os, time, signal, math
from typing import Optional, Tuple, List

import numpy as np
import cv2

# >>> Use YOUR camera.py for vision <<<
import camera as camlib  # detect_grid_by_lines, warp_board, classify_cell

# -------------------- ENV / defaults --------------------
DOBOT_PORT   = os.environ.get("DOBOT_PORT", "").strip()
CAM_INDEX    = int(os.environ.get("CAM_INDEX", "3"))
Z_DRAW       = float(os.environ.get("Z_DRAW", "-35"))
Z_SAFE       = float(os.environ.get("Z_SAFE", "25"))
GRID_W       = float(os.environ.get("GRID_W", "150"))
GRID_H       = float(os.environ.get("GRID_H", "150"))
FEED_DRAW    = float(os.environ.get("FEED_DRAW", "60"))
FEED_TRAVEL  = float(os.environ.get("FEED_TRAVEL", "120"))

BOARD_SIZE   = int(os.environ.get("BOARD_SIZE", "540"))
SHOW_DEBUG   = os.environ.get("SHOW_DEBUG", "1").strip() == "1"
STABILIZE_N  = int(os.environ.get("STABILIZE_N", "3"))
STRIKE_OVERSHOOT = float(os.environ.get("STRIKE_OVERSHOOT", "0.10"))  # as board fraction

# Strict validation is ALWAYS ON
STRICT_VALIDATE = 1

# -------------------- Dobot wrapper ---------------------
try:
    import serial.tools.list_ports
    from pydobot import Dobot as _Dobot
except Exception:
    serial = None
    _Dobot = None

class DobotLite:
    def __init__(self, port: str):
        self._dev = None
        if _Dobot is None:
            raise RuntimeError("pydobot not available. pip install pydobot")
        if not port:
            port = self._auto_port()
        print(f"[DOBOT] Connecting on {port} ...")
        self._dev = _Dobot(port=port, verbose=False)
        self._dev.speed(FEED_TRAVEL, FEED_TRAVEL)
        try:
            if hasattr(self._dev, "acceleration"):
                self._dev.acceleration(400, 400)
        except Exception:
            pass
        time.sleep(0.2)

    def _auto_port(self) -> str:
        if serial is None:
            return "/dev/ttyACM0"
        ports = list(serial.tools.list_ports.comports())
        # heuristic pick
        for p in ports:
            s = (p.device + " " + (p.description or "")).lower()
            if any(k in s for k in ["dobot", "wch", "ch340", "cp210", "usb serial"]):
                return p.device
        for p in ports:
            if "/dev/ttyACM" in p.device or "/dev/ttyUSB" in p.device:
                return p.device
        return ports[0].device if ports else "/dev/ttyACM0"

    def pose(self):
        return self._dev.pose()

    def speed(self, v: float, a: float):
        self._dev.speed(float(v), float(a))

    def move_to(self, x, y, z, feed=None, wait=False):
        if feed is not None:
            self._dev.speed(float(feed), float(feed))
        self._dev.move_to(float(x), float(y), float(z), 0.0, wait=wait)

    def close(self):
        try:
            self._dev.close()
        except Exception:
            pass

# -------------------- Board (axis-aligned) ---------------
class RectBoard:
    """Axis‑aligned rectangular board in robot XY so grid lines are perfectly straight."""
    def __init__(self, cx, cy, w_mm, h_mm, z_draw):
        self.cx, self.cy = float(cx), float(cy)
        self.w, self.h = float(w_mm), float(h_mm)
        self.zd = float(z_draw)

    def center(self):
        return self.cx, self.cy, self.zd

    def to_mm(self, u, v):
        # u: 0..1 left->right, v: 0..1 bottom->top
        x = self.cx + (u - 0.5) * self.w
        y = self.cy + (v - 0.5) * self.h
        return x, y, self.zd

    def ts_to_mm(self, t, s):
        # t: top->bottom, s: left->right (convert to u,v)
        return self.to_mm(s, 1.0 - t)

    def grid_fracs(self):
        return [0.0, 1/3, 2/3, 1.0], [0.0, 1/3, 2/3, 1.0]

    def cell_center(self, r, c):
        return self.ts_to_mm((r + 0.5) / 3.0, (c + 0.5) / 3.0)

    def win_line_endpoints(self, kind, overshoot=STRIKE_OVERSHOOT):
        """
        Convert a win descriptor {'type': 'row'|'col'|'diag_main'|'diag_anti','idx':int|None}
        into two (x,y,z) endpoints that extend outside the board by 'overshoot' fraction.
        """
        if kind['type'] == 'row':
            t = (kind['idx'] + 0.5) / 3.0
            return self.ts_to_mm(t, -overshoot), self.ts_to_mm(t, 1 + overshoot)
        if kind['type'] == 'col':
            s = (kind['idx'] + 0.5) / 3.0
            return self.ts_to_mm(-overshoot, s), self.ts_to_mm(1 + overshoot, s)
        if kind['type'] == 'diag_main':
            return self.ts_to_mm(-overshoot, -overshoot), self.ts_to_mm(1 + overshoot, 1 + overshoot)
        if kind['type'] == 'diag_anti':
            return self.ts_to_mm(-overshoot, 1 + overshoot), self.ts_to_mm(1 + overshoot, -overshoot)
        raise ValueError("Unknown win kind")

# -------------------- Pen plotter ------------------------
class PenPlotter:
    def __init__(self, robot: DobotLite, board: RectBoard):
        self.r = robot
        self.b = board

    def _travel(self, x, y, z=None):
        self.r.move_to(x, y, self.b.zd + Z_SAFE if z is None else z, feed=FEED_TRAVEL, wait=False)

    def _draw(self, x, y, z=None):
        self.r.move_to(x, y, self.b.zd if z is None else z, feed=FEED_DRAW, wait=False)

    def line(self, p0, p1):
        x0, y0, z0 = p0
        x1, y1, z1 = p1
        self._travel(x0, y0)
        self._draw(x0, y0, z0)   # pen down
        self._draw(x1, y1, z1)   # draw
        self._travel(x1, y1)     # lift

    def draw_grid(self):
        rows, cols = self.b.grid_fracs()
        # horizontal lines
        for t in (rows[1], rows[2]):
            p0 = self.b.ts_to_mm(t, 0.0); p1 = self.b.ts_to_mm(t, 1.0)
            self.line(p0, p1)
        # vertical lines
        for s in (cols[1], cols[2]):
            p0 = self.b.ts_to_mm(0.0, s); p1 = self.b.ts_to_mm(1.0, s)
            self.line(p0, p1)
        print("[GRID] Straight grid drawn.")

    def draw_X(self, r, c, frac=0.18):
        TL = self.b.ts_to_mm(r/3.0, c/3.0)
        TR = self.b.ts_to_mm(r/3.0, (c+1)/3.0)
        BR = self.b.ts_to_mm((r+1)/3.0, (c+1)/3.0)
        BL = self.b.ts_to_mm((r+1)/3.0, c/3.0)
        def inset(P, Q, Rpt):
            Px, Py, _ = P; Qx, Qy, _ = Q; Rx, Ry, _ = Rpt
            vx, vy = Qx - Px, Qy - Py
            wx, wy = Rx - Px, Ry - Py
            return (Px + frac*vx + frac*wx, Py + frac*vy + frac*wy, self.b.zd)
        TLp = inset(TL, TR, BL); TRp = inset(TR, TL, BR)
        BRp = inset(BR, TR, BL); BLp = inset(BL, TL, BR)
        self.line(TLp, BRp); self.line(TRp, BLp)

    def draw_O(self, r, c, frac=0.33, segments=96):
        cx, cy, cz = self.b.cell_center(r, c)
        rad = frac * min(self.b.w/3.0, self.b.h/3.0)
        pts = [(cx + rad*math.cos(2*math.pi*k/segments),
                cy + rad*math.sin(2*math.pi*k/segments),
                cz) for k in range(segments+1)]
        for a,b in zip(pts[:-1], pts[1:]):
            self.line(a,b)

    def draw_E_on_side(self, side='right', offset=20.0, height=25.0, width=15.0):
        cx, cy, cz = self.b.center()
        if side == 'right':
            midx, midy, _ = self.b.ts_to_mm(0.5, 1.0); nx, ny = 1.0, 0.0;  tx, ty = 0.0, 1.0
        elif side == 'left':
            midx, midy, _ = self.b.ts_to_mm(0.5, 0.0); nx, ny = -1.0, 0.0; tx, ty = 0.0, 1.0
        elif side == 'top':
            midx, midy, _ = self.b.ts_to_mm(0.0, 0.5); nx, ny = 0.0, 1.0;  tx, ty = 1.0, 0.0
        else:
            midx, midy, _ = self.b.ts_to_mm(1.0, 0.5); nx, ny = 0.0, -1.0; tx, ty = 1.0, 0.0
        startx = midx + nx*offset - tx*(height/2); starty = midy + ny*offset - ty*(height/2)
        endx   = startx + tx*height; endy = starty + ty*height
        top    = (startx, starty, cz)
        midpt  = (startx + tx*(height*0.5), starty + ty*(height*0.5), cz)
        bottom = (endx, endy, cz)
        self.line((top[0], top[1], cz), (top[0]+nx*width, top[1]+ny*width, cz))
        self.line((midpt[0], midpt[1], cz), (midpt[0]+nx*width*0.7, midpt[1]+ny*width*0.7, cz))
        self.line((bottom[0], bottom[1], cz), (bottom[0]+nx*width, bottom[1]+ny*width, cz))
        self.line(top, bottom)

    def draw_D_on_side(self, side='top', offset=20.0, height=25.0, width=18.0, segments=90):
        cx, cy, cz = self.b.center()
        if side == 'right':
            midx, midy, _ = self.b.ts_to_mm(0.5, 1.0); nx, ny = 1.0, 0.0;  tx, ty = 0.0, 1.0
        elif side == 'left':
            midx, midy, _ = self.b.ts_to_mm(0.5, 0.0); nx, ny = -1.0, 0.0; tx, ty = 0.0, 1.0
        elif side == 'top':
            midx, midy, _ = self.b.ts_to_mm(0.0, 0.5); nx, ny = 0.0, 1.0;  tx, ty = 1.0, 0.0
        else:
            midx, midy, _ = self.b.ts_to_mm(1.0, 0.5); nx, ny = 0.0, -1.0; tx, ty = 1.0, 0.0

        startx = midx + nx*offset - tx*(height/2); starty = midy + ny*offset - ty*(height/2)
        endx   = startx + tx*height; endy = starty + ty*height
        top    = (startx, starty, cz)
        bottom = (endx, endy, cz)
        self.line(top, bottom)

        cxm = midx + nx*offset
        cym = midy + ny*offset
        R = width/2.0
        pts = []
        for k in range(segments+1):
            theta = (-0.5 + k/segments) * math.pi  # -pi/2 .. +pi/2
            px = cxm + nx*(R*math.cos(theta)) + tx*( (height/2.0)*math.sin(theta) )
            py = cym + ny*(R*math.cos(theta)) + ty*( (height/2.0)*math.sin(theta) )
            pts.append( (px, py, cz) )
        for a,b in zip(pts[:-1], pts[1:]):
            self.line(a,b)

    def draw_win_strike(self, kind):
        a, b = self.b.win_line_endpoints(kind)
        self.line(a, b)

# -------------------- Vision (uses camera.py) ------------
def _open_camera(index: int):
    cap = cv2.VideoCapture(index, getattr(cv2, "CAP_V4L2", 0))
    if not cap or not cap.isOpened():
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera index {index}.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    except Exception:
        pass
    for _ in range(6):
        cap.read()
    return cap

class Vision:
    """
    Auto-calibration + classification powered by your camera.py:
      - detect_grid_by_lines()
      - warp_board()
      - classify_cell()
    """
    def __init__(self, index=0):
        self.cap = _open_camera(index)
        self.quad = None  # TL,TR,BR,BL in pixel coords
        self.board_size = BOARD_SIZE

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("Camera read failed")
        return frame

    def calibrate(self):
        best = None
        for _ in range(60):
            frame = self.read()
            quad, overlay = camlib.detect_grid_by_lines(frame, debug_draw=True)
            if quad is not None:
                best = quad
                if SHOW_DEBUG:
                    disp = frame.copy()
                    cv2.polylines(disp, [quad.reshape(-1,1,2).astype(int)], True, (0,255,0), 2)
                    cv2.imshow("Board (calib)", disp); cv2.waitKey(1)
                break
            time.sleep(0.05)
        if best is None:
            raise RuntimeError("Grid detection failed; ensure lines are visible.")
        self.quad = best

    def classify_board(self):
        frame = self.read()
        if self.quad is None:
            self.calibrate()
        warped = camlib.warp_board(frame, self.quad, out=self.board_size)
        h = w = self.board_size
        cell_h = h // 3; cell_w = w // 3
        board = [['' for _ in range(3)] for _ in range(3)]
        overlay = warped.copy()
        for r in range(3):
            for c in range(3):
                y0,y1 = r*cell_h,(r+1)*cell_h
                x0,x1 = c*cell_w,(c+1)*cell_w
                cell = warped[y0:y1, x0:x1]
                res = camlib.classify_cell(cell)  # returns CellResult('X'|'O'|'.', debug) or 'X'/'O'/'.'
                lab = res.label if hasattr(res, "label") else (res or "")
                lab = '' if lab == '.' else lab
                board[r][c] = lab
                if SHOW_DEBUG:
                    col = (0,255,0) if lab in ('','.') else ((0,0,255) if lab=='X' else (255,0,0))
                    cv2.rectangle(overlay, (x0,y0), (x1,y1), col, 2)
                    if lab in ('X','O'):
                        cv2.putText(overlay, lab, (x0 + cell_w//2 - 18, y0 + cell_h//2 + 18),
                                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,255), 3, cv2.LINE_AA)
        if SHOW_DEBUG:
            grid_vis = overlay.copy()
            camlib.draw_grid(grid_vis) if hasattr(camlib, 'draw_grid') else None
            cv2.imshow("Board (warped)", grid_vis); cv2.waitKey(1)
        return board

    def stabilized_board(self, n: int = STABILIZE_N, timeout: float = 5.0):
        """
        Return a board state seen identically 'n' times in a row (debounce human drawing).
        Timeout returns the last seen.
        """
        last = None; cnt = 0; t0 = time.time()
        while True:
            b = self.classify_board()
            if b == last: cnt += 1
            else: last, cnt = b, 1
            if cnt >= n: return b
            if (time.time() - t0) > timeout: return b
            time.sleep(0.06)

    def release(self):
        try:
            self.cap.release()
        except Exception:
            pass
        if SHOW_DEBUG:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

# -------------------- Game logic (minimax) ----------------
def winner_with_info(b):
    for r in range(3):
        if b[r][0] == b[r][1] == b[r][2] != "":
            return b[r][0], {'type':'row','idx':r}
    for c in range(3):
        if b[0][c] == b[1][c] == b[2][c] != "":
            return b[0][c], {'type':'col','idx':c}
    if b[0][0] == b[1][1] == b[2][2] != "":
        return b[0][0], {'type':'diag_main','idx':None}
    if b[0][2] == b[1][1] == b[2][0] != "":
        return b[0][2], {'type':'diag_anti','idx':None}
    return None, None

def is_draw(b):
    return all(b[r][c] in ('X','O') for r in range(3) for c in range(3))

def available_moves(b):
    return [(r,c) for r in range(3) for c in range(3) if b[r][c] == ""]

def _minimax(b, ai, human, ai_turn, depth):
    w,_ = winner_with_info(b)
    if w == ai: return 10 - depth, None
    if w == human: return depth - 10, None
    if is_draw(b): return 0, None
    if ai_turn:
        best=-1e9; mv=None
        for r,c in available_moves(b):
            b[r][c] = ai
            sc,_ = _minimax(b, ai, human, False, depth+1)
            b[r][c] = ""
            if sc > best: best, mv = sc, (r,c)
        return best, mv
    else:
        worst=1e9; mv=None
        for r,c in available_moves(b):
            b[r][c] = human
            sc,_ = _minimax(b, ai, human, True, depth+1)
            b[r][c] = ""
            if sc < worst: worst, mv = sc, (r,c)
        return worst, mv

def robot_move_minimax(board, ai_symbol, human_symbol):
    _, mv = _minimax(board, ai_symbol, human_symbol, True, 0)
    return mv

def diff(prev, curr):
    plus  = {'X':0, 'O':0}
    minus = {'X':0, 'O':0}
    for r in range(3):
        for c in range(3):
            a,b = prev[r][c], curr[r][c]
            if a == b: continue
            if a == '' and b in ('X', 'O'):
                plus[b] += 1
            if a in ('X','O') and b != a:
                minus[a] += 1
    return plus, minus

def explain_invalid_move(plus, minus, human_symbol, robot_symbol):
    """Return list of human-readable reasons why a human move is invalid (strict rules)."""
    reasons = []
    total_added = plus.get('X', 0) + plus.get('O', 0)
    # Wrong symbol drawn
    if plus.get(robot_symbol, 0) >= 1:
        reasons.append(f"wrong symbol drawn: '{robot_symbol}' (robot) placed instead of '{human_symbol}' (human).")
    # Multiple additions
    if total_added >= 2:
        reasons.append(f"multiple new marks detected ({total_added}).")
    # Human-specific validations
    if plus.get(human_symbol, 0) == 0:
        reasons.append(f"no new '{human_symbol}' mark detected.")
    elif plus.get(human_symbol, 0) > 1:
        reasons.append(f"more than one new '{human_symbol}' mark ({plus.get(human_symbol,0)}).")
    if minus.get(human_symbol, 0) > 0:
        reasons.append(f"human symbol changed/erased in {minus.get(human_symbol,0)} cell(s).")
    # Robot symbol change
    if minus.get(robot_symbol, 0) > 0:
        reasons.append(f"robot symbol changed/erased in {minus.get(robot_symbol,0)} cell(s).")
    if not reasons:
        reasons.append("ambiguous snapshot: draw clearly inside one empty cell.")
    return reasons

# -------------------- Main auto workflow -----------------
def main():
    rob = DobotLite(DOBOT_PORT)
    (x,y,z, *_j) = rob.pose()
    print(f"[POSE] Current pen pose x={x:.1f} y={y:.1f} z={z:.1f}")
    board = RectBoard(x, y, GRID_W, GRID_H, Z_DRAW)
    pen = PenPlotter(rob, board)
    cam = Vision(CAM_INDEX)

    def cleanup(*_):
        try: cam.release()
        except Exception: pass
        try: rob.close()
        except Exception: pass
        print("\\n[EXIT] Clean shutdown."); os._exit(0)
    signal.signal(signal.SIGINT, cleanup)

    # 1) Draw straight grid automatically
    print("[AUTO] Drawing grid at current XY (axis-aligned).")
    pen.draw_grid()

    # 2) Auto-calibrate camera to the grid (camera.py)
    print("[AUTO] Calibrating camera to grid (camera.py) ...")
    cam.calibrate()
    print("[AUTO] Calibration complete.")

    # 3) Who goes first? (prompt)
    while True:
        first = input("Who goes first? (r=robot, h=human): ").strip().lower()
        if first in ('r','h'): break
        print("Please enter 'r' or 'h'.")

    started_by = 'robot' if first == 'r' else 'human'

    human_symbol = None
    robot_symbol = None
    prev = [['' for _ in range(3)] for _ in range(3)]

    # If robot starts: play instantly (minimax)
    if started_by == 'robot':
        robot_symbol, human_symbol = 'X', 'O'
        mv = robot_move_minimax([row[:] for row in prev], robot_symbol, human_symbol) or (1,1)
        r,c = mv
        print(f"[OPEN] Robot ({robot_symbol}) → ({r},{c})")
        pen.draw_X(r, c)
        # confirm by quick stabilized scan
        seen = cam.stabilized_board()
        if 0 <= r < 3 and 0 <= c < 3 and seen[r][c] != robot_symbol:
            seen[r][c] = robot_symbol
        prev = seen
        first_human_pending = False
    else:
        # human starts; wait for first clear mark (no error on this first move)
        first_human_pending = True

    print("[RUN] Hands-free mode: watching for human move...")

    # 4) Automatic game loop
    while True:
        # Wait for a stable board
        curr = cam.stabilized_board()
        if curr == prev:
            time.sleep(0.06)
            continue

        plus, minus = diff(prev, curr)

        # ----- FIRST HUMAN MOVE (only when human starts): ONLY check multi-mark error -----
        if first_human_pending and started_by == 'human':
            plus_total = plus['X'] + plus['O']

            if plus_total >= 2:
                print(f"[ERROR] First human move invalid: multiple marks detected ({plus_total}).")
                pen.draw_E_on_side(side='right', offset=25.0, height=25.0, width=15.0)
                time.sleep(0.10)
                continue

            if plus['X'] == 1 and plus['O'] == 0:
                human_symbol, robot_symbol = 'X', 'O'
                print("[SETUP] Detected human symbol = X → Robot = O")
                prev = curr
                first_human_pending = False
            elif plus['O'] == 1 and plus['X'] == 0:
                human_symbol, robot_symbol = 'O', 'X'
                print("[SETUP] Detected human symbol = O → Robot = X")
                prev = curr
                first_human_pending = False
            else:
                # No new mark yet — keep waiting without error
                print("[WAIT] First human move: no new mark detected; waiting.")
                time.sleep(0.10)
                continue

        else:
            # ----- HUMAN MOVE VALIDATION (all later turns): error if multi-marks OR wrong symbol -----
            if human_symbol is None or robot_symbol is None:
                # Robot started case: symbols should already be known as X (robot) / O (human)
                human_symbol = human_symbol or 'O'
                robot_symbol = robot_symbol or 'X'

            plus_total = plus['X'] + plus['O']

            # Condition A: multiple new marks
            if plus_total >= 2:
                print(f"[ERROR] Human move invalid: multiple marks detected ({plus_total}).")
                pen.draw_E_on_side(side='right', offset=25.0, height=25.0, width=15.0)
                time.sleep(0.10)
                continue

            # Condition B: wrong symbol (human placed robot's symbol)
            if plus[robot_symbol] >= 1:
                print(f"[ERROR] Human move invalid: wrong symbol '{robot_symbol}' placed instead of '{human_symbol}'.")
                pen.draw_E_on_side(side='right', offset=25.0, height=25.0, width=15.0)
                time.sleep(0.10)
                continue

            # Accept exactly one correct human mark; otherwise keep waiting
            if plus[human_symbol] == 1:
                prev = curr
            else:
                print("[WAIT] No new valid human mark detected; ignoring this snapshot.")
                time.sleep(0.06)
                continue


        # Terminal checks (after accepting human move)
        w, info = winner_with_info(prev)
        if w == (human_symbol or ''):
            print("[RESULT] Human wins — strike-through.")
            pen.draw_win_strike(info)
            break
        if is_draw(prev):
            print("[RESULT] Draw detected — writing 'D' on the side.")
            pen.draw_D_on_side(side='top', offset=20.0, height=25.0, width=18.0)
            break

        # Robot move via minimax
        mv = robot_move_minimax([row[:] for row in prev], robot_symbol or 'X', human_symbol or 'O')
        if mv is None:
            print("[WARN] No legal move? Ending."); break
        r, c = mv
        print(f"[PLAY] Robot ({robot_symbol or 'X'}) → ({r},{c})")
        if (robot_symbol or 'X') == 'X':
            pen.draw_X(r, c)
        else:
            pen.draw_O(r, c)

        # Confirm our mark by stabilized scan
        seen = cam.stabilized_board()
        if 0 <= r < 3 and 0 <= c < 3 and seen[r][c] != (robot_symbol or 'X'):
            seen[r][c] = (robot_symbol or 'X')
        prev = seen

        # Terminal checks after robot move
        w, info = winner_with_info(prev)
        if w == (robot_symbol or 'X'):
            print(f"[RESULT] Robot ({robot_symbol or 'X'}) wins — strike-through.")
            pen.draw_win_strike(info)
            break
        if is_draw(prev):
            print("[RESULT] Draw detected — writing 'D' on the side.")
            pen.draw_D_on_side(side='top', offset=20.0, height=25.0, width=18.0)
            break

        # Small idle delay to reduce CPU
        time.sleep(0.06)

    cam.release()
    rob.close()
    print("[DONE] Bye.")

if __name__ == "__main__":
    main()
