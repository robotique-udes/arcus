'''
Authors: Ramtin B. Meidani, Charles McCabe
Modified: Multi-waypoint selection, path smoothing

This code aims to determine the most optimal path using A* algorithm for shortest path
and the brushfire algorithm for the safest path. The aim is to combine both to get the
shortest and safest path through a series of user-defined waypoints.

The following program computes:

INPUT:  .yaml and .pgm/.png file
OUTPUT: .csv of the coordinates in metres

Workflow
--------
1. Open an interactive window — left-click to place intermediate waypoints and
   the final destination, right-click to undo the last point, then press
   "Confirm waypoints" to proceed.
2. A* is run on each consecutive pair (start->wp1, wp1->wp2, ..., wpN->end) and
   the segments are concatenated into a full raceline.
3. The raw raceline is smoothed with a parametric cubic B-spline.
4. The smoothed path is saved as (x, y) world coordinates in metres.

Dependencies
------------
pip install pillow pyyaml numpy matplotlib opencv-python scipy
'''

import yaml
import os
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import heapq
import math
import csv
import cv2
import re
from scipy.interpolate import splprep, splev

UNKNOWN  = -1
FREE     = 0
OCCUPIED = 100


# ============================================================================
# File I/O helpers
# ============================================================================

def pgm_opener(path):
    '''Opens an image file and normalises pixel values to [0, 1].'''
    img = Image.open(path).convert("L")
    return np.array(img) / 255.0


def yaml_opener(path):
    '''Loads a YAML file and returns its contents as a dict.'''
    with open(path, "r") as f:
        return yaml.safe_load(f)


def saveCSV(data, filename):
    '''
    Saves a list of tuples to a CSV file.

    Parameters
    ----------
    data     : list of tuples  e.g. [(x0,y0), (x1,y1), ...]
    filename : str             path WITHOUT the .csv extension
    '''
    filepath = filename + ".csv"
    with open(filepath, "w", newline='') as f:
        writer = csv.writer(f, delimiter=",")
        writer.writerows(data)


# ============================================================================
# Grid generation
# ============================================================================

def grid_generator(loaded_yaml, img):
    '''
    Builds a discrete occupancy grid from a grayscale map image and its
    associated YAML metadata.

    Returns
    -------
    np.ndarray of int8 with values UNKNOWN(-1), FREE(0), OCCUPIED(100)
    '''
    mode     = loaded_yaml["mode"]
    negate   = loaded_yaml["negate"]
    occ_thr  = loaded_yaml["occupied_thresh"]
    free_thr = loaded_yaml["free_thresh"]

    p = img if negate else 1.0 - img

    height, width  = img.shape
    occupancy_grid = np.full((height, width), UNKNOWN, dtype=np.int8)

    if mode == "trinary":
        occupancy_grid[p >= occ_thr]  = OCCUPIED
        occupancy_grid[p <= free_thr] = FREE
    elif mode == "scale":
        occupancy_grid = np.clip((1.0 - img) * 100, 0, 100).astype(np.int8)
    elif mode == "raw":
        occupancy_grid = (img * 255).astype(np.int16)
    else:
        raise ValueError(f"Unknown map mode: {mode}")

    return occupancy_grid


# ============================================================================
# Coordinate conversion
# ============================================================================

def world_to_grid(world_pos, origin, resolution, height):
    '''
    Converts real-world (x, y) metres into (row, col) grid indices.

    The Y axis is flipped because image rows grow downward while the ROS
    world frame grows upward.
    '''
    x, y   = world_pos[:2]
    ox, oy = origin[:2]

    col = int((x - ox) / resolution)
    row = int((y - oy) / resolution)
    row = height - 1 - row          # flip Y

    return (row, col)


def grid_to_world(grid_pos, origin, resolution, height):
    '''
    Converts (row, col) grid indices back to real-world (x, y) in metres.
    Exact inverse of world_to_grid.
    '''
    row, col = grid_pos
    ox, oy   = origin[:2]

    x = col * resolution + ox
    y = (height - 1 - row) * resolution + oy

    return (x, y)


# ============================================================================
# Visualisation helpers
# ============================================================================

def _build_vis(occupancy_grid):
    '''
    Converts the occupancy grid to an RGB uint8 image for display.
    Paths and markers are drawn separately by the callers via matplotlib.
    '''
    vis = np.zeros((*occupancy_grid.shape, 3), dtype=np.uint8)
    vis[occupancy_grid == UNKNOWN] = [60,  60,  80 ]
    vis[occupancy_grid == FREE]    = [230, 230, 245]
    vis[occupancy_grid == OCCUPIED]= [20,  20,  30 ]
    return vis


def show_grid(occupancy_grid, raw_raceline=None, smooth_raceline=None, waypoints=None):
    '''
    Renders the occupancy grid with optional overlays:
      - raw A* path (muted green)
      - smoothed spline path (cyan)
      - numbered intermediate waypoints (orange squares)
      - start (green circle) and end (red circle) markers

    Parameters
    ----------
    occupancy_grid  : np.ndarray
    raw_raceline    : list of (row, col)  A* output before smoothing
    smooth_raceline : list of (row, col)  spline-smoothed path
    waypoints       : list of (row, col)  user-chosen intermediate checkpoints
    '''
    vis = _build_vis(occupancy_grid)

    # Draw raw path in muted green using OpenCV (fast for large arrays)
    if raw_raceline and len(raw_raceline) >= 2:
        pts = np.array([(c, r) for r, c in raw_raceline], dtype=np.int32)
        cv2.polylines(vis, [pts.reshape(-1, 1, 2)], isClosed=False,
                      color=(60, 140, 80), thickness=2, lineType=cv2.LINE_AA)

    # Draw smoothed path in bright cyan
    if smooth_raceline and len(smooth_raceline) >= 2:
        pts = np.array([(int(round(c)), int(round(r)))
                        for r, c in smooth_raceline], dtype=np.int32)
        cv2.polylines(vis, [pts.reshape(-1, 1, 2)], isClosed=False,
                      color=(0, 210, 220), thickness=3, lineType=cv2.LINE_AA)

    fig, ax = plt.subplots(figsize=(14, 11))
    fig.patch.set_facecolor('#1a1a2e')
    ax.set_facecolor('#1a1a2e')
    ax.imshow(vis, origin="lower")
    ax.set_title("Computed Path  (muted green = raw A*  |  cyan = smoothed)",
                 color='white', fontsize=12, pad=14)
    ax.tick_params(colors='#888888')
    for spine in ax.spines.values():
        spine.set_edgecolor('#333355')

    # Start / Finish marker (same point for a loop)
    if raw_raceline:
        sr, sc = raw_raceline[0]
        ax.plot(sc, sr, 'o', color='#00e676', markersize=13,
                markeredgewidth=2, markeredgecolor='white', zorder=6,
                label='Start / Finish')
        ax.annotate('S/F', xy=(sc, sr), xytext=(sc + 8, sr + 8),
                    color='#00e676', fontsize=8, fontweight='bold', zorder=7,
                    arrowprops=dict(arrowstyle='->', color='#00e676', lw=1.2))

    # All intermediate checkpoints (orange squares)
    if waypoints:
        for i, (wr, wc) in enumerate(waypoints):
            ax.plot(wc, wr, 's', color='#ffab00', markersize=11,
                    markeredgewidth=1.5, markeredgecolor='white', zorder=6)
            ax.text(wc + 5, wr + 5, str(i + 1),
                    color='#ffab00', fontsize=8, fontweight='bold', zorder=7)

    # Legend
    handles = []
    if raw_raceline:
        handles.append(plt.Line2D([0],[0], color='#3c8c50', lw=2, label='Raw A* path'))
    if smooth_raceline:
        handles.append(plt.Line2D([0],[0], color='#00d2dc', lw=2, label='Smoothed path'))
    if handles:
        ax.legend(handles=handles, loc='upper right', framealpha=0.5,
                  facecolor='#1a1a2e', edgecolor='#444466',
                  labelcolor='white', fontsize=9)

    ax.grid(False)
    plt.tight_layout()
    plt.show()


# ============================================================================
# Interactive multi-waypoint picker
# ============================================================================

def pick_waypoints(occupancy_grid, start_pos):
    '''
    Opens an interactive map window where the user places intermediate
    checkpoints around the track and optionally moves the start/finish point.
    The path always closes back to the start/finish automatically.

    Controls
    --------
    Left-click (normal mode)       : place a numbered checkpoint
    Right-click (normal mode)      : undo the last checkpoint
    "Move Start/Finish" button     : enter move mode (button turns red)
    Left-click (move mode)         : relocate the S/F marker, exit move mode
    Right-click (move mode)        : cancel move mode without changing S/F
    "Confirm" button               : accept and close (needs >= 1 checkpoint)

    Parameters
    ----------
    occupancy_grid : np.ndarray
    start_pos      : (row, col)  default start/finish from the YAML origin

    Returns
    -------
    tuple( list of (row, col), (row, col) ) | None
        (checkpoints, final_start_pos), or None if cancelled.
        checkpoints is ordered [cp1, cp2, ..., cpN].
        final_start_pos may differ from the input start_pos if the user moved it.
    '''
    vis = _build_vis(occupancy_grid)

    fig, ax = plt.subplots(figsize=(14, 11))
    fig.patch.set_facecolor('#1a1a2e')
    ax.set_facecolor('#1a1a2e')
    plt.subplots_adjust(bottom=0.13)

    ax.imshow(vis, origin="lower")
    ax.tick_params(colors='#888888')
    for spine in ax.spines.values():
        spine.set_edgecolor('#333355')

    # ── Shared mutable state ─────────────────────────────────────────────────
    state = {
        'start_pos'  : start_pos,   # (row, col) — updated if user moves S/F
        'waypoints'  : [],          # list of (row, col) checkpoints
        'markers'    : [],          # checkpoint matplotlib artists
        'labels'     : [],          # checkpoint annotation artists
        'lines'      : [],          # connector line artists
        'sf_marker'  : None,        # S/F circle artist
        'sf_label'   : None,        # S/F annotation artist
        'move_mode'  : False,       # True while waiting for S/F relocation click
        'confirmed'  : False,
    }

    # ── Title ────────────────────────────────────────────────────────────────
    ax.set_title(
        'LEFT-CLICK to add checkpoints  |  RIGHT-CLICK to undo last  |  '
        '"Move S/F" to relocate start/finish  |  "Confirm" when done\n'
        'Green = start/finish  |  Yellow squares = intermediate checkpoints',
        color='white', fontsize=11, pad=12)

    # ── Status bar ───────────────────────────────────────────────────────────
    status_text = ax.text(
        0.5, -0.055,
        'Left-click to place your first checkpoint',
        transform=ax.transAxes, ha='center', va='top',
        color='#aaaacc', fontsize=10)

    # ── Buttons ──────────────────────────────────────────────────────────────
    # "Move Start/Finish" — left of centre
    ax_move = plt.axes([0.18, 0.02, 0.22, 0.055])
    btn_move = Button(ax_move, 'Move Start / Finish',
                      color='#22224a', hovercolor='#1a3a1a')
    btn_move.label.set_color('white')
    btn_move.label.set_fontsize(10)

    # "Confirm" — right of centre
    ax_conf = plt.axes([0.60, 0.02, 0.20, 0.055])
    btn_conf = Button(ax_conf, 'Confirm waypoints',
                      color='#22224a', hovercolor='#3333aa')
    btn_conf.label.set_color('white')
    btn_conf.label.set_fontsize(10)

    # ── S/F marker helpers ───────────────────────────────────────────────────
    def _draw_sf(row, col):
        '''Remove old S/F artists and draw fresh ones at (row, col).'''
        if state['sf_marker'] is not None:
            state['sf_marker'].remove()
        if state['sf_label'] is not None:
            state['sf_label'].remove()

        mk, = ax.plot(col, row, 'o',
                      color='#00e676', markersize=14,
                      markeredgewidth=2, markeredgecolor='white', zorder=7)
        lbl = ax.annotate(
            'S/F',
            xy=(col, row), xytext=(col + 10, row + 10),
            color='#00e676', fontsize=9, fontweight='bold', zorder=8,
            arrowprops=dict(arrowstyle='->', color='#00e676', lw=1.5))

        state['sf_marker'] = mk
        state['sf_label']  = lbl

    # Draw the initial S/F marker
    _draw_sf(start_pos[0], start_pos[1])

    # ── Connector lines ──────────────────────────────────────────────────────
    def _redraw_connectors():
        '''Dashed lines: S/F -> cp1 -> ... -> cpN -> S/F (closing the loop).'''
        for ln in state['lines']:
            ln.remove()
        state['lines'].clear()

        sp = state['start_pos']
        all_pts = [sp] + state['waypoints'] + [sp]
        for i in range(len(all_pts) - 1):
            r0, c0 = all_pts[i]
            r1, c1 = all_pts[i + 1]
            is_closing = (i == len(all_pts) - 2)
            style = '-.' if is_closing else '--'
            color = '#00e67644' if is_closing else '#ffffff44'
            ln, = ax.plot([c0, c1], [r0, r1],
                          style, color=color, lw=1.2, zorder=3)
            state['lines'].append(ln)

    # ── Status update ────────────────────────────────────────────────────────
    def _update_status():
        if state['move_mode']:
            status_text.set_text(
                'MOVE MODE — left-click to set the new Start/Finish position  |  '
                'right-click to cancel')
            status_text.set_color('#00e676')
            fig.canvas.draw_idle()
            return

        n = len(state['waypoints'])
        if n == 0:
            msg, color = 'Left-click to place your first checkpoint', '#aaaacc'
        elif n == 1:
            msg   = '1 checkpoint placed  |  right-click to undo  |  confirm or add more'
            color = '#69f0ae'
        else:
            msg   = f'{n} checkpoints placed  |  right-click to undo  |  confirm when ready'
            color = '#69f0ae'
        status_text.set_text(msg)
        status_text.set_color(color)
        fig.canvas.draw_idle()

    # ── Click handler ────────────────────────────────────────────────────────
    def _validate_cell(row, col):
        '''Returns True and sets status if the cell is not usable.'''
        h_g, w_g = occupancy_grid.shape
        if not (0 <= row < h_g and 0 <= col < w_g):
            return False
        if occupancy_grid[row, col] == OCCUPIED:
            status_text.set_text('That point is inside an obstacle — choose a free (light) area')
            status_text.set_color('#ff6d00')
            fig.canvas.draw_idle()
            return False
        if occupancy_grid[row, col] == UNKNOWN:
            status_text.set_text('That point is in unknown space — choose a free (light) area')
            status_text.set_color('#ff6d00')
            fig.canvas.draw_idle()
            return False
        return True

    def on_click(event):
        if event.inaxes is not ax:
            return

        col = int(round(event.xdata))
        row = int(round(event.ydata))

        # ── Move mode ────────────────────────────────────────────────────────
        if state['move_mode']:
            if event.button == 3:
                # Right-click cancels move mode without changing anything
                state['move_mode'] = False
                btn_move.color = '#22224a'
                btn_move.hovercolor = '#1a3a1a'
                btn_move.ax.set_facecolor('#22224a')
                _update_status()
                fig.canvas.draw_idle()
                return

            if event.button == 1:
                if not _validate_cell(row, col):
                    return
                state['start_pos'] = (row, col)
                state['move_mode'] = False
                btn_move.color = '#22224a'
                btn_move.hovercolor = '#1a3a1a'
                btn_move.ax.set_facecolor('#22224a')
                _draw_sf(row, col)
                _redraw_connectors()
                _update_status()
                fig.canvas.draw_idle()
            return

        # ── Normal mode ──────────────────────────────────────────────────────
        if event.button == 3:
            # Undo last checkpoint
            if not state['waypoints']:
                return
            state['waypoints'].pop()
            state['markers'].pop().remove()
            state['labels'].pop().remove()
            _redraw_connectors()
            _update_status()
            fig.canvas.draw_idle()
            return

        if event.button != 1:
            return

        if not _validate_cell(row, col):
            return

        state['waypoints'].append((row, col))
        n = len(state['waypoints'])

        mk, = ax.plot(col, row, 's',
                      color='#ffab00', markersize=12,
                      markeredgewidth=2, markeredgecolor='white', zorder=6)
        state['markers'].append(mk)

        lbl = ax.annotate(
            str(n),
            xy=(col, row), xytext=(col + 7, row + 7),
            color='#ffab00', fontsize=8, fontweight='bold', zorder=7,
            arrowprops=dict(arrowstyle='->', color='#ffab00', lw=1.2))
        state['labels'].append(lbl)

        _redraw_connectors()
        _update_status()
        fig.canvas.draw_idle()

    # ── Button handlers ──────────────────────────────────────────────────────
    def on_move_sf(_event):
        '''Toggle move mode on/off.'''
        state['move_mode'] = not state['move_mode']
        if state['move_mode']:
            btn_move.ax.set_facecolor('#1a3a1a')   # green tint = active
        else:
            btn_move.ax.set_facecolor('#22224a')
        _update_status()
        fig.canvas.draw_idle()

    def on_confirm(_event):
        if not state['waypoints']:
            status_text.set_text('Place at least one checkpoint before confirming')
            status_text.set_color('#ff6d00')
            fig.canvas.draw_idle()
            return
        state['confirmed'] = True
        plt.close(fig)

    fig.canvas.mpl_connect('button_press_event', on_click)
    btn_move.on_clicked(on_move_sf)
    btn_conf.on_clicked(on_confirm)

    plt.show()   # blocks until window is closed

    if state['confirmed'] and state['waypoints']:
        return state['waypoints'], state['start_pos']
    return None


# ============================================================================
# Brushfire
# ============================================================================

def brushfire_algo(occupancyGrid, use8CellWindow=True):
    '''
    Wave-expansion from every obstacle cell outward. Labels each free cell
    with its distance (in pixels) to the nearest obstacle.
    '''
    nRows, nCols = occupancyGrid.shape
    a = 0

    while np.any(occupancyGrid == 0):
        a += 1
        for iRow in range(nRows):
            for iCol in range(nCols):
                if occupancyGrid[iRow, iCol] == a:
                    for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                        nr, nc = iRow+dr, iCol+dc
                        if 0 <= nr < nRows and 0 <= nc < nCols:
                            if occupancyGrid[nr, nc] == 0:
                                occupancyGrid[nr, nc] = a + 1
                    if use8CellWindow:
                        for dr, dc in [(-1,-1),(-1,1),(1,-1),(1,1)]:
                            nr, nc = iRow+dr, iCol+dc
                            if 0 <= nr < nRows and 0 <= nc < nCols:
                                if occupancyGrid[nr, nc] == 0:
                                    occupancyGrid[nr, nc] = a + 1
    return occupancyGrid


# ============================================================================
# A* with brushfire weights  (single segment)
# ============================================================================

def A_star_algo(occupancy_grid, brushfire_weights, safety_weight, start_pos, end_pos):
    '''
    Finds the shortest, safest path from start_pos to end_pos using A* and
    brushfire distance weights as a safety cost term.

    Returns
    -------
    list of (row, col) or None if no path exists
    '''
    ROW = len(occupancy_grid)
    COL = len(occupancy_grid[0])

    for pos, name in [(start_pos, "Start"), (end_pos, "End")]:
        if not (0 <= pos[0] < ROW and 0 <= pos[1] < COL):
            print(f"{name} position {pos} is outside the map.")
            return None
        if occupancy_grid[pos[0]][pos[1]] == OCCUPIED:
            print(f"{name} position {pos} is inside an obstacle.")
            return None

    closed = [[False]*COL for _ in range(ROW)]
    f      = [[float('inf')]*COL for _ in range(ROW)]
    g      = [[float('inf')]*COL for _ in range(ROW)]
    parent = [[None]*COL for _ in range(ROW)]

    def h(r, c):
        return math.sqrt((r - end_pos[0])**2 + (c - end_pos[1])**2)

    sr, sc         = start_pos
    g[sr][sc]      = 0.0
    f[sr][sc]      = h(sr, sc)
    parent[sr][sc] = (sr, sc)

    open_list = []
    heapq.heappush(open_list, (f[sr][sc], sr, sc))

    directions = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    while open_list:
        _, r, c = heapq.heappop(open_list)
        closed[r][c] = True

        for dr, dc in directions:
            nr, nc = r+dr, c+dc
            if not (0 <= nr < ROW and 0 <= nc < COL): continue
            if occupancy_grid[nr][nc] == OCCUPIED:     continue
            if closed[nr][nc]:                         continue

            if (nr, nc) == tuple(end_pos):
                parent[nr][nc] = (r, c)
                path, cur = [], (nr, nc)
                while True:
                    path.append(cur)
                    if parent[cur[0]][cur[1]] == cur:
                        break
                    cur = parent[cur[0]][cur[1]]
                path.reverse()
                return path

            d           = brushfire_weights[nr][nc]
            d_safe      = np.clip(d, 1e-6, None)
            safety_cost = safety_weight / (d_safe**2)
            move_cost   = math.sqrt(2) if (dr != 0 and dc != 0) else 1.0
            g_new       = g[r][c] + move_cost + safety_cost
            f_new       = g_new + h(nr, nc)

            if f[nr][nc] > f_new:
                g[nr][nc]      = g_new
                f[nr][nc]      = f_new
                parent[nr][nc] = (r, c)
                heapq.heappush(open_list, (f_new, nr, nc))

    return None


# ============================================================================
# Multi-segment path planning
# ============================================================================

def plan_full_path(occupancy_grid, brushfire_grid, safety_weight,
                   start_pos, waypoints):
    '''
    Runs A* on each consecutive pair of positions and concatenates the
    resulting segments into one continuous closed-loop path.

    Segment chain:  start_pos -> cp[0] -> cp[1] -> ... -> cp[-1] -> start_pos

    start_pos is always appended as the final target so the raceline forms
    a complete loop regardless of what the caller passes in.

    The junction point shared between consecutive segments appears only
    once in the output (the duplicate head is removed on each segment).

    Parameters
    ----------
    occupancy_grid  : np.ndarray
    brushfire_grid  : np.ndarray
    safety_weight   : float
    start_pos       : (row, col)
    waypoints       : list of (row, col)   intermediate checkpoints [cp1, ..., cpN]

    Returns
    -------
    list of (row, col) or None if any segment has no solution
    '''
    # Always close the loop: last target is the start position
    all_positions = [start_pos] + list(waypoints) + [start_pos]
    full_path     = []

    n_segs = len(all_positions) - 1
    for i in range(n_segs):
        seg_start = all_positions[i]
        seg_end   = all_positions[i + 1]

        label = "closing segment (back to start)" if i == n_segs - 1 else f"segment {i+1}/{n_segs}"
        print(f"  {label}:  {seg_start} -> {seg_end}")

        segment = A_star_algo(occupancy_grid, brushfire_grid,
                              safety_weight, seg_start, seg_end)

        if segment is None:
            print(f"  No path found for {label}. "
                  f"Try repositioning checkpoint {i+1}.")
            return None

        print(f"  OK  ({len(segment)} cells)")

        if full_path:
            segment = segment[1:]   # drop duplicate junction point

        full_path.extend(segment)

    return full_path


# ============================================================================
# Path smoothing
# ============================================================================

def smooth_path(path, smoothing_factor=None, num_points=None, closed=False):
    '''
    Smooths a raw (row, col) path using a parametric cubic B-spline
    (scipy splprep / splev).

    The spline approximates (rather than interpolates) the input points,
    naturally rounding corners and removing staircase artefacts from the
    grid-based A* path.

    Parameters
    ----------
    path             : list of (row, col)
    smoothing_factor : float or None
        Controls spline tightness:
          0            -> interpolating (passes through every point exactly)
          larger value -> smoother, further from original points
          None         -> scipy auto-selects based on len(path)
        A good starting value for racing circuits: len(path) * 0.5
    num_points       : int or None
        Number of output waypoints. Defaults to len(path).
    closed           : bool
        If True the spline is periodic (smooth closed loop).
        Set this to True when planning a full lap raceline.

    Returns
    -------
    list of (float, float)  as (row, col) floats
    '''
    if len(path) < 4:
        print("[smooth_path] Path too short to smooth (need >= 4 points). "
              "Returning raw path.")
        return path

    rows = np.array([p[0] for p in path], dtype=float)
    cols = np.array([p[1] for p in path], dtype=float)

    if num_points is None:
        num_points = len(path)

    if smoothing_factor is None:
        # scipy default: len(path) which is often a good middle ground
        smoothing_factor = float(len(path))

    try:
        tck, _ = splprep(
            [rows, cols],
            s   = smoothing_factor,
            per = closed,    # periodic = closed loop
            k   = 3)         # cubic spline

        u_new          = np.linspace(0, 1, num_points)
        rows_s, cols_s = splev(u_new, tck)

        return list(zip(rows_s.tolist(), cols_s.tolist()))

    except Exception as e:
        print(f"[smooth_path] Spline fitting failed ({e}). Returning raw path.")
        return path


def find_latest_map_pair(map_folder):
    '''
    Finds the newest timestamped YAML/PGM pair in map_folder.

    Expected filename style includes a timestamp token like YYYYMMDD_HHMMSS,
    e.g. slam_map_20260314_130422.yaml and matching .pgm.

    Returns
    -------
    tuple(str, str)
        (yaml_filename, pgm_filename)
    '''
    ts_pattern = re.compile(r"(\d{8}_\d{6})")
    candidates = []

    for entry in os.listdir(map_folder):
        if not entry.lower().endswith('.yaml'):
            continue

        stem = os.path.splitext(entry)[0]
        m = ts_pattern.search(stem)
        if not m:
            continue

        pgm_name = stem + '.pgm'
        pgm_path = os.path.join(map_folder, pgm_name)
        if not os.path.isfile(pgm_path):
            continue

        candidates.append((m.group(1), entry, pgm_name))

    if not candidates:
        raise FileNotFoundError(
            f"No timestamped YAML/PGM map pairs found in '{map_folder}'.")

    _, yaml_name, pgm_name = max(candidates, key=lambda x: x[0])
    return yaml_name, pgm_name



if __name__ == "__main__":

    # ── Map files ─────────────────────────────────────────────────────────────
    MAP_FOLDER  = "Maps/"
    if not os.path.isdir(MAP_FOLDER) and os.path.isdir("maps/"):
        MAP_FOLDER = "maps/"

    # Set this to a map basename (without extension) to force a specific map.
    FORCED_MAP_BASENAME = ""

    if FORCED_MAP_BASENAME.strip():
        map_base = FORCED_MAP_BASENAME.strip()
        YAML_FILE = map_base + ".yaml"
        PGM_FILE  = map_base + ".pgm"

        full_yaml_path = os.path.join(MAP_FOLDER, YAML_FILE)
        full_pgm_path  = os.path.join(MAP_FOLDER, PGM_FILE)

        if not os.path.isfile(full_yaml_path):
            raise FileNotFoundError(f"Forced YAML file not found: {full_yaml_path}")
        if not os.path.isfile(full_pgm_path):
            raise FileNotFoundError(f"Forced PGM file not found: {full_pgm_path}")

        print(f"Using forced map pair from '{MAP_FOLDER}':")
        print(f"  YAML: {YAML_FILE}")
        print(f"  PGM : {PGM_FILE}")
    else:
        YAML_FILE, PGM_FILE = find_latest_map_pair(MAP_FOLDER)
        print(f"Using latest map pair from '{MAP_FOLDER}':")
        print(f"  YAML: {YAML_FILE}")
        print(f"  PGM : {PGM_FILE}")

    # ── Output ────────────────────────────────────────────────────────────────
    # Folder and filename for the saved waypoint CSV (no extension needed)
    CSV_FOLDER  = "saved/"
    CSV_NAME    = "raceline"

    # ── Default start / finish position (world coordinates in metres) ─────────
    # This is where the S/F marker is placed when the window first opens.
    START_WORLD = [0.0, 0.0]

    # ── A* safety weight ──────────────────────────────────────────────────────
    # Controls how strongly the path avoids walls.
    # Higher = more cautious
    SAFETY_WEIGHT = 20

    # ── Spline smoothing factor ───────────────────────────────────────────────
    # Controls how closely the smoothed path follows the raw A* output.
    # High  = very smooth but may cut corners aggressively
    SMOOTHING_FACTOR = 2000.0

    # ── Load map ──────────────────────────────────────────────────────────────
    full_yaml_path = os.path.join(MAP_FOLDER, YAML_FILE)
    full_pgm_path  = os.path.join(MAP_FOLDER, PGM_FILE)

    loaded_yaml    = yaml_opener(full_yaml_path)
    loaded_img     = pgm_opener(full_pgm_path)
    occupancy_grid = grid_generator(loaded_yaml, loaded_img)

    height, width  = occupancy_grid.shape
    origin         = loaded_yaml["origin"]
    res            = loaded_yaml["resolution"]

    # ── Default start position ────────────────────────────────────────────────
    start_pos = world_to_grid(START_WORLD, origin, res, height)
    print(f"Default start position (grid): row={start_pos[0]}, col={start_pos[1]}")

    # ── Interactive waypoint selection ────────────────────────────────────────
    print("\nOpening map window.")
    print("  Left-click       : place intermediate checkpoints around the track")
    print("  Right-click      : undo the last checkpoint")
    print("  Move Start/Finish: relocate the S/F marker")
    print("  Confirm          : accept and proceed\n")

    result = pick_waypoints(occupancy_grid, start_pos)

    if not result:
        print("No checkpoints selected. Exiting.")
        exit(0)

    waypoints, start_pos = result

    sx, sy = grid_to_world(start_pos, origin, res, height)
    print(f"\nStart/Finish (grid): row={start_pos[0]}, col={start_pos[1]}  "
          f"world=({sx:.2f} m, {sy:.2f} m)")
    print(f"{len(waypoints)} checkpoint(s) confirmed (loop closes back to start):")
    for i, wp in enumerate(waypoints):
        wx, wy = grid_to_world(wp, origin, res, height)
        print(f"  [checkpoint {i+1}]  grid=({wp[0]}, {wp[1]})  "
              f"world=({wx:.2f} m, {wy:.2f} m)")

    # ── Brushfire ─────────────────────────────────────────────────────────────
    binary_grid                      = np.zeros_like(occupancy_grid, dtype=np.uint8)
    binary_grid[occupancy_grid == OCCUPIED] = 1

    brushfire_grid                   = np.zeros_like(binary_grid, dtype=np.int32)
    brushfire_grid[binary_grid == 1] = 1

    print("\nRunning brushfire... (may take a moment on large maps)")
    brushfire_grid = brushfire_algo(brushfire_grid)

    # ── A* ────────────────────────────────────────────────────────────────────
    print("\nRunning A* across all segments...")
    raw_raceline = plan_full_path(occupancy_grid, brushfire_grid,
                                  SAFETY_WEIGHT, start_pos, waypoints)

    if raw_raceline is None:
        print("\nNo complete path found. Try repositioning a checkpoint near a narrow gap.")
        exit(1)

    print(f"\nRaw raceline: {len(raw_raceline)} waypoints")

    # ── Smoothing ─────────────────────────────────────────────────────────────
    smooth_raceline = smooth_path(
        raw_raceline,
        smoothing_factor = SMOOTHING_FACTOR,
        num_points       = len(raw_raceline),
        closed           = True,             # always True: tracks are loops
    )
    print(f"Smoothed raceline: {len(smooth_raceline)} waypoints")

    # ── Save CSV ──────────────────────────────────────────────────────────────
    path_csv = os.path.join(CSV_FOLDER, CSV_NAME)
    os.makedirs(CSV_FOLDER, exist_ok=True)

    smooth_world = [grid_to_world(pt, origin, res, height) for pt in smooth_raceline]
    saveCSV(smooth_world, path_csv)

    print(f"\nSmoothed path saved -> {path_csv}.csv  "
          f"({len(smooth_world)} waypoints, metres)")
    print(f"  Start : x={smooth_world[0][0]:.3f} m,  y={smooth_world[0][1]:.3f} m")
    print(f"  End   : x={smooth_world[-1][0]:.3f} m,  y={smooth_world[-1][1]:.3f} m")

    # ── Display ───────────────────────────────────────────────────────────────
    show_grid(occupancy_grid,
              raw_raceline    = raw_raceline,
              smooth_raceline = smooth_raceline,
              waypoints       = waypoints)