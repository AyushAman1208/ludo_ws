"""Ludo board geometry: which world position belongs to which step of a piece's journey.

Pure Python, no ROS. Must match ludo_sim/models/board.xml (15x15 grid, 0.05 m cells,
grid (7, 7) is the board centre, x = (i - 7) * 0.05, y = (j - 7) * 0.05).

Progress of a piece
    0        in the yard
    1        on its own start cell
    2..51    further along the 52-cell outer loop (counter-clockwise)
    52..56   the five cells of its own home lane
    57       finished (placed in its triangle at the centre)
"""
CELL = 0.05
COLORS = ("red", "green", "yellow", "blue")
TURN_ORDER = ("red", "blue", "yellow", "green")      # counter-clockwise around the board
FINISH = 57
LOOP_LEN = 52


def grid_xy(i, j):
    return ((i - 7) * CELL, (j - 7) * CELL)


def _rot(c):                      # rotate a grid cell 90 deg counter-clockwise about the centre
    i, j = c
    return (14 - j, i)


def _build_loop():
    cells = []
    cells += [(i, 6) for i in range(1, 6)]            # left arm, lower row, towards the centre
    cells += [(6, j) for j in range(5, -1, -1)]       # bottom arm, left column, outwards
    cells += [(7, 0)]
    cells += [(8, j) for j in range(0, 6)]            # bottom arm, right column, inwards
    cells += [(i, 6) for i in range(9, 15)]           # right arm, lower row, outwards
    cells += [(14, 7)]
    cells += [(i, 8) for i in range(14, 8, -1)]       # right arm, upper row, inwards
    cells += [(8, j) for j in range(9, 15)]           # top arm, right column, outwards
    cells += [(7, 14)]
    cells += [(6, j) for j in range(14, 8, -1)]       # top arm, left column, inwards
    cells += [(i, 8) for i in range(5, -1, -1)]       # left arm, upper row, outwards
    cells += [(0, 7), (0, 6)]
    return cells


LOOP = _build_loop()
assert len(LOOP) == LOOP_LEN and len(set(LOOP)) == LOOP_LEN

START_IDX = {"red": 0, "blue": 13, "yellow": 26, "green": 39}
for _c, _i in START_IDX.items():            # sanity: the layout really is 4-fold symmetric
    assert LOOP[(_i + 13) % LOOP_LEN] == _rot(LOOP[_i])

LANE = {"red": [(k, 7) for k in range(1, 6)]}
for _a, _b in zip(("red", "blue", "yellow"), ("blue", "yellow", "green")):
    LANE[_b] = [_rot(c) for c in LANE[_a]]

# cells a piece cannot be captured on (and where it can't land if an opponent stands there)
SAFE_IDX = set(START_IDX.values()) | {(i + 8) % LOOP_LEN for i in START_IDX.values()}

_SIGN = {"red": (-1, -1), "green": (-1, 1), "yellow": (1, 1), "blue": (1, -1)}
YARD_XY = {c: (sx * 0.225, sy * 0.225) for c, (sx, sy) in _SIGN.items()}
FINISH_XY = {"red": (-0.045, 0.0), "green": (0.0, 0.045), "yellow": (0.045, 0.0), "blue": (0.0, -0.045)}


def loop_index(color, progress):
    """Index into LOOP for progress 1..51, else None."""
    if 1 <= progress <= 51:
        return (START_IDX[color] + progress - 1) % LOOP_LEN
    return None


def position_xy(color, progress):
    """World (x, y) where this colour's piece stands at the given progress."""
    if progress <= 0:
        return YARD_XY[color]
    if progress >= FINISH:
        return FINISH_XY[color]
    if progress <= 51:
        return grid_xy(*LOOP[loop_index(color, progress)])
    return grid_xy(*LANE[color][progress - 52])
