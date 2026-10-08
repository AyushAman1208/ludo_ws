"""Ludo rules for one piece per player (pure Python, no ROS).

* A piece leaves the yard on a 6 and lands on its start cell.
* Then it travels 51 cells round the loop and up its own 5-cell home lane to the centre.
* Finishing needs an exact roll (overshooting is not a legal move).
* Landing on an opponent captures it (it goes back to its yard) ...
* ... except on a safe cell (the four start cells and the four star cells): the arm can't
  put two pieces on one cell, so landing there is simply not allowed while it is occupied.
* Rolling a 6 gives the same player another roll.
* First piece to reach the centre wins.
"""
import random
from dataclasses import dataclass
from typing import Dict, List, Optional

from ludo_game import board_map as B


@dataclass(frozen=True)
class Move:
    color: str
    roll: int
    old_progress: int
    new_progress: int
    captured: Optional[str] = None       # colour of the captured opponent, if any


class Dice:
    """Seedable die; `sequence` (optional) is consumed first, for reproducible tests."""

    def __init__(self, seed: Optional[int] = None, sequence: Optional[List[int]] = None):
        self.rng = random.Random(seed)
        self.sequence = list(sequence or [])

    def roll(self) -> int:
        return self.sequence.pop(0) if self.sequence else self.rng.randint(1, 6)


class Game:
    def __init__(self, players=B.TURN_ORDER, first=None):
        self.players = [c for c in B.TURN_ORDER if c in players]
        self.turn = self.players.index(first) if first else 0
        self.progress: Dict[str, int] = {c: 0 for c in self.players}
        self.winner: Optional[str] = None

    @property
    def current(self) -> str:
        return self.players[self.turn]

    # ---------------------------------------------------------------- moves
    def propose(self, color: str, roll: int) -> Optional[Move]:
        """The move this roll gives `color`, or None if there is none."""
        p = self.progress[color]
        if p == 0:
            if roll != 6:
                return None
            new = 1
        else:
            new = p + roll
            if new > B.FINISH:
                return None                      # needs an exact roll to finish
        captured = None
        idx = B.loop_index(color, new)
        if idx is not None:
            for other in self.players:
                if other == color:
                    continue
                if B.loop_index(other, self.progress[other]) == idx:
                    if idx in B.SAFE_IDX:
                        return None              # blocked: cell is safe and occupied
                    captured = other
        return Move(color, roll, p, new, captured)

    def apply(self, move: Move):
        self.progress[move.color] = move.new_progress
        if move.captured:
            self.progress[move.captured] = 0
        if move.new_progress == B.FINISH:
            self.winner = move.color

    def end_turn(self, roll: int):
        """Advance to the next player (a 6 keeps the turn)."""
        if roll != 6 and not self.winner:
            self.turn = (self.turn + 1) % len(self.players)

    def snapshot(self) -> dict:
        return {"turn": self.current, "progress": dict(self.progress), "winner": self.winner}
