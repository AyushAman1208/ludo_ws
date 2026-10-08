import pytest

from ludo_game import board_map as B
from ludo_game.rules import Dice, Game


def test_loop_is_52_cells_and_symmetric():
    assert len(B.LOOP) == 52
    for c, i in B.START_IDX.items():
        assert B.position_xy(c, 1) == B.grid_xy(*B.LOOP[i])


def test_start_cells_match_the_painted_board():
    # start cells on board.xml: red (1,6), blue (8,1), yellow (13,8), green (6,13)
    assert B.LOOP[B.START_IDX["red"]] == (1, 6)
    assert B.LOOP[B.START_IDX["blue"]] == (8, 1)
    assert B.LOOP[B.START_IDX["yellow"]] == (13, 8)
    assert B.LOOP[B.START_IDX["green"]] == (6, 13)


def test_home_lanes_lead_to_the_centre():
    assert B.LANE["red"] == [(1, 7), (2, 7), (3, 7), (4, 7), (5, 7)]
    assert B.LANE["blue"][0] == (7, 1) and B.LANE["yellow"][0] == (13, 7) and B.LANE["green"][0] == (7, 13)


def test_last_loop_cell_is_next_to_own_lane():
    # progress 51 is the cell right before the lane (red: (0,7) -> (1,7))
    assert B.LOOP[B.loop_index("red", 51)] == (0, 7)
    for c in B.COLORS:
        i, j = B.LOOP[B.loop_index(c, 51)]
        li, lj = B.LANE[c][0]
        assert abs(i - li) + abs(j - lj) == 1


def test_needs_six_to_leave_yard():
    g = Game()
    assert g.propose("red", 5) is None
    m = g.propose("red", 6)
    assert m.new_progress == 1


def test_exact_finish_required():
    g = Game()
    g.progress["red"] = 55
    assert g.propose("red", 3) is None
    m = g.propose("red", 2)
    assert m.new_progress == 57
    g.apply(m)
    assert g.winner == "red"


def test_capture_sends_opponent_home():
    g = Game()
    g.progress["red"] = 10                       # loop idx 9
    g.progress["blue"] = 0
    # put blue on loop idx 11 -> blue progress p with (13 + p - 1) % 52 == 11  => p = 51
    g.progress["blue"] = 51
    m = g.propose("red", 2)
    assert m.captured == "blue"
    g.apply(m)
    assert g.progress["blue"] == 0 and g.progress["red"] == 12


def test_safe_cell_is_blocked_when_occupied():
    g = Game()
    g.progress["blue"] = 1                       # blue on its start (idx 13, safe)
    g.progress["red"] = 10                       # red on idx 9 -> a roll of 4 lands on idx 13
    assert g.propose("red", 4) is None
    g.progress["blue"] = 0
    assert g.propose("red", 4).new_progress == 14


def test_six_keeps_the_turn():
    g = Game()
    g.end_turn(6)
    assert g.current == "red"
    g.end_turn(3)
    assert g.current == "blue"


def test_dice_sequence_then_random():
    d = Dice(seed=1, sequence=[6, 6, 2])
    assert [d.roll() for _ in range(3)] == [6, 6, 2]
    assert 1 <= d.roll() <= 6


def test_a_game_always_terminates():
    for seed in range(5):
        g, d, n = Game(), Dice(seed), 0
        while not g.winner and n < 5000:
            r = d.roll()
            m = g.propose(g.current, r)
            if m:
                g.apply(m)
            g.end_turn(r)
            n += 1
        assert g.winner, f"seed {seed} never finished"
