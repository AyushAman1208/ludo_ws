import math

import pytest

from ludo_robot import kinematics as K

ALL = [K.ArmGeometry.create_for_color(c) for c in K.COLORS]


def outer_track_cells():
    lane = set()
    for k in range(1, 6):
        lane |= {(k, 7), (7, k), (14 - k, 7), (7, 14 - k)}
    return [(i, j) for i in range(15) for j in range(15)
            if ((6 <= j <= 8) != (6 <= i <= 8)) and (i, j) not in lane]


@pytest.mark.parametrize("geo", ALL)
def test_fk_ik_roundtrip(geo):
    for x, y, z in [(0.0, 0.0, 0.06), (-0.05, 0.35, 0.2), (0.35, 0.0, 0.062), (0.2, -0.3, 0.1)]:
        res = geo.ik(x, y, z)
        if not res.ok:
            continue
        tcp = geo.fk(res.q)["tcp"]
        assert tcp == pytest.approx((x, y, z), abs=1e-6)
        assert sum(res.q[1:]) == pytest.approx(math.pi, abs=1e-9)   # gripper points down


@pytest.mark.parametrize("geo", ALL)
def test_home_pose_over_own_yard(geo):
    tcp = geo.fk(geo.home_q())["tcp"]
    assert tcp[:2] == pytest.approx(geo.yard_center, abs=1e-6)
    assert tcp[2] == pytest.approx(K.HOVER_Z, abs=1e-6)


def test_every_robot_reaches_every_outer_track_cell():
    cells = outer_track_cells()
    assert len(cells) == 52
    for geo in ALL:
        for i, j in cells:
            x, y = (i - 7) * 0.05, (j - 7) * 0.05
            for z in (K.GRASP_Z, K.PLACE_Z, K.HOVER_Z):
                assert geo.reachable(x, y, z), (geo.color, i, j, z, geo.ik(x, y, z).reason)


def test_unreachable_target_rejected():
    assert not ALL[0].ik(2.0, 2.0, 0.1).ok


def test_gaze_points_at_target():
    geo = K.ArmGeometry.create_for_color("red")
    pan, tilt = geo.gaze(*geo.yard_center, K.BRICK_REST_Z)
    assert abs(pan) < 0.7 and 0.0 < tilt < 1.4
