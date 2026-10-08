import math

from ludo_robot import kinematics as K
from ludo_robot import trajectory as T


def run_perfect_plant(geo, steps, grasp_width=0.005):
    """Executor against an ideal arm (measured == commanded) that grips a piece fine."""
    ex = T.MotionExecutor(geo)
    ex.begin(steps)
    tcp_log = []
    for _ in range(20000):
        if not ex.busy:
            break
        cmd = ex.commands()
        meas = {"yaw": cmd["yaw"], "shoulder": cmd["shoulder"], "elbow": cmd["elbow"],
                "wrist": cmd["wrist"], "finger_l": min(cmd["grip"], grasp_width),
                "finger_r": min(cmd["grip"], grasp_width)}
        ex.update(0.02, meas)
        tcp_log.append(geo.fk(ex.cmd_q)["tcp"])
    return ex, tcp_log


def test_pick_and_place_visits_source_and_target():
    geo = K.ArmGeometry.for_color("yellow")
    src, dst = geo.yard_center, (-0.30, -0.05)
    steps = T.pick_and_place_steps(src, dst)
    assert T.validate_steps(geo, steps, geo.home_q()) is None
    ex, log = run_perfect_plant(geo, steps)
    assert not ex.failed and not ex.busy
    lows = [p for p in log if p[2] < K.GRASP_Z + 0.002]
    assert any(math.dist(p[:2], src) < 0.002 for p in lows)
    assert any(p[2] < K.PLACE_Z + 0.002 and math.dist(p[:2], dst) < 0.002 for p in log)
    assert min(p[2] for p in log) >= K.GRASP_Z - 1e-6          # never lower than the grasp height
    assert ex.cmd_q == list(geo.home_q()) or max(abs(a - b) for a, b in zip(ex.cmd_q, geo.home_q())) < 1e-9


def test_missed_grasp_is_detected():
    geo = K.ArmGeometry.for_color("red")
    steps = T.pick_and_place_steps(geo.yard_center, (0.2, -0.05))
    ex, _ = run_perfect_plant(geo, steps, grasp_width=T.GRIP_CLOSE)   # fingers close fully: nothing held
    assert ex.failed and "grasp" in ex.message


def test_unreachable_destination_rejected_before_moving():
    geo = K.ArmGeometry.for_color("red")
    steps = T.pick_and_place_steps(geo.yard_center, (1.5, 1.5))
    assert "unreachable" in T.validate_steps(geo, steps, geo.home_q())
