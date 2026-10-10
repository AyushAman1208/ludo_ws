import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from ludo_description import geometry as G
from ludo_description.model_codegen import (
    generated_files,
    render_board_xml,
    render_robot_xml,
)


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]


def test_generated_files_are_current():
    stale = [
        path.relative_to(WORKSPACE_ROOT)
        for path, content in generated_files(WORKSPACE_ROOT).items()
        if not path.is_file() or path.read_text() != content
    ]
    assert stale == []


@pytest.mark.parametrize("color", G.COLORS)
def test_generated_robot_xml_is_valid_and_uses_canonical_geometry(color):
    root = ET.fromstring(render_robot_xml(color))
    body = root.find(f"./worldbody/body[@name='{color}_base']")
    assert body is not None
    actuator_names = [
        actuator.attrib["name"] for actuator in root.findall("./actuator/position")
    ]
    assert actuator_names == [
        f"{color}_a_yaw",
        f"{color}_a_shoulder",
        f"{color}_a_elbow",
        f"{color}_a_wrist",
        f"{color}_a_grip_l",
        f"{color}_a_grip_r",
        f"{color}_a_head_pan",
        f"{color}_a_head_tilt",
    ]

    x, y, yaw = G.base_pose_for_color(color)
    assert tuple(map(float, body.attrib["pos"].split())) == pytest.approx((x, y, 0.0))
    assert float(body.attrib["euler"].split()[2]) == pytest.approx(math.degrees(yaw))

    upper = root.find(f".//body[@name='{color}_upper']")
    assert upper is not None
    assert tuple(map(float, upper.attrib["pos"].split())) == pytest.approx(
        (0.0, 0.0, G.SHOULDER_OFFSET_Z)
    )


def test_canonical_home_keyframe_is_generated_for_every_robot():
    scene_path = WORKSPACE_ROOT / "src" / "ludo_sim" / "models" / "ludo.xml"
    generated_scene = generated_files(WORKSPACE_ROOT)[scene_path]
    assert (
        generated_scene.count(
            "<!-- Generated from ludo_description.geometry; do not edit by hand. -->"
        )
        == 1
    )
    root = ET.fromstring(generated_scene)
    key = root.find("./keyframe/key[@name='home']")
    assert key is not None

    qpos = list(map(float, key.attrib["qpos"].split()))
    controls = list(map(float, key.attrib["ctrl"].split()))
    assert len(qpos) == 60
    assert len(controls) == 32
    for index, color in enumerate(G.COLORS):
        arm_q = G.home_arm_q(color)
        arm_start = 28 + index * 8
        assert qpos[arm_start : arm_start + 4] == pytest.approx(arm_q)
        assert controls[index * 8 : index * 8 + 4] == pytest.approx(arm_q)


def test_yards_and_brick_defaults_follow_canonical_geometry():
    board_path = WORKSPACE_ROOT / "src" / "ludo_sim" / "models" / "board.xml"
    root = ET.fromstring(render_board_xml(board_path.read_text()))

    for color in G.COLORS:
        x, y = G.yard_center_for_color(color)
        brick = root.find(f".//body[@name='{color}_brick']")
        assert brick is not None
        assert tuple(map(float, brick.attrib["pos"].split())) == pytest.approx(
            (x, y, G.BRICK_REST_Z)
        )
        geom = root.find(f".//geom[@name='{color}_brick_body']")
        assert geom is not None
        assert tuple(map(float, geom.attrib["size"].split())) == pytest.approx(
            G.BRICK_SIZE
        )
        assert float(geom.attrib["mass"]) == pytest.approx(G.BRICK_MASS)

    slots = [
        geom
        for geom in root.findall(".//geom")
        if geom.attrib.get("type") == "cylinder"
        and float(geom.attrib["size"].split()[0]) == pytest.approx(G.YARD_SLOT_RADIUS)
    ]
    assert len(slots) == len(G.COLORS) * 4
