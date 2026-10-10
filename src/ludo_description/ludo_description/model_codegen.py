"""Generate checked-in MuJoCo robot models and their geometry-dependent home pose."""

import argparse
import math
import re
import sys
from pathlib import Path
from typing import Dict
import xml.etree.ElementTree as ET

from ludo_description import geometry as G


def _fmt(value):
    """Format model numbers consistently without unnecessary trailing zeroes."""
    if abs(value) < 1e-12:
        return "0"
    return format(value, ".8g")


def _vector(values):
    return " ".join(_fmt(value) for value in values)


def _rgba(values):
    return _vector(values)


def _joint_range(name):
    return _vector(G.JOINT_LIMITS_DEG[name])


def _actuator(color, name, suffix=None, joint_suffix=None):
    """Render one position actuator using the shared gain and control-range data."""
    suffix = suffix or name
    joint_suffix = joint_suffix or suffix
    kp, kv = G.ACTUATOR_GAINS[name]
    lo, hi = G.ACTUATOR_LIMITS[name]
    return (
        f'    <position name="{color}_a_{suffix}" '
        f'joint="{color}_{joint_suffix}" kp="{_fmt(kp)}" kv="{_fmt(kv)}" '
        f'ctrlrange="{_fmt(lo)} {_fmt(hi)}"/>'
    )


def render_robot_xml(color):
    """Render one robot include from canonical dimensions and joint specifications."""
    if color not in G.COLORS:
        raise ValueError(f"unknown robot color: {color}")

    base_x, base_y, base_yaw = G.base_pose_for_color(color)
    arm_rgba = _rgba(G.ROBOT_RGBA[color])
    neutral_rgba = _rgba(G.NEUTRAL_RGBA)
    lens_rgba = _rgba(G.LENS_RGBA)
    yaw_range = _joint_range("yaw")
    shoulder_range = _joint_range("shoulder")
    elbow_range = _joint_range("elbow")
    wrist_range = _joint_range("wrist")
    head_pan_range = _joint_range("head_pan")
    head_tilt_range = _joint_range("head_tilt")
    finger_range = _vector(G.FINGER_RANGE)
    actuator_lines = "\n".join(
        [
            _actuator(color, "yaw"),
            _actuator(color, "shoulder"),
            _actuator(color, "elbow"),
            _actuator(color, "wrist"),
            _actuator(color, "grip", "grip_l", "finger_l"),
            _actuator(color, "grip", "grip_r", "finger_r"),
            _actuator(color, "head_pan"),
            _actuator(color, "head_tilt"),
        ]
    )

    return f"""<mujoco>
  <!-- Generated from ludo_description.geometry; edit canonical values, then regenerate. -->
  <worldbody>
    <body name="{color}_base" pos="{_fmt(base_x)} {_fmt(base_y)} 0" euler="0 0 {_fmt(math.degrees(base_yaw))}">
      <geom contype="2" conaffinity="1" type="cylinder" size="{_fmt(G.BASE_FOOT_RADIUS)} {_fmt(G.BASE_FOOT_HALF_HEIGHT)}" pos="0 0 {_fmt(G.BASE_FOOT_Z)}" rgba="{neutral_rgba}"/>
      <geom contype="2" conaffinity="1" type="cylinder" size="{_fmt(G.BASE_PEDESTAL_RADIUS)} {_fmt(G.BASE_PEDESTAL_HALF_HEIGHT)}" pos="0 0 {_fmt(G.BASE_PEDESTAL_Z)}" rgba="{arm_rgba}"/>
      <body name="{color}_turret" pos="0 0 {_fmt(G.TURRET_Z)}" gravcomp="1">
        <joint name="{color}_yaw" type="hinge" axis="0 0 1" range="{yaw_range}" damping="{_fmt(G.JOINT_DYNAMICS["yaw"][0])}" armature="{_fmt(G.JOINT_DYNAMICS["yaw"][1])}"/>
        <geom contype="2" conaffinity="1" type="sphere" size="{_fmt(G.TURRET_RADIUS)}" rgba="{neutral_rgba}"/>
        <body name="{color}_upper" pos="0 0 {_fmt(G.SHOULDER_OFFSET_Z)}" gravcomp="1">
          <joint name="{color}_shoulder" type="hinge" axis="0 1 0" range="{shoulder_range}" damping="{_fmt(G.JOINT_DYNAMICS["shoulder"][0])}" armature="{_fmt(G.JOINT_DYNAMICS["shoulder"][1])}"/>
          <geom contype="2" conaffinity="1" type="capsule" fromto="0 0 0 0 0 {_fmt(G.L1)}" size="{_fmt(G.UPPER_RADIUS)}" rgba="{arm_rgba}"/>
          <body name="{color}_fore" pos="0 0 {_fmt(G.L1)}" gravcomp="1">
            <joint name="{color}_elbow" type="hinge" axis="0 1 0" range="{elbow_range}" damping="{_fmt(G.JOINT_DYNAMICS["elbow"][0])}" armature="{_fmt(G.JOINT_DYNAMICS["elbow"][1])}"/>
            <geom contype="2" conaffinity="1" type="sphere" size="{_fmt(G.ELBOW_RADIUS)}" rgba="{neutral_rgba}"/>
            <geom contype="2" conaffinity="1" type="capsule" fromto="0 0 0 0 0 {_fmt(G.L2)}" size="{_fmt(G.FOREARM_RADIUS)}" rgba="{arm_rgba}"/>
            <body name="{color}_wrist" pos="0 0 {_fmt(G.L2)}" gravcomp="1">
              <joint name="{color}_wrist" type="hinge" axis="0 1 0" range="{wrist_range}" damping="{_fmt(G.JOINT_DYNAMICS["wrist"][0])}" armature="{_fmt(G.JOINT_DYNAMICS["wrist"][1])}"/>
              <geom contype="2" conaffinity="1" type="sphere" size="{_fmt(G.WRIST_RADIUS)}" rgba="{neutral_rgba}"/>
              <geom contype="2" conaffinity="1" type="box" size="{_vector(G.HAND_BOX_HALF_SIZE)}" pos="0 0 {_fmt(G.HAND_BOX_Z)}" rgba="{neutral_rgba}"/>
              <body name="{color}_finger_l" pos="0 {_fmt(G.FINGER_BODY_Y)} {_fmt(G.FINGER_BODY_Z)}" gravcomp="1">
                <joint name="{color}_finger_l" type="slide" axis="0 -1 0" range="{finger_range}" damping="{_fmt(G.JOINT_DYNAMICS["finger"][0])}"/>
                <geom contype="2" conaffinity="1" type="box" size="{_vector(G.FINGER_BOX_HALF_SIZE)}" pos="0 0 {_fmt(G.FINGER_BOX_Z)}" rgba="{arm_rgba}"/>
              </body>
              <body name="{color}_finger_r" pos="0 -{_fmt(G.FINGER_BODY_Y)} {_fmt(G.FINGER_BODY_Z)}" gravcomp="1">
                <joint name="{color}_finger_r" type="slide" axis="0 1 0" range="{finger_range}" damping="{_fmt(G.JOINT_DYNAMICS["finger"][0])}"/>
                <geom contype="2" conaffinity="1" type="box" size="{_vector(G.FINGER_BOX_HALF_SIZE)}" pos="0 0 {_fmt(G.FINGER_BOX_Z)}" rgba="{arm_rgba}"/>
              </body>
              <camera name="{color}_hand_cam" pos="0 0 {_fmt(G.HAND_CAMERA_Z)}" xyaxes="0 1 0 1 0 0" fovy="90" resolution="320 240"/>
            </body>
          </body>
        </body>
      </body>
      <geom contype="2" conaffinity="1" type="capsule" fromto="{_fmt(G.HEAD_LOCAL_XY[0])} {_fmt(G.HEAD_LOCAL_XY[1])} {_fmt(G.HEAD_POST_START_Z)} {_fmt(G.HEAD_LOCAL_XY[0])} {_fmt(G.HEAD_LOCAL_XY[1])} {_fmt(G.HEAD_POST_END_Z)}" size="{_fmt(G.HEAD_POST_RADIUS)}" rgba="{neutral_rgba}"/>
      <body name="{color}_head" pos="{_fmt(G.HEAD_LOCAL_XY[0])} {_fmt(G.HEAD_LOCAL_XY[1])} {_fmt(G.HEAD_PAN_Z)}" gravcomp="1">
        <joint name="{color}_head_pan" type="hinge" axis="0 0 1" range="{head_pan_range}" damping="{_fmt(G.JOINT_DYNAMICS["head"][0])}" armature="{_fmt(G.JOINT_DYNAMICS["head"][1])}"/>
        <geom contype="2" conaffinity="1" type="sphere" size="{_fmt(G.HEAD_SPHERE_RADIUS)}" rgba="{neutral_rgba}"/>
        <body name="{color}_neck" pos="0 0 {_fmt(G.HEAD_TILT_OFFSET_Z)}" gravcomp="1">
          <joint name="{color}_head_tilt" type="hinge" axis="0 1 0" range="{head_tilt_range}" damping="{_fmt(G.JOINT_DYNAMICS["head"][0])}" armature="{_fmt(G.JOINT_DYNAMICS["head"][1])}"/>
          <geom contype="2" conaffinity="1" type="box" size="{_vector(G.HEAD_NECK_BOX_HALF_SIZE)}" pos="{_fmt(G.HEAD_NECK_BOX_X)} 0 0" rgba="{arm_rgba}"/>
          <geom contype="2" conaffinity="1" type="cylinder" fromto="{_fmt(G.HEAD_LENS_START_X)} 0 0 {_fmt(G.HEAD_LENS_END_X)} 0 0" size="{_fmt(G.HEAD_LENS_RADIUS)}" rgba="{lens_rgba}"/>
          <camera name="{color}_head_cam" pos="{_fmt(G.HEAD_CAMERA_X)} 0 0" xyaxes="0 -1 0 0 0 1" fovy="70" resolution="640 480"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
{actuator_lines}
  </actuator>
</mujoco>
"""


def render_home_keyframe():
    """Render initial brick poses and robot commands from the same geometry as IK."""
    qpos = []
    controls = []
    for color in G.COLORS:
        x, y = G.yard_center_for_color(color)
        yaw = G.base_pose_for_color(color)[2]
        qpos.extend(
            (x, y, G.BRICK_REST_Z, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        )

    for color in G.COLORS:
        arm_q = G.home_arm_q(color)
        qpos.extend((*arm_q, 0.0, 0.0, *G.HOME_HEAD))
        controls.extend((*arm_q, 0.0, 0.0, *G.HOME_HEAD))

    return (
        "  <!-- Generated from ludo_description.geometry; do not edit by hand. -->\n"
        "  <keyframe>\n"
        f'    <key name="home" qpos="{_vector(qpos)}" ctrl="{_vector(controls)}"/>\n'
        "  </keyframe>"
    )


def _replace_attribute(tag, name, value):
    """Replace one XML attribute while preserving the rest of its source formatting."""
    return re.sub(
        rf'(\b{name}=")[^"]*(")',
        lambda match: f"{match.group(1)}{value}{match.group(2)}",
        tag,
        count=1,
    )


def _color_for_rgba(value):
    rgba = tuple(round(float(component), 6) for component in value.split())
    for color, expected in G.ROBOT_RGBA.items():
        if rgba == tuple(round(component, 6) for component in expected):
            return color
    return None


def render_board_xml(source):
    """Update generated yard markers and home-brick defaults from canonical geometry."""
    yard_boxes = {color: 0 for color in G.COLORS}
    yard_insets = 0
    yard_slots = {color: 0 for color in G.COLORS}

    def update_geom(match):
        nonlocal yard_insets
        tag = match.group(0)
        element = ET.fromstring(tag)
        attrs = element.attrib
        position = [float(value) for value in attrs.get("pos", "").split()]
        size = [float(value) for value in attrs.get("size", "").split()]
        if len(position) != 3 or len(size) < 2:
            return tag

        color = _color_for_rgba(attrs.get("rgba", ""))
        if (
            attrs.get("type") == "box"
            and math.isclose(size[0], G.YARD_HALF_SIZE, abs_tol=1e-9)
            and math.isclose(size[1], G.YARD_HALF_SIZE, abs_tol=1e-9)
            and math.isclose(position[2], G.YARD_BACKGROUND_Z, abs_tol=1e-9)
            and color is not None
        ):
            center = G.yard_center_for_color(color)
            yard_boxes[color] += 1
            tag = _replace_attribute(
                tag,
                "size",
                _vector(
                    (G.YARD_HALF_SIZE, G.YARD_HALF_SIZE, G.YARD_BACKGROUND_HALF_HEIGHT)
                ),
            )
            return _replace_attribute(tag, "pos", _vector((*center, position[2])))

        if (
            attrs.get("type") == "box"
            and math.isclose(size[0], G.YARD_INSET_HALF_SIZE, abs_tol=1e-9)
            and math.isclose(size[1], G.YARD_INSET_HALF_SIZE, abs_tol=1e-9)
            and math.isclose(position[2], G.YARD_INSET_Z, abs_tol=1e-9)
        ):
            if yard_insets >= len(G.COLORS):
                raise ValueError("found more yard inset boxes than configured colors")
            center = G.yard_center_for_color(G.COLORS[yard_insets])
            yard_insets += 1
            tag = _replace_attribute(
                tag,
                "size",
                _vector(
                    (
                        G.YARD_INSET_HALF_SIZE,
                        G.YARD_INSET_HALF_SIZE,
                        G.YARD_INSET_HALF_HEIGHT,
                    )
                ),
            )
            return _replace_attribute(tag, "pos", _vector((*center, position[2])))

        if (
            attrs.get("type") == "cylinder"
            and math.isclose(size[0], G.YARD_SLOT_RADIUS, abs_tol=1e-9)
            and math.isclose(position[2], G.YARD_SLOT_Z, abs_tol=1e-9)
            and color is not None
        ):
            slot = yard_slots[color]
            if slot >= 4:
                raise ValueError(f"found more than four yard slots for {color}")
            center_x, center_y = G.yard_center_for_color(color)
            offsets = (
                (-G.BOARD_CELL, -G.BOARD_CELL),
                (-G.BOARD_CELL, G.BOARD_CELL),
                (G.BOARD_CELL, -G.BOARD_CELL),
                (G.BOARD_CELL, G.BOARD_CELL),
            )
            dx, dy = offsets[slot]
            yard_slots[color] += 1
            tag = _replace_attribute(
                tag, "size", _vector((G.YARD_SLOT_RADIUS, G.YARD_SLOT_HALF_HEIGHT))
            )
            return _replace_attribute(
                tag, "pos", _vector((center_x + dx, center_y + dy, position[2]))
            )
        return tag

    source = re.sub(r"<geom\b[^>]*/>", update_geom, source)
    if yard_boxes != {color: 1 for color in G.COLORS}:
        raise ValueError(
            f"expected one colored yard background per color, found {yard_boxes}"
        )
    if yard_insets != len(G.COLORS):
        raise ValueError(
            f"expected {len(G.COLORS)} yard inset boxes, found {yard_insets}"
        )
    if yard_slots != {color: 4 for color in G.COLORS}:
        raise ValueError(f"expected four yard slots per color, found {yard_slots}")

    for color in G.COLORS:
        x, y = G.yard_center_for_color(color)
        yaw = math.degrees(G.base_pose_for_color(color)[2])
        body_pattern = re.compile(rf'<body name="{color}_brick"[^>]*>')
        source, body_count = body_pattern.subn(
            lambda match: _replace_attribute(
                _replace_attribute(
                    match.group(0), "pos", _vector((x, y, G.BRICK_REST_Z))
                ),
                "euler",
                f"0 0 {_fmt(yaw)}",
            ),
            source,
            count=1,
        )
        if body_count != 1:
            raise ValueError(f"expected one {color}_brick body in board.xml")

        geom_pattern = re.compile(
            rf'<geom\b(?=[^>]*\bname="{color}_brick_body")[^>]*/>'
        )

        def update_brick_geom(match):
            tag = match.group(0)
            tag = _replace_attribute(tag, "size", _vector(G.BRICK_SIZE))
            tag = _replace_attribute(tag, "mass", _fmt(G.BRICK_MASS))
            tag = _replace_attribute(tag, "friction", _vector(G.BRICK_FRICTION))
            return _replace_attribute(tag, "rgba", _rgba(G.ROBOT_RGBA[color]))

        source, geom_count = geom_pattern.subn(update_brick_geom, source, count=1)
        if geom_count != 1:
            raise ValueError(f"expected one {color}_brick_body geom in board.xml")

    marker = "    <!-- Yard markings and starting pieces are generated from ludo_description.geometry. -->"
    if marker not in source:
        source = source.replace("<worldbody>", f"<worldbody>\n{marker}", 1)
    return source


def generated_files(workspace_root):
    """Return the generated-file contents without modifying the source tree."""
    model_dir = workspace_root / "src" / "ludo_sim" / "models"
    scene_path = model_dir / "ludo.xml"
    board_path = model_dir / "board.xml"
    scene = scene_path.read_text()
    board = render_board_xml(board_path.read_text())
    keyframe = render_home_keyframe()
    scene, substitutions = re.subn(
        r"(?:  <!-- Generated from ludo_description\.geometry; do not edit by hand\. -->\n)*"
        r"  <keyframe>.*?  </keyframe>",
        keyframe,
        scene,
        count=1,
        flags=re.DOTALL,
    )
    if substitutions != 1:
        raise ValueError(f"expected exactly one home keyframe in {scene_path}")

    generated: Dict[Path, str] = {
        model_dir / "robots" / f"robot_{color}.xml": render_robot_xml(color)
        for color in G.COLORS
    }
    generated[board_path] = board
    generated[scene_path] = scene
    return generated


def find_workspace_root(start):
    """Find a source workspace by its top-level MuJoCo scene."""
    for candidate in (start, *start.parents):
        if (candidate / "src" / "ludo_sim" / "models" / "ludo.xml").is_file():
            return candidate
    raise FileNotFoundError(
        "could not find src/ludo_sim/models/ludo.xml; run from the workspace or pass --workspace-root"
    )


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Generate MuJoCo robot XML and its geometry-dependent home keyframe."
    )
    parser.add_argument(
        "--workspace-root",
        type=Path,
        help="workspace containing src/ludo_sim (default: search current directory and parents)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail if generated files differ from checked-in output; do not write files",
    )
    args = parser.parse_args(argv)

    try:
        root = (
            args.workspace_root.resolve()
            if args.workspace_root
            else find_workspace_root(Path.cwd())
        )
        files = generated_files(root)
        stale = [
            path
            for path, content in files.items()
            if not path.is_file() or path.read_text() != content
        ]
        if args.check:
            if stale:
                for path in stale:
                    print(
                        f"stale generated file: {path.relative_to(root)}",
                        file=sys.stderr,
                    )
                return 1
            print(f"generated MuJoCo models are up to date ({len(files)} files)")
            return 0

        for path, content in files.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        print(
            f"generated {len(files)} MuJoCo model files under {root / 'src' / 'ludo_sim' / 'models'}"
        )
        return 0
    except (OSError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
