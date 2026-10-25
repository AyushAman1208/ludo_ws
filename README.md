# Ludo environment with four robot arms (ROS 2 + MuJoCo)

```
src/
├── ludo_description/  Canonical geometry for the board and robot arms.
├── ludo_sim/          MuJoCo world; publishes robot, camera, and piece state.
├── ludo_robot/        Kinematics and motion-planning library.
├── ludo_game/         Random dice-roll publisher.
└── player/            Reserved for player logic; reads state/dice and issues manipulation commands.
```

## Install & run
```bash
pip install mujoco                     # once; not available through rosdep
cp -r src ~/ludo_ws/ && cd ~/ludo_ws
PYTHONPATH=src/ludo_description python3 -m ludo_description.model_codegen --workspace-root .
colcon build && source install/setup.bash
ros2 launch ludo_game play.launch.py                        # environment + dice publisher
ros2 launch ludo_game play.launch.py viewer:=false speed:=3 dice_seed:=42
```
Headless camera rendering needs `export MUJOCO_GL=egl`; add `render_cameras:=true` to publish images.
The dice publisher emits a random value from 1 to 6 at 0.2 Hz by default. Set `dice_rate` to change
the publication rate, or `dice_seed` to make the sequence reproducible. The player package does not
yet subscribe to dice/state topics or send manipulation commands.

## Robot geometry source of truth
`src/ludo_description/ludo_description/geometry.py` is the canonical source for robot dimensions,
mounting positions, joint limits, task heights, and shared yard/board-scale values. The generator
updates robot XML, yard markers and starting-piece defaults in `src/ludo_sim/models/board.xml`, and
the home keyframe in `src/ludo_sim/models/ludo.xml`. Do not hand-edit those generated values;
regenerate after changing canonical model values:

```bash
PYTHONPATH=src/ludo_description python3 -m ludo_description.model_codegen --workspace-root .
PYTHONPATH=src/ludo_description python3 -m ludo_description.model_codegen --workspace-root . --check
colcon build --symlink-install
source install/setup.bash
```

The check command is also covered by the `ludo_description` tests. Generated XML is checked in so
the simulator can load it directly; rebuilding after generation refreshes installed model files.
The kinematics solver and MuJoCo model generator use the same arm IK implementation for the ready
pose, avoiding a separately maintained home-joint vector. The board's static track artwork remains
hand-authored; the generator owns its yard markers and starting-piece defaults. After building the
package, `ros2 run ludo_description generate_robot_models` is also available, but the source-module
command above avoids accidentally running a previously installed copy of the geometry.

## Topics
| Topic | Type | Who |
|---|---|---|
| `/mujoco/joint_states` | JointState | simulator -> everyone |
| `/mujoco/brick_poses` | PoseArray | simulator -> everyone |
| `/<color>/joint_states`, `/<color>/brick_pose` | JointState, PoseStamped | simulator -> everyone |
| `/<color>/head_cam/image_raw`, `/<color>/hand_cam/image_raw` | Image | simulator -> everyone (if camera rendering is enabled) |
| `/ludo/state` | String (JSON) | simulator state publisher -> everyone; physical piece positions |
| `/ludo/dice` | Int32 | dice publisher -> everyone |
| `/<color>/joint_command` | JointState | player -> simulator; position targets (yaw, shoulder, elbow, wrist, grip, head_pan, head_tilt) |

The simulator publishes environment state and accepts low-level joint commands, but does not decide
game turns or plan manipulation. The player package will consume the state/dice topics and publish
joint commands when that behavior is implemented. The simulator also provides `/mujoco/reset`
(`std_srvs/Empty`).

## Rules (one piece per player)
6 to leave the yard · 51 cells round the loop + 5-cell home lane · exact roll to finish · landing on an
opponent captures it · safe cells (start + star cells) can't be landed on while occupied (two bricks can't
share a cell) · a 6 rolls again · first to the centre wins.

## Tests
```bash
cd src/ludo_robot && python -m pytest test     # kinematics, planner
cd src/ludo_game  && python -m pytest test     # rules, board map
```
