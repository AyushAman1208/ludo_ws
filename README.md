# Ludo with four robot arms (ROS 2 + MuJoCo)

```
src/
├── ludo_sim/    MuJoCo world: board, 4 arms with head/hand cameras, 4 bricks. Publishes state, takes joint commands.
├── ludo_robot/  Kinematics + pick-and-place for ONE arm (run one controller per colour).
└── ludo_game/   Rules + turn orchestration. Rolls the die, tells robots what to move, verifies the result.
```

## Install & run
```bash
pip install mujoco                     # once; not available through rosdep
cp -r src ~/ludo_ws/ && cd ~/ludo_ws
PYTHONPATH=src/ludo_description python3 -m ludo_description.model_codegen --workspace-root .
colcon build && source install/setup.bash
ros2 launch ludo_game play.launch.py                        # whole game, with viewer
ros2 launch ludo_game play.launch.py viewer:=false speed:=3 dice_seed:=42
```
Headless camera rendering needs `export MUJOCO_GL=egl`; add `render_cameras:=true` to publish images.
Keep `speed` <= 3: the controllers update at 50 Hz wall-clock, so very fast sims get coarse motion.

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

## How a turn works
```
game_manager --roll--> rules.Game.propose()  --(capture?)-->  /<victim>/task   move_piece -> own yard
                                             -->  /<mover>/task    move_piece -> new cell
robot_controller:  IK -> pick (hover, descend, grip, lift) -> carry -> place -> release -> back to ready pose
game_manager: waits for task_status "done", checks /<color>/brick_pose is within 3 cm of the cell, then next player
```

## Topics
| Topic | Type | Who |
|---|---|---|
| `/<color>/task` | String (JSON) | game -> robot. `{"id":1,"type":"move_piece","to":[x,y]}` (optional `"from"`), `{"type":"park"}`, `{"type":"look_at","at":[x,y]}` |
| `/<color>/task_status` | String (JSON) | robot -> game: accepted / running / done / failed |
| `/<color>/joint_command` | JointState | robot -> sim (yaw, shoulder, elbow, wrist, grip, head_pan, head_tilt) |
| `/<color>/joint_states`, `/<color>/brick_pose` | JointState, PoseStamped | sim -> everyone |
| `/ludo/dice` | Int32 | every roll |
| `/ludo/game_state` | String (JSON) | phase, turn, last roll, progress, winner |
| `/ludo/state` | String (JSON) | physical truth: where each piece is |

The latest dice roll is also shown in the top-right corner of the MuJoCo viewer.

Services: `/ludo/start`, `/ludo/new_game` (Trigger), `/mujoco/reset` (Empty).

## Rules (one piece per player)
6 to leave the yard · 51 cells round the loop + 5-cell home lane · exact roll to finish · landing on an
opponent captures it · safe cells (start + star cells) can't be landed on while occupied (two bricks can't
share a cell) · a 6 rolls again · first to the centre wins.

## Tests
```bash
cd src/ludo_robot && python -m pytest test     # kinematics, planner
cd src/ludo_game  && python -m pytest test     # rules, board map
```
