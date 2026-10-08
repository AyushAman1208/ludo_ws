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
colcon build && source install/setup.bash
ros2 launch ludo_game play.launch.py                        # whole game, with viewer
ros2 launch ludo_game play.launch.py viewer:=false speed:=3 dice_seed:=42
```
Headless camera rendering needs `export MUJOCO_GL=egl`; add `render_cameras:=true` to publish images.
Keep `speed` <= 3: the controllers update at 50 Hz wall-clock, so very fast sims get coarse motion.

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
