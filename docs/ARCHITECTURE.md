# System Architecture

## Component Overview
```
┌─────────────────────────────────────────────────────────────┐
│                      OPERATOR INPUT                          │
│              RViz 2D Goal Pose  /  /swarm/goal topic         │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                   SWARM CONTROLLER NODE                      │
│  • Delta movement calculation                               │
│  • Sequential dispatch (leader first, then followers)       │
│  • Spiral search auto-recovery on goal failure              │
│  • Goal cancellation on new command                         │
│  • Map bounds clamping                                       │
└──────┬──────────────┬──────────────┬──────────────┬─────────┘
       │              │              │              │
       ▼              ▼              ▼              ▼
  ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐
  │ robot1  │   │ robot2  │   │ robot3  │   │ robot4  │
  │  Nav2   │   │  Nav2   │   │  Nav2   │   │  Nav2   │
  │  Stack  │   │  Stack  │   │  Stack  │   │  Stack  │
  └─────────┘   └─────────┘   └─────────┘   └─────────┘
   Leader         Follower      Follower      Follower
   y = 0.0        y = +1.0      y = -1.2      y = 0.0 (rear)
```

## Per-Robot Nav2 Stack

Each robot runs a fully independent Nav2 stack in its own namespace:

| Node | Purpose |
|------|---------|
| `map_server` | Serves the occupancy grid map |
| `planner_server` | NavFn global path planner (Dijkstra) |
| `controller_server` | Regulated Pure Pursuit local controller |
| `behavior_server` | Recovery behaviors (spin, backup, wait) |
| `bt_navigator` | Behavior Tree navigate_to_pose action server |
| `velocity_smoother` | Smooths cmd_vel output |
| `lifecycle_manager` | Manages node lifecycle states |

## TF Tree
```
map
├── robot1/odom → robot1/base_footprint → robot1/base_link → robot1/base_scan
├── robot2/odom → robot2/base_footprint → robot2/base_link → robot2/base_scan
├── robot3/odom → robot3/base_footprint → robot3/base_link → robot3/base_scan
└── robot4/odom → robot4/base_footprint → robot4/base_link → robot4/base_scan
```

- `map → robotX/odom` — static transform published at spawn by `odom_to_tf.py`
- `robotX/odom → robotX/base_footprint` — dynamic transform from odometry by `odom_tf_broadcaster.py`

## Delta Movement Algorithm
```
1. Read robot1 actual position (r1x, r1y) from TF
2. dx = goal_x - r1x
   dy = goal_y - r1y
3. Send robot1 to (goal_x, goal_y) — wait for SUCCESS
4. Read robot1 actual final position from TF
5. For each follower:
      follower_goal = follower_current_pos + actual_delta
```

Using robot1's actual final position (not commanded goal) ensures followers
stay correctly spaced even when robot1 takes a recovery path.

## Spiral Search Recovery

When Nav2 returns ABORTED (status=6):
```
attempt 1-8:   ring=1, radius=0.2m, 8 directions (45° apart)
attempt 9-16:  ring=2, radius=0.4m, 8 directions
attempt 17-24: ring=3, radius=0.6m, 8 directions
...up to MAX_RETRIES
```

## Laser Scan Filtering

Each robot's scan_filter.py node:
1. Subscribes to /robotX/scan (raw)
2. Looks up TF positions of all other robots
3. Removes scan points within 0.25m of any teammate
4. Publishes /robotX/scan_filtered

This prevents Nav2 from treating teammates as static obstacles in costmaps.

## Key Topics and Actions

| Topic / Action | Type | Purpose |
|----------------|------|---------|
| `/swarm/goal` | `geometry_msgs/PoseStamped` | CLI goal input |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz 2D Goal Pose output |
| `/robotX/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Nav2 action per robot |
| `/robotX/scan` | `sensor_msgs/LaserScan` | Raw laser scan |
| `/robotX/scan_filtered` | `sensor_msgs/LaserScan` | Teammate-filtered scan |
| `/robotX/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/robotX/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
