# AutoNav Tutor Simulation

A ROS 2 simulation package demonstrating automated mobile robot navigation in a Gazebo environment. This project implements a differential drive robot with Lidar sensor, a custom enclosed arena with obstacles, and a modular control architecture designed for testing navigation algorithms (PID) and reinforcement learning.

## 📌 Project Overview

The **AutoNav Tutor** simulation spawns a custom differential drive robot (`box_bot`) into an arena. The system randomly generates target coordinates (visualized as green markers), and a control node attempts to drive the robot to these coordinates. An environment monitor runs in the background to detect collisions via Lidar and reset the simulation automatically, providing an ideal setup for reinforcement learning training.

## 🚀 Key Features

* **Custom Robot (URDF/Xacro):**
    * Differential drive kinematics with caster wheel support
    * Integrated Lidar sensor (Ray sensor) with 360° coverage and 3.5m range
    * Odometry and TF publishing via `libgazebo_ros_diff_drive`

* **Simulation Environment:**
    * **Gazebo World:** A 6x6 meter walled arena (`arena.world`) containing static obstacles (cylinders and boxes) to challenge navigation
    * **Automatic Resets:** Collision detection using Lidar data (< 0.35m clearance) triggers automatic robot pose and goal resets

* **Modular Node Architecture:**
    * **Goal Manager:** Randomly spawns visual goals and handles goal relocation upon reset
    * **PID Driver:** Proportional controller for linear and angular velocity control
    * **Lidar Monitor:** Collision detection and safety monitoring
    * **RL Training Nodes:** Environment wrappers for reinforcement learning integration

## 📦 Dependencies

* **ROS 2** (Tested on Humble/Foxy)
* `gazebo_ros`
* `xacro`
* `robot_state_publisher`
* `geometry_msgs`, `nav_msgs`, `sensor_msgs`

## 🛠️ Installation

1. **Clone the repository** into your ROS 2 workspace `src` folder:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/hadissuryaalamin/rl-obstacle-nav-ros2 tutor
   ```

2. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select tutor
   ```

3. **Source the setup script:**
   ```bash
   source install/setup.bash
   ```

## ▶️ Usage

### Quick Start

The package supports multiple launch configurations for different use cases. Follow the guides below for your specific needs.

### 1. Launch Normally with PID Controller Only

Use this for basic robot control testing with PID-based navigation without the full simulation environment.

```bash
# Terminal 1: Start ROS 2 node
ros2 run tutor controller.py

# Terminal 2: Start the PID driver
ros2 run tutor pid_driver.py

# Terminal 3: Start the goal manager (publishes random targets)
ros2 run tutor goal_manager.py
```

**What this does:**
- Launches only the essential ROS nodes
- Robot follows PID-based velocity commands
- No Gazebo simulation required
- Useful for testing control logic in isolation
- No collision detection or environment monitoring

**Topics:**
- `/cmd_vel` - Robot velocity commands
- `/odom` - Odometry feedback
- `/goal_coordinates` - Target goal positions

---

### 2. Launch with Arena and Simple PID Driver

This is the recommended setup for testing basic navigation with obstacles and PID control.

```bash
ros2 launch tutor gz.launch.py
```

**What this does:**
- Spawns Gazebo with the arena world
- Loads the differential drive robot
- Starts the robot state publisher
- Automatically launches PID controller, goal manager, and lidar monitor
- Includes collision detection via Lidar
- Resets robot on collision

**Features:**
- 6x6 meter arena with obstacles
- Automatic goal generation and reset
- Visual markers for goals in Gazebo/RViz
- Lidar-based safety monitoring
- Episode reset on collision or timeout

**Monitoring:**
```bash
# In another terminal, launch RViz for visualization
rviz2 -d $(ros2 pkg share tutor)/rviz/config.rviz
```

---

### 3. Launch with Lidar for Calibration and Testing

Use this to verify Lidar sensor operation and obstacle detection before training.

```bash
ros2 launch tutor lidar.launch.py
```

**What this does:**
- Launches Gazebo with arena
- Loads robot with Lidar sensor
- Enables Lidar data publishing
- Runs lidar.py for processing sensor data
- Performs basic collision detection tests

**Verify Lidar is working:**
```bash
# Check Lidar scan data
ros2 topic echo /scan

# Monitor distance measurements
ros2 topic echo /lidar_distances

# View point cloud in RViz
rviz2
# Add: LaserScan -> /scan topic
# Add: PointCloud -> /scan_cloud topic
```

**Lidar Specifications:**
- Range: 0.1 - 3.5 meters
- Samples: 360 rays (1° resolution)
- Collision threshold: < 0.35m

**Troubleshooting Lidar:**
```bash
# Check Lidar is publishing
ros2 topic list | grep -i lidar

# View frequency and statistics
ros2 topic hz /scan

# Verify sensor in URDF
grep -A 20 "lidar" $(ros2 pkg share tutor)/urdf/robot.urdf.xacro
```

---

### 4. Launch Training Mode with RL

Use this to train a reinforcement learning agent to navigate autonomously.

```bash
ros2 launch tutor train.launch.py
```

This will start a separate terminal running the training script.

**Or run training directly:**
```bash
python3 $(ros2 pkg share tutor)/scripts/rl_train.py
```

**What this does:**
- Launches Gazebo with arena
- Loads robot with all sensors
- Spawns the RL environment (TutorBotEnv)
- Starts PPO training with callbacks
- Saves checkpoints periodically
- Logs training metrics (rewards, FPS, episode length)

**Training Configuration:**
Edit `scripts/rl_train.py` to adjust:
```python
# Training parameters
TOTAL_TIMESTEPS = 500_000  # Total training steps
SAVE_FREQ = 10_000         # Save checkpoint every N steps
LEARNING_RATE = 3e-4       # PPO learning rate
BATCH_SIZE = 64            # Training batch size
```

**Monitor Training Progress:**
```bash
# In another terminal, watch logs in real-time
tail -f training_logs/episode_log.txt

# Check saved models
ls -lh models/PPO/
```

**Expected Output:**
```
[INFO] Starting PPO training for 500,000 timesteps...
[Episode 1] Reward: 15.32 | Steps: 125 | FPS: 85
[Episode 2] Reward: 32.18 | Steps: 215 | FPS: 82
...
[Checkpoint] Saved model at step 10,000
```

**Training time:** ~1-2 hours for 500K timesteps on a typical GPU

---

### 5. Launch Training and Load Trained Model

Use this workflow to train and then immediately evaluate your trained model.

**Step 1: Train the model**
```bash
python3 $(ros2 pkg share tutor)/scripts/rl_train.py
```
Wait for training to complete (check terminal for completion message).

**Step 2: Benchmark the trained model**
```bash
python3 $(ros2 pkg share tutor)/scripts/load.py \
  --model models/PPO/final_model.zip \
  --stats models/PPO/vec_normalize.pkl \
  --tests 50
```

**Or use deployment mode for continuous testing:**
```bash
python3 $(ros2 pkg share tutor)/scripts/load.py \
  --model models/PPO/final_model.zip \
  --stats models/PPO/vec_normalize.pkl \
  --deploy --episodes 20
```

**Complete Training + Evaluation Workflow:**
```bash
#!/bin/bash
# train_and_eval.sh

echo "Step 1: Training RL agent..."
python3 scripts/rl_train.py

echo "Step 2: Benchmarking trained model..."
python3 scripts/load.py \
  --model models/PPO/final_model.zip \
  --stats models/PPO/vec_normalize.pkl \
  --tests 100 \
  --max-steps 500 \
  --output-dir benchmark_results

echo "Training complete! Results saved to benchmark_results/"
```

Run it:
```bash
chmod +x train_and_eval.sh
./train_and_eval.sh
```

**Load.py Options:**
```bash
# Run benchmark with specific settings
python3 scripts/load.py \
  --model <path_to_model.zip> \
  --stats <path_to_stats.pkl> \
  --tests 100                    # Run 100 test episodes
  --max-steps 500                # Max steps per episode
  --output-dir results/          # Save results here

# Deploy model for real-time inference
python3 scripts/load.py \
  --model <path_to_model.zip> \
  --deploy                       # Deployment mode
  --episodes 10                  # Run 10 episodes

# Use different algorithm (A2C, DQN)
python3 scripts/load.py \
  --model-type A2C \
  --model models/A2C/model.zip
```

**Output Files:**
```
benchmark_results/
├── benchmark_20250126-143052.csv  # Detailed episode results
└── benchmark_20250126-143052.txt  # Summary statistics
```

**CSV Format:**
```
Episode,Outcome,Final_Reward,Steps
1,WIN,95.23,127
2,WIN,98.45,112
3,CRASH,-95.12,89
4,TIMEOUT,15.67,500
...
```

**Interpret Results:**
- **WIN**: Episode completed successfully (reward ≥ 90)
- **CRASH**: Robot collided with obstacle (reward ≤ -90)
- **TIMEOUT**: Episode ended without reaching goal
- **Accuracy**: % of successful episodes = (Wins / Total) × 100

---

### Launch Comparison Table

| Scenario | Command | Gazebo | PID | Lidar | RL | Use Case |
|----------|---------|--------|-----|-------|----|---------| 
| PID Only | `controller.py` + `pid_driver.py` | ❌ | ✅ | ❌ | ❌ | Control testing |
| Arena + PID | `ros2 launch tutor gz.launch.py` | ✅ | ✅ | ✅ | ❌ | Navigation testing |
| Lidar Check | `ros2 launch tutor lidar.launch.py` | ✅ | ❌ | ✅ | ❌ | Sensor calibration |
| RL Training | `ros2 launch tutor train.launch.py` | ✅ | ❌ | ✅ | ✅ | Model training |
| Load Model | `python3 load.py` | ✅ | ❌ | ✅ | ✅ | Model evaluation |

## 📁 Project Structure

```
.
├── CMakeLists.txt                 # ROS 2 build configuration
├── package.xml                    # Package metadata
├── README.md                       # This file
├── include/tutor/                 # C++ header files (if any)
├── launch/                         # ROS 2 launch files
│   ├── gz.launch.py              # Main simulation launch
│   ├── train.launch.py           # Training-specific launch
│   └── lidar.launch.py           # Lidar sensor launch
├── urdf/
│   └── robot.urdf.xacro          # Robot description (URDF/Xacro)
├── worlds/
│   └── arena.world               # Gazebo world with obstacles
├── rviz/                          # RViz configuration files
├── scripts/                       # Python nodes and utilities
│   ├── controller.py             # Odometry and control utilities
│   ├── goal_manager.py           # Random goal spawning
│   ├── lidar.py                  # Lidar data processing
│   ├── pid_driver.py             # PID controller for navigation
│   ├── robot_env.py              # Base robot environment wrapper
│   ├── rl_env.py                 # Reinforcement learning environment
│   ├── rl_train.py               # RL training script
│   └── load.py                   # Utility for loading models/configs
└── src/                           # C++ source files (if any)
```

## 🤖 Scripts Overview

### Core Navigation Nodes
- **`controller.py`**: Handles odometry subscriptions and provides control utilities (quaternion to Euler conversions)
- **`pid_driver.py`**: Implements PID-based velocity control toward goal positions
- **`goal_manager.py`**: Manages random goal generation and visualization in Gazebo

### Environment & Sensing
- **`lidar.py`**: Processes Lidar sensor data for collision detection and distance measurements
- **`robot_env.py`**: Base environment wrapper providing robot state and sensor interfaces

### Reinforcement Learning
- **`rl_env.py`**: RL-specific environment with reward shaping and episode management
- **`rl_train.py`**: Training script for RL agents (compatible with OpenAI Gym interface)

### Utilities
- **`load.py`**: Helper functions for loading configurations and models

## 🔧 Configuration

The robot and simulation can be customized by editing:
- **Robot parameters:** `urdf/robot.urdf.xacro` (wheel radius, mass, sensor parameters)
- **Arena layout:** `worlds/arena.world` (obstacle positions, environment physics)
- **Control gains:** `scripts/pid_driver.py` (PID coefficients)
- **RL settings:** `scripts/rl_train.py` (training hyperparameters)

## 📊 RViz Visualization

Launch RViz with the included configuration:
```bash
rviz2 -d share/tutor/rviz/config.rviz
```

Monitor:
- Robot pose and odometry
- Lidar point cloud
- Goal positions
- Path trajectory

## 🧠 Reinforcement Learning Setup

The package includes RL training infrastructure for goal-reaching tasks:

```bash
python3 scripts/rl_train.py
```

The RL environment provides:
- State: robot position, goal position, Lidar distances
- Action: linear and angular velocities
- Reward: distance reduction, goal reaching, collision penalties
- Episode reset: automatic on collision or goal reached

## 🐛 Troubleshooting

**Gazebo crashes or hangs:**
- Ensure GPU drivers are properly installed
- Try running with `gazebo --verbose` for debug output

**Robot doesn't move:**
- Check that `gz.launch.py` has started all required nodes
- Verify topics with `ros2 topic list`

**Lidar not detecting obstacles:**
- Check Lidar range in `robot.urdf.xacro` (default 3.5m)
- Ensure collision distance threshold in `lidar.py` matches your use case

## 📝 License

See LICENSE file for details.

## 👤 Authors

- Andreas Sahir - [algebraic.sum19@gmail.com](mailto:algebraic.sum19@gmail.com)
- Hadis - [@hadissuryaalamin](https://github.com/hadissuryaalamin)

## 🔗 Repositories

- [hadissuryaalamin/rl-obstacle-nav-ros2](https://github.com/hadissuryaalamin/rl-obstacle-nav-ros2)
- [andreas-sahir/obstacle_avoidance-robot](https://github.com/andreas-sahir/obstacle_avoidance-robot)
