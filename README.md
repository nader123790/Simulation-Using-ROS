# Drone Simulation with ROS, Gazebo, and Autonomous Navigation

---

## Overview

This chapter documents the design and implementation of a drone simulation environment using ROS (Robot Operating System) and Gazebo. The system integrates an Iris quadrotor model with a camera and LiDAR sensor suite, implements obstacle avoidance via laser scan processing, performs autonomous waypoint navigation, and applies YOLO-based object detection through the `darknet_ros` package.

---

## Objectives

- Configure a custom Gazebo world containing an Iris drone with attached sensors
- Integrate a camera and LiDAR (Hokuyo) sensor into the drone's SDF model
- Implement autonomous obstacle avoidance using LiDAR scan data
- Execute waypoint-based square flight paths using the `iq_gnc` API
- Enable real-time object detection via YOLO pretrained models

---

## System Description

The simulation stack is built on three primary packages:

| Package | Role |
|---|---|
| `iq_sim` | Gazebo world definition, drone SDF model, sensor configuration, launch files |
| `iq_gnc` | Guidance, Navigation, and Control — obstacle avoidance and waypoint scripts |
| `darknet_ros` | Real-time object detection using YOLO models over ROS topics |

The Iris drone model (`iris_with_standoffs_demo`) is loaded into a custom Gazebo world along with a sun and ground plane. Two sensors — a downward-facing camera and a Hokuyo LiDAR — are attached to the drone via fixed joints. The drone is controlled through MAVROS and the `gnc_api`, which abstracts FCU communication.

---

## Architecture

```
┌──────────────────────────────────────────────────┐
│                  Gazebo World                    │
│  ┌─────────────┐   ┌──────────┐  ┌───────────┐  │
│  │  Iris Model │───│  Camera  │  │   LiDAR   │  │
│  │ (SDF/URDF)  │   │ (webcam) │  │ (hokuyo)  │  │
│  └──────┬──────┘   └────┬─────┘  └─────┬─────┘  │
└─────────┼───────────────┼──────────────┼─────────┘
          │               │              │
      MAVROS          /webcam/       /spur/laser/scan
          │          image_raw             │
          ▼               │               ▼
     ┌─────────┐    ┌──────────┐   ┌───────────────┐
     │ iq_gnc  │    │darknet_  │   │  obs_avoider  │
     │ (C++/Py)│    │   ros    │   │  (Python)     │
     └─────────┘    └──────────┘   └───────────────┘
```

---

## Sensor Configuration

### Camera Sensor

The camera link is positioned at `(0, -0.01, 0.070)` with a 1.57 rad rotation. It is attached to the drone's gimbal tilt link via a revolute joint (`base_camera_joint`).

**Camera Parameters:**

| Parameter | Value |
|---|---|
| Horizontal FOV | 1.0472 rad (~60°) |
| Resolution | 640 × 480 |
| Clip Near / Far | 0.05 m / 1000 m |
| Update Rate | 10 Hz |
| ROS Plugin | `libgazebo_ros_camera.so` |
| Image Topic | `/webcam/image_raw` |

**Camera Joint:**
```xml
<joint type="revolute" name="base_camera_joint">
  <parent>iris::iris_demo::gimbal_small_2d::tilt_link</parent>
  <child>camera</child>
  <axis>
    <xyz>0 0 1</xyz>
    <use_parent_model_frame>true</use_parent_model_frame>
  </axis>
</joint>
```

---

### LiDAR Sensor (Hokuyo)

The Hokuyo LiDAR is used for obstacle detection and avoidance. It is attached to the drone's base link via a fixed joint.

**LiDAR Parameters:**

| Parameter | Value |
|---|---|
| Sensor Type | Ray |
| Scan Samples | 1024 |
| Angular Range | −π to +π (360°) |
| Min Range | 0.1 m |
| Max Range | 30 m |
| Range Resolution | 0.1 m |
| Update Rate | 10 Hz |
| ROS Plugin | `libgazebo_ros_laser.so` |
| Scan Topic | `/spur/laser/scan` |

**LiDAR Joint:**
```xml
<joint name="hokuyo_joint" type="fixed">
  <parent>iris::iris_demo::iris::base_link</parent>
  <child>hokuyo_link</child>
</joint>
```

---

## Rotor Aerodynamics Configuration

Each rotor blade is configured via the `libLiftDragPlugin.so` Gazebo plugin. Four rotors are defined (rotors 0–3), with alternating spin directions encoded via the `<cp>` and `<forward>` fields.

**Shared Blade Parameters:**

| Parameter | Value |
|---|---|
| Zero-lift AOA (`a0`) | 0.3 |
| Stall angle (`alpha_stall`) | 1.4 rad |
| Lift slope (`cla`) | 4.2500 |
| Drag coefficient (`cda`) | 0.10 |
| Blade area | 0.002 m² |
| Air density | 1.2041 kg/m³ |

Motor velocity controllers use PID with `p_gain = 0.20` and a multiplier of `±838` (sign encodes rotation direction).

---

## World Configuration

The Gazebo world uses an ODE physics solver with the following settings:

```xml
<physics type="ode">
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>
      <sor>1.0</sor>
    </solver>
    <constraints>
      <erp>0.9</erp>
    </constraints>
  </ode>
  <real_time_update_rate>-1</real_time_update_rate>
</physics>
```

The world includes `model://sun`, `model://ground_plane`, and `model://iris_with_standoffs_demo`.

**Launch command:**
```bash
roslaunch iq_sim droneOnly.launch
```

---

## Obstacle Avoidance — `obs_avoider.py`

The obstacle avoidance node subscribes to `/spur/laser/scan` and computes a repulsive velocity vector based on detected obstacles within a threshold distance.

**Algorithm Summary:**

1. For each scan ray `i`, if `range[i] < d0 (3.0 m)` and `range[i] > 0.35 m`:
   - Compute repulsive force contribution in the sensor frame
   - Accumulate `avoid_x`, `avoid_y` vectors
2. Rotate the avoidance vector from sensor frame to world frame using the drone's current heading
3. If avoidance is triggered and `dist > 3 m`, normalize the vector to magnitude 3
4. Send the avoidance goal: `drone.set_destination(avoid_x + cur_pose.x, avoid_y + cur_pose.y, 2, 0)`

```python
u = (-0.5 * k * pow((1/cr_scan.ranges[i]) - (1/d0), 2.0))
avoid_x += (x * u)
avoid_y += (y * u)
```

**Startup sequence:**
```python
drone.wait4connect()
drone.wait4start()
drone.initialize_local_frame()
drone.takeoff(2)
rospy.spin()
```

---

## Waypoint Navigation — `square.cpp`

The waypoint mission flies the drone in a square pattern at 3 m altitude using the C++ `gnc_api`.

**Waypoints:**

| # | x | y | z | ψ (°) |
|---|---|---|---|---|
| 1 | 0 | 0 | 3 | 0 |
| 2 | 5 | 0 | 3 | −90 |
| 3 | 5 | 5 | 3 | 0 |
| 4 | 0 | 5 | 3 | 90 |
| 5 | 0 | 0 | 3 | 180 |

**Run command:**
```bash
rosrun iq_gnc square.cpp
```

---

## Object Detection — `darknet_ros`

The `darknet_ros` package enables YOLO-based object detection over the `/webcam/image_raw` topic.

**Installation:**
```bash
sudo apt install nvidia-cuda-toolkit
cd ~/drone_ws/src
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
catkin build -DCMAKE_BUILD_TYPE=Release
```

Configure the image topic in `darknet_ros/config/ros.yaml`:
```yaml
camera_reading:
  topic: /webcam/image_raw
```

**YOLO Model Options:**

| Model | Notes |
|---|---|
| `yolov1` | Deprecated — not recommended |
| `yolov2` | Fast and accurate — general use |
| `yolov3` | Most accurate; high GPU RAM requirement |
| `tiny-yolo` | Fastest; recommended for edge devices (Nvidia Jetson) |

Select the model in `darknet_ros/launch/darknet_ros.launch` via the `network_param_file` argument.

---

## Technologies Used

| Technology | Purpose |
|---|---|
| ROS (Noetic/Melodic) | Middleware for sensor/actuator communication |
| Gazebo | Physics-based 3D drone simulation |
| ArduPilot / MAVROS | Flight controller interface |
| Python (rospy) | Obstacle avoidance node |
| C++ (roscpp) | Waypoint navigation script |
| SDF (XML) | Robot and world model description |
| YOLO via darknet_ros | Real-time object detection |
| CUDA (optional) | GPU acceleration for YOLO inference |
| Hokuyo LiDAR model | Simulated 2D laser ranging |

---

## Implementation Steps

1. **Build workspace** — run `catkin build` in `~/drone_ws` and source `devel/setup.bash`
2. **Launch simulation** — `roslaunch iq_sim droneOnly.launch`
3. **Verify sensors** — confirm `/webcam/image_raw` and `/spur/laser/scan` topics are active
4. **Run obstacle avoidance** — `rosrun iq_gnc obs_avoider.py`
5. **Run waypoint mission** — `rosrun iq_gnc square.cpp`
6. **Enable object detection** — launch `darknet_ros` with selected YOLO model

---

## Results

- Gazebo world successfully launches with the Iris drone, ground plane, and environmental lighting
- Camera stream published on `/webcam/image_raw` at 10 Hz with 640×480 resolution
- LiDAR scan published on `/spur/laser/scan` with 1024-sample 360° coverage
- Drone executes autonomous square flight pattern through five waypoints at 3 m altitude
- Obstacle avoidance node actively deflects the drone away from detected obstacles
- YOLO object detection runs on the camera feed and annotates detected objects in real time

---

## Challenges & Solutions

| Problem | Cause | Solution |
|---|---|---|
| LiDAR mass disturbing drone dynamics | Even small inertia values affect quadrotor balance | Set very low inertia values (`ixx = iyy = izz = 0.0001`) with explicit comment |
| Camera orientation mismatch | Default SDF frame alignment incorrect for downward-facing view | Applied `−1.57` pitch rotation in sensor pose |
| YOLO running slowly without GPU | CPU-only inference | Installed CUDA toolkit; fallback to `tiny-yolo` for low-RAM GPUs |
| Drone sluggishness under high message rate | FCU overloaded by control loop | Set `ros::Rate` to low frequency in waypoint loop |

---

## Future Improvements

- Integrate 3D point cloud sensor (e.g., VLP-16) for volumetric obstacle mapping
- Replace reactive obstacle avoidance with a global planner (RRT*, A*)
- Train a custom YOLO model on domain-specific objects (e.g., landing pads, people)
- Add GPS-denied localization using visual-inertial odometry
- Port the stack to ROS 2 with PX4 SITL for improved real-to-sim transfer

---

## Conclusion

This chapter establishes a complete, sensor-equipped drone simulation pipeline in ROS and Gazebo. The Iris quadrotor is augmented with a camera and Hokuyo LiDAR, enabling both reactive obstacle avoidance and computer vision-based object detection. The `iq_gnc` API abstracts FCU control, allowing concise Python and C++ scripts to command autonomous flight. The modular architecture cleanly separates perception, planning, and actuation, providing a robust foundation for further development in autonomous aerial robotics.
