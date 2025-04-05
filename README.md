# ros2_optitrack_bridge

A ROS 2 package providing four nodes for utilizing [OptiTrack](https://optitrack.com/cameras/) (Prime Cameras) pose data at robot system and PX4.

This package connects to an OptiTrack server (via NatNet) and transform data to suit the purpose of each node.
<br/></br>
## Prerequisites

- **ROS 2** (tested on Humble)
- **px4_msgs** & **px4_ros_com** (only for `px4` node)
<br/></br>
## List of ROS2 Executables

- **pose**

  Publishes `geometry_msgs/PoseStamped` type ROS2 topic for each tracked body. This contains **timestamp**, **position**, **orientation**(quaternion).
  
- **odometry**

  Apply a linear Kalman Filter to the received pose data to estimate the linear velocity. Then, publish `nav_msgs/Odometry` type ROS2 topic for each tracked body. This contains **timestamp**, **position**, **orientation**, **linear velocity**.
  
- **px4**

   Subscribe OptiTrack pose data, converts it into `px4_msgs/VehicleOdometry` messages in NED frame, and publishes on `/fmu/in/vehicle_visual_odometry` topic. This allows using OptiTrack data instead of GPS in PX4. For more details, please refer to [this site](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry.html).
  
- **dummy**

   Publishes dummy messages at `/fmu/in/vehicle_visual_odometry` topic for testing PX4 integration without an OptiTrack server connection.
<br/></br>
## Installation

Clone this repository into your ROS 2 workspace `src` folder:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/jwleesnu/ros2_optitrack_bridge.git
   ```

## Usage

### pose node

Publishes per-body `geometry_msgs/PoseStamped` topics from OptiTrack.

```bash
ros2 run ros2_optitrack_bridge pose
```

- Topics: `optitrack<BodyName>` (e.g., `/optitrackDrone1`)

- Structure :

  ```Yaml
  geometry_msgs/msg/PoseStamped:
  header:
    stamp:
      sec: int32
      nanosec: uint32
    frame_id: string
  pose:
    position:
      x: float64
      y: float64
      z: float64
    orientation:
      x: float64
      y: float64
      z: float64
      w: float64
  ```
  

### optitrack node

Publishes per-body `nav_msgs/Odometry` topics.

```bash
ros2 run ros2_optitrack_bridge odometry
```

- Topics: `optitrack<BodyName>` (e.g., `/optitrackDrone1`)

- Structure :

  ```Yaml
  nav_msgs/msg/Odometry:
    header:
      stamp:
        sec: int32
        nanosec: uint32
      frame_id: string
    child_frame_id: string
    pose:
      pose:
        position:
          x: float64
          y: float64
          z: float64
        orientation:
          x: float64
          y: float64
          z: float64
          w: float64
      covariance: 
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    twist:
      twist:
        linear:
          x: float64
          y: float64
          z: float64
        angular:
          x: 0.0
          y: 0.0
          z: 0.0
      covariance: 
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  ```

### px4 node

Converts OptiTrack pose into PX4 `VehicleOdometry` message and publishes on `/fmu/in/vehicle_visual_odometry`.

```bash
ros2 run ros2_optitrack_bridge px4
```

- Topic: `/fmu/in/vehicle_visual_odometry` (`px4_msgs/VehicleOdometry` type)

- Structure :

  ```Yaml
  px4_msgs/msg/VehicleOdometry:
    timestamp: uint64
    timestamp_sample: uint64
    pose_frame: 1
    position: 
      - float32  # x
      - float32  # y
      - float32  # z
    q: 
      - float32  # x
      - float32  # y
      - float32  # z
      - float32  # w
    velocity_frame: 1
    velocity:
      - NaN
      - NaN
      - NaN
    angular_velocity:
      - NaN
      - NaN
      - NaN
    position_variance:
      - NaN
      - NaN
      - NaN
    orientation_variance:
      - NaN
      - NaN
      - NaN
    velocity_variance:
      - NaN
      - NaN
      - NaN
    reset_counter: uint8
    quality: int8
  ```
### dummy node

Publishes dummy `VehicleOdometry` messages for testing without an OptiTrack server.

```bash
ros2 run ros2_optitrack_bridge dummy
```

- Topic: `/fmu/in/vehicle_visual_odometry`

- Structure :

  ```Yaml
  px4_msgs/msg/VehicleOdometry:
    timestamp: uint64
    timestamp_sample: uint64
    pose_frame: 1
    position: 
      - 1.0
      - -2.0
      - -3.0
    q: 
      - 0.0
      - 0.0
      - 0.0
      - 1.0
    velocity_frame: 1
    velocity:
      - NaN
      - NaN
      - NaN
    angular_velocity:
      - NaN
      - NaN
      - NaN
    position_variance:
      - NaN
      - NaN
      - NaN
    orientation_variance:
      - NaN
      - NaN
      - NaN
    velocity_variance:
      - NaN
      - NaN
      - NaN
    reset_counter: 0
    quality: 0
  ```

