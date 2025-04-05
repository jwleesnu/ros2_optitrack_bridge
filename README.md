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

  Publishes `geometry_msgs/PoseStamped` type ROS2 topic for each tracked body.
  
- **odometry**

  Apply a linear Kalman Filter to the received pose data to estimate the linear velocity. Then, publish `nav_msgs/Odometry` type ROS2 topic for each tracked body.
  
- **px4**

   Subscribe OptiTrack pose data, converts it into `px4_msgs/VehicleOdometry` messages in NED frame, and publishes on `/fmu/in/vehicle_visual_odometry`. This allows using OptiTrack data instead of GPS in PX4. For more details, please refer to [this site](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry.html).
  
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

### px4 node

Converts OptiTrack pose into PX4 `VehicleOdometry` and publishes on `/fmu/in/vehicle_visual_odometry`.

```bash
ros2 run ros2_optitrack_bridge px4
```

- Topic: `/fmu/in/vehicle_visual_odometry` (`px4_msgs/VehicleOdometry`)

### dummy node

Publishes dummy `VehicleOdometry` messages for testing without an OptiTrack server.

```bash
ros2 run ros2_optitrack_bridge dummy
```

- Topic: `/fmu/in/vehicle_visual_odometry`

## Parameters

All nodes accept the following common parameter:

| Name         | Type   | Default   | Description                         |
| ------------ | ------ | --------- | ----------------------------------- |
| `hz`         | double | `100`     | Publishing rate (Hz)                |
| `pose_prefix`| string | `optitrack` | Prefix for per-body topics (pose) |
| `frame_id`   | string | `world`   | Frame ID for published messages     |

## Topics

### pose node

| Topic                  | Type                               | Description                   |
| ---------------------- | ---------------------------------- | ----------------------------- |
| `/<prefix><BodyName>`  | `geometry_msgs/PoseStamped`        | Raw OptiTrack body pose       |

### px4 & dummy nodes

| Topic                                    | Type                                  | Description                             |
| ---------------------------------------- | ------------------------------------- | --------------------------------------- |
| `/fmu/in/vehicle_visual_odometry`        | `px4_msgs/VehicleOdometry`            | Visual odometry for PX4 offboard control |

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
