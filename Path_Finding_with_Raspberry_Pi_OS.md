# ROS 2 Humble Drone with TFmini-S LiDAR Wall Detection and MAVROS Control on Raspberry Pi OS

**Platform:** Raspberry Pi OS (64-bit)
**Purpose:** SLAM mapping, wall detection using TFmini-S LiDAR, and motor control via Pixhawk

---

## 1️⃣ System Requirements & Setup

### Hardware

* Raspberry Pi 4/5 (Raspberry Pi OS 64-bit)
* Pixhawk flight controller
* TFmini-S LiDAR (UART)
* Camera (optional for SLAM)
* Drone frame + motors

### Software Prerequisites

* Raspberry Pi OS (64-bit)
* ROS 2 Humble installed
* Python 3.10+
* MAVROS for Pixhawk
* Python serial library (`pyserial`) for TFmini-S

---

## 2️⃣ Install ROS 2 Humble on Raspberry Pi OS

```bash
# Update system
sudo apt update && sudo apt upgrade -y
sudo apt install curl gnupg2 lsb-release software-properties-common -y

# Add ROS 2 apt repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

sudo apt update

# Install ROS 2 Humble desktop
sudo apt install ros-humble-desktop -y

# Source ROS 2 setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify Installation:**

```bash
ros2 --version
```

---

## 3️⃣ Install MAVROS

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras -y

# Install geographiclib datasets required by MAVROS
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```

---

## 4️⃣ Setup ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create the main package
ros2 pkg create --build-type ament_python drone_controller
```

---

## 5️⃣ TFmini-S Node (Reads LiDAR via UART & publishes ROS 2 Range)

**File:** `drone_controller/drone_controller/tfmini_node.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial

class TFminiNode(Node):
    def __init__(self):
        super().__init__('tfmini_node')
        self.publisher_ = self.create_publisher(Range, '/tfmini/range', 10)
        # Adjust serial port to Raspberry Pi UART, e.g., /dev/serial0
        self.ser = serial.Serial('/dev/serial0', 115200, timeout=1)
        self.timer = self.create_timer(0.05, self.read_data)  # 20 Hz

    def read_data(self):
        data = self.ser.read(9)
        if len(data) != 9:
            return
        if data[0] != 0x59 or data[1] != 0x59:
            return
        low = data[2]
        high = data[3]
        distance = (high << 8) + low
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.05
        msg.min_range = 0.3
        msg.max_range = 12.0
        msg.range = distance / 100.0
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TFminiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Dependencies:**

```bash
sudo apt install python3-serial ros-humble-sensor-msgs
```

---

## 6️⃣ Wall Avoidance Node (Subscribes to TFmini + /scan + MAVROS pose)

**File:** `drone_controller/drone_controller/wall_avoider_node.py`

```python
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile

class WallAvoider(Node):
    def __init__(self):
        super().__init__('wall_avoider')
        self.declare_parameter('front_threshold', 2.0)
        self.declare_parameter('side_threshold', 1.2)
        self.declare_parameter('step_distance', 1.0)
        self.declare_parameter('back_distance', 0.8)
        self.declare_parameter('hover_alt', 2.0)
        self.declare_parameter('setpoint_rate', 20.0)

        self.front_threshold = self.get_parameter('front_threshold').value
        self.side_threshold = self.get_parameter('side_threshold').value
        self.step_distance = self.get_parameter('step_distance').value
        self.back_distance = self.get_parameter('back_distance').value
        self.hover_alt = self.get_parameter('hover_alt').value
        self.rate_hz = self.get_parameter('setpoint_rate').value

        self.latest_scan = None
        self.latest_pose = None
        self.latest_tfmini = None

        qos = QoSProfile(depth=10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos)
        self.tfmini_sub = self.create_subscription(Range, '/tfmini/range', self.tfmini_cb, qos)

        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_cb)

    def scan_cb(self, msg):
        self.latest_scan = msg

    def pose_cb(self, msg):
        self.latest_pose = msg

    def tfmini_cb(self, msg):
        self.latest_tfmini = msg.range

    def timer_cb(self):
        if self.latest_pose is None:
            return
        pose = self.latest_pose
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = pose.header.frame_id
        target.pose.position.x = pose.pose.position.x
        target.pose.position.y = pose.pose.position.y
        target.pose.position.z = self.hover_alt
        target.pose.orientation = pose.pose.orientation

        wall_detected = False
        if self.latest_scan:
            mid_index = len(self.latest_scan.ranges) // 2
            forward = [r for r in self.latest_scan.ranges[mid_index-5:mid_index+5] if r>0]
            if forward and min(forward) < self.front_threshold:
                wall_detected = True

        if self.latest_tfmini and self.latest_tfmini < self.front_threshold:
            wall_detected = True

        if wall_detected:
            left_clear = right_clear = True
            if self.latest_scan:
                left_min = min([r for r in self.latest_scan.ranges[len(self.latest_scan.ranges)//2+10:len(self.latest_scan.ranges)//2+30] if r>0], default=10)
                right_min = min([r for r in self.latest_scan.ranges[len(self.latest_scan.ranges)//2-30:len(self.latest_scan.ranges)//2-10] if r>0], default=10)
                left_clear = left_min > self.side_threshold
                right_clear = right_min > self.side_threshold

            if left_clear or right_clear:
                if left_clear and right_clear:
                    side = 'left' if left_min >= right_min else 'right'
                elif left_clear:
                    side = 'left'
                else:
                    side = 'right'

                if side == 'left':
                    target.pose.position.y += self.step_distance
                else:
                    target.pose.position.y -= self.step_distance
            else:
                target.pose.position.x -= self.back_distance

        self.setpoint_pub.publish(target)


def main(args=None):
    rclpy.init(args=args)
    node = WallAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 7️⃣ Setup Package Entry Points

Edit `setup.py` in `drone_controller`:

```python
entry_points={
    'console_scripts': [
        'tfmini_node = drone_controller.tfmini_node:main',
        'wall_avoider = drone_controller.wall_avoider_node:main',
    ],
},
```

---

## 8️⃣ Build Workspace

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 9️⃣ Running the System

### Start MAVROS

```bash
ros2 launch mavros mavros_launch.py fcu_url:=/dev/ttyACM0:57600
```

### Start SLAM (optional if you have camera)

```bash
ros2 launch rtabmap_ros rtabmap.launch.py
```

### Start TFmini Node

```bash
ros2 run drone_controller tfmini_node
```

### Start Wall Avoider Node

```bash
ros2 run drone_controller wall_avoider
```

---

## 🔹 Notes & Safety

* Ensure Pixhawk is in **OFFBOARD mode** before flying.
* Test with **props removed** first.
* Verify `/tfmini/range` values in ROS topic before flight.
* Tune thresholds (`front_threshold`, `side_threshold`, `step_distance`) for your drone speed and environment.
* Ensure serial port (`/dev/serial0`) matches your connection to TFmini-S.

---

## ✅ Outcome

* Drone can **detect walls** with TFmini-S + LaserScan.
* It **sidesteps left/right** or **backs up** automatically.
* **SLAM + TFmini-S** combined improves **collision avoidance**.
* Fully **documented and reusable** on Raspberry Pi OS.
