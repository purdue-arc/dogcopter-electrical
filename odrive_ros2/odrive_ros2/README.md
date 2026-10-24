# odrive_ros2 — ROS2 Humble + ODrive on NVIDIA Jetson

A minimal, production-ready ROS2 Humble package that lets an NVIDIA Jetson
drive a brushless motor through an ODrive controller over USB.

---

## Prerequisites

### 1. ROS2 Humble on Jetson
Follow the official ARM64 install guide:
```
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
```

### 2. ODrive Python library
```bash
pip3 install odrive
```

### 3. USB permissions (required on Jetson / Ubuntu)
```bash
sudo bash -c "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules \
  > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"
```
Re-plug the ODrive USB cable after running the above.

### 4. (ODrive v3.x only) Run calibration once before using this node
```bash
odrivetool
# Inside odrivetool:
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# Wait until idle, then save
odrv0.save_configuration()
```
ODrive Pro / S1 can skip manual calibration if using absolute encoders.

---

## Build

```bash
mkdir -p ~/ros2_ws/src
cp -r odrive_ros2 ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select odrive_ros2
source install/setup.bash
```

---

## Run

```bash
ros2 launch odrive_ros2 odrive.launch.py
```

---

## Commanding the motor

### Velocity (turns/second)
```bash
ros2 topic pub /odrive/velocity_cmd std_msgs/Float64 "data: 2.0"
```

### Position (turns)
```bash
ros2 topic pub /odrive/control_mode std_msgs/String "data: position"
ros2 topic pub /odrive/position_cmd std_msgs/Float64 "data: 5.0"
```

### Torque (Nm)
```bash
ros2 topic pub /odrive/control_mode std_msgs/String "data: torque"
ros2 topic pub /odrive/torque_cmd std_msgs/Float64 "data: 0.1"
```

### Emergency stop
```bash
ros2 topic pub /odrive/estop std_msgs/Bool "data: true"
# Clear estop and re-arm:
ros2 topic pub /odrive/estop std_msgs/Bool "data: false"
```

---

## Monitor feedback
```bash
ros2 topic echo /odrive/status
ros2 topic echo /odrive/velocity
ros2 topic echo /odrive/position
```

---

## Tuning parameters
Edit `config/odrive_params.yaml` (or pass via launch args):

| Parameter             | Default | Description                        |
|-----------------------|---------|------------------------------------|
| `axis_index`          | 0       | ODrive axis (0 or 1)               |
| `control_mode`        | velocity| Starting mode                      |
| `vel_limit`           | 10.0    | Max velocity (turns/s)             |
| `current_limit`       | 10.0    | Phase current limit (A)            |
| `vel_ramp_rate`       | 2.0     | Velocity ramp rate (turns/s²)      |
| `publish_rate_hz`     | 50.0    | Feedback publish rate              |
| `odrive_serial_number`| ""      | Hex serial, blank = first found    |
| `reconnect_on_error`  | true    | Auto clear non-fatal errors        |

---

## Troubleshooting

**"ODrive not found"**  
- Check `lsusb | grep ODrive`; if missing, check USB cable and power.  
- Re-run udev rules and replug.

**Axis stays in error state / won't arm**  
- Run `odrivetool` and check `odrv0.axis0.error` / `odrv0.axis0.motor.error`.  
- Most common cause: calibration not done, or encoder not detected.

**Permission denied on /dev/ttyUSBx**  
- Add yourself to the `dialout` group: `sudo usermod -aG dialout $USER` then log out/in.

---

## Package layout

```
odrive_ros2/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── odrive_params.yaml
├── launch/
│   └── odrive.launch.py
└── scripts/
    └── odrive_node.py          ← main ROS2 node
```
