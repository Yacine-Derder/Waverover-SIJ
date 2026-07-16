# ros2waverover

This is a ROS 2 UART bridge for Waveshare Wave Rover.

It subscribes to namespaced `cmd_vel`, sends JSON velocity commands to the
rover, reads UART feedback, and publishes namespaced raw IMU data.

## Control modes

The launch file reads defaults from
`waverover/config/robot_defaults.yaml`, reads per-machine `robot_name` from
the ignored identity file, and derives `/waverover_<ID>`. The
bridge uses relative topic names, so robot 29 subscribes to
`/waverover_29/cmd_vel`, publishes `/waverover_29/imu/data_raw`, and uses
`/waverover_29/manual_lr` in manual mode.

The default `control_mode` is `fixed_wing`. The same parameter name is used by
the waypoint controller.

`fixed_wing` maps teleop-style Twist commands to straight `(0.25, 0.25)`, bank
left `(-0.1, 0.5)`, and bank right `(0.5, -0.1)`. A zero or otherwise
non-turning Twist commands straight. An explicit stop marker (`angular.x: 1.0`,
all other Twist fields zero) maps to stop `(0.0, 0.0)` for waypoint waiting, TF
failure, and shutdown.

The bridge also skips its destructor emergency-stop while in `fixed_wing` mode,
so shutting down the bridge itself does not introduce another stopping path.
Destructor emergency-stop behavior remains enabled in `twist` and `manual_lr`.

`manual_lr` subscribes to relative topic `manual_lr` as
`std_msgs/msg/Float32MultiArray` containing `[left, right]`. Values are sent
directly as WaveRover JSON wheel speeds and clamped to `[-0.5, 0.5]`.

Run the bridge in manual mode:
```
ros2 launch ros2waverover wave_rover_launch.py \
  robot_name:=29 control_mode:=manual_lr
```

Publish one manual command:
```
ros2 topic pub --once /waverover_29/manual_lr \
  std_msgs/msg/Float32MultiArray "{data: [0.3, 0.7]}"
```

Run the SSH-friendly terminal UI:
```
ros2 run waverover waverover_manual_lr
```

The wrapper derives the namespace and UI tuning from the central stack
configuration. Use `--robot-name 30` for a one-run override.

UI controls: `a/z` left up/down, `k/m` right up/down, `w/s` both up/down, uppercase for larger steps, `0` or space to reset, and `q` to send zero and quit. The UI defaults to `[-0.5, 0.5]` for calibration.

In `manual_lr` mode the bridge sends zero speeds on startup and zeros the wheels if no manual command arrives before `manual_lr_timeout_sec`.

## Requirements
If you don't use docker. Otherwise, just install docker.
```
sudo apt install -y qtcreator qtbase5-dev qt5-qmake cmake libqt5serialport5-dev
```

## Run the ROS bridge directly with a docker image on the robot
```
docker run --privileged -v /dev/ttyS0:/dev/ttyS0 -it --rm ros2waverover ros2 launch ros2waverover wave_rover_launch.py UART_address:=/dev/ttyS0
```
If you add `--restart=always`, it will start when your robot starts.

## Docker build - native or raspberry pi

### QEMU if you are cross compiling
```
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

### The actual build
(remove the platform you don't need, or not)
```
docker buildx build \
          --push \
          --tag whatever-you-chose \
          --platform linux/amd64,linux/arm64 .
```

## Debugging setup

### Create a serial port, for testing purpose
```
socat -v -d -d PTY,raw,echo=0,b115200,cs8 PTY,raw,echo=0,b115200,cs8
```

### For the real one, to test it:
```
stty -F /dev/ttyUSB0 1000000 # set the baud rate
cat /dev/ttyUSB0 # will receive commands that are sent from the robot
echo -ne '{"T":1, "L":0.2, "R":0.2}' > /dev/ttyUSB0 # will send commands
```

### To create the udev rule
```
sudo cp 99-waverover.rules /etc/udev/rules.d/99-waverover.rules
```

### Give the rights to the serial port
The easy way:

sudoedit /etc/udev/rules.d/50-myusb.rules

Save this text:
```
KERNEL=="ttyUSB[0-9]*",MODE="0666"
KERNEL=="ttyACM[0-9]*",MODE="0666"
```
Then unplug and replug the device.

### Send messages manually
Send a twist message
```
ros2 topic pub /waverover_29/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```
