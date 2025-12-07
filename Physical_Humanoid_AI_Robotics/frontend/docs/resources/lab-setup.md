---
sidebar_position: 2
title: 'Lab Setup Guide'
---

# Lab Setup Guide

This guide provides detailed instructions for setting up the laboratory environment for the Physical AI & Humanoid Robotics course, including both simulation and physical robot configurations.

## Prerequisites

Before beginning the lab setup, ensure you have:

### Software Prerequisites
- **Operating System**: Ubuntu 22.04 LTS (recommended)
- **Disk Space**: Minimum 100GB free space (200GB+ recommended)
- **Internet Connection**: Stable broadband connection for downloads
- **User Permissions**: Administrative access for installation
- **SSH Keys**: Set up for GitHub/GitLab access

### Hardware Prerequisites
- Workstation meeting [Hardware Requirements](./hardware-requirements)
- Robot platform (simulated or physical)
- Network infrastructure with DHCP
- Power outlets and surge protection

## Development Environment Setup

### 1. System Preparation

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install basic development tools
sudo apt install build-essential cmake git curl wget gnupg lsb-release \
                 software-properties-common python3-dev python3-pip \
                 python3-colcon-common-extensions python3-rosdep \
                 python3-vcstool libasio-dev libtinyxml2-dev \
                 libcunit1-dev libopencv-dev pkg-config \
                 unzip zip build-essential libssl-dev \
                 libbz2-dev libreadline-dev libsqlite3-dev \
                 libncursesw5-dev xz-utils tk-dev libxml2-dev \
                 libxmlsec1-dev libffi-dev liblzma-dev \
                 clang-format cppcheck

# Install pip packages
pip3 install setuptools vcstool rospkg numpy empy pyyaml lark \
             defusedxml netifaces cryptography flask \
             flask-cors websocket-client tornado \
             transforms3d psutil

# Install Node.js for web interfaces
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install -y nodejs
```

### 2. ROS 2 Installation

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop-full
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Isaac Sim Installation

```bash
# Install NVIDIA drivers (if not already installed)
sudo apt install nvidia-driver-535

# Install Isaac Sim prerequisites
sudo apt install mesa-utils libgl1-mesa-glx libglib2.0-0 libsm6 libxext6 libxrender-dev libgomp1

# Download and install Isaac Sim
# Visit https://developer.nvidia.com/isaac-sim and download the latest version
# Extract to home directory (~/isaac-sim)

# Set up Isaac Sim environment
echo "export ISAACSIM_PATH=$HOME/isaac-sim" >> ~/.bashrc
echo "export PYTHONPATH=$ISAACSIM_PATH/python:$PYTHONPATH" >> ~/.bashrc
source ~/.bashrc

# Install Isaac Sim Python dependencies
cd ~/isaac-sim
python3 -m pip install -e .
```

### 4. Isaac ROS Setup

```bash
# Install Isaac ROS dependencies
sudo apt install ros-humble-isaac-ros-dev

# Install specific Isaac ROS packages
sudo apt install ros-humble-isaac-ros-visual-slam
sudo apt install ros-humble-isaac-ros-apriltag
sudo apt install ros-humble-isaac-ros-dnn-inference
sudo apt install ros-humble-isaac-ros-segment-any-thing
sudo apt install ros-humble-isaac-ros-point-cloud
sudo apt install ros-humble-isaac-ros-image-pipeline
```

### 5. Development Workspace Setup

```bash
# Create ROS workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Install colcon tools
pip3 install colcon-ros-bundle colcon-bundle-tools

# Initialize workspace
colcon build --symlink-install

# Source workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/ros2_ws/install/setup.bash
```

## Simulation Environment Configuration

### 1. Isaac Sim Configuration

```bash
# Create configuration directory
mkdir -p ~/isaac-sim-config

# Create basic simulation configuration
cat > ~/isaac-sim-config/default_config.usd << 'EOF'
#usda 1.0
(
    doc = "Default Isaac Sim configuration for robotics simulation"
    metersPerUnit = 1.0
    upAxis = "Y"
)

def Xform "World"
{
    def Xform "GroundPlane"
    {
        add references = @./assets/ground_plane.usd@
    }

    def Xform "Robot"
    {
        add references = @./assets/robot_model.usd@
        prepend apiSchemas = ["PhysicsSchemaAPI"]
    }

    def Xform "Environment"
    {
        add references = @./assets/office_environment.usd@
    }
}
EOF
```

### 2. Navigation Simulation Setup

```bash
# Create navigation simulation world
mkdir -p ~/ros2_ws/src/navigation_simulation/worlds
cat > ~/ros2_ws/src/navigation_simulation/worlds/simple_office.world << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_office">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Office environment -->
    <model name="wall_1">
      <pose>0 5 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2.5</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Add furniture and obstacles -->
    <model name="table_1">
      <pose>2 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.5 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.5 0.8 0.8</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
EOF
```

### 3. Perception Simulation Setup

```bash
# Create perception simulation configuration
mkdir -p ~/ros2_ws/src/perception_simulation/config
cat > ~/ros2_ws/src/perception_simulation/config/sensors.yaml << 'EOF'
# Sensor configuration for simulation
camera:
  type: "rgb"
  topic: "/camera/rgb/image_raw"
  frame_id: "camera_rgb_optical_frame"
  update_rate: 30
  resolution:
    width: 640
    height: 480
  distortion:
    k1: 0.0
    k2: 0.0
    p1: 0.0
    p2: 0.0
    k3: 0.0

depth_camera:
  type: "depth"
  topic: "/camera/depth/image_raw"
  frame_id: "camera_depth_optical_frame"
  update_rate: 30
  resolution:
    width: 640
    height: 480

lidar:
  type: "ray"
  topic: "/scan"
  frame_id: "lidar_frame"
  update_rate: 10
  scan:
    horizontal:
      samples: 720
      resolution: 1
      min_angle: -3.14159
      max_angle: 3.14159
    vertical:
      samples: 1
      resolution: 1
      min_angle: 0
      max_angle: 0
  range:
    min: 0.1
    max: 30.0
    resolution: 0.01
EOF
```

## Physical Robot Setup

### 1. Jetson Orin Configuration

```bash
# SSH into Jetson Orin device
ssh jetson@<jetson_ip_address>

# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install ros-humble-ros-base
sudo apt install python3-rosinstall python3-rosdep

# Source ROS environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Sensor Integration

```bash
# Install camera drivers
sudo apt install v4l-utils

# Test camera
v4l2-ctl --list-devices

# Install RealSense drivers
sudo apt install ros-humble-realsense2-camera

# Test RealSense camera
roslaunch realsense2_camera rs_camera.launch
```

### 3. Network Configuration

```bash
# Configure static IP for robot
sudo nano /etc/netplan/01-network-manager-all.yaml

# Add configuration:
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: false
      addresses:
        - 192.168.1.100/24
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]

# Apply configuration
sudo netplan apply

# Verify configuration
ip addr show eth0
```

## Voice Command System Setup

### 1. Audio System Configuration

```bash
# Install audio packages
sudo apt install pulseaudio alsa-utils pavucontrol

# Test microphone
arecord -d 5 -f cd test.wav
aplay test.wav

# Install Whisper for speech recognition
pip3 install openai-whisper
pip3 install sounddevice pyaudio
```

### 2. Audio Configuration for Robotics

```bash
# Create audio configuration
cat > ~/.config/pulse/daemon.conf << 'EOF'
default-sample-rate = 16000
alternate-sample-rate = 44100
resample-method = speex-fixed-7
high-priority = yes
nice-level = -11
realtime-scheduling = yes
realtime-priority = 5
rlimit-rtprio = 99
rlimit-rttime = 200000
EOF

# Restart PulseAudio
pulseaudio -k
pulseaudio --start
```

## Development Tools Setup

### 1. IDE Configuration

```bash
# Install VS Code
wget -qO - https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
sudo install -o root -g root -m 644 microsoft.gpg /usr/share/keyrings/microsoft.gpg
sudo sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'

sudo apt update
sudo apt install code

# Install ROS extension
code --install-extension ms-iot.vscode-ros
code --install-extension ms-python.python
```

### 2. Git Configuration for Robotics

```bash
# Configure Git for robotics development
git config --global user.name "Robotics Developer"
git config --global user.email "robotics@example.com"
git config --global core.editor "code --wait"
git config --global merge.tool "vscode"
git config --global mergetool.vscode.cmd 'code --wait $MERGED'

# Install Git extensions
sudo apt install git-lfs

# Configure Git LFS for large binary files
git lfs install
git lfs track "*.usd" "*.usda" "*.usdc" "*.fbx" "*.dae" "*.stl" "*.obj"
```

## Testing and Validation

### 1. Basic ROS 2 Test

```bash
# Test ROS 2 installation
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener &
sleep 5
killall talker listener
```

### 2. Isaac Sim Test

```bash
# Test Isaac Sim
cd ~/isaac-sim
python3 scripts/run_simple_example.py
```

### 3. Navigation Test

```bash
# Test navigation stack
source ~/ros2_ws/install/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

### 4. Voice Command Test

```bash
# Test voice command pipeline
# Create test script
cat > ~/test_voice.py << 'EOF'
import whisper
import pyaudio
import numpy as np

# Load model
model = whisper.load_model("tiny.en")

# Audio setup
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000

p = pyaudio.PyAudio()

stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)

print("Recording... Speak now.")
frames = []

for i in range(0, int(RATE / CHUNK * 5)):  # Record for 5 seconds
    data = stream.read(CHUNK)
    frames.append(data)

print("Finished recording.")

stream.stop_stream()
stream.close()
p.terminate()

# Convert to numpy array
audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
audio_float = audio_data.astype(np.float32) / 32768.0

# Transcribe
result = model.transcribe(audio_float)
print(f"Transcribed: {result['text']}")
EOF

python3 ~/test_voice.py
```

## Troubleshooting

### Common Issues and Solutions

#### ROS 2 Issues
```bash
# Issue: Cannot find ROS packages
# Solution: Source the correct setup files
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Issue: Permission denied for serial devices
# Solution: Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect
```

#### Isaac Sim Issues
```bash
# Issue: Isaac Sim won't start
# Solution: Check NVIDIA driver and OpenGL
nvidia-smi
glxinfo | grep "OpenGL renderer"

# Issue: Poor performance
# Solution: Optimize Isaac Sim settings
export ISAACSIM_HEADLESS=0
export ISAACSIM_SKIP_NUCLEUS_LOGIN=1
```

#### Audio Issues
```bash
# Issue: No audio input detected
# Solution: Check audio device and permissions
arecord -l
sudo usermod -a -G audio $USER

# Issue: High latency
# Solution: Configure real-time audio
echo 'default-fragments = 8' >> ~/.config/pulse/daemon.conf
echo 'default-fragment-size-msec = 10' >> ~/.config/pulse/daemon.conf
```

## Maintenance and Updates

### 1. Regular Maintenance Script

```bash
# Create maintenance script
cat > ~/robotics_maintenance.sh << 'EOF'
#!/bin/bash

echo "Starting robotics environment maintenance..."

# Update system packages
sudo apt update && sudo apt upgrade -y

# Clean up ROS logs
rm -rf ~/.ros/log/*

# Clean up Isaac Sim cache
rm -rf ~/isaac-sim/cache/*

# Update pip packages
pip3 list --outdated --format=freeze | grep -v '^\-e' | cut -d = -f 1 | xargs -n1 pip3 install -U

# Check disk space
df -h

echo "Maintenance completed!"
EOF

chmod +x ~/robotics_maintenance.sh
```

### 2. Backup Strategy

```bash
# Create backup script
cat > ~/backup_robotics_env.sh << 'EOF'
#!/bin/bash

BACKUP_DIR="$HOME/robotics_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"

# Backup configurations
cp -r ~/.bashrc ~/.profile ~/isaac-sim-config "$BACKUP_DIR/"

# Backup ROS workspace (excluding build artifacts)
cp -r ~/ros2_ws/src "$BACKUP_DIR/ros2_ws_src"

# Backup custom code
cp -r ~/custom_robotics_projects "$BACKUP_DIR/" 2>/dev/null || echo "No custom projects found"

echo "Backup created at: $BACKUP_DIR"
EOF

chmod +x ~/backup_robotics_env.sh
```

## Next Steps

After completing this lab setup:

1. **Verify Installation**: Run all test commands to confirm proper setup
2. **Module 1 Practice**: Begin with ROS 2 fundamentals exercises
3. **Simulation Testing**: Test navigation and perception in Isaac Sim
4. **Voice Integration**: Experiment with voice command processing
5. **Documentation**: Review the course documentation and examples

Your laboratory environment is now ready for the Physical AI & Humanoid Robotics course. Proceed with the course modules in sequence, starting with Module 1: ROS 2 - The Robotic Nervous System.