---
sidebar_position: 1
title: 'Course Resources'
---

# Course Resources

This section contains essential resources for the Physical AI & Humanoid Robotics course, including hardware requirements, lab setup guides, and robot options.

## Hardware Requirements

### Digital Twin Workstation
- **GPU**: RTX 4070 Ti+ (or equivalent with 12GB+ VRAM)
- **CPU**: Multi-core processor (Intel i7 or AMD Ryzen 7 equivalent)
- **RAM**: 64GB or more (128GB recommended for advanced tasks)
- **Storage**: 1TB+ SSD (NVMe recommended for simulation performance)
- **OS**: Ubuntu 22.04 LTS
- **Network**: Gigabit Ethernet or better

### Physical AI Edge Kit
- **Compute**: NVIDIA Jetson Orin AGX (64GB) or equivalent
- **Camera**: Intel RealSense D435i or similar RGB-D camera
- **Audio**: ReSpeaker Mic Array v2.0 or equivalent
- **Connectivity**: WiFi 6, Bluetooth 5.0+, Gigabit Ethernet
- **Power**: 19V/65W DC or PoE+ where available

## Robot Platform Options

### Budget Option
- **Platform**: Unitree Go2 Quadruped
- **Specifications**: 12 DoF, 1.6 m/s max speed, 5kg payload
- **Use Case**: Locomotion research, outdoor navigation
- **Price**: ~$16,000-20,000

### Miniature Humanoid
- **Options**: ROBOTIS OP3 or Unitree G1 Mini
- **Specifications**: 20-32 DoF, basic manipulation capabilities
- **Use Case**: Humanoid research, HRI studies
- **Price**: ~$12,000-20,000

### Premium Option
- **Platform**: Unitree G1 Humanoid
- **Specifications**: 32 DoF, 133cm height, advanced manipulation
- **Use Case**: Full humanoid research, capstone projects
- **Price**: ~$100,000-160,000

## Software Requirements

### Core Frameworks
- **ROS 2**: Humble Hawksbill (main framework)
- **Isaac Sim**: For simulation and perception
- **Isaac ROS**: ROS 2 packages for perception and navigation
- **OpenAI Whisper**: For speech recognition
- **Large Language Model**: For command interpretation (GPT-4, Claude, etc.)
- **MoveIt**: For manipulation planning
- **Nav2**: For navigation planning

### Development Tools
- **IDE**: VS Code with ROS extensions
- **Version Control**: Git with Git LFS for large files
- **Build System**: Colcon for ROS 2 packages
- **Containerization**: Docker for environment management

## Lab Setup Guide

The lab setup guide provides detailed instructions for configuring your development environment:

1. **System Preparation**: OS setup and basic development tools
2. **ROS 2 Installation**: Core ROS 2 framework and packages
3. **Isaac Sim Setup**: Simulation environment configuration
4. **Physical Robot Integration**: Hardware and sensor setup
5. **Voice Command System**: Audio processing and recognition
6. **Testing and Validation**: Verification procedures

## Development Environment

### Recommended Configuration
- **Editor**: VS Code with ROS, Python, and C++ extensions
- **Terminal**: GNOME Terminal or similar with multiple tabs
- **File Manager**: Standard file manager with Git integration
- **Browser**: Firefox or Chrome for documentation and web tools

### Virtual Environment Setup
```bash
# Create Python virtual environment
python3 -m venv ~/robotics_env
source ~/robotics_env/bin/activate
pip install --upgrade pip

# Install core packages
pip install numpy scipy matplotlib jupyter notebook
pip install openai-whisper torch torchvision torchaudio
pip install opencv-python open3d
```

## Cloud Computing Options

For training and simulation when local hardware is insufficient:

### AWS EC2 GPU Instances
- **g5.xlarge**: 1×A10G GPU, good for small models
- **p4d.24xlarge**: 8×A100 GPUs, for large-scale training

### Google Cloud Platform
- **A2 instances**: NVIDIA A100 GPUs
- **Vertex AI**: Managed training and deployment

### Microsoft Azure
- **ND A100 v4**: NVIDIA A100 80GB GPUs
- **Azure ML**: Managed machine learning services

## Troubleshooting Resources

### Common Issues
1. **ROS 2 Communication**: Check network configuration and firewall settings
2. **Isaac Sim Performance**: Verify GPU drivers and OpenGL support
3. **Audio Input**: Check permissions and device configuration
4. **Simulation Stability**: Increase system resources or reduce simulation complexity

### Diagnostic Commands
```bash
# Check ROS 2 installation
source /opt/ros/humble/setup.bash
ros2 doctor

# Check Isaac Sim
cd ~/isaac-sim
python3 scripts/run_simple_example.py

# Check audio devices
arecord -l
```

## Documentation and References

### Official Documentation
- [ROS 2 Documentation](https://docs.ros.org/)
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [Whisper Documentation](https://github.com/openai/whisper)

### Tutorials and Examples
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [MoveIt Tutorials](https://moveit.picknik.ai/humble/doc/tutorials.html)
- [Nav2 Tutorials](https://navigation.ros.org/tutorials/)

## Community and Support

### Forums and Communities
- **ROS Answers**: For ROS 2 specific questions
- **NVIDIA Developer Forums**: For Isaac Sim and Isaac ROS
- **GitHub Issues**: For specific project repositories
- **Discord/Slack**: Course-specific communication channels

### Academic Resources
- **Research Papers**: Links to relevant robotics and AI papers
- **Conference Proceedings**: RSS, ICRA, IROS conference papers
- **University Courses**: Similar courses at other institutions
- **Open Source Projects**: Related robotics projects on GitHub

## Course Materials

### Lecture Slides and Videos
- Module-specific presentations
- Recorded lectures and demos
- Guest speaker presentations
- Industry partner showcases

### Assignments and Projects
- Weekly assignments with increasing complexity
- Group project guidelines
- Capstone project specifications
- Evaluation rubrics and deadlines

### Supplementary Materials
- Reading lists for each module
- Additional tutorials for specific topics
- Research paper summaries
- Industry best practices guides

This comprehensive resource section will help you successfully navigate the Physical AI & Humanoid Robotics course, from initial setup through final capstone project implementation.