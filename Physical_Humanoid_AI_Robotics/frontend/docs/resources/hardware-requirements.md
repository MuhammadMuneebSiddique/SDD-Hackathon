---
sidebar_position: 1
title: 'Hardware Requirements'
---

# Hardware Requirements

This section details the hardware requirements for the Physical AI & Humanoid Robotics course, including different options for various budgets and capabilities.

## Digital Twin Workstation

For developing and testing the robotics system in simulation before deployment to physical hardware.

### Minimum Requirements
- **GPU**: NVIDIA RTX 4070 (12GB VRAM)
- **CPU**: Intel i7-12700K or AMD Ryzen 7 5800X
- **RAM**: 32GB DDR4-3200
- **Storage**: 1TB NVMe SSD
- **OS**: Ubuntu 22.04 LTS
- **Network**: Gigabit Ethernet

### Recommended Configuration
- **GPU**: NVIDIA RTX 4080/4090 (16-24GB VRAM) or RTX 6000 Ada Generation
- **CPU**: Intel i9-13900K or AMD Ryzen 9 7950X
- **RAM**: 64GB DDR5-5200
- **Storage**: 2TB NVMe SSD (PCIe 4.0)
- **OS**: Ubuntu 22.04 LTS
- **Network**: 2.5GbE or higher

### High-Performance Option
- **GPU**: NVIDIA RTX 6000 Ada Generation (48GB) or dual RTX 4090
- **CPU**: AMD Threadripper PRO 5975WX (32 cores) or Intel Xeon
- **RAM**: 128GB ECC DDR5
- **Storage**: 4TB+ NVMe SSD RAID 0
- **Cooling**: Liquid cooling system
- **PSU**: 1600W+ Gold rated

## Physical AI Edge Kit

For deployment of trained models and real-world testing of the humanoid robot system.

### Compute Platform
- **Primary**: NVIDIA Jetson Orin AGX (64GB) or Orin Ultra
  - 2048 CUDA cores
  - 64GB LPDDR5 memory
  - 32 TOPS AI performance (INT8)
  - 275 TOPS with DLAs
  - Power: 15-60W (configurable)

- **Alternative**: NVIDIA Jetson Orin NX (8GB)
  - 1024 CUDA cores
  - 8GB LPDDR5 memory
  - 73 TOPS AI performance (INT8)
  - Power: 10-25W (configurable)

### Sensors and Perception
- **Camera**: Intel RealSense D435i or D455
  - RGB: 1920×1080 @ 30fps
  - Depth: 1280×720 @ 90fps
  - IMU: Accelerometer and gyroscope
  - Connectivity: USB 3.0

- **Alternative Camera**: Stereolabs ZED 2i
  - 2K resolution at 30fps
  - Built-in IMU
  - 3D depth sensing

### Audio System
- **Microphone**: ReSpeaker Mic Array v2.0 (6-mic circular array)
  - Far-field voice capture
  - Acoustic echo cancellation
  - USB connectivity

- **Speakers**: Small form-factor stereo speakers
  - 2×3W or higher
  - USB or 3.5mm jack connectivity

### Connectivity and Power
- **WiFi**: WiFi 6 (802.11ax) for high-bandwidth communication
- **Bluetooth**: 5.0+ for peripheral devices
- **Ethernet**: Gigabit Ethernet for stable communication
- **Power**: 19V/65W DC barrel connector or PoE+ where available

## Robot Platforms

Various robot options to suit different budgets and capabilities for the course.

### Budget Option: Unitree Go2 Quadruped
- **Type**: Quadruped robot
- **Price**: ~$16,000-20,000
- **Features**:
  - 12 DoF (3 DoF per leg)
  - Max speed: 1.6 m/s
  - Payload: 5kg
  - Battery life: 2+ hours
  - IP65 protection rating
  - ROS 2 support
  - SDK for custom applications

### Miniature Humanoid: ROBOTIS OP3 or Unitree G1 Mini
#### ROBOTIS OP3
- **Type**: Small humanoid (42cm tall)
- **Price**: ~$12,000-15,000
- **Features**:
  - 20 DoF (12 for legs, 6 for arms, 2 for head)
  - Open-source software
  - ROS 1/2 support
  - Vision and speech capabilities
  - Modular design

#### Unitree G1 Mini
- **Type**: Small humanoid
- **Price**: ~$16,000-20,000
- **Features**:
  - 32 DoF
  - Walking and balance capabilities
  - ROS 2 support
  - Built-in perception systems

### Premium Option: Unitree G1 Humanoid
- **Type**: Full-size humanoid
- **Price**: ~$100,000-160,000
- **Features**:
  - 32 DoF
  - Height: 133cm, Weight: 35kg
  - Advanced walking and balance
  - High payload capacity
  - Long battery life (2+ hours)
  - ROS 2 support
  - Advanced perception suite
  - Cloud connectivity

### Alternative Premium: Boston Dynamics Atlas (Research)
- **Type**: Advanced humanoid
- **Price**: ~$100,000+ (research licenses)
- **Features**:
  - Hydraulic actuation
  - Advanced mobility
  - Complex manipulation
  - Cloud-connected AI

## Cloud Computing Options

For training and simulation when local hardware is insufficient.

### AWS EC2 GPU Instances
- **g5.xlarge**: 1×A10G GPU, 4 vCPUs, 16GB RAM - Good for small models
- **g5.2xlarge**: 1×A10G GPU, 8 vCPUs, 32GB RAM - Good for medium models
- **g5.12xlarge**: 4×A10G GPUs, 48 vCPUs, 192GB RAM - Good for large models
- **p4d.24xlarge**: 8×A100 GPUs, 96 vCPUs, 1152GB RAM - For large-scale training

### Google Cloud Platform
- **A2 instances**: NVIDIA A100 GPUs
- **A100-80GB instances**: For memory-intensive workloads
- **Vertex AI**: Managed training and deployment

### Microsoft Azure
- **ND A100 v4**: NVIDIA A100 80GB GPUs
- **NCv3**: NVIDIA V100 GPUs
- **Azure ML**: Managed machine learning services

## Recommended Setup by Use Case

### Academic Research Lab
- **Workstation**: RTX 6000 Ada Generation with Threadripper CPU
- **Edge Kit**: Jetson Orin AGX with RealSense D435i
- **Robot**: Unitree Go2 for initial development, upgrade to G1 for advanced work
- **Backup**: Cloud credits for large-scale training

### Individual Developer
- **Workstation**: RTX 4080 with mid-range CPU
- **Edge Kit**: Jetson Orin NX with ReSpeaker mic array
- **Robot**: Simulator initially, then OP3 or Go2
- **Cloud**: Spot instances for heavy training

### Startup/SME
- **Workstation**: Dual RTX 4090 with high-end CPU
- **Edge Kit**: Multiple Jetson Orin AGX units
- **Robot**: Unitree G1 for demonstrations
- **Cloud**: Reserved instances for predictable costs

## Compatibility Matrix

| Component | ROS 2 Humble | Isaac Sim | Isaac ROS | Open3D | PyTorch |
|-----------|--------------|-----------|-----------|--------|---------|
| RTX 4090 | ✅ | ✅ | ✅ | ✅ | ✅ |
| RTX 4080 | ✅ | ✅ | ✅ | ✅ | ✅ |
| RTX 4070 | ✅ | ⚠️ | ⚠️ | ✅ | ✅ |
| Jetson Orin AGX | ✅ | ✅ | ✅ | ⚠️ | ✅ |
| Jetson Orin NX | ✅ | ⚠️ | ⚠️ | ⚠️ | ⚠️ |

✅ = Fully supported, ⚠️ = Limited support, ❌ = Not supported

## Procurement Tips

1. **Buy in Bulk**: Academic discounts and bulk purchases can reduce costs by 10-20%
2. **Lease Options**: Consider leasing expensive hardware for short-term projects
3. **Refurbished**: Certified refurbished options for cost savings
4. **Timing**: Purchase during academic discount periods or end-of-year sales
5. **Warranty**: Extended warranties for expensive components
6. **Support**: Ensure vendor support for robotics-specific applications

## Maintenance and Support

### Local Hardware
- **Cleaning**: Regular dust removal from GPU fans
- **Thermal Paste**: Replace every 2-3 years for air-cooled systems
- **Power Protection**: UPS for power conditioning and backup
- **Backup**: Regular system backups of development environments

### Cloud Infrastructure
- **Cost Monitoring**: Set up billing alerts and budgets
- **Resource Management**: Automate shutdown of unused instances
- **Security**: VPN access and encrypted storage for sensitive data
- **Compliance**: Ensure cloud usage meets institutional requirements

This hardware configuration will support the full range of activities in the Physical AI & Humanoid Robotics course, from simulation to real-world deployment.