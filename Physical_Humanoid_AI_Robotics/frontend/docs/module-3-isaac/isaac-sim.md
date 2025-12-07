---
sidebar_position: 2
title: 'Isaac Sim: Photorealistic Simulation'
---

# Isaac Sim: Photorealistic Simulation

This section covers NVIDIA Isaac Sim, a reference application for robotics simulation that combines NVIDIA's graphics and physics simulation capabilities with a robotics-focused interface.

## Learning Objectives

After completing this section, you will be able to:
- Install and configure Isaac Sim for robotics applications
- Create photorealistic environments for robot training
- Implement domain randomization techniques for sim-to-real transfer
- Configure sensors with realistic noise models
- Generate synthetic datasets for AI model training

## Introduction to Isaac Sim

Isaac Sim is NVIDIA's robotics simulation application built on the Omniverse platform. It provides:
- Physically accurate simulation with NVIDIA PhysX
- Photorealistic rendering with RTX technology
- Extensive robot and sensor models
- Integration with popular robotics frameworks (ROS, ROS 2, Isaac ROS)
- Synthetic data generation capabilities

### Key Features
- **USD-based Scene Description**: Universal Scene Description for scalable scene management
- **Realistic Physics**: NVIDIA PhysX 4.1 for accurate rigid body dynamics
- **RTX Rendering**: Hardware-accelerated ray tracing for photorealistic visuals
- **Extensible Framework**: Python and C++ APIs for custom extensions
- **Robot Simulation**: Support for complex articulated robots with accurate kinematics

## Installation and Setup

### System Requirements
- NVIDIA GPU with RTX or GTX 1080/2080/3080/4080 series or higher
- CUDA-compatible GPU with compute capability 6.0+
- 16GB+ RAM (32GB+ recommended)
- Windows 10/11 or Ubuntu 20.04 LTS
- Latest NVIDIA graphics drivers

### Installation Process
1. Download Isaac Sim from NVIDIA Developer website
2. Install Omniverse Launcher
3. Add Isaac Sim app through the launcher
4. Install required dependencies (CUDA, graphics drivers)

### Basic Launch
```bash
# Launch Isaac Sim with default settings
isaac-sim --exec "omni.isaac.examples.simple_world.simple_world"

# Launch with custom configuration
isaac-sim --config "custom_config.yaml"
```

## Creating Photorealistic Environments

### USD Scene Structure

Isaac Sim uses Universal Scene Description (USD) as its scene format. Here's a basic robot environment:

```python
# Example: Creating a simple environment in Isaac Sim
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import get_prim_at_path
import numpy as np

# Create world instance
world = World(stage_units_in_meters=1.0)

# Add ground plane
ground_plane = world.scene.add_default_ground_plane()

# Add a simple robot (using existing asset)
assets_root_path = get_assets_root_path()
if assets_root_path is not None:
    # Add a simple robot from Isaac Sim's asset library
    add_reference_to_stage(
        usd_path=assets_root_path + "/Isaac/Robots/Franka/franka_instanceable.usd",
        prim_path="/World/Robot"
    )

# Add objects to the scene
from omni.isaac.core.objects import DynamicCuboid
world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=np.array([0.5, 0.5, 0.5]),
        size=np.array([0.1, 0.1, 0.1]),
        mass=0.1
    )
)

# Reset world to apply changes
world.reset()
```

### Advanced Environment Design

```python
# Example: Creating a complex indoor environment
import omni
from pxr import Gf, Sdf, UsdGeom, UsdShade
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.prims import create_prim

def create_office_environment():
    stage = get_current_stage()

    # Create office room with walls
    create_prim(
        prim_path="/World/Wall_1",
        prim_type="Capsule",
        position=[0, 5, 1.5],
        orientation=[0, 0, 0, 1],
        scale=[0.1, 5, 3],
        usd_path="omniverse://localhost/NVIDIA/Assets/ArchVis/Commercial/Walls/"
    )

    # Add furniture
    create_prim(
        prim_path="/World/Desk",
        prim_type="Cuboid",
        position=[2, 0, 0.4],
        scale=[1.5, 0.8, 0.8],
        color=[0.8, 0.6, 0.4]
    )

    # Add lighting
    from omni.isaac.core.utils.prims import create_prim
    create_prim(
        prim_path="/World/KeyLight",
        prim_type="DistantLight",
        position=[0, 0, 10],
        orientation=[-0.3, 0, 0, 1],
        attributes={"color": [0.9, 0.9, 0.9], "intensity": 3000}
    )

# Execute environment creation
create_office_environment()
```

## Domain Randomization

Domain randomization is crucial for improving sim-to-real transfer by training models on diverse synthetic data.

### Lighting Randomization

```python
import random
from omni.isaac.core.utils.prims import get_prim_at_path

class LightingRandomizer:
    def __init__(self):
        self.light_prims = ["/World/KeyLight", "/World/FillLight", "/World/RimLight"]

    def randomize_lighting(self):
        for light_path in self.light_prims:
            light_prim = get_prim_at_path(light_path)
            if light_prim:
                # Randomize light intensity
                intensity = random.uniform(1000, 5000)
                light_prim.GetAttribute("inputs:intensity").Set(intensity)

                # Randomize light color temperature
                color_temp = random.uniform(4000, 8000)  # Kelvin
                # Convert to RGB approximation
                rgb = self.color_temperature_to_rgb(color_temp)
                light_prim.GetAttribute("inputs:color").Set(Gf.Vec3f(*rgb))

    def color_temperature_to_rgb(self, temp_k):
        """Convert color temperature in Kelvin to RGB values"""
        temp_k /= 100
        if temp_k <= 66:
            red = 255
        else:
            red = temp_k - 60
            red = 329.698727446 * (red ** -0.1332047592)

        if temp_k <= 66:
            green = temp_k
            green = 99.4708025861 * np.log(green) - 161.1195681661
        else:
            green = temp_k - 60
            green = 288.1221695283 * (green ** -0.0755148492)

        if temp_k >= 66:
            blue = 255
        elif temp_k <= 19:
            blue = 0
        else:
            blue = temp_k - 10
            blue = 138.5177312231 * np.log(blue) - 305.0447927307

        return [max(0, min(255, c)) / 255.0 for c in [red, green, blue]]
```

### Material Randomization

```python
import random
from pxr import UsdShade

class MaterialRandomizer:
    def __init__(self):
        self.material_paths = ["/World/Materials/Floor", "/World/Materials/Wall", "/World/Materials/Object"]

    def randomize_materials(self):
        for material_path in self.material_paths:
            material_prim = get_prim_at_path(material_path)
            if material_prim:
                # Randomize base color
                base_color = [random.random() for _ in range(3)]
                material_prim.GetAttribute("inputs:diffuse_tint").Set(Gf.Vec3f(*base_color))

                # Randomize roughness (0.0 = glossy, 1.0 = matte)
                roughness = random.uniform(0.1, 0.9)
                material_prim.GetAttribute("inputs:roughness").Set(roughness)

                # Randomize metallic (0.0 = non-metallic, 1.0 = metallic)
                metallic = random.choice([0.0, 1.0]) if random.random() > 0.7 else random.uniform(0.0, 0.3)
                material_prim.GetAttribute("inputs:metallic").Set(metallic)

# Usage in simulation loop
material_randomizer = MaterialRandomizer()

def simulation_step():
    if random.random() < 0.1:  # Randomize every 10th step
        material_randomizer.randomize_materials()
```

### Texture Randomization

```python
import random
from omni.isaac.core.utils.semantics import add_semantic_group

class TextureRandomizer:
    def __init__(self):
        self.texture_options = [
            "omniverse://localhost/NVIDIA/Assets/Textures/Patterns/Pattern_1.png",
            "omniverse://localhost/NVIDIA/Assets/Textures/Patterns/Pattern_2.png",
            "omniverse://localhost/NVIDIA/Assets/Textures/Materials/Metal_1.png",
            "omniverse://localhost/NVIDIA/Assets/Textures/Materials/Wood_1.png",
        ]

    def randomize_textures(self, prim_path):
        # Apply random texture to a prim
        material_prim = get_prim_at_path(prim_path)
        if material_prim:
            random_texture = random.choice(self.texture_options)
            # Set texture to material
            material_prim.GetAttribute("inputs:diffuse_texture:file").Set(random_texture)
```

## Sensor Configuration in Isaac Sim

### RGB Camera Setup

```python
from omni.isaac.sensor import Camera
import numpy as np

def setup_rgb_camera(robot_prim_path, position, orientation):
    # Create camera attached to robot
    camera = Camera(
        prim_path=robot_prim_path + "/camera",
        position=position,  # [x, y, z] in meters
        orientation=orientation,  # [w, x, y, z] quaternion
        frequency=30,  # Hz
        resolution=(640, 480)
    )

    # Configure camera properties
    camera.set_focal_length(24.0)  # mm
    camera.set_horizontal_aperture(20.955)  # mm
    camera.set_vertical_aperture(15.2908)  # mm

    return camera

# Example usage
rgb_camera = setup_rgb_camera(
    robot_prim_path="/World/Robot",
    position=[0.2, 0, 0.1],  # 20cm forward, 10cm up from robot origin
    orientation=[1, 0, 0, 0]  # Looking forward
)
```

### Depth Camera Setup

```python
def setup_depth_camera(robot_prim_path, position, orientation):
    # Depth camera with realistic noise
    depth_camera = Camera(
        prim_path=robot_prim_path + "/depth_camera",
        position=position,
        orientation=orientation,
        frequency=30,
        resolution=(640, 480)
    )

    # Enable depth data
    depth_camera.add_depth_data_to_frame()

    # Add realistic noise
    depth_camera.add_noise_to_depth_data(
        mean=0.0,
        std_dev=0.01  # 1cm standard deviation
    )

    return depth_camera

# Example usage
depth_camera = setup_depth_camera(
    robot_prim_path="/World/Robot",
    position=[0.2, 0, 0.1],
    orientation=[1, 0, 0, 0]
)
```

### LiDAR Sensor Setup

```python
from omni.isaac.range_sensor import LidarRtx
import numpy as np

def setup_lidar(robot_prim_path, position, orientation):
    # Create LiDAR sensor
    lidar = LidarRtx(
        prim_path=robot_prim_path + "/lidar",
        translation=position,
        orientation=orientation,
        config="Example_Rotary_M16",  # Uses M16 configuration
        rotation_frequency=10,  # 10 Hz
        samples=1000,  # Number of samples per rotation
        channels=16,  # Number of vertical channels
        up_axis="Z"  # Up axis direction
    )

    # Configure range properties
    lidar.set_max_range(25.0)  # Maximum range in meters
    lidar.set_min_range(0.1)   # Minimum range in meters

    # Add noise properties
    lidar.add_noise_to_lidar_data(
        mean=0.0,
        std_dev=0.02  # 2cm standard deviation
    )

    return lidar

# Example usage
lidar = setup_lidar(
    robot_prim_path="/World/Robot",
    position=[0.1, 0, 0.3],  # On top of robot
    orientation=[1, 0, 0, 0]
)
```

## Synthetic Data Generation

Isaac Sim excels at generating synthetic training data for AI models:

```python
import cv2
import numpy as np
from PIL import Image
import json

class SyntheticDataGenerator:
    def __init__(self, output_dir="synthetic_data"):
        self.output_dir = output_dir
        self.frame_counter = 0

    def capture_frame_data(self, rgb_camera, depth_camera, lidar):
        """Capture synchronized data from all sensors"""
        # Get RGB image
        rgb_data = rgb_camera.get_rgb()

        # Get depth data
        depth_data = depth_camera.get_depth_data()

        # Get LiDAR data
        lidar_data = lidar.get_point_cloud()

        # Save data with metadata
        self.save_frame(rgb_data, depth_data, lidar_data)

        self.frame_counter += 1

    def save_frame(self, rgb_data, depth_data, lidar_data):
        """Save frame data with annotations"""
        # Create frame directory
        frame_dir = f"{self.output_dir}/frame_{self.frame_counter:06d}"
        import os
        os.makedirs(frame_dir, exist_ok=True)

        # Save RGB image
        rgb_image = Image.fromarray(rgb_data)
        rgb_image.save(f"{frame_dir}/rgb.png")

        # Save depth data
        depth_array = np.array(depth_data)
        np.save(f"{frame_dir}/depth.npy", depth_array)

        # Save LiDAR point cloud
        np.save(f"{frame_dir}/pointcloud.npy", np.array(lidar_data))

        # Save metadata
        metadata = {
            "frame_id": self.frame_counter,
            "timestamp": self.get_timestamp(),
            "sensor_config": {
                "rgb_resolution": rgb_data.shape[:2],
                "depth_range": [0.1, 25.0],
                "lidar_range": [0.1, 25.0]
            }
        }

        with open(f"{frame_dir}/metadata.json", 'w') as f:
            json.dump(metadata, f)

    def get_timestamp(self):
        import time
        return time.time()

# Example usage in simulation loop
data_generator = SyntheticDataGenerator()

def simulation_loop():
    while True:
        # Step simulation
        world.step(render=True)

        # Capture data periodically
        if world.current_time_step_index % 100 == 0:  # Every 100 steps
            data_generator.capture_frame_data(rgb_camera, depth_camera, lidar)
```

## Lab Activity: Isaac Sim Environment

Create a complete Isaac Sim environment with:
1. Photorealistic indoor scene
2. Robot with multiple sensors
3. Domain randomization enabled
4. Synthetic data generation pipeline

### Steps:
1. Install Isaac Sim and verify installation
2. Create a complex indoor environment
3. Add a robot with RGB camera, depth camera, and LiDAR
4. Implement domain randomization for lighting and materials
5. Set up synthetic data capture pipeline
6. Generate diverse training datasets

### Expected Outcome:
- Functional Isaac Sim environment
- Robot with properly configured sensors
- Domain randomization working
- Synthetic data generation pipeline operational
- Diverse dataset for AI training

## Best Practices

1. **Performance Optimization**: Use appropriate level of detail for real-time simulation
2. **Realistic Parameters**: Configure sensors with parameters matching real hardware
3. **Domain Randomization**: Apply appropriate level of randomization for sim-to-real transfer
4. **Data Quality**: Ensure synthetic data is of high quality and properly annotated
5. **Validation**: Compare simulation results with real-world data when possible
6. **Documentation**: Maintain clear documentation of environment configurations

## Checklist

- [ ] Isaac Sim installation and setup completed
- [ ] Photorealistic environment created
- [ ] Domain randomization implemented
- [ ] Sensor configurations properly set up
- [ ] Synthetic data generation pipeline established
- [ ] Lab activity completed successfully

## Next Steps

In the next section, we'll explore Isaac ROS, which provides ROS 2 packages for perception tasks including Visual Simultaneous Localization and Mapping (VSLAM).