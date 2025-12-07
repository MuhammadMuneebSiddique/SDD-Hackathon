import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 - The Robotic Nervous System',
      items: [
        'module-1-ros2/index',
        'module-1-ros2/nodes-topics-services',
        'module-1-ros2/rclpy-bridge',
        'module-1-ros2/urdf-basics',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo + Unity)',
      items: [
        'module-2-simulation/index',
        'module-2-simulation/physics-simulation',
        'module-2-simulation/environment-building',
        'module-2-simulation/hri-visualization',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac - The AI-Robot Brain',
      items: [
        'module-3-isaac/index',
        'module-3-isaac/isaac-sim',
        'module-3-isaac/isaac-ros',
        'module-3-isaac/nav2-movement',
        'module-3-isaac/sim-to-real',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA - Vision-Language-Action',
      items: [
        'module-4-vla/index',
        'module-4-vla/whisper-integration',
        'module-4-vla/llm-reasoning',
        'module-4-vla/ros-actions',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/index',
      ],
    },
    {
      type: 'category',
      label: 'Resources',
      items: [
        'resources/index',
        'resources/hardware-requirements',
        'resources/lab-setup',
        'resources/robot-options',
      ],
    },
  ],
};

export default sidebars;
