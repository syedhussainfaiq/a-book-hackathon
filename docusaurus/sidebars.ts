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
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/intro',
        'module-1/chapter-1-nodes-topics-services',
        'module-1/chapter-2-rclpy-bridge',
        'module-1/chapter-3-urdf-humanoids'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/intro',
        'module-2/chapter-1-gazebo-physics',
        'module-2/chapter-2-unity-hri',
        'module-2/chapter-3-sensor-simulation'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/intro',
        'module-3/chapter-1-isaac-sim-synthetic-data',
        'module-3/chapter-2-isaac-ros-vslam',
        'module-3/chapter-3-nav2-humanoid'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Visual Language Agent (VLA) Integration',
      items: [
        'module-4/intro',
        'module-4/chapter-1-vla-robotic-systems',
        'module-4/chapter-2-real-time-vla-hri',
        'module-4/chapter-3-advanced-vla-applications'
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
