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
  // Module 1: ROS 2 Fundamentals for Humanoid Robotics
  module1Sidebar: [
    'intro', // Course overview
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module1/chapter1-ros2-core',      // Chapter 1: ROS 2 Core Concepts
        'module1/chapter2-agent-bridge',   // Chapter 2: Agent Bridging
        'module1/chapter3-urdf-model',     // Chapter 3: URDF Modeling
      ],
    },
  ],

  // Module 3: The AI-Robot Brain (NVIDIA Isaac)
  module3Sidebar: [
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module3/intro',                      // Introduction & Learning Path
        'module3/chapter1-isaac-sim',         // Chapter 1: Isaac Sim Fundamentals
        'module3/chapter2-synthetic-data',    // Chapter 2: Synthetic Data Generation
        'module3/chapter3-vslam',             // Chapter 3: Isaac ROS VSLAM
        'module3/chapter4-nav2',              // Chapter 4: Nav2 Path Planning
      ],
    },
  ],
};

export default sidebars;
