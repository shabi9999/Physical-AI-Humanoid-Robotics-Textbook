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
  // Main comprehensive sidebar with all modules
  docs: [
    'intro', // Course overview

    // Setup guides
    {
      type: 'category',
      label: 'Setup & Getting Started',
      items: [
        'setup/setup-intro',
        'setup/setup-workstation',
        'setup/setup-edge-kit',
        'setup/setup-cloud',
      ],
    },

    // Module 1: ROS 2 Fundamentals for Humanoid Robotics
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module1/intro',
        'module1/ch1-ros2-core',      // Chapter 1: ROS 2 Core Concepts
        'module1/ch2-agent-bridge',   // Chapter 2: Autonomous Agents
        'module1/ch3-urdf-model',     // Chapter 3: Robot Structure (URDF)
      ],
    },

    // Module 2: The Digital Twin (Gazebo & Unity Simulation)
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module2/intro',                       // Introduction & Learning Path
        'module2/ch1-digital-twin-concepts',    // Chapter 1: Digital Twin Fundamentals
        'module2/ch2-gazebo-physics',           // Chapter 2: Gazebo Physics Engine
        'module2/ch3-world-building',           // Chapter 3: Building Custom Worlds
        'module2/ch4-sensor-simulation',        // Chapter 4: Simulating Sensors
        'module2/ch5-unity-visualization',      // Chapter 5: Unity Visualization
      ],
    },

    // Module 3: The AI-Robot Brain (NVIDIA Isaac)
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module3/intro',                       // Introduction & Learning Path
        'module3/ch1-isaac-sim-fundamentals',  // Chapter 1: Isaac Sim Fundamentals
        'module3/chapter2-synthetic-data',     // Chapter 2: Synthetic Data Generation
        'module3/chapter3-vslam',              // Chapter 3: Visual SLAM (VSLAM)
        'module3/chapter4-nav2',               // Chapter 4: Navigation 2 (Nav2)
      ],
    },

    // Module 4: Vision-Language-Action Pipeline
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Pipeline',
      items: [
        'module4/intro',                       // Introduction & Learning Path
        'module4/ch1-whisper',         // Chapter 1: Speech Recognition with Whisper
        'module4/ch2-llm-planning',            // Chapter 2: LLM Cognitive Planning
        'module4/ch3-ros2-actions',            // Chapter 3: ROS 2 Action Integration
        'module4/ch4-complete-vla',            // Chapter 4: Complete VLA Pipeline
      ],
    },

    // Glossary/References
    {
      type: 'category',
      label: 'Reference',
      items: [
        'glossary',  // Robotics Glossary
      ],
    },
  ],
};

export default sidebars;
