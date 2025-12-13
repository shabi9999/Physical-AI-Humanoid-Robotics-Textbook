// import React from 'react';
// import Link from '@docusaurus/Link';

// const moduleData = [
//   {
//     weeks: 'Weeks 3-5',
//     title: 'Module 1: The Robotic Nervous System (ROS 2)',
//     description: 'Master ROS 2 architecture, communication patterns, and robot modeling. Build distributed robotic systems using nodes, topics, services, and actions.',
//     outcomes: [
//       'Explain the ROS 2 computation graph and its components',
//       'Create publishers, subscribers, and service clients using rclpy',
//       'Define robot structure using URDF and visualize in RViz2',
//     ],
//     link: '/module1/intro',
//   },
//   {
//     weeks: 'Weeks 6-7',
//     title: 'Module 2: Digital Twins - Simulation & Sensors',
//     description: 'Build digital twins for robotic systems using Gazebo and Unity. Simulate sensors, physics, and environments for testing before deploying to physical hardware.',
//     outcomes: [
//       'Create Gazebo simulation environments with physics and sensors',
//       'Integrate Unity for photorealistic sensor simulation',
//       'Test navigation and perception algorithms in simulation',
//     ],
//     link: '/module2/intro',
//   },
//   {
//     weeks: 'Weeks 8-10',
//     title: 'Module 3: NVIDIA Isaac - Perception & Navigation',
//     description: 'Leverage NVIDIA Isaac Sim for GPU-accelerated robotics. Implement VSLAM, Nav2 navigation stacks, and reinforcement learning for autonomous behaviors.',
//     outcomes: [
//       'Set up and configure NVIDIA Isaac Sim environments',
//       'Implement Visual SLAM for robot localization',
//       'Deploy Nav2 navigation stack for autonomous navigation',
//     ],
//     link: '/module3/intro',
//   },
//   {
//     weeks: 'Weeks 11-13',
//     title: 'Module 4: VLA & Humanoid Robotics',
//     description: 'Integrate Vision-Language-Action models with humanoid robots. Master humanoid kinematics, manipulation, and conversational AI for natural human-robot interaction.',
//     outcomes: [
//       'Calculate forward and inverse kinematics for humanoid robots',
//       'Implement manipulation primitives for pick-and-place tasks',
//       'Integrate conversational AI with robot action planning',
//     ],
//     link: '/module4/intro',
//   },
// ];

// function ModuleCard({ weeks, title, description, outcomes, link }) {
//   return (
//     <div className="bg-white border border-gray-200 rounded-xl p-8 hover:border-[#16a34a] hover:shadow-lg hover:-translate-y-1 transition-all duration-300">
//       {/* Week Badge */}
//       <div className="text-xs font-semibold text-gray-500 uppercase tracking-wide mb-3">
//         {weeks}
//       </div>

//       {/* Module Title */}
//       <h3 className="text-xl font-bold text-[#059669] mb-4 leading-snug hover:text-[#047857] transition-colors">
//         {title}
//       </h3>

//       {/* Description */}
//       <p className="text-gray-600 leading-relaxed mb-6">
//         {description}
//       </p>

//       {/* Learning Outcomes */}
//       <div className="border-t border-gray-200 pt-5">
//         <h4 className="text-xs font-semibold text-[#16a34a] uppercase tracking-wide mb-3">
//           Learning Outcomes
//         </h4>
//         <ul className="space-y-2 mb-6">
//           {outcomes.map((outcome, idx) => (
//             <li key={idx} className="flex items-start gap-2 text-sm text-gray-700">
//               <span className="text-[#16a34a] font-bold mt-0.5 flex-shrink-0">✓</span>
//               <span>{outcome}</span>
//             </li>
//           ))}
//         </ul>
//       </div>

//       {/* Learn More Link */}
//       <Link
//         to={link}
//         className="inline-flex items-center gap-1 text-[#059669] hover:text-[#047857] font-semibold text-sm hover:underline transition-colors"
//       >
//         Learn more
//         <span>→</span>
//       </Link>
//     </div>
//   );
// }

// export default function HomepageFeatures() {
//   return (
//     <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
//       {moduleData.map((module, idx) => (
//         <ModuleCard key={idx} {...module} />
//       ))}
//     </div>
//   );
// }

// part 2

import React from 'react';
import Link from '@docusaurus/Link';

const moduleData = [
  {
    weeks: 'Weeks 3-5',
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    description: 'Master ROS 2 architecture, communication patterns, and robot modeling. Build distributed robotic systems using nodes, topics, services, and actions.',
    outcomes: [
      'Explain the ROS 2 computation graph and its components',
      'Create publishers, subscribers, and service clients using rclpy',
      'Define robot structure using URDF and visualize in RViz2',
    ],
    link: '/module1/intro',
  },
  {
    weeks: 'Weeks 6-7',
    title: 'Module 2: Digital Twins - Simulation & Sensors',
    description: 'Build digital twins for robotic systems using Gazebo and Unity. Simulate sensors, physics, and environments for testing before deploying to physical hardware.',
    outcomes: [
      'Create Gazebo simulation environments with physics and sensors',
      'Integrate Unity for photorealistic sensor simulation',
      'Test navigation and perception algorithms in simulation',
    ],
    link: '/module2/intro',
  },
  {
    weeks: 'Weeks 8-10',
    title: 'Module 3: NVIDIA Isaac - Perception & Navigation',
    description: 'Leverage NVIDIA Isaac Sim for GPU-accelerated robotics. Implement VSLAM, Nav2 navigation stacks, and reinforcement learning for autonomous behaviors.',
    outcomes: [
      'Set up and configure NVIDIA Isaac Sim environments',
      'Implement Visual SLAM for robot localization',
      'Deploy Nav2 navigation stack for autonomous navigation',
    ],
    link: '/module3/intro',
  },
  {
    weeks: 'Weeks 11-13',
    title: 'Module 4: VLA & Humanoid Robotics',
    description: 'Integrate Vision-Language-Action models with humanoid robots. Master humanoid kinematics, manipulation, and conversational AI for natural human-robot interaction.',
    outcomes: [
      'Calculate forward and inverse kinematics for humanoid robots',
      'Implement manipulation primitives for pick-and-place tasks',
      'Integrate conversational AI with robot action planning',
    ],
    link: '/module4/intro',
  },
];

function ModuleCard({ weeks, title, description, outcomes, link }) {
  return (
    <div className="bg-white border border-gray-200 rounded-xl p-8 hover:border-[#16a34a] hover:shadow-md transition-all duration-200">
      {/* Week Badge */}
      <div className="text-xs font-semibold text-gray-500 uppercase tracking-wide mb-3">
        {weeks}
      </div>

      {/* Module Title */}
      <h3 className="text-xl font-bold text-[#059669] mb-4 leading-snug hover:text-[#047857] transition-colors">
        {title}
      </h3>

      {/* Description */}
      <p className="text-gray-600 leading-relaxed mb-6">
        {description}
      </p>

      {/* Learning Outcomes */}
      <div className="border-t border-gray-200 pt-5 mb-6">
        <h4 className="text-xs font-semibold text-[#16a34a] uppercase tracking-wide mb-3">
          Learning Outcomes
        </h4>
        <ul className="space-y-2.5">
          {outcomes.map((outcome, index) => (
            <li key={index} className="flex items-start text-sm text-gray-600 leading-relaxed">
              <span className="text-[#16a34a] font-bold text-base mr-3 mt-0.5">✓</span>
              <span>{outcome}</span>
            </li>
          ))}
        </ul>
      </div>

      {/* Learn More Link */}
      <Link
        to={link}
        className="inline-flex items-center gap-1 text-[#059669] hover:text-[#047857] font-semibold text-sm hover:underline transition-colors"
      >
        Learn more <span>→</span>
      </Link>
    </div>
  );
}

export default function HomepageFeatures(): JSX.Element {
  return (
    <section>
      <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
        {moduleData.map((module, index) => (
          <ModuleCard key={index} {...module} />
        ))}
      </div>
    </section>
  );
}