import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

interface Module {
  weeks: string;
  title: string;
  description: string;
  outcomes: string[];
  link: string;
}

function HeroSection() {
  return (
    <header className={styles.heroSection}>
      <div className={styles.heroContainer}>
        <div className={styles.heroBadge}>
          <span>🎓 Industry-Grade Robotics Education</span>
        </div>
        <h1 className={styles.heroTitle}>
          Physical AI & Humanoid Robotics
        </h1>
        <p className={styles.heroSubtitle}>
          Comprehensive 13-Week Course for Industry Practitioners
        </p>
        <p className={styles.heroDescription}>
          Master ROS 2 architecture, digital twin simulation, NVIDIA Isaac Sim, and Vision-Language-Action models through hands-on, project-based learning
        </p>
        <div className={styles.heroButtons}>
          <Link
            to="/docs/intro"
            className={styles.heroButtonPrimary}
          >
            Start Learning
            <span>→</span>
          </Link>
          <a
            href="https://github.com/shabi9999"
            target="_blank"
            rel="noopener noreferrer"
            className={styles.heroButtonSecondary}
          >
            📄 View on GitHub
          </a>
        </div>
      </div>
    </header>
  );
}

function ModuleCards() {
  const modules: Module[] = [
    {
      weeks: 'Weeks 3-5',
      title: 'Module 1: The Robotic Nervous System (ROS 2)',
      description: 'Master ROS 2 architecture, communication patterns, and robot modeling. Build distributed robotic systems using nodes, topics, services, and actions.',
      outcomes: [
        'Explain the ROS 2 computation graph and its components',
        'Create publishers, subscribers, and service clients using rclpy',
        'Define robot structure using URDF and visualize in RViz2',
      ],
      link: '/docs/module1/intro',
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
      link: '/docs/module2/intro',
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
      link: '/docs/module3/intro',
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
      link: '/docs/module4/intro',
    },
  ];

  return (
    <section className={styles.moduleCardsSection}>
      <h2 className={styles.moduleCardsTitle}>Course Modules</h2>
      <p className={styles.moduleCardsDescription}>
        A comprehensive 13-week learning path designed to take you from ROS 2 fundamentals to advanced vision-language-action models for humanoid robots
      </p>
      <div className={styles.moduleGrid}>
        {modules.map((module, index) => (
          <div key={index} className={styles.moduleCard}>
            <div className={styles.moduleWeekBadge}>{module.weeks}</div>
            <h3 className={styles.moduleCardTitle}>{module.title}</h3>
            <p className={styles.moduleCardDescription}>{module.description}</p>
            <div className={styles.moduleOutcomes}>
              <h4 className={styles.moduleOutcomesTitle}>Learning Outcomes</h4>
              <ul className={styles.moduleOutcomesList}>
                {module.outcomes.map((outcome, idx) => (
                  <li key={idx} className={styles.moduleOutcomeItem}>
                    <span className={styles.moduleOutcomeCheck}>✓</span>
                    <span>{outcome}</span>
                  </li>
                ))}
              </ul>
            </div>
            <Link
              to={module.link}
              className={styles.moduleCardLink}
            >
              Learn more <span>→</span>
            </Link>
          </div>
        ))}
      </div>
    </section>
  );
}

interface QuickLink {
  title: string;
  href: string;
}

function QuickLinks() {
  const links: QuickLink[] = [
    { title: 'Workstation Setup', href: '/docs/setup/setup-workstation' },
    { title: 'Edge Kit Setup', href: '/docs/setup/setup-edge-kit' },
    { title: 'Cloud Setup', href: '/docs/setup/setup-cloud' },
    { title: 'Glossary', href: '/docs/glossary' },
    { title: 'Module 1: ROS 2', href: '/docs/module1/intro' },
    { title: 'Module 2: Digital Twin', href: '/docs/module2/intro' },
    { title: 'Module 3: Isaac Sim', href: '/docs/module3/intro' },
    { title: 'Module 4: VLA & Humanoids', href: '/docs/module4/intro' },
  ];

  return (
    <aside className={styles.quickLinks}>
      <h3 className={styles.quickLinksTitle}>Quick Links</h3>
      <ul className={styles.quickLinksList}>
        {links.map((link, index) => (
          <li key={index} className={styles.quickLinksItem}>
            <Link
              to={link.href}
              className={styles.quickLinksLink}
            >
              {link.title}
            </Link>
          </li>
        ))}
      </ul>
    </aside>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Comprehensive 13-Week Course for Industry Practitioners - Master ROS 2, Digital Twins, NVIDIA Isaac Sim, and Vision-Language-Action Models"
    >
      <HeroSection />
      <main className={styles.mainContent}>
        <div className={styles.contentContainer}>
          <div className={styles.mainColumn}>
            <ModuleCards />
          </div>
          <div className={styles.sidebarColumn}>
            <QuickLinks />
          </div>
        </div>
      </main>
    </Layout>
  );
}
