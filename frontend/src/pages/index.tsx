import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

import RoboticsHeroSection from '@site/src/components/RoboticsHeroSection';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Docusaurus Tutorial - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <div className={styles.homepageContainer}>
        {/* Main Content Area */}
        <main className={styles.mainContent}>
          <header className={styles.mainContentHeader}>
            <h1 className={styles.bookTitle}>Physical AI & Humanoid Robotics</h1>
            <p className={styles.bookSubtitle}>An AI-Native Textbook</p>
            <div className={styles.buttons}>
              <Link
                className={styles.primaryCtaButton}
                to="/docs/intro">
                Start Learning Now
              </Link>
            </div>
          </header>

          <div className={styles.sectionDivider}></div>

          {/* Interactive Feature Cards */}
          <section className={styles.featureCardsSection}>
            <h2 className={styles.sectionHeading}>Explore Key Concepts</h2>
            <div className="row">
              {/* Card 1: AI */}
              <div className="col col--4">
                <div className={clsx("padding-horiz--md", styles.featureCard)}>
                  <div className={styles.cardHeader}>
                    <div className={styles.cardAccentBorder}></div>
                    <h3>Artificial Intelligence</h3>
                  </div>
                  <div className={styles.cardBody}>
                    <p>Dive deep into the algorithms and models driving intelligent systems and humanoid robots.</p>
                  </div>
                  <div className={styles.cardFooter}>
                    <button type="button" className={styles.smallCtaButton}>Learn More &rarr;</button>
                  </div>
                </div>
              </div>
              {/* Card 2: Robotics */}
              <div className="col col--4">
                <div className={clsx("padding-horiz--md", styles.featureCard)}>
                  <div className={styles.cardHeader}>
                    <div className={styles.cardAccentBorder}></div>
                    <h3>Humanoid Robotics</h3>
                  </div>
                  <div className={styles.cardBody}>
                    <p>Understand the mechanics, control systems, and design principles of advanced humanoid forms.</p>
                  </div>
                  <div className={styles.cardFooter}>
                    <button type="button" className={styles.smallCtaButton}>Learn More &rarr;</button>
                  </div>
                </div>
              </div>
              {/* Card 3: Machine Learning */}
              <div className="col col--4">
                <div className={clsx("padding-horiz--md", styles.featureCard)}>
                  <div className={styles.cardHeader}>
                    <div className={styles.cardAccentBorder}></div>
                    <h3>Machine Learning & Perception</h3>
                  </div>
                  <div className={styles.cardBody}>
                    <p>Explore how robots learn from data and perceive their environment through cutting-edge ML techniques.</p>
                  </div>
                  <div className={styles.cardFooter}>
                    <button type="button" className={styles.smallCtaButton}>Learn More &rarr;</button>
                  </div>
                </div>
              </div>
            </div>
          </section>
        </main>
      </div>

      <RoboticsHeroSection />
    </Layout>
  );
}

