import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

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
                🚀 Start Learning Now
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
                    <Link to="/docs/chapter-1" className={styles.smallCtaButton}>Learn More &rarr;</Link>
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
                    <Link to="/docs/chapter-2" className={styles.smallCtaButton}>Learn More &rarr;</Link>
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
                    <Link to="/docs/chapter-3" className={styles.smallCtaButton}>Learn More &rarr;</Link>
                  </div>
                </div>
              </div>
            </div>
          </section>
        </main>
      </div>

      {/* Pre-Footer Section */}
      <section className={styles.preFooterSection}>
        <div className="container">
          <div className="row">
            {/* Column 1: Left - 60% width */}
            <div className="col col--7"> {/* Docusaurus uses col--x for width, ~60% is col--7 */}
              <h2 className={styles.preFooterMainHeading}>Master AI Robotics Today</h2>
              <p className={styles.preFooterSubheading}>Join thousands of developers and engineers learning cutting-edge robotics technology</p>
              <ul className={styles.preFooterFeatureList}>
                <li><span className={styles.featureIcon}>✅</span> Comprehensive tutorials</li>
                <li><span className={styles.featureIcon}>💡</span> Practical projects</li>
                <li><span className={styles.featureIcon}>🚀</span> Latest AI techniques</li>
                <li><span className={styles.featureIcon}>🤝</span> Community support</li>
              </ul>
            </div>

            {/* Column 2: Right - 40% width */}
            <div className="col col--5"> {/* ~40% is col--5 */}
              <div className={styles.preFooterCtaCard}>
                <h3 className={styles.ctaCardTitle}>Get Free Chapter</h3>
                <p className={styles.ctaCardSubtitle}>Start learning immediately</p>
                <div className={styles.emailInputContainer}>
                  <input type="email" placeholder="Your email address" className={styles.emailInputField} />
                </div>
                <button className={styles.downloadChapterButton}>Download Chapter</button>
                <p className={styles.ctaCardNote}>No spam. Unsubscribe anytime.</p>
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* Footer Section */}
      <footer className={styles.homepageFooter}>
        <div className="container">
          <div className="row">
            <div className="col col--4">
              <h4 className={styles.footerHeading}>Explore</h4>
              <ul className={styles.footerLinks}>
                <li><Link to="/docs/intro">Getting Started</Link></li>
                <li><Link to="/blog">Blog</Link></li>
                <li><Link to="/community">Community</Link></li>
              </ul>
            </div>
            <div className="col col--4">
              <h4 className={styles.footerHeading}>Resources</h4>
              <ul className={styles.footerLinks}>
                <li><Link to="/docs/api">API Reference</Link></li>
                <li><Link to="/docs/contributing">Contribute</Link></li>
              </ul>
            </div>
            <div className="col col--4">
              <h4 className={styles.footerHeading}>Connect</h4>
              <ul className={styles.footerLinks}>
                <li><a href="https://twitter.com/docusaurus" target="_blank" rel="noopener noreferrer">Twitter</a></li>
                <li><a href="https://github.com/facebook/docusaurus" target="_blank" rel="noopener noreferrer">GitHub</a></li>
              </ul>
            </div>
          </div>
          <div className={styles.footerCopyright}>
            <p>Copyright © {new Date().getFullYear()} AI Robotics Book. Built with Docusaurus.</p>
          </div>
        </div>
      </footer>
    </Layout>
  );
}
