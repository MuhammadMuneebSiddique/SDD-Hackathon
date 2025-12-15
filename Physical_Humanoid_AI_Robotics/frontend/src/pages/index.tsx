import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <Heading as="h1" className={styles.heroTitle}>
              {siteConfig.title}
            </Heading>
            <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Start Reading the Book →
              </Link>
            </div>
          </div>
          <div className={styles.heroBookVisual}>
            <div className={styles.bookIcon}>
              <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor" width="100%" height="100%">
                <path d="M22 9.24l-7.19-.62L12 2 9.19 8.63 2 9.24l5.46 4.73L5.82 21 12 17.27 18.18 21l-1.63-7.03L22 9.24zM12 15.4l-3.76 2.25 1-4.28-3.32-2.88 4.38-.38L12 6.1l1.71 4.04 4.38.38-3.32 2.88 1 4.28L12 15.4z"/>
              </svg>
            </div>
          </div>
        </div>

        <div className={styles.section}>
          <div className="container">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <Heading as="h2" className={styles.sectionTitle}>
                  About This Project
                </Heading>
                <p className={styles.sectionText}>
                  This project introduces Physical AI and Humanoid Robotics through a structured, Spec-driven learning experience.
                </p>

                <Heading as="h3" className={styles.sectionSubTitle}>
                  Why This Project
                </Heading>
                <div className={styles.benefits}>
                  <div className={styles.benefitItem}>
                    <span className={styles.benefitEmoji}>🤖</span>
                    <span className={styles.benefitText}>Learn robotics with AI-driven methods</span>
                  </div>
                  <div className={styles.benefitItem}>
                    <span className={styles.benefitEmoji}>⚡</span>
                    <span className={styles.benefitText}>Modern simulation workflows</span>
                  </div>
                  <div className={styles.benefitItem}>
                    <span className={styles.benefitEmoji}>🚀</span>
                    <span className={styles.benefitText}>Future-ready humanoid automation</span>
                  </div>
                  <div className={styles.benefitItem}>
                    <span className={styles.benefitEmoji}>📋</span>
                    <span className={styles.benefitText}>Spec-driven educational design</span>
                  </div>
                </div>

                <div className={clsx('hero hero--primary', styles.finalCTA)}>
                  <Heading as="h3" className={styles.ctaTitle}>
                    Ready to dive in?
                  </Heading>
                  <p className={styles.ctaText}>
                    Explore the comprehensive guide to Physical AI and Humanoid Robotics
                  </p>
                  <div className={styles.buttons}>
                    <Link
                      className="button button--secondary button--lg"
                      to="/docs/intro">
                      💥 Open the Book → /docs
                    </Link>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics - A comprehensive guide to robotics, AI, and humanoid systems">
      <HomepageHeader />
    </Layout>
  );
}
