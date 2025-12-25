import React, { useEffect, useRef } from 'react';
import styles from './RoboticsHeroSection.module.css';

const RoboticsHeroSection = () => {
    const ctaButtonRef = useRef(null);

    useEffect(() => {
        const ctaButton = ctaButtonRef.current;
        if (!ctaButton) return;

        ctaButton.classList.add(styles.pulsing);

        const handleCtaClick = (e) => {
            e.preventDefault();
            const button = ctaButton;
            const ripple = document.createElement('span');
            const diameter = Math.max(button.clientWidth, button.clientHeight);
            const radius = diameter / 2;

            ripple.style.width = ripple.style.height = `${diameter}px`;
            ripple.style.left = `${e.clientX - (button.getBoundingClientRect().left + radius)}px`;
            ripple.style.top = `${e.clientY - (button.getBoundingClientRect().top + radius)}px`;
            ripple.classList.add(styles.ripple);

            const existingRipple = button.getElementsByClassName(styles.ripple)[0];
            if (existingRipple) {
                existingRipple.remove();
            }
            button.appendChild(ripple);

            button.classList.remove(styles.pulsing);
        };

        ctaButton.addEventListener('click', handleCtaClick);

        return () => {
            ctaButton.removeEventListener('click', handleCtaClick);
        };
    }, []);

    return (
        <>
            <div className={styles.floatingElement}></div>
            <div className={styles.floatingElement}></div>
            <div className={styles.floatingElement}></div>

            <div className={styles.container}>
                <div className={styles.mainContentArea}>
                    <div className={styles.leftColumn}>
                        <h1>Master AI Robotics Today</h1>
                        <p className={styles.description}>Join thousands of developers and engineers learning cutting-edge robotics technology</p>
                        <ul className={styles.featuresList}>
                            <li><i className="fas fa-graduation-cap"></i> Comprehensive tutorials</li>
                            <li><i className="fas fa-project-diagram"></i> Practical projects</li>
                            <li><i className="fas fa-brain"></i> Latest AI techniques</li>
                            <li><i className="fas fa-users"></i> Community support</li>
                        </ul>
                        <a href="#" className={styles.ctaButton} ref={ctaButtonRef}>Get Free Chapter</a>
                        <p className={styles.ctaNote}>Start learning immediately. No spam. Unsubscribe anytime.</p>
                    </div>

                    <div className={styles.rightColumn}>
                        <div className={styles.navCard}>
                            <h3><i className="fas fa-compass"></i> Explore</h3>
                            <ul>
                                <li><a href="#"><i className="fas fa-play-circle"></i> Getting Started</a></li>
                                <li><a href="#"><i className="fas fa-blog"></i> Blog</a></li>
                                <li><a href="#"><i className="fas fa-users"></i> Community</a></li>
                            </ul>
                        </div>

                        <div className={styles.navCard}>
                            <h3><i className="fas fa-toolbox"></i> Resources</h3>
                            <ul>
                                <li><a href="#"><i className="fas fa-code"></i> API Reference</a></li>
                                <li><a href="#"><i className="fas fa-handshake"></i> Contribute</a></li>
                            </ul>
                        </div>

                        <div className={styles.navCard}>
                            <h3><i className="fas fa-link"></i> Connect</h3>
                            <div className={styles.socialButtons}>
                                <a href="#" className={styles.socialButton}><i className="fab fa-twitter"></i></a>
                                <a href="#" className={styles.socialButton}><i className="fab fa-github"></i></a>
                                <a href="#" className={styles.socialButton}><i className="fab fa-discord"></i></a>
                                <a href="#" className={styles.socialButton}><i className="fab fa-stack-overflow"></i></a>
                            </div>
                        </div>
                    </div>
                </div>


            </div>
        </>
    );
};

export default RoboticsHeroSection;