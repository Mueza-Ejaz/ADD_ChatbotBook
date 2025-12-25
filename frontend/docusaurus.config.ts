import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'Tutorial Blog',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  url: 'https://example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/Bookplus_Chatbot/',

  scripts: [
    'https://cdn.platform.openai.com/deployments/chatkit/chatkit.js',
  ],
  stylesheets: [
    'https://fonts.googleapis.com/css2?family=Inter:wght@400;500;700&display=swap',
  ],

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: '<YOUR_GITHUB_USERNAME_OR_ORG>', // Usually your GitHub org/user name.
  projectName: 'Bookplus_Chatbot', // Usually your repo name.

  onBrokenLinks: 'ignore',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Humanoid Robotics',
      items: [
        {
          to: '/',
          label: 'Home',
          position: 'left',
        },
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Tutorials',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          href: 'https://github.com/Mueza-Ejaz',
          label: 'GitHub',
          position: 'right',
          className: 'header-github-link', // Use 'header-github-link' for consistency
          'aria-label': 'GitHub repository',
        },
        { // Docusaurus built-in search from our branch
          type: 'search',
          position: 'right',
          className: 'header-search-icon', // Custom class for styling
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Tutorial',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/docusaurus',
            },
            {
              label: 'LinkedIn',
              href: 'https://www.linkedin.com/in/mueza-ejaz086',
            },
            { label: 'X', href: 'https://x.com/' }
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/Mueza-Ejaz',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Mueza Ejaz.`,
    },
    prism: {
      theme: prismThemes.vsLight,
      darkTheme: prismThemes.vsDark,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
