import { defineConfig } from 'vitepress'
import mathjax3 from 'markdown-it-mathjax3'

export default defineConfig({
  lang: 'zh-CN',
  title: "OpenAskDragon's Docs",
  description: 'Make SLAM more Adaptive and Robust!',

  head: [
    ['link', { rel: 'icon', href: '/images/favicon.ico' }],
  ],

  // Exclude repo metadata files from the site
  srcExclude: ['README.md', 'LICENSE', 'CNAME', 'package.json', 'package-lock.json'],

  // MathJax3 for LaTeX math rendering
  markdown: {
    config: (md) => {
      md.use(mathjax3)
    },
  },

  themeConfig: {
    // Top nav bar
    nav: [
      { text: '首页', link: '/' },
      { text: '文章', link: '/posts/' },
      { text: '项目', link: '/projects/' },
      { text: '归档', link: '/archive' },
      { text: '标签', link: '/tags' },
      { text: '关于', link: '/about' },
    ],

    // Sidebar - different config per section
    sidebar: {
      '/posts/': [
        {
          text: '文章',
          items: [
            { text: 'SDV LOAM', link: '/posts/2024-04-06-sdv-loam' },
          ],
        },
      ],
      '/projects/': [
        {
          text: '项目文档',
          items: [
            { text: 'R-VIO2 算法详解', link: '/projects/rvio2_algorithm' },
            { text: '3DGS 核心原理与技术路线', link: '/projects/3dgs_algorithm' },
            { text: 'StreetCrafter 算法逻辑与工程流程', link: '/projects/algorithm_logic_flow.zh' },
          ],
        },
      ],
    },

    // Built-in search
    search: {
      provider: 'local',
    },

    // Footer
    footer: {
      message: 'Released under the MIT License.',
      copyright: 'Copyright © 2024 OpenAskDragon',
    },

    // Social links
    socialLinks: [
      { icon: 'github', link: 'https://github.com/OpenAskDragon' },
    ],

    // Edit link
    editLink: {
      pattern: 'https://github.com/OpenAskDragon/OpenAskDragon.github.io/edit/master/:path',
      text: '在 GitHub 上编辑此页',
    },

    // Last updated
    lastUpdated: {
      text: '最后更新',
      formatOptions: { dateStyle: 'short' },
    },
  },
})
