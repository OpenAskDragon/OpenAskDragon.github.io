<script setup>
import DefaultTheme from 'vitepress/theme'
import { useData } from 'vitepress'
import { onMounted } from 'vue'

const { frontmatter } = useData()

// Busuanzi counter
onMounted(() => {
  // Load busuanzi script
  const script = document.createElement('script')
  script.src = '//busuanzi.ibruce.info/busuanzi/2.3/busuanzi.pure.mini.js'
  script.async = true
  document.head.appendChild(script)
})

// Gitalk loader (only if frontmatter.gitalk is true)
onMounted(() => {
  if (frontmatter.value?.gitalk) {
    const gitalkCss = document.createElement('link')
    gitalkCss.rel = 'stylesheet'
    gitalkCss.href = 'https://cdn.jsdelivr.net/npm/gitalk@1/dist/gitalk.css'
    document.head.appendChild(gitalkCss)

    const gitalkJs = document.createElement('script')
    gitalkJs.src = 'https://cdn.jsdelivr.net/npm/gitalk@1/dist/gitalk.min.js'
    gitalkJs.onload = () => {
      const container = document.getElementById('gitalk-container')
      if (container && window.Gitalk) {
        new window.Gitalk({
          clientID: '{{GITALK_CLIENT_ID}}',
          clientSecret: '{{GITALK_CLIENT_SECRET}}',
          repo: 'OpenAskDragon.github.io',
          owner: 'OpenAskDragon',
          admin: ['OpenAskDragon'],
          id: location.pathname,
          distractionFreeMode: false,
          language: 'zh-CN',
        }).render('gitalk-container')
      }
    }
    document.head.appendChild(gitalkJs)
  }
})
</script>

<template>
  <DefaultTheme.Layout>
    <template #doc-footer-before>
      <!-- Gitalk comment area -->
      <div
        v-if="frontmatter.gitalk"
        id="gitalk-container"
        class="gitalk-container"
      />
    </template>
    <template #layout-bottom>
      <!-- Busuanzi visitor counter -->
      <div class="busuanzi-counter">
        <span id="busuanzi_container_site_pv">
          View: <span id="busuanzi_value_site_pv" />&nbsp;
        </span>
        <span id="busuanzi_container_site_uv">
          User: <span id="busuanzi_value_site_uv" />
        </span>
      </div>
    </template>
  </DefaultTheme.Layout>
</template>
