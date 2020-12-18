<template>
  <div class="demo">
    <div class="scroll-list-wrap">
      <cube-scroll
        ref="scroll"
        :data="items"
        :options="scrollbarOptions"
        @click="download">
      </cube-scroll>
    </div>
  </div>
</template>

<script>
export default {
  data() {
    return {
      items: ['scan1.xyz', 'scan2.xyz',],
      scrollbarOptions: {
        scrollbar: {
          fade: false
        },
      }
    }
  },
  methods: {
    download(fname) {
      window.open(`/${process.env.VUE_APP_BASE_URL}/download/${fname}`, '_self');
    }
  }
}
</script>

<style lang="scss">
.scroll-list-wrap {
  height: 350px;
  border: 1px solid rgba(0, 0, 0, 0.1);
  border-radius: 5px;
  transform: rotate(0deg);
  overflow: hidden;
}
</style>