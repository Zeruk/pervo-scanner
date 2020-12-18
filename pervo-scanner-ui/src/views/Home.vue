<template>
  <div>
    <cube-button :light="true" icon="cubeic-scan" v-if="actionable" @click="startScan">Start scan</cube-button>
    <div class="pb-handler" v-if="scanStatus !== 'off'">
      <div class="pb" :style="{width: `${scanProcess}%`}"></div>
    </div>
    <cube-button :light="true" icon="cubeic-close" v-if="scanStatus !== 'off'" @click="stopScan()">Stop scan</cube-button>
    <cube-button :light="true" icon="cubeic-back" v-if="actionable" @click="rotate()">Rotate CCW</cube-button>
    <cube-button :light="true" icon="cubeic-arrow" v-if="actionable" @click="rotate('forward')">Rotate CW</cube-button>
    <cube-button :light="true" icon="cubeic-close" v-if="rotation !== 'off'" @click="rotate('stop')">Stop rotation</cube-button>
  </div>
</template>

<script>
import axios from 'axios'

export default {
  name: 'Home',
  data() {
    return {
      scanStatus: 'off',
      scanProcess: 0,
      rotation: 'off',
      scanUpdaterInterval: -1,
    }
  },
  methods: {
    scanStatusUpdate() {
      axios.get('/api/v1/scan/status').then(res => {
        this.scanProcess = res.data.process
      }).catch()
    },
    startScan() {
      if (!this.startToast) this.startToast = this.$createToast({
        txt: 'Starting scan process',
        mask: true
      })
      this.startToast.show()
      this.scanProcess = 0
      axios.post('/api/v1/scan/start').finally(() => {this.scanStatus = 'scanning';this.startToast.hide()})
      this.scanUpdaterInterval = setInterval(this.scanStatusUpdate, 500)
    },
    stopScan() {
      if(!this.stopToast) this.stopToast = this.$createToast({
        txt: 'Stopping scan process',
        mask: true
      })
      this.stopToast.show()
      axios.post('/api/v1/scan/stop').finally(() => {this.scanStatus = 'off';this.stopToast.hide()})
      clearInterval(this.scanUpdaterInterval)
      this.scanUpdaterInterval = -1
    },
    rotate(dir = 'backward') {
      // dir == backward/forward/stop
      axios.post(`/api/v1/rotate/${dir}`)
      this.rotation = dir === 'stop' ? 'off' : dir
    },
  },
  computed: {
    actionable() {
      return this.rotation === 'off' && this.scanStatus === 'off'
    }
  },
}
</script>

<style lang="scss">
.pb-handler {
  width: 100%;
  background-color: grey;
}

.pb {
  width: 0;
  transition: width 0.5s;
  height: 30px;
  background-color: green;
}
</style>
