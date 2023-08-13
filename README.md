## Video

Test Video source

```
gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! 'video/x-raw(memory:NVMM),width=3820, height=2464, framerate=21/1,format=NV12' ! fakesink
```

## Multi Stage Build Options

https://www.docker.com/blog/dockerfiles-now-support-multiple-build-contexts/