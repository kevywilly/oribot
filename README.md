## Video

Test Video source

```
gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! 'video/x-raw(memory:NVMM),width=3820, height=2464, framerate=21/1,format=NV12' ! fakesink
```

## Multi Stage Build Options

https://www.docker.com/blog/dockerfiles-now-support-multiple-build-contexts/


## Jetson Orin Pins

board #: 7 tegra: GP167
board #: 11 tegra: GP72_UART1_RTS_N
board #: 12 tegra: GP122
board #: 13 tegra: GP36_SPI3_CLK
board #: 15 tegra: GP88_PWM1
board #: 16 tegra: GP40_SPI3_CS1_N
board #: 18 tegra: GP39_SPI3_CS0_N
board #: 19 tegra: GP49_SPI1_MOSI
board #: 21 tegra: GP48_SPI1_MISO
board #: 22 tegra: GP37_SPI3_MISO
board #: 23 tegra: GP47_SPI1_CLK
board #: 24 tegra: GP50_SPI1_CS0_N
board #: 26 tegra: GP51_SPI1_CS1_N
board #: 29 tegra: GP65
board #: 31 tegra: GP66
board #: 32 tegra: GP113_PWM7
board #: 33 tegra: GP115
board #: 35 tegra: GP125
board #: 36 tegra: GP73_UART1_CTS_N
board #: 37 tegra: GP38_SPI3_MOSI
board #: 38 tegra: GP124
board #: 40 tegra: GP123