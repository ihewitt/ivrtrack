# ivrtrack
GPS running tracker for A9G board.
Uses SSD1306 OLED for display.

More details at https://blog.ivor.org/2020/10/tracking-running-part-2.html

# BUILDING

With a setup A9G [GPRS SDK](https://ai-thinker-open.github.io/GPRS_C_SDK_DOC/en/c-sdk/installation_linux.html) directory, create code subdirectory "gps_monitor" then

```
./build.sh gps_monitor
```

Next as usual flash:
`(first time) hex/gps_monitor/gps_monitor_B2129_debug.lod`
followed by
`(each build) hex/gps_monitor/gps_monitor_flash_debug.lod`
