# ivrtrack
GPS running tracker for A9G board.
Uses SSD1306 OLED for display.

More details at https://blog.ivor.org/2020/10/tracking-running-part-2.html

The OLED fields: ![OLED](doc/oled.jpg?raw=true)

# BUILDING

With a setup A9G [GPRS SDK](https://ai-thinker-open.github.io/GPRS_C_SDK_DOC/en/c-sdk/installation_linux.html) directory, create code subdirectory "gps_monitor" then

```
./build.sh gps_monitor
```

Next as usual flash:
`(first time) hex/gps_monitor/gps_monitor_B2129_debug.lod`
followed by
`(each build) hex/gps_monitor/gps_monitor_flash_debug.lod`

# Miscellaneous
At one point needed to retrieve/restore IMEI from a dead A9G, so used https://gist.github.com/ihewitt/7ef825261cc642398cf795f394af7539 to dump all the flash contents.
Attempting to create a separate extract (and later upload) flash utility using the HST UART interface, this isn't working yet but this is a start: https://gist.github.com/ihewitt/5969b7d427fc7248306cb894ec20cace 

Flash map: https://ai-thinker-open.github.io/GPRS_C_SDK_DOC/zh/more/flash_map.html
