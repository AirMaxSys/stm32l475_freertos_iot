LVGL font load at run time
===========================

LVGL converts TTF fonts to binary file tool [lv_font_conv](https://github.com/lvgl/lv_font_conv/)

Font convertion script

- font size 14

```sh
env DEBUG=* lv_font_conv --font Montserrat-Medium.ttf -r 0x20-0x7F --size 14 --format bin --bpp 4 --no-compress -o stm32l475_freertos_iot/assets/font/mont14.bin
```

- font size 48 (bpp set to 1)

```sh
env DEBUG=* lv_font_conv --font Montserrat-Medium.ttf -r 0x20-0x7F --size 48 --format bin --bpp 1 --no-compress -o stm32l475_freertos_iot/assets/font/mont48.bin
```
