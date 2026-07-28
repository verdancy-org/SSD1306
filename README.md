# SSD1306

SSD1306 OLED panel backend that subscribes to `DisplaySurface::Frame` messages and
flushes a 128x64 monochrome vertical-tiled framebuffer over I2C.

## Required Hardware
- `i2c_oled`

## Constructor Arguments
- `i2c_alias`: hardware container alias for the OLED I2C bus
- `address`: SSD1306 I2C address, default `0x3C`
- `frame_topic_name`: topic name for `DisplaySurface::Frame`, default `display_frame`
- `chunk_bytes`: maximum I2C data payload per transfer

## Template Arguments
None

## Depends
- `DisplaySurface`
