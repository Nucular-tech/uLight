# uLight
micro Light controller

Code published as example use of LEVCAN libraries

If you erased whole chip, flash Bootloader hex file from releases page

Used:
 - eclipse IDE 2024-12
 - GNU MCU Eclipse
 - GNU Tools ARM Embedded 8 2019-q3-update
 - Sysprogs OpenOCD
 - GNU MCU Build tools 2.12-20190422
 
To build Debug run:

setup Debug --cross-file meson.cross --reconfigure

compile -C Debug

To build Release run:

setup Release --cross-file meson.cross --reconfigure --buildtype=release --optimization=2

compile -C Release
