#!/bin/bash
avr-objcopy -Oihex -R.eeprom \
    target/avr-atmega32u4/release/firmware.elf \
    target/avr-atmega32u4/release/firmware.hex

if ! dfu-programmer atmega32u4 get bootloader-version >/dev/null 2>/dev/null; then
    printf "Waiting for bootloader..."
    while ! dfu-programmer atmega32u4 get bootloader-version >/dev/null 2>/dev/null; do
        printf "." ;
        sleep 0.5
    done
    printf "\n"
fi

dfu-programmer atmega32u4 erase
dfu-programmer atmega32u4 flash target/avr-atmega32u4/release/firmware.hex
dfu-programmer atmega32u4 reset
