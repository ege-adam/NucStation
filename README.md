# NucStation
A STM32 Game Console for Nucleo - L073RZ board. Done as final project for Microcontrollers course.

It has a custom bootloader and "GAME CARTRIDGES!"

I've used EEPROM's to store game data. Bootloader reads from when "cartridge" connected, and flashes Nucleo board on fly according to data in EEPROM.
