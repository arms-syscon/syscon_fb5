# Firebird 5 C code for ATMEGA2560 MCU
## Setup:
Install the following modules using commands given

Linux

`sudo apt install avr-libc`

`sudo apt install avrdude`

## Compiling
Make appropiate edits to make file as per your c code
keep make and c file in same folder.
Open Terminal and Run following
`make`
## set fuses
Use following commands change the high fuse and low fuse to right setting

`avrdude -c usbasp -p m2560 -U lfuse:w:0xFF:m`

`avrdude -c usbasp -p m2560 -U hfuse:w:0xDB:m`

## upload hex fire to ATMEGA2560
`avrdude -c usbasp -p m2560 -U flash:w:<name of c file>.hex -B5`

For example

`avrdude -c usbasp -p m2560 -U flash:w:fb5.hex -B5`
