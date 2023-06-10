all: compile upload monitor
compile:
	arduino-cli compile -b arduino:avr:nano:cpu=atmega328old
upload:
	arduino-cli upload -b arduino:avr:nano:cpu=atmega328old -p /dev/ttyUSB0
monitor:
	arduino-cli monitor -p /dev/ttyUSB0
