

SOURCES = $(wildcard $(SRC)/*.ino)

all: build
	echo "Scatch: $(SOURCES)"

build:
	arduino-cli compile --fqbn  arduino:avr:pro $(SRC)

upload: build
	arduino-cli upload -p /dev/ttyUSB0 --fqbn  arduino:avr:pro $(SRC)

monitor: all
	arduino-cli monitor -p /dev/ttyUSB0 --fqbn  arduino:avr:pro --config baudrate=115200
