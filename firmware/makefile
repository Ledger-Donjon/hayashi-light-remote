# This is a very poor makefile to build the binary and call objcopy after.

all:
	cargo build --release
	cargo objcopy --release --bin hayashi-light-remote -- -O binary hayashi-light-remote.bin

flash:
	stm32flash -w hayashi-light-remote.bin -v -g 0 /dev/ttyUSB0 

