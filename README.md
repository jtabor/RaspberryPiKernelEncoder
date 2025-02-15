# RaspberryPiKernelEncoder

This is a kernel driver to read up to 4 quadrature encoders.  The driver is interrupt-driven, and only uses about 2-5% CPU usage with 2 quadrature encoders counting at 12Khz each.  Any suggestions/extra code/whatever else welcome!

Tested on Rasbian Buster on a Raspberry Pi Zero W.  I expect it will work on other versions, but haven't tested it.

## Software Initial Setup:
- sudo apt-get -y install raspberrypi-kernel-headers
- If that doesn't work do:
	- sudo apt-get -y install --reinstall raspberrypi-bootloader raspberrypi-kernel raspberrypi-kernel-headers


## Compiling and installing kernel driver:
- Go to the repo's directory
- make  (compiles kernel module)
- sudo make load (installs kernel module)
- sudo make unload (remove kernel module)

## Electrical Hookup:
- Pin number definitions in device_file.c
- Change pin definitions at your own risk.  Changes sometimes interfere with other interrupts and cause the driver not to work properly.
- Pin numbers Encoder 1
	- A - 17, B - 27
- Encoder 2
	- A - 23, B - 24
- Encoder 3
	- A - 5, B - 6
- Encoder 4
	- A - 16, B - 26


## Testing:
- test.c prints out encoder counts
	- gcc -i test.c -o test (then sudo ./test to run)
- reset.c resets encoder counts by writing to char driver in /dev
	- gcc -i reset.c -o reset (then sudo ./reset to run)


## Usage:
- sudo make load creates a device driver /dev/encoder-driver
	- To read the counts, simply read sizeof(int) * 4 bytes from the driver.  Those will be the counts for the four encoder channels in binary (cast to int to read)  No matter what you read, it will always return that number of bytes.
	- To reset the counts, write anything to the driver


## Update 2/14/25

The kernel module has been updated to work with kernel version 6.6 which deprecates the old GPIO API.  It was tested on a Raspberry Pi 4B with Bookworm (kernel 6.6.51) and 3 motors + encoders.
