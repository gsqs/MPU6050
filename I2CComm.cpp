/** I2C communcations service, used with BeagleBone Black.
Author: GSQ
Creation date: 12/22/2015
Changelog:
12/22/2015 - Creation

**/

#include "I2CComm.h"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <iostream>
#include <string.h>
#include <errno.h>

using namespace std;

static char devName[] = "/dev/i2c-0";
static int file;


I2CComm::I2CComm() {
	devName[9] = '0';
}

/* Initialize the I2C device port. */
I2CComm::I2CComm(uint8_t devPort) {
	switch (devPort) {
		case 1:
			devName[9] = '1';
		break;
		case 2:
			devName[9] = '2';
		break;
		case 3:
			devName[9] = '3';
		break;
		default:
			devName[9] = '0';
		break;
	}
}

/* Read some number of bytes from the device, starting at regAddr. */
int8_t I2CComm::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length,
	uint8_t *data, uint16_t timeout, uint8_t readAddress) {
    int8_t count = 0;
    int fd = open(devName, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
        return(-1);
    }
    if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
        fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
        close(fd);
        return(-1);
    }
	uint8_t buffer[1];
	buffer[0] = readAddress;
	//if (write(fd, &regAddr, 1) != 1) {
    if (write(fd, buffer, 1) != 1) {
        fprintf(stderr, "Failed to write reg: %s\n", strerror(errno));
        close(fd);
        return(-1);
    }
    count = read(fd, data, length);
    if (count < 0) {
        fprintf(stderr, "Failed to read device(%d): %s\n", count, ::strerror(errno));
        close(fd);
        return(-1);
    } else if (count != length) {
        fprintf(stderr, "Short read  from device, expected %d, got %d\n", length, count);
        close(fd);
        return(-1);
    }
    close(fd);

    return count;
}

/* Write some number of bytes to the device, at regAddr. */
bool I2CComm::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length,
	uint8_t* data) {
    int8_t count = 0;
    uint8_t buffer[128];
    int fd;
    if (length > 127) {
        fprintf(stderr, "Byte write count (%d) > 127\n", length);
        return(false);
    }
    fd = open(devName, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
        return(false);
    }
    if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
        fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
        close(fd);
        return(false);
    }
	
    buffer[0] = regAddr;
    memcpy(buffer+1,data,length);
    count = write(fd, buffer, length+1);
    if (count < 0) {
        fprintf(stderr, "Failed to write device(%d): %s\n", count, ::strerror(errno));
        close(fd);
        return(false);
    } else if (count != length+1) {
        fprintf(stderr, "Short write to device, expected %d, got %d\n", length+1, count);
        close(fd);
        return(false);
    }
    close(fd);

    return true;
}
