#ifndef I2CCOMM_H

#include <iostream>
#include <string>
#include <array>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#define I2CCOMM_H

class I2CComm
{
    public:
        I2CComm();
		I2CComm(uint8_t devPort);
		int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout, uint8_t readAddress);
		bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data);
    protected:
    private:

};

#endif // I2CCOMM_H
