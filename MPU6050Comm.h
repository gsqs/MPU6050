#ifndef MPU6050COMM_H

#include <iostream>
#include <string>
#include <array>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include "I2CComm.h"
#define MPU6050COMM_H

class MPU6050Comm
{
    public:
        MPU6050Comm();
		MPU6050Comm(uint8_t port);
		float convertAccel(int16_t rawValue);
		float convertGyro(int16_t rawValue);
		float convertTemp(int16_t rawValue);
		void getData(float* data);
		void getOffset();
		void getAngles(float* acquiredData, float* angles, float* angles360, unsigned long dt);
    protected:
    private:

};

#endif // MPU6050COMM_H