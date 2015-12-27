/** Main for testing MPU6050, used with BeagleBone Black.
Author: GSQ
Creation date: 12/26/2015
Changelog:
12/26/2015 - Creation

Compile as follows: g++ Main.cpp MPU6050Comm.cpp I2CComm.cpp -o Main -std=c++0x -lrt
**/

#include <iostream>
#include <string>
#include <unistd.h>
#include <array>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include "MPU6050Comm.h"

using namespace std;

const unsigned long long nano = 1000000000;

int main() {
	printf("Beginning MPU6050 communications service testing: \n\r");
	MPU6050Comm mpuObject(1);
	float* MPU_data = new float[7];
	float* angles = new float[3];
	float* angles360 = new float[3];
	
	/* Get offset. */
	mpuObject.getOffset();
	
	/* Timer related code. */
	struct timespec tm;
	
	unsigned long t1, t2, delay;
	clock_gettime(CLOCK_REALTIME, &tm);
	t1 = tm.tv_nsec + tm.tv_sec * nano;
	int count = 100;
	while (true) {
		if (count > 99) {
			printf("======RESULTS=======\n\r");
			printf("accelx = %0.2fg\n\r",MPU_data[0]);
			printf("accely = %0.2fg\n\r",MPU_data[1]);
			printf("accelz = %0.2fg\n\r",MPU_data[2]);
			printf("temp = %0.2f deg C\n\r",MPU_data[3]);
			printf("gyrox = %0.2f deg/s\n\r",MPU_data[4]);
			printf("gyroy = %0.2f deg/s\n\r",MPU_data[5]);
			printf("gyroz = %0.2f deg/s\n\r",MPU_data[6]);
			printf("anglex = %0.2f deg\n\r",angles360[0]);
			printf("angley = %0.2f deg\n\r",angles360[1]);
			printf("anglez = %0.2f deg\n\r",angles360[2]);
			printf("====================\n\r");
			count = 0;
		} else {
			count += 1;
		}
		mpuObject.getData(MPU_data);
		usleep(5000);
		clock_gettime( CLOCK_REALTIME, &tm );
		t2 = tm.tv_nsec + tm.tv_sec * nano;
		delay = t2 - t1;
		clock_gettime( CLOCK_REALTIME, &tm );
		t1 = tm.tv_nsec + tm.tv_sec * nano;
		mpuObject.getAngles(MPU_data, angles, angles360, delay);
	}
	return 0;
}