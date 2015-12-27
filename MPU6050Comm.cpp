/** MPU6050 communcations service, used with BeagleBone Black.
Author: GSQ
Creation date: 12/26/2015
Changelog:
12/26/2015 - Creation

Compile as follows: g++ MPU6050Comm.cpp I2CComm.cpp -o MPU6050 -std=c++0x -lrt
**/
//#define standalone

#include <time.h>
#include <math.h>
#include "MPU6050Comm.h"

#define MPU6050_ADDRESS 0x68
#define PWR_MANAGEMENT_ADDRESS 0x6B
#define ACCELX_ADDRESS 0x3B
#define accel_dynamic_range 2.0 //(pm) g
#define gyro_dynamic_range 250.0 //(pm) deg/s
#define PI 3.141592
#define SIGNIFICANCE_FACTOR 0.95

static uint8_t I2CPort;
const unsigned long long nano = 1000000000;
static float gyrox_off;
static float gyroy_off;
static float gyroz_off;
static uint8_t* buf = new uint8_t[14];

/* Default constructor. */
MPU6050Comm::MPU6050Comm() {
	I2CPort = 0;
}

/* Constructor specifying I2C library port. */
MPU6050Comm::MPU6050Comm(uint8_t port) {
	if (port <= 255 && port >= 0) {
		I2CPort = port;
	} else {
		I2CPort = 0;
	}
}

/* Tester function for conversion to acceleration, returns G's. */
float MPU6050Comm::convertAccel(int16_t rawValue) {
	return (((float) ((float) rawValue)/((float) 65536.0)))*((float) accel_dynamic_range)*2.0;
}

/* Tester function for conversion to angular rotation velocity, returns deg/s. */
float MPU6050Comm::convertGyro(int16_t rawValue) {
	return (((float) ((float) rawValue)/((float) 65536.0)))*((float) gyro_dynamic_range)*2.0;
}

/* Tester function for conversion to degrees Celsius. */
float MPU6050Comm::convertTemp(int16_t rawValue) {
	return ((float) rawValue)/340.0 + 36.53;
}

/* Obtains 3-axis acceleration, temperature, and 3-axis angular velocity. */
void MPU6050Comm::getData(float* data) {
	I2CComm i2cObject(I2CPort);
	uint8_t* somedata = new uint8_t[1];
	somedata[0] = 0;
	i2cObject.writeBytes(MPU6050_ADDRESS, PWR_MANAGEMENT_ADDRESS, 1, somedata);
	i2cObject.readBytes(MPU6050_ADDRESS, ACCELX_ADDRESS, 14, buf, 0, 0x3B | 0b10000000);
	int16_t accelx_raw = buf[0] << 8 | buf[1];
	int16_t accely_raw = buf[2] << 8 | buf[3];
	int16_t accelz_raw = buf[4] << 8 | buf[5];
	int16_t temp_raw = buf[6] << 8 | buf[7];
	int16_t gyrox_raw = buf[8] << 8 | buf[9];
	int16_t gyroy_raw = buf[10] << 8 | buf[11];
	int16_t gyroz_raw = buf[12] << 8 | buf[13];
	float accelx = convertAccel(accelx_raw);
	float accely = convertAccel(accely_raw);
	float accelz = convertAccel(accelz_raw);
	float temp = convertTemp(temp_raw);
	float gyrox = convertGyro(gyrox_raw) - gyrox_off;
	float gyroy = convertGyro(gyroy_raw) - gyroy_off;
	float gyroz = convertGyro(gyroz_raw) - gyroz_off;
	data[0] = accelx;
	data[1] = accely;
	data[2] = accelz;
	data[3] = temp;
	data[4] = gyrox;
	data[5] = gyroy;
	data[6] = gyroz;
}

/* Polls the MPU n times at rest and averages the offset. */
void MPU6050Comm::getOffset() {
	int n = 100;
	float* data = new float[7];
	float curr_gyrox;
	float curr_gyroy;
	float curr_gyroz;
	for (int i = 0; i < n; i += 1) {
		getData(data);
		curr_gyrox = data[4];
		curr_gyroy = data[5];
		curr_gyroz = data[6];
		gyrox_off += curr_gyrox;
		gyroy_off += curr_gyroy;
		gyroz_off += curr_gyroz;
	}
	gyrox_off = gyrox_off/n;
	gyroy_off = gyroy_off/n;
	gyroz_off = gyroz_off/n;
	
	delete data;
}

/* Function used to obtain angle of the MPU6050, corrected by a complementary filter, dt in us. */
void MPU6050Comm::getAngles(float* acquiredData, float* angles, float* angles360, unsigned long dt) {
	float accelx = acquiredData[0];
	float accely = acquiredData[1];
	float accelz = acquiredData[2];
	float gyrox = acquiredData[4];
	float gyroy = acquiredData[5];
	float gyroz = acquiredData[6];
	float anglex = angles[0];
	float angley = angles[1];
	float anglez = angles[2];
	anglex += (gyrox)*((float) dt/1000000000.0);
	angley += (gyroy)*((float) dt/1000000000.0);
	anglez += (gyroz)*((float) dt/1000000000.0);
	float xAcc_ang = atan2(accely, accelz)*180/PI;
	float yAcc_ang = atan2(accelz, accelx)*180/PI;
	float zAcc_ang = atan2(accelx, accely)*180/PI;
	anglex = anglex*SIGNIFICANCE_FACTOR + (1-SIGNIFICANCE_FACTOR)*xAcc_ang;
	
	angley = angley*SIGNIFICANCE_FACTOR + (1-SIGNIFICANCE_FACTOR)*yAcc_ang;
	
	anglez = anglez*SIGNIFICANCE_FACTOR + (1-SIGNIFICANCE_FACTOR)*zAcc_ang;
	
	angles[0] = anglex;
	angles[1] = angley;
	angles[2] = anglez;
	
	float angles360x = anglex;
	float angles360y = angley;
	float angles360z = anglez;

	while (angles360x > 360.0) {
		angles360x -= 360.0;
	}
	while (angles360x < 0) {
		angles360x += 360.0;
	}
	while (angles360y > 360.0) {
		angles360y -= 360.0;
	}
	while (angles360y < 0) {
		angles360y += 360.0;
	}
	while (angles360z > 360.0) {
		angles360z -= 360.0;
	}
	while (angles360z < 0) {
		angles360z += 360.0;
	}
	angles360[0] = angles360x;
	angles360[1] = angles360y;
	angles360[2] = angles360z;
}

#ifdef standalone

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

#endif
