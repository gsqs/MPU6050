CC = g++
CFLAGS = -std=c++0x -lrt

imu:
	$(CC) -o imu Main.cpp MPU6050Comm.cpp I2CComm.cpp $(CFLAGS)

clean:
	rm imu

run:
	./imu
