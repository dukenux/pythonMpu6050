
#include <Python.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <cstdint>
#include <csignal>
#include <pthread.h>
#include <inttypes.h>
#include <time.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


typedef struct
{
	// orientation/motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float euler[3];         // [psi, theta, phi]    Euler angle container
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	float velocity[3];
	float acclin[3];
    int32_t gyro[3];	
	pthread_t thread_imu;
	pthread_mutex_t mutex_imu;
	int init_ok;
}
imu_t;
 
static imu_t mpu; 


typedef unsigned char uint8_t;


// Le gyro renvoit un angle de 0 à 180 ou de 0 à -180. On met tout en positif
static float convert_360(float in_angle)
{
	if (in_angle<0) return (in_angle+360);
	else return in_angle;
}

/* Fonction pour le thread de l'imu. */
static void * fn_imu (void * p_data)
{
	MPU6050 mpu6050;
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer
	VectorInt16 aaReal;    // [x, y, z]            world-frame accel sensor measurements
	
    // initialize device
    //printf("Initializing I2C devices...\n");
    mpu6050.initialize();

    // verify connection
    //printf("Testing device connections...\n");
    //printf(mpu6050.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // load and configure the DMP
    //printf("Initializing DMP...\n");
    devStatus = mpu6050.dmpInitialize();

     if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //printf("Enabling DMP...\n");
        mpu6050.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu6050.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
	   pthread_mutex_lock (& mpu.mutex_imu);	   
	   //printf("Préparartion imu en cours\n");
	   mpu.init_ok=1;
	   pthread_mutex_unlock (& mpu.mutex_imu);      
       //printf("DMP ready!\n");
       // get expected DMP packet size for later comparison
       packetSize = mpu6050.dmpGetFIFOPacketSize();
       //printf("Full scale: %d\n",mpu6050.getFullScaleAccelRange());       
	   while (1)
	   {
		  // get current FIFO count
		  fifoCount = mpu6050.getFIFOCount();
		  if (fifoCount == 1024) {
			  // reset so we can continue cleanly
			  mpu6050.resetFIFO();
			  //printf("FIFO overflow!\n");
		  // otherwise, check for DMP data ready interrupt (this should happen frequently)
          } else if (fifoCount >= 42) {
			pthread_mutex_lock (& mpu.mutex_imu);
			// read a packet from FIFO
			mpu6050.getFIFOBytes(fifoBuffer, packetSize);
            mpu6050.dmpGetQuaternion(&mpu.q, fifoBuffer);
            mpu6050.dmpGetAccel(&mpu.aa, fifoBuffer);
            mpu6050.dmpGetGravity(&mpu.gravity, &mpu.q);
            mpu6050.dmpGetEuler(mpu.euler, &mpu.q);
            mpu6050.dmpGetLinearAccel(&aaReal, &mpu.aa, &mpu.gravity);
            mpu6050.dmpGetYawPitchRoll(mpu.ypr,&mpu.q,&mpu.gravity);
			mpu6050.dmpGetGyro(mpu.gyro, fifoBuffer);   
            //printf("areal %6d %6d %6d    \n", gyro.aa.x, gyro.aa.y, gyro.aa.z);
			mpu.acclin[0]=aaReal.x*0.000598f; // 9.81/16394 (full scale is 2g = 16384); //aaReal = m/s²
			mpu.acclin[1]=aaReal.y*0.000598f;
			mpu.acclin[2]=aaReal.z*0.000598f;
			pthread_mutex_unlock (& mpu.mutex_imu);      
		  }	   
		  usleep(12000);
	   }
   } else printf("DMP Initialization failed (code %d)\n", devStatus);
}



extern "C" {
	static PyObject* ypr(PyObject* self)
	{
		float yawPitchRoll[]={999.,999.,999.};
		pthread_mutex_lock (& mpu.mutex_imu);
		yawPitchRoll[0]=mpu.ypr[0]*180/M_PI;
		yawPitchRoll[1]=mpu.ypr[1]*180/M_PI;
		yawPitchRoll[2]=mpu.ypr[2]*180/M_PI;
		pthread_mutex_unlock (& mpu.mutex_imu);
		return Py_BuildValue("ddd",convert_360(yawPitchRoll[0]),convert_360(yawPitchRoll[1]),convert_360(yawPitchRoll[2]));
	}

	static PyObject* acc(PyObject* self)
	{
		float acc[]={999.,999.,999.};
		pthread_mutex_lock (& mpu.mutex_imu);
		acc[0]=mpu.acclin[0];
		acc[1]=mpu.acclin[1];
		acc[2]=mpu.acclin[2];
		pthread_mutex_unlock (& mpu.mutex_imu);
		return Py_BuildValue("ddd",acc[0],acc[1],acc[2]);
	}

	static PyObject* gyr(PyObject* self)
	{
		int32_t g[]={999,999,999};
		pthread_mutex_lock (& mpu.mutex_imu);
		g[0]=mpu.gyro[0];
		g[1]=mpu.gyro[1];
		g[2]=mpu.gyro[2];
		pthread_mutex_unlock (& mpu.mutex_imu);
		return Py_BuildValue("iii",g[0],g[1],g[2]);
	}

	static PyObject* quaternion(PyObject* self)
	{
		float q[]={999.,999.,999.,.999};
		pthread_mutex_lock (& mpu.mutex_imu);
		q[0]=mpu.q.w;
		q[1]=mpu.q.x;
		q[2]=mpu.q.y;
		q[3]=mpu.q.z;
		pthread_mutex_unlock (& mpu.mutex_imu);
		return Py_BuildValue("dddd",q[0],q[1],q[2],q[3]);
	}

	static PyObject* euler(PyObject* self)
	{
		float e[]={999.,999.,999.,.999};
		pthread_mutex_lock (& mpu.mutex_imu);
		e[0]=mpu.euler[0]*180/M_PI;
		e[1]=mpu.euler[1]*180/M_PI;
		e[2]=mpu.euler[2]*180/M_PI;
		pthread_mutex_unlock (& mpu.mutex_imu);
		return Py_BuildValue("ddd",convert_360(e[0]),convert_360(e[1]),convert_360(e[2]));
	}	

	static char imu_docs[] = "acc: fonctions IMU utiles!!\n";

	static PyMethodDef imu_funcs[] = {
		{"ypr", (PyCFunction)ypr, 
		 METH_NOARGS, imu_docs},
		{"acc", (PyCFunction)acc, 
		 METH_NOARGS, imu_docs},
		{"gyro", (PyCFunction)gyr, 
		 METH_NOARGS, imu_docs},
		{"quaternion", (PyCFunction)quaternion, 
		 METH_NOARGS, imu_docs},
		{"euler", (PyCFunction)euler, 
		 METH_NOARGS, imu_docs},
		{ NULL, NULL, 0, NULL }
	};

	DL_EXPORT(void) initimu(void)
	{
		Py_InitModule3("imu", imu_funcs,"Inertial Module Unit");
		/* Creation du thread pour l'imu. */
		mpu.mutex_imu = PTHREAD_MUTEX_INITIALIZER;
		//printf ("Creation du thread imu !\n");
		int ret = pthread_create (
		  & mpu.thread_imu, NULL,
		  fn_imu, NULL
		);
		if (ret) {
			printf("Impossible de créer le thread imu\n");
			return;
		}

		//printf("IMU is now ready, CTRL +C to stop\n");
	}
}
