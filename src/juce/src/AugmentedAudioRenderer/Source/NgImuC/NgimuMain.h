// NgimuMain.h: Main scrip starting NGIMU callback functions.
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once
#include "NgimuReceive.h"
#include <stdlib.h>
#include <stdio.h>
#include "kalman.h"

#ifdef __cplusplus
extern "C" {
#endif
	// NGIMU
	////////////////////////////////////////////////////////////////////////////////////////////////////
	// paramters
	extern bool ngimuPrintSensors; // print read imu data to console
	extern bool ngimuInitialized; // has imu reported first value
	//extern bool ngimuHorizontal; // IMU is used horizontal or vertical (switches Yaw, Pitch and x,y,z coordinate)
	extern float ngimuUpdateRate_ms; // update rate of Ngimu in ms
	extern long ngimuLinearBiasBufferN; // number of elements in bias calibration moving mean buffer
	extern bool ngimuLinearPosOn; // activate position variable
	extern bool ngimuLinearMeanOn; // active mean smoothing variable
	extern long ngimuLinearMeanBufferN; // length of filter used for mean smoothing
	extern float acousticUpdateRateMax_ms; // maximal loop update rate of acoustic head tracker in ms

	// sensor value
	extern float ngimuBarometer; // hPa: barometric pressure
	extern float ngimuEulerRoll; // °: orientation of the device relative to the Earth (North(x), West(y), Up(z) convention)
	extern float ngimuEulerPitch; // °: orientation of the device relative to the Earth (North(x), West(y), Up(z) convention)
	extern float ngimuEulerYaw; // °: orientation of the device relative to the Earth (North(x), West(y), Up(z) convention)
	extern float ngimuLinearX; // g: gravity-free acceleration in the sensor coordinate frame
	extern float ngimuLinearY; // g: gravity-free acceleration in the sensor coordinate frame
	extern float ngimuLinearZ; // g: gravity-free acceleration in the sensor coordinate frame
	extern float ngimuEarthX; // g: gravity-free acceleration in the earth coordinate frame
	extern float ngimuEarthY; // g: gravity-free acceleration in the earth coordinate frame
	extern float ngimuEarthZ; // g: gravity-free acceleration in the earth coordinate frame
	
	// calculated position
	extern struct kals* ngimuLinearPosX; // position of the sensor in the sensor coordinate frame
	extern struct kals* ngimuLinearPosY; // position of the sensor in the sensor coordinate frame
	extern struct kals* ngimuLinearPosZ; // position of the sensor in the sensor coordinate frame

	// calculated mean
	extern float ngimuLinearXMean; // g: average gravity-free acceleration in the sensor coordinate frame
	extern float ngimuLinearYMean; // g: average gravity-free acceleration in the sensor coordinate frame
	extern float ngimuLinearZMean; // g: average gravity-free acceleration in the sensor coordinate frame

	// caculated bias
	extern float ngimuLinearXBias; // bias for gravity-free acceleration in the sensor coordinate frame
	extern float ngimuLinearYBias; // bias for gravity-free acceleration in the sensor coordinate frame
	extern float ngimuLinearZBias; // bias for gravity-free acceleration in the sensor coordinate frame
	extern float ngimuLinearXBiasVar;
	extern float ngimuLinearYBiasVar;
	extern float ngimuLinearZBiasVar;
	extern float* ngimuLinearXBiasBuffer;
	extern float* ngimuLinearYBiasBuffer;
	extern float* ngimuLinearZBiasBuffer;



	// callback functions
	/////////////////////////////////////////////////////////////////////////

	// This function is called each time there is a receive error
	void ngimuReceiveErrorCallback(const char* const errorMessage);

	// This function is called each time a "/sensors" message is received
	void ngimuSensorsCallback(const NgimuSensors ngimuSensors);

	// This function is called each time a "/quaternion" message is received
	void ngimuQuaternionCallback(const NgimuQuaternion ngimuQuaternion);

	// This function is called each time a "/euler" message is received.
	void ngimuEulerCallback(const NgimuEuler ngimuEuler);

	// This function is called each time a "/linear" message is received.
	void ngimuLinearCallback(const NgimuLinear ngimuLinear);

	// This function is called each time a "/earth" message is received.
	void ngimuEarthCallback(const NgimuEarth ngimuEarth);

	// Position tracking (by kalman integration)
	/////////////////////////////////////////////////////////////////////////

	// init position tracking by integrating accleration sensor with kalman filter (position is in x variable of kalman struct for each derivative)
	void ngimuLinearPosCallbackInit();

	// update position tracking by integrating accleration sensor with kalman filter (position is in x variable of kalman struct for each derivative)
	void ngimuLinearPosCallbackUpdate();

	// Mean estimation
	/////////////////////////////////////////////////////////////////////////
	// init all mean buffers
	void ngimuLinearMeanCallbackInit();

	// add new input x to moving mean (imu has to be in rest when called and in the correct pitch and roll coordinate)
	void ngimuLinearMeanCallbackUpdate();

	// destroy all mean buffers
	void ngimuLinearMeanCallbackFinish();


	// bias estimation functions
	/////////////////////////////////////////////////////////////////////////
	// init all bias calibration buffers
	void ngimuLinearCalibrateInit();

	// calibrate all biases with new input x using moving mean (imu has to be in rest when called and in the correct pitch and roll coordinate)
	void ngimuLinearCalibrateUpdate();

	// destroy all bias calibration buffers
	void ngimuLinearCalibrateFinish();


	// buffer functions
	/////////////////////////////////////////////////////////////////////////

	// init single buffer
	float* ngimuBufferInitSingle(long N);

	// calibrate single buffer with new input x using moving mean (imu has to be in rest when called and in the correct pitch and roll coordinate)
	void ngimuBufferUpdateSingle(float x, float* xBiasBuffer, long N, float* xBias, float* xBiasVar);
	
	// destroy single buffer
	void ngimuBufferFinishSingle(float* xBiasBuffer);

	// moving mean and variance buffer initialization
	void ngimu_sample_mean_var_moving_init(float x, float* x_buffer_p, long Nwindow);

	// moving mean and variance buffer update
	void ngimu_sample_mean_var_moving(float x, float* x_buffer_p, long Nwindow, float* mean, float* var);

	// moving mean and variance buffer update using Npast number of observations x (slower as not running calculation)
	void ngimu_sample_mean_var_moving_Npast(float x, float* x_buffer_p, long Nwindow, long Npast, float* mean, float* var);

#ifdef __cplusplus
};
#endif
//------------------------------------------------------------------------------
// End of file
