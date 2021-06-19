// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "NgimuMain.h"

// parameters
bool ngimuPrintSensors = false;
bool ngimuInitialized = false;
//bool ngimuHorizontal = false;
float ngimuUpdateRate_ms = 1.0f;
long ngimuLinearBiasBufferN = 100;
bool ngimuLinearPosOn = false;
bool ngimuLinearMeanOn = false;
long ngimuLinearMeanBufferN = 0;
float acousticUpdateRateMax_ms = 300.0f; // automatical set if not rapid measurement technique

// variables

// sensor value
float ngimuBarometer = 0.0f;
float ngimuEulerRoll = 0.0f;
float ngimuEulerPitch = 0.0f;
float ngimuEulerYaw = 0.0f;
float ngimuLinearX = 0.0f;
float ngimuLinearY = 0.0f;
float ngimuLinearZ = 0.0f;
float ngimuEarthX = 0.0f;
float ngimuEarthY = 0.0f;
float ngimuEarthZ = 0.0f;

// calculated linear position
struct kals* ngimuLinearPosX;
struct kals* ngimuLinearPosY;
struct kals* ngimuLinearPosZ;


// calculated linear mean
float ngimuLinearXMean = 0.0f;
float ngimuLinearYMean = 0.0f;
float ngimuLinearZMean = 0.0f;
float ngimuLinearXVar = 0.0f;
float ngimuLinearYVar = 0.0f;
float ngimuLinearZVar = 0.0f;
float* ngimuLinearXMeanBuffer;
float* ngimuLinearYMeanBuffer;
float* ngimuLinearZMeanBuffer;


// calculated linear bias
float ngimuLinearXBias = 0.0f;
float ngimuLinearYBias = 0.0f;
float ngimuLinearZBias = 0.0f;
float ngimuLinearXBiasVar = 0.0f;
float ngimuLinearYBiasVar = 0.0f;
float ngimuLinearZBiasVar = 0.0f;
float* ngimuLinearXBiasBuffer;
float* ngimuLinearYBiasBuffer;
float* ngimuLinearZBiasBuffer;


// callback functions
/////////////////////////////////////////////////////////////////////////

void ngimuReceiveErrorCallback(const char* const errorMessage) {
	if (ngimuPrintSensors) {
		printf(errorMessage);
		printf("\r\n");
	}
}

void ngimuSensorsCallback(const NgimuSensors ngimuSensors) {
	if (ngimuPrintSensors) {
		printf("/sensors, ");
		printf("%3.3f", ngimuSensors.gyroscopeX);
		printf(", ");
		printf("%3.3f", ngimuSensors.gyroscopeY);
		printf(", ");
		printf("%3.3f", ngimuSensors.gyroscopeZ);
		printf(", ");
		printf("%3.3f", ngimuSensors.accelerometerX);
		printf(", ");
		printf("%3.3f", ngimuSensors.accelerometerY);
		printf(", ");
		printf("%3.3f", ngimuSensors.accelerometerZ);
		printf(", ");
		printf("%3.3f", ngimuSensors.magnetometerX);
		printf(", ");
		printf("%3.3f", ngimuSensors.magnetometerY);
		printf(", ");
		printf("%3.3f", ngimuSensors.magnetometerZ);
		printf("%3.3f", ngimuSensors.barometer);
		printf("\r\n");
	}
	// show that imu reads values
	ngimuInitialized = true;

	// assign
	ngimuBarometer = ngimuSensors.barometer;
}

void ngimuQuaternionCallback(const NgimuQuaternion ngimuQuaternion) {
	if (ngimuPrintSensors) {
		printf("/quaternion, ");
		printf("%3.3f", ngimuQuaternion.w);
		printf(", ");
		printf("%3.3f", ngimuQuaternion.x);
		printf(", ");
		printf("%3.3f", ngimuQuaternion.y);
		printf(", ");
		printf("%3.3f", ngimuQuaternion.z);
		printf("\r\n");
	}
}

void ngimuEulerCallback(const NgimuEuler ngimuEuler) {
	if (ngimuPrintSensors) {
		printf("/euler, ");
		printf("%3.3f", ngimuEuler.roll);
		printf(", ");
		printf("%3.3f", ngimuEuler.pitch);
		printf(", ");
		printf("%3.3f", ngimuEuler.yaw);
		printf("\r\n");
	}
	// show that imu reads values
	ngimuInitialized = true;

	// assign
		ngimuEulerRoll = ngimuEuler.roll;
		ngimuEulerPitch = ngimuEuler.pitch;
		ngimuEulerYaw = ngimuEuler.yaw;

}

void ngimuLinearCallback(const NgimuLinear ngimuLinear) {
	if (ngimuPrintSensors) {
		printf("/linear, ");
		printf("%3.3f", ngimuLinear.x);
		printf(", ");
		printf("%3.3f", ngimuLinear.y);
		printf(", ");
		printf("%3.3f", ngimuLinear.z);
		printf("\r\n");
	}
	// show that imu reads values
	ngimuInitialized = true;

	// assign
	ngimuLinearX = ngimuLinear.x - ngimuLinearXBias;
	ngimuLinearY = ngimuLinear.y - ngimuLinearYBias;
	ngimuLinearZ = ngimuLinear.z - ngimuLinearZBias;

	// update position
	if (ngimuLinearPosOn) {
		ngimuLinearPosCallbackUpdate();
	}
	if (ngimuLinearMeanOn) {
		ngimuLinearMeanCallbackUpdate();
	}
}

void ngimuEarthCallback(const NgimuEarth ngimuEarth) {
	if (ngimuPrintSensors) {
		printf("/linear, ");
		printf("%3.3f", ngimuEarth.x);
		printf(", ");
		printf("%3.3f", ngimuEarth.y);
		printf(", ");
		printf("%3.3f", ngimuEarth.z);
		printf("\r\n");
	}
	// show that imu reads values
	ngimuInitialized = true;



	// update position
	//ngimuPosUpdate(ngimuEarthX, ngimuEarthY, ngimuEarthZ);
}

// Acceleration integration with Kalman kinematics model
/////////////////////////////////////////////////////////////////////////

void ngimuLinearPosCallbackInit() {
	// parameters
	float ngimu_rq = 1.0f; // only observation model
	float* ngimu_r = ar_value(A_Nderivative, A_roff * A_roff);
	ngimu_r[A_acc] = A_ron * A_ron;
	float ngimu_T = ngimuUpdateRate_ms / 1000.0f;
	float ngimu_refresh_rq = false;
	int ngimu_Ninitsteps = 1000;

	// init
	ngimuLinearPosX = (struct kals *)malloc(sizeof(struct kals));
	ngimuLinearPosY = (struct kals *)malloc(sizeof(struct kals));
	ngimuLinearPosZ = (struct kals *)malloc(sizeof(struct kals));

	// assign kalman filter
	init_kalman_1dof(ngimuLinearPosX, ngimu_rq, ngimu_r, ngimu_T, NULL, ngimu_refresh_rq, ngimu_Ninitsteps);
	init_kalman_1dof(ngimuLinearPosY, ngimu_rq, ngimu_r, ngimu_T, NULL, ngimu_refresh_rq, ngimu_Ninitsteps);
	init_kalman_1dof(ngimuLinearPosZ, ngimu_rq, ngimu_r, ngimu_T, NULL, ngimu_refresh_rq, ngimu_Ninitsteps);

	// free memory
	free(ngimu_r);

	// update flags
	ngimuLinearPosOn = true;
}

void ngimuLinearPosCallbackUpdate() {
	ngimuLinearPosX->y[A_acc] = ngimuLinearX;
	ngimuLinearPosY->y[A_acc] = ngimuLinearY;
	ngimuLinearPosZ->y[A_acc] = ngimuLinearZ;
	update_kalman_1dof(ngimuLinearPosX);
	update_kalman_1dof(ngimuLinearPosY);
	update_kalman_1dof(ngimuLinearPosZ);
}

// Mean calculation
/////////////////////////////////////////////////////////////////////////

void ngimuLinearMeanCallbackInit() {
	if (!rapid_on) {
		acousticUpdateRateMax_ms = rec_ms; // known update rate if not-rapid ir measurement technique
	}
	ngimuLinearMeanBufferN = (long)roundf(acousticUpdateRateMax_ms / ngimuUpdateRate_ms);
	ngimuLinearXMeanBuffer = ngimuBufferInitSingle(ngimuLinearMeanBufferN);
	ngimuLinearYMeanBuffer = ngimuBufferInitSingle(ngimuLinearMeanBufferN);
	ngimuLinearZMeanBuffer = ngimuBufferInitSingle(ngimuLinearMeanBufferN);

	// update flags
	ngimuLinearMeanOn = true;
}

void ngimuLinearMeanCallbackUpdate() {
	if (rapid_on) {
		long ngimuLinearMeanBufferNpast = (long)roundf(update_Tdiff * 1000.0f);
		ngimu_sample_mean_var_moving_Npast(ngimuLinearX, ngimuLinearXMeanBuffer, ngimuLinearMeanBufferN, ngimuLinearMeanBufferNpast, &ngimuLinearXMean, &ngimuLinearXVar);
		ngimu_sample_mean_var_moving_Npast(ngimuLinearY, ngimuLinearYMeanBuffer, ngimuLinearMeanBufferN, ngimuLinearMeanBufferNpast, &ngimuLinearYMean, &ngimuLinearYVar);
		ngimu_sample_mean_var_moving_Npast(ngimuLinearZ, ngimuLinearZMeanBuffer, ngimuLinearMeanBufferN, ngimuLinearMeanBufferNpast, &ngimuLinearZMean, &ngimuLinearZVar);
	}
	else {
		ngimuBufferUpdateSingle(ngimuLinearX, ngimuLinearXMeanBuffer, ngimuLinearMeanBufferN, &ngimuLinearXMean, &ngimuLinearXVar);
		ngimuBufferUpdateSingle(ngimuLinearY, ngimuLinearYMeanBuffer, ngimuLinearMeanBufferN, &ngimuLinearYMean, &ngimuLinearYVar);
		ngimuBufferUpdateSingle(ngimuLinearZ, ngimuLinearZMeanBuffer, ngimuLinearMeanBufferN, &ngimuLinearZMean, &ngimuLinearZVar);
	}
}

void ngimuLinearMeanCallbackFinish() {
	ngimuBufferFinishSingle(ngimuLinearXMeanBuffer);
	ngimuBufferFinishSingle(ngimuLinearYMeanBuffer);
	ngimuBufferFinishSingle(ngimuLinearZMeanBuffer);
}


// bias estimation functions
/////////////////////////////////////////////////////////////////////////

void ngimuLinearCalibrateInit() {
	ngimuLinearXBiasBuffer = ngimuBufferInitSingle(ngimuLinearBiasBufferN);
	ngimuLinearYBiasBuffer = ngimuBufferInitSingle(ngimuLinearBiasBufferN);
	ngimuLinearZBiasBuffer = ngimuBufferInitSingle(ngimuLinearBiasBufferN);
}

void ngimuLinearCalibrateUpdate() {
	ngimuBufferUpdateSingle(ngimuLinearX, ngimuLinearXBiasBuffer, ngimuLinearBiasBufferN, &ngimuLinearXBias, &ngimuLinearXBiasVar);
	ngimuBufferUpdateSingle(ngimuLinearY, ngimuLinearYBiasBuffer, ngimuLinearBiasBufferN, &ngimuLinearYBias, &ngimuLinearYBiasVar);
	ngimuBufferUpdateSingle(ngimuLinearZ, ngimuLinearZBiasBuffer, ngimuLinearBiasBufferN, &ngimuLinearZBias, &ngimuLinearZBiasVar);
}

void ngimuLinearCalibrateFinish() {
	ngimuBufferFinishSingle(ngimuLinearXBiasBuffer);
	ngimuBufferFinishSingle(ngimuLinearYBiasBuffer);
	ngimuBufferFinishSingle(ngimuLinearZBiasBuffer);
}

// buffer functions
/////////////////////////////////////////////////////////////////////////

float* ngimuBufferInitSingle(long N) {
	float* xBiasBuffer = (float*)malloc((N + 3) * sizeof(float));
	ngimu_sample_mean_var_moving_init(0.0f, xBiasBuffer, N);
	return xBiasBuffer;
}


void ngimuBufferUpdateSingle(float x, float* xBiasBuffer, long N, float *xBias, float *xBiasVar) {
	ngimu_sample_mean_var_moving(x, xBiasBuffer, N, xBias, xBiasVar);
}

void ngimuBufferFinishSingle(float* xBiasBuffer) {
	free(xBiasBuffer);
}

void ngimu_sample_mean_var_moving_init(float x, float* x_buffer_p, long Nwindow) {
	for (long n = 0; n < Nwindow; n++) { 
		x_buffer_p[n] = x; // set all past values to current value (constant observation x)
	}
	x_buffer_p[Nwindow] = (float)(Nwindow - 1); // pointer to oldest value (start at last window point)
	x_buffer_p[Nwindow + 1] = x * Nwindow; // past sum of all values
	x_buffer_p[Nwindow + 2] = x * x * Nwindow; // past sum of all squared values
}

void ngimu_sample_mean_var_moving(float x, float * x_buffer_p, long Nwindow, float* mean, float* var) {
	// background: https://www.dsprelated.com/showthread/comp.dsp/97276-1.php

	float x2;
	float x_p; 
	float x2_p;
	long Np; // oldest bin from buffer
	float x_sum;
	float x2_sum;
	float x_sum_p;
	float x2_sum_p;
	float Nwindowf;
	Nwindowf = (float)Nwindow;

	// load past values from buffer
	Np = (long)x_buffer_p[Nwindow]; // pointer to oldest value (start at 0)
	x_p = x_buffer_p[Np];
	x_sum_p = x_buffer_p[Nwindow + 1]; // past sum of all values
	x2_sum_p = x_buffer_p[Nwindow + 2]; // past sum of all squared values

	// init new sum element
	x2 = x * x;
	x2_p = x_p * x_p;

	// correct past sum
	x_sum = x_sum_p + x - x_p;
	x2_sum = x2_sum_p + x2 - x2_p;

	// update buffer values
	x_buffer_p[Np] = x;
	x_buffer_p[Nwindow + 1] = x_sum;
	x_buffer_p[Nwindow + 2] = x2_sum;

	// increment pointer to point to last x value in buffer
	Np = Np + 1;
	if (Np >= Nwindow) {
		Np = 0;
	}
	x_buffer_p[Nwindow] = (float)Np;

	// calculate (moving) mean
	*mean = x_sum / Nwindowf;

	// calculate (moving) variance
	// *var = (Nwindowf * x2_sum - (x_sum * x_sum)) / (Nwindowf * (Nwindowf - 1.0f));
	*var = x2_sum / Nwindowf - (x_sum / Nwindowf * x_sum / Nwindowf);

}

void ngimu_sample_mean_var_moving_Npast(float x, float* x_buffer_p, long Nwindow,  long Npast, float* mean, float* var) {
	// init
	long n;
	float tmp;
	float tmp_square;
	long nwindow;
	long Np;
	tmp = 0.0f;
	tmp_square = 0.0f;
	Np = (long)x_buffer_p[Nwindow];
	nwindow = Np;

	// calculate mean and variance
	if (Npast != 0) {
		for (n = 0; n < Npast; n++) {
			nwindow -= 1;
			if (nwindow == -1) {
				nwindow = Nwindow - 1;
			}
			tmp += x_buffer_p[nwindow];
			tmp_square += tmp * tmp;

		}

		tmp /= (float)Npast;
		tmp_square /= (float)Npast;

		*mean = tmp;
		*var = tmp_square - tmp * tmp;
	}

	// update buffer values
	x_buffer_p[Np] = x;

	// increment pointer to point to oldest x value in buffer
	Np = Np + 1;
	if (Np >= Nwindow) {
		Np = 0;
	}
	x_buffer_p[Nwindow] = (float)Np;
}


#ifdef __cplusplus
};
#endif