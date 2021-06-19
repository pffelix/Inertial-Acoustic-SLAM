// kalman.h: Kalman filter for predicting head position.
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once
#include "params.h"
#include "state.h"
#include "array.h"
#include "debugging.h"


#ifdef __cplusplus
extern "C" {
#endif

	//variables
	//////////////////////////////
	//// number of states
	//extern int M;
	//// state matrix
	//extern float **A;
	//// state matrix jacobian
	//extern float **A_J;
	//// state error jacobian
	//extern float **W;
	//// observation matrix
	//extern float *C;
	//// observation matrix jacobian
	//extern float **C_J;
	//// óbservation error jacobian
	//extern float **V;
	//// temp variables
	//extern float **divisor;
	//extern float **numerator;
	//extern float **eyearray;
	//// kalman gain
	//extern float **K_new;
	//// states
	//extern float *x_new;
	//extern float *x;
	//// states apriori
	//extern float *xap_new;
	//extern float *xap;
	//// state error
	//extern float *e_new;
	//extern float *e;
	//// error covariance
	//extern float **P_new;
	//extern float **P;
	//// error covariance apriori
	//extern float **Pap;
	//// Q / R ratio
	//extern float R_n;
	//extern float Q_n;
	//// Process noise covariance
	//extern float **Q;
	//// observation noise covariance
	//extern float **R;
	//// initial error covariance estimate
	//extern float P_n;


	//functions
	// for better understanding of the (Extended) Kalman filter checkout: https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
	//////////////////////////////

	/*  init 1 degree of freedom kalman head tracking filter (semester thesis)
		rq: ratio between noise covariance of observation R vs. noise covariance of kinematics motion model Q (high -> kinematics motion model is considered stronger than observation)
		r: noise covariance of absolute, velocity and acceleration observation (e.g. one value can be set to 1 and the other relative to it as ratio)
		T: time in s between two updates of kalman filter
		limit: state lower and upper limit
	*/
	void init_kalman_1dof(struct kals * k, float rq, float* r, float T, float** limit, bool refresh_rq, int Ninitsteps);

	///*  update kalman filter (memory unsafe)
	//	before calling function updated k->y_new: array with 3 entries containing absoulte, differential and 2nd differential feature (0 if unknown)
	//*/
	//void update_kalman_memory_unsafe(struct kals * k);

	//  pre-initializalize kalman gain and kalman covariance using repeated update of both variables in Nsteps
	void preinit_kalman_gain_cov_1dof(struct kals* k, int Ninitsteps);

	/*  update 1 degree of freedom kalman head tracking filter (semester thesis)
		before calling function updated k->y_new: array with 3 entries containing absoulte, differential and 2nd differential feature (0 if unknown)
	*/
	void update_kalman_1dof(struct kals* k);

	// predict step kalman filter
	void update_kalman_1dof_predict(struct kals* k);

	// correct step kalman filter
	void update_kalman_1dof_correct(struct kals* k);

	// update kalman gain
	void update_kalman_gain_1dof(struct kals* k);

	// update kalman state covariance
	void update_kalman_cov_1dof(struct kals* k);

	/*  init 6 degree of freedom kalman head tracking filter (semester thesis)
		rq: ratio between translation, rotation, and mapping noise covariance of observation R vs. noise covariance of kinematics motion model Q (high -> kinematics motion model is considered stronger than observation)
		r: noise covariance of all observations (e.g. one value can be set to 1 and the other relative to it as ratio)
		T: time in s between two updates of kalman filter
		limit: state lower and upper limit
	*/
	void init_kalman_6dof(struct kal6s * k, float* rq, float* r, float T, int Mframes, float** limit, bool refresh_rq, int Ninitsteps);

	///*  update kalman filter (memory unsafe)
	//	before calling function updated k->y_new: array with 3 entries containing absoulte, differential and 2nd differential feature (0 if unknown)
	//*/
	//void update_kalman_memory_unsafe(struct kals * k);

	//  pre-initializalize kalman gain and kalman covariance using repeated update of both variables in Nsteps
	void preinit_kalman_gain_cov_6dof(struct kal6s* k, int Ninitsteps);

	/*  update 6 degree of freedom kalman head tracking filter (semester thesis)
		before calling function updated k->y_new: array with 3 entries containing absoulte, differential and 2nd differential feature (0 if unknown)
	*/
	void update_kalman_6dof(struct kal6s * k);

	// update kalman gain
	void update_kalman_gain_6dof(struct kal6s* k);

	// update kalman state covariance
	void update_kalman_cov_6dof(struct kal6s* k);
#ifdef __cplusplus
};
#endif