// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "kalman.h"

//variables
//////////////////////////////


//functions
//////////////////////////////


void init_kalman_1dof(struct kals * k, float rq, float* r, float T, float** limit, bool refresh_rq, int Ninitsteps) {

	///////////////////////////////////
	///////////////////////////////////

	// init 
	int m, MN;

	// set fixed configuration
	// number of states
	k->M = 3;

	// number of observations
	k->N = k->M;

	// observation vector
	k->y = ar_zeros(k->N); // y_new

	// state matrix
	k->A = ar_valuediag2d(k->M, 1.0f);
	k->A[0][1] = T;
	k->A[0][2] = T * T / 2;
	k->A[1][2] = T;

	// state matrix jacobian
	k->A_J = ar_valuediag2d(k->M, 1.0f);
	k->A_J[0][1] = T;
	k->A_J[0][2] = T * T / 2;
	k->A_J[1][2] = T;

	// state error jacobian
	k->W = ar_valuediag2d(k->M, 1.0f);

	///////////////////////////////////
	// set observation matrices
	// observation matrix
	k->C = ar_valuediag2d(k->M, 1.0f); // k->C = ar_zeros(k->M); 
	///k->C[2][2] = 0;

	// observation matrix jacobian
	k->C_J = ar_valuediag2d(k->M, 1.0f);
	//k->C_J[2][2] = 0;
	
	// observation error jacobian
	k->V = ar_valuediag2d(k->N, 1.0f);

	///////////////////////////////////
	//Init further matrices

	// kalman gain
	k->K = ar_zeros2d(k->M, k->N);

	// states
	//k->x_new = ar_zeros(k->M);
	k->x = ar_zeros(k->M);

	// states apriori
	k->xap = ar_zeros(k->M);
	//k->xap = ar_zeros(k->M);

	// state input
	k->u = ar_zeros(k->M);

	// state error
	//k->e_new = ar_zeros(k->M);
	k->e = ar_zeros(k->M);

	// state error covariance
	//k->P_new = ar_zeros2d(k->M, k->M);
	k->P_n = 1.0f;
	k->P = ar_valuediag2d(k->M, k->P_n);

	// error covariance apriori
	k->Pap = ar_zeros2d(k->M, k->M);

	// limit of state variables
	if (limit) {
		k->limit_on = true;
		k->limit = ar_declare2d(k->M, A_limit_N);
		for (m = 0; m < k->M; m++) {
			k->limit[m][A_limit_low] = limit[m][A_limit_low];
			k->limit[m][A_limit_high] = limit[m][A_limit_high];
		}
	} else {
		k->limit_on = false;
	}

	// update rq metrics
	k->refresh_rq = refresh_rq;

	// loop number
	k->Nupdate = 0;

	///////////////////////////////////
	///////////////////////////////////
	//Set adaptable configuration

	// R / Q ratio (how important is model relative to observation)
	k->R_m = 1.0f;
	k->Q_m = k->R_m / rq;

	// Process noise covariance
	k->Q = ar_valuediag2d(k->M, k->Q_m);
	k->Q[0][0] = k->Q[0][0] * T * T * T / 6;
	k->Q[1][1] = k->Q[1][1] * T * T / 2;
	k->Q[2][2] = k->Q[2][2] * T;

	// initial observation noise covariance
	k->R0 = ar_valuediag2d(k->N, k->R_m);
	k->R0[A_ref][A_ref] = k->R0[A_ref][A_ref] * r[A_ref] * T * T * T / 6;
	k->R0[A_vel][A_vel] = k->R0[A_vel][A_vel] * r[A_vel] * T * T / 2;
	k->R0[A_acc][A_acc] = k->R0[A_acc][A_acc] * r[A_acc] * T;

	// observation noise covariance
	k->R = ar_valuediag2d(k->N, k->R_m);
	k->R[A_ref][A_ref] = k->R[A_ref][A_ref] * r[A_ref] * T * T * T / 6;
	k->R[A_vel][A_vel] = k->R[A_vel][A_vel] * r[A_vel] * T * T / 2;
	k->R[A_acc][A_acc] = k->R[A_acc][A_acc] * r[A_acc] * T;

	///////////////////////////////////
	// initial values

	// initial state estimate
	k->x[0] = 0.0f; // 0.01f
	k->x[1] = 0.0f; //0.5f
	k->x[2] = 0.0f;
	
	// initial state input
	k->u[0] = 0.0f;
	k->u[1] = 0.0f;
	k->u[2] = 0.0f;

	// temporary variables
	MN = (k->M > k->N) ? k->M : k->N;
	k->divisor = ar_zeros2d(k->N, k->N);
	k->numerator = ar_zeros2d(k->M, k->N);
	k->eyearray = ar_valuediag2d(k->M, 1.0f);
	k->tmp1_1d = ar_zeros(MN);
	k->tmp1_2d = ar_zeros2d(MN, MN);
	k->tmp2_2d = ar_zeros2d(MN, MN);
	k->tmp3_2d = ar_zeros2d(MN, MN);
	k->tmp4_2d =  ar_zeros2d(MN, MN);

	// preinit kalman gain K and covariance P
	preinit_kalman_gain_cov_1dof(k, Ninitsteps);

};

//// update kalman filter
//void update_kalman_memory_unsafe(struct kals * k) {
//	// temporary variables
//
//	// update apriori estimate
//	k->xap = ar_m2d1dM3 (k->A, k->x);
//
//	// update observation matrix jacobian
//	k->C = k->xap;
//
//	// update apriori error covariance estimate
//	k->Pap = ar_s2dM3(ar_m2dM3(ar_m2dM3(k->A_J, k->P), ar_tp2dM3(k->A_J)), 
//	ar_m2dM3(ar_m2dM3(k->W, k->Q), ar_tp2dM3(k->W)));
//
//	// update Kalman gain
//	k->divisor = ar_s2dM3(ar_m2dM3(ar_m2dM3(k->C_J, k->Pap), 
//	ar_tp2dM3(k->C_J)), ar_m2dM3(ar_m2dM3(k->V, k->R), ar_tp2dM3(k->V)));
//	k->numerator = ar_m2dM3(k->Pap, ar_tp2dM3(k->C_J));
//	k->K = ar_m2dM3(k->numerator, ar_inv2dM3(k->divisor));
//
//	// update error
//	k->e = ar_sM3(k->y_new, ar_negM3(k->C));
//
//	// update aposteriori state estimate
//	k->x_new = ar_sM3(k->xap, ar_m2d1dM3(k->K, k->e));
//
//	// update aposteriori error covariance estimate
//	k->P_new = ar_s2dM3(k->eyearray, ar_neg2dM3(ar_m2dM3(ar_m2dM3(k->K, k->C_J), k->Pap)));
//
//	// update variables
//	//memcpy(k->x, k->x_new, M * sizeof(*fe->itd));
//
//	k->x = k->x_new;
//	//k->xap = k->xap_new;
//	//k->e = k->e_new;
//	k->P = k->P_new;
//};

void preinit_kalman_gain_cov_1dof(struct kals* k, int Ninitsteps) {
	// Debugging
	//bool deb_disk_save = deb_disk;
	//deb_disk = false;

	int n;
	for (n = 0; n < Ninitsteps; n++) {
		update_kalman_gain_1dof(k);
		update_kalman_cov_1dof(k);
	}

	// Debugging
	//deb_disk = deb_disk_save;
}

void update_kalman_1dof(struct kals* k) {
	update_kalman_1dof_predict(k);
	update_kalman_1dof_correct(k);
};

void update_kalman_1dof_predict(struct kals* k) {

	// Debugging
	//bool deb_disk_save = deb_disk;
	//deb_disk = false;
	//write_txt_debug(k->x, k->M, 0);
	//write_txt_debug2d(k->A, k->M, k->M, 0);

	// init
	int m;

	// update apriori state estimate
	ar_m2d1dM3(k->A, k->x, k->xap);
	//write_txt_debug(k->xap, k->M, 0);

	// add input to apriori state estimate
	ar_sM3(k->xap, k->u, k->xap);
	//write_txt_debug(k->xap, k->M, 0);

	// limit apriori state estimate
	if (k->limit_on) {
		for (m = 0; m < k->M; m++) {
			if (k->limit[m][A_limit_high] != INFINITY) {
				if (k->limit[m][A_limit_low] == 0.0f && k->limit[m][A_limit_high] == 360.0f) {
					correct_limit_circular(&k->xap[m], k->limit[m][A_limit_low], k->limit[m][A_limit_high]);
				}
				else {
					correct_limit(&k->xap[m], k->limit[m][A_limit_low], k->limit[m][A_limit_high]);
				}
			}
		}
	}

	// new kalman gain
	if (k->refresh_rq) {
		update_kalman_gain_1dof(k);
	}

	// update observation estimate
	ar_m2d1dM3(k->C, k->xap, k->tmp1_1d); // k->C[0] = k->xap[0]; k->C[1] = k->xap[1]; k->C[2] = 0.0f;
	//write_txt_debug(k->tmp1_1d, k->N, 0); // write_txt_debug(k->xap, k->M, 0);

	// update state error
	ar_negM3(k->tmp1_1d, k->tmp1_1d); // ar_negM3(k->C, k->tmp1_1d); 
	//write_txt_debug(k->tmp1_1d, k->N, 0);

}

void update_kalman_1dof_correct(struct kals* k) {
	// init
	int m;

	ar_sM3(k->y, k->tmp1_1d, k->e); // y_new
	//write_txt_debug(k->e, k->N, 0);

	// adapt estimation error to 360 degree circular error
	if (k->limit_on) {
		for (m = 0; m < k->M; m++) {
			if (k->limit[m][A_limit_low] == 0.0f && k->limit[m][A_limit_high] == 360.0f) {
				angdiff_to_circular360(&k->e[m]);
			}
		}
	}

	// update aposteriori state estimate
	ar_m2d1dM3(k->K, k->e, k->tmp1_1d);
	//write_txt_debug(k->tmp1_1d, k->M, 0);

	ar_sM3(k->xap, k->tmp1_1d, k->x); // x_new
	//write_txt_debug(k->x, k->M, 0); // x_new

	// limit aposteriori state estimate
	if (k->limit_on) {
		for (m = 0; m < k->M; m++) {
			if (k->limit[m][A_limit_high] != INFINITY) {
				if (k->limit[m][A_limit_low] == 0.0f && k->limit[m][A_limit_high] == 360.0f) {
					correct_limit_circular(&k->xap[m], k->limit[m][A_limit_low], k->limit[m][A_limit_high]);
				}
				else {
					correct_limit(&k->xap[m], k->limit[m][A_limit_low], k->limit[m][A_limit_high]);
				}
			}
		}
	}

	// new state covariance
	if (k->refresh_rq) {
		update_kalman_cov_1dof(k);
	}

	// update update number
	k->Nupdate += 1;


	// Debugging
	//deb_disk = deb_disk_save;
}

void update_kalman_gain_1dof(struct kals * k) {

	// update apriori state error covariance estimate
	// first summand (P after movement) (identical if Q && R no change)
	ar_m2dM3(k->A_J, k->P, k->tmp1_2d);
	//write_txt_debug2d(k->tmp1_2d, k->M, k->M, 0);

	ar_tp2dM3(k->A_J, k->tmp2_2d);
	//write_txt_debug2d(k->tmp2_2d, k->M, k->M, 0);

	ar_m2dM3(k->tmp1_2d, k->tmp2_2d, k->tmp3_2d);
	//write_txt_debug2d(k->tmp3_2d, k->M, k->M, 0);

	// second summand (Q) (identical if Q no change)
	ar_m2dM3(k->W, k->Q, k->tmp1_2d);
	//write_txt_debug2d(k->tmp1_2d, k->M, k->M, 0);

	ar_tp2dM3(k->W, k->tmp2_2d);
	//write_txt_debug2d(k->tmp2_2d, k->M, k->M, 0);

	ar_m2dM3(k->tmp1_2d, k->tmp2_2d, k->tmp4_2d);
	//write_txt_debug2d(k->tmp4_2d, k->M, k->M, 0);

	// update (identical if Q && R no change)
	ar_s2dM3(k->tmp3_2d, k->tmp4_2d, k->Pap);
	//write_txt_debug2d(k->Pap, k->M, k->M, 0);


	// update Kalman gain divisor
	// first summand (P to P_y) (identical if Q && R no change)
	ar_m2dM3(k->C_J, k->Pap, k->tmp1_2d);
	//write_txt_debug2d(k->tmp1_2d, k->N, k->M, 0);

	ar_tp2dM3(k->C_J, k->tmp2_2d);
	//write_txt_debug2d(k->tmp2_2d, k->M, k->N, 0);

	ar_m2dM3(k->tmp1_2d, k->tmp2_2d, k->tmp3_2d);
	//write_txt_debug2d(k->tmp3_2d, k->N, k->N, 0);

	// second summand (R) (identical if R no change)
	ar_m2dM3(k->V, k->R, k->tmp1_2d);
	//write_txt_debug2d(k->tmp1_2d, k->N, k->N, 0);

	ar_tp2dM3(k->V, k->tmp2_2d);
	//write_txt_debug2d(k->tmp2_2d, k->N, k->N, 0);

	ar_m2dM3(k->tmp1_2d, k->tmp2_2d, k->tmp4_2d);
	//write_txt_debug2d(k->tmp4_2d, k->N, k->N, 0);

	// update (identical if Q && R no change)
	ar_s2dM3(k->tmp3_2d, k->tmp4_2d, k->divisor);
	//write_txt_debug2d(k->divisor, k->N, k->N, 0);


	// update Kalman gain numerator (identical if Q && R no change)
	ar_tp2dM3(k->C_J, k->tmp1_2d);
	//write_txt_debug2d(k->tmp1_2d, k->M, k->N, 0);

	ar_m2dM3(k->Pap, k->tmp1_2d, k->numerator);
	//write_txt_debug2d(k->numerator, k->M, k->N, 0);



	// invert Kalman gain divisor (identical if Q && R no change)
	ar_inv2dM3(k->divisor, k->tmp1_2d);
	//write_txt_debug2d(k->tmp1_2d, k->N, k->N, 0);


	// update Kalman gain (identical if Q && R no change)
	ar_m2dM3(k->numerator, k->tmp1_2d, k->K);
	//write_txt_debug2d(k->K, k->M, k->N, 0);

}

void update_kalman_cov_1dof(struct kals * k) {

	// update aposteriori state error covariance estimate
	// second term (identical if Q && R no change)
	ar_m2dM3(k->K, k->C_J, k->tmp1_2d);
	//write_txt_debug2d(k->tmp1_2d, k->M, k->M, 0);

	ar_neg2dM3(k->tmp1_2d, k->tmp2_2d);
	//write_txt_debug2d(k->tmp2_2d, k->M, k->M, 0);

	// difference term (identical if Q && R no change)
	ar_s2dM3(k->eyearray, k->tmp2_2d, k->tmp3_2d); // P_new
	//write_txt_debug2d(k->tmp3_2d, k->M, k->M, 0); // P_new

	// multiply (identical if Q && R no change)
	ar_m2dM3(k->tmp3_2d, k->Pap, k->P);
	//write_txt_debug2d(k->P, k->M, k->M, 0);

}

void init_kalman_6dof(struct kal6s * k, float* rq, float* r, float T, int Mframes, float ** limit, bool refresh_rq, int Ninitsteps) {

	///////////////////////////////////
	///////////////////////////////////
	// Debugging
	//bool deb_disk_save = deb_disk;
	//deb_disk = false;

	// definitions
	int NXtrans;
	int NXrot;
	int NXrotmap;
	int NYtrans;
	int NYrotimu;
	int NYrotacu;

	int NXpartial;
	int NYpartial;
	int* Xpartial;
	int* Ypartial;

	int Nderivative;
	int Ndof;

	float** A_kin;
	float* Q_kin;
	int m;
	int n;
	int i;
	int j;
	int MN;


	// set fixed configuration
	NXtrans = 3 * 3; // (3 x,y,z translation * 3 derivatives)
	NXrot = 3 * 3; // (3 yaw,pitch,roll rotation * 3 derivatives)
	NXrotmap = 3 * Mframes; // (3 map yaw,pitch,roll imu/acoustic rotation * Mframes)
	NYtrans = 3 * 3; // (3 x,y,z imu/acoustic translation * 3 derivatives)
	NYrotimu = 3; //  (3 yaw,pitch,roll imu rotation)
	NYrotacu = 3 * Mframes; // (3 yaw,pitch,roll acoustic rotation * Mframes)

	Nderivative = 3;
	Ndof = 3;

	NXpartial = 3; // 4 partial states
	NYpartial = 3; // 3 partial observations
	Xpartial = ar_declarei(NXpartial);
	Ypartial = ar_declarei(NYpartial);
	Xpartial[Xtrans] = 0;
	Xpartial[Xrot] = Xpartial[0] + NXtrans;
	Xpartial[Xrotmap] = Xpartial[1] + NXrot;
	Ypartial[Ytrans] = 0;
	Ypartial[Yrotimu] = Ypartial[0] + NYtrans;
	Ypartial[Yrotacu] = Ypartial[1] + NYrotimu;


	// number of states
	k->M = NXtrans + NXrot + NXrotmap;

	// number of observations
	k->N = NYtrans + NYrotimu + NYrotacu;

	// observation vector
	k->y = ar_zeros(k->N);

	// state matrix
	A_kin = ar_valuediag2d(k->M, 1.0f);
	A_kin[0][1] = T;
	A_kin[0][2] = T * T / 2;
	A_kin[1][2] = T;
	k->A = ar_valuediag2d(k->M, 1.0f);
	for (i = 0; i < 2 * Ndof; i++) {
		m = Xpartial[Xtrans] + i * Nderivative;
		k->A[0 + m][1 + m] = A_kin[0][1];
		k->A[0 + m][2 + m] = A_kin[0][2];
		k->A[1 + m][2 + m] = A_kin[1][2];
	}
	//write_txt_debug2d(k->A, k->M, k->M, 0);

	// state matrix jacobian
	k->A_J = ar_valuediag2d(k->M, 1.0f);
	for (i = 0; i < 2 * Ndof; i++) {
		m = Xpartial[Xtrans] + i * Nderivative;
		k->A_J[0 + m][1 + m] = A_kin[0][1];
		k->A_J[0 + m][2 + m] = A_kin[0][2];
		k->A_J[1 + m][2 + m] = A_kin[1][2];
	}


	// state error jacobian
	k->W = ar_valuediag2d(k->M, 1.0f);

	///////////////////////////////////
	// set observation matrices

	// observation matrix
	k->C = ar_zeros2d(k->N, k->M);
	for (i = 0; i < NXtrans; i++) {
		m = Xpartial[Xtrans] + i;
		n = Ypartial[Ytrans] + i;
		k->C[n][m] = 1.0f;
	}
	for (i = 0; i < NYrotacu; i++) {
		m = Xpartial[Xrotmap] + i;
		n = Ypartial[Yrotacu] + i;
		k->C[n][m] = 1.0f;
	}
	for (i = 0; i < Ndof; i++) {
		m = Xpartial[Xrot] + i * Nderivative;
		n = Ypartial[Yrotimu] + i;
		k->C[n][m] = 1.0f;
	}
	for (j = 0; j < Mframes; j++){
		for (i = 0; i < Ndof; i++) {
			m = Xpartial[Xrot] + i * Nderivative;
			n = Ypartial[Yrotacu] + j * Ndof +  i;
			k->C[n][m] = 1.0f;
		}
	}
	//write_txt_debug2d(k->C, k->N, k->M, 0);

	// observation matrix jacobian
	k->C_J = ar_zeros2d(k->N, k->M);
	for (i = 0; i < NXtrans; i++) {
		m = Xpartial[Xtrans] + i;
		n = Ypartial[Ytrans] + i;
		k->C_J[n][m] = 1.0f;
	}
	for (i = 0; i < NYrotacu; i++) {
		m = Xpartial[Xrotmap] + i;
		n = Ypartial[Yrotacu] + i;
		k->C_J[n][m] = 1.0f;
	}
	for (i = 0; i < Ndof; i++) {
		m = Xpartial[Xrot] + i * Nderivative;
		n = Ypartial[Yrotimu] + i;
		k->C_J[n][m] = 1.0f;
	}
	for (j = 0; j < Mframes; j++) {
		for (i = 0; i < Ndof; i++) {
			m = Xpartial[Xrot] + i * Nderivative;
			n = Ypartial[Yrotacu] + j * Ndof +  i;
			k->C_J[n][m] = 1.0f;
		}
	}

	// observation error jacobian
	k->V = ar_valuediag2d(k->N, 1.0f);

	///////////////////////////////////
	//Init further matrices

	// kalman gain
	k->K = ar_zeros2d(k->M, k->N);

	// states
	//k->x_new = ar_zeros(k->M);
	k->x = ar_zeros(k->M);

	// states apriori
	k->xap = ar_zeros(k->M);
	//k->xap = ar_zeros(k->M);

	// state input
	k->u = ar_zeros(k->M);

	// state error
	//k->e_new = ar_zeros(k->M);
	k->e = ar_zeros(k->M);

	// state error covariance
	//k->P_new = ar_zeros2d(k->M, k->M);
	k->P_n = 1.0f;
	k->P = ar_valuediag2d(k->M, k->P_n);

	// error covariance apriori
	k->Pap = ar_zeros2d(k->M, k->M);

	// limit of state variables
	if (limit) {
		k->limit_on = true;
		k->limit = ar_declare2d(k->M, A_limit_N);
		for (m = 0; m < k->M; m++) {
			k->limit[m][A_limit_low] = limit[m][A_limit_low];
			k->limit[m][A_limit_high] = limit[m][A_limit_high];
		}
	} else {
		k->limit_on = false;
	}

	// update rq metrics
	k->refresh_rq = refresh_rq;

	// loop number
	k->Nupdate = 0;

	///////////////////////////////////
	///////////////////////////////////
	//Set adaptable configuration

	// Q / R ratio (how important is observation relative to model)
	k->R_m = 1.0f;

	// Process noise covariance
	Q_kin = ar_declare(k->M);
	Q_kin[0] = T * T * T / 6;
	Q_kin[1] = T * T / 2;
	Q_kin[2] = T;
	k->Q = ar_zeros2d(k->M, k->M);
	for (i = Xpartial[Xtrans]; i < Xpartial[Xrot]; i++) {
		m = modulus(i, Nderivative);
		k->Q_m = k->R_m / rq[Xtrans];
		k->Q[i][i] = k->Q_m * Q_kin[m];
	}
	for (i = Xpartial[Xrot]; i < Xpartial[Xrotmap]; i++) {
		m = modulus(i, Nderivative);
		k->Q_m = k->R_m / rq[Xrot];
		k->Q[i][i] = k->Q_m * Q_kin[m];
	}
	for (i = Xpartial[Xrotmap]; i < k->M; i++) {
		m = (i - Xpartial[Xrotmap]) / Ndof;
		k->Q_m = k->R_m / rq[Xrotmap];
		k->Q[i][i] = k->Q_m; // + m
	}
	//write_txt_debug2d(k->Q, k->M, k->M, 0);

	// initial observation noise covariance
	k->R0 = ar_valuediag2d(k->N, k->R_m);
	for (i = 0; i < Ndof; i++) {
		n = Ypartial[Ytrans] + i * Nderivative;
		k->R0[n][n] = k->R0[n + A_ref][n + A_ref] * r[A_ref];
		k->R0[n + A_vel][n + A_vel] = k->R0[n + A_vel][n + A_vel] * r[A_vel];
		k->R0[n + A_acc][n + A_acc] = k->R0[n + A_acc][n + A_acc] * r[A_acc];
	}
	//write_txt_debug2d(k->R0, k->N, k->N, 0);

	// observation noise covariance
	k->R = ar_valuediag2d(k->N, k->R_m);
	for (i = 0; i < Ndof; i++) {
		n = Ypartial[Ytrans] + i * Nderivative;
		k->R[n][n] = k->R[n + A_ref][n + A_ref] * r[A_ref];
		k->R[n + A_vel][n + A_vel] = k->R[n + A_vel][n + A_vel] * r[A_vel];
		k->R[n + A_acc][n + A_acc] = k->R[n + A_acc][n + A_acc] * r[A_acc];
	}
	//write_txt_debug2d(k->R, k->N, k->N, 0);


	// free memory
	free(Xpartial);
	free(Ypartial);

	///////////////////////////////////
	// initial values

	// initial state estimate
	for (m = 0; m < k->M; m++) {
		k->x[m] = 0.0f;
	}

	// initial state input
	for (m = 0; m < k->M; m++) {
		k->u[m] = 0.0f;
	}

	// temporary variables
	MN = (k->M > k->N) ? k->M : k->N;
	k->divisor = ar_zeros2d(k->N, k->N);
	k->numerator = ar_zeros2d(k->M, k->N);
	k->eyearray = ar_valuediag2d(k->M, 1.0f);
	k->tmp1_1d = ar_zeros(MN);
	k->tmp1_2d = ar_zeros2d(MN, MN);
	k->tmp2_2d = ar_zeros2d(MN, MN);
	k->tmp3_2d = ar_zeros2d(MN, MN);
	k->tmp4_2d =  ar_zeros2d(MN, MN);

	// preinit kalman gain K and covariance P
	preinit_kalman_gain_cov_6dof(k, Ninitsteps);

	// Debugging
	//deb_disk = deb_disk_save;

};

void preinit_kalman_gain_cov_6dof(struct kal6s* k, int Ninitsteps) {
	// Debugging
	bool deb_disk_save = deb_disk;
	deb_disk = false;

	int n;
	for (n = 0; n < Ninitsteps; n++) {
		update_kalman_gain_6dof(k);
		update_kalman_cov_6dof(k);
	}

	// Debugging
	deb_disk = deb_disk_save;
}

// update kalman filter
void update_kalman_6dof(struct kal6s * k) {

	// Debugging
	//bool deb_disk_save = deb_disk;
	//deb_disk = false;

	long timer_start = clock_tick();
	//write_txt_debug(k->x, k->M, 0);
	//write_txt_debug2d(k->A, k->M, k->M, 0);

	// init
	int m;

	// update apriori state estimate
	ar_m2d1d (k->A, k->x, k->M, k->M, k->xap);
	//write_txt_debug(k->xap, k->M, 0);

	// add input to apriori state estimate
	ar_s(k->xap, k->u, k->M, k->xap);
	//write_txt_debug(k->xap, k->M, 0);

	// limit apriori state estimate
	if (k->limit_on) {
		for (m = 0; m < k->M; m++) {
			if (k->limit[m][A_limit_high] != INFINITY) {
				if (k->limit[m][A_limit_low] == 0.0f && k->limit[m][A_limit_high] == 360.0f) {
					correct_limit_circular(&k->xap[m], k->limit[m][A_limit_low], k->limit[m][A_limit_high]);
				}
				else {
					correct_limit(&k->xap[m], k->limit[m][A_limit_low], k->limit[m][A_limit_high]);
				}
			}
		}
	}

	// new kalman gain
	if (k->refresh_rq) {
		update_kalman_gain_6dof(k);
	}

	// update observation estimate
	ar_m2d1d(k->C, k->xap, k->N, k->M, k->tmp1_1d);
	//write_txt_debug(k->tmp1_1d, k->N, 0);

	// update state error
	ar_neg(k->tmp1_1d, k->N, k->tmp1_1d);
	//write_txt_debug(k->tmp1_1d, k->N, 0);

	ar_s(k->y, k->tmp1_1d, k->N, k->e); // y_new
	//write_txt_debug(k->e, k->N, 0);

	// adapt estimation error to 360 degree circular error
	if (k->limit_on) {
		for (m = 0; m < k->M; m++) {
			if (k->limit[m][A_limit_low] == 0.0f && k->limit[m][A_limit_high] == 360.0f) {
				angdiff_to_circular360(&k->e[m]);
			}
		}
	}

	// update aposteriori state estimate
	ar_m2d1d(k->K, k->e, k->M, k->N, k->tmp1_1d);
	//write_txt_debug(k->tmp1_1d, k->M, 0);

	ar_s(k->xap, k->tmp1_1d, k->M, k->x); // x_new
	//write_txt_debug(k->x, k->M, 0); // x_new

	// limit aposteriori state estimate
	if (k->limit_on) {
		for (m = 0; m < k->M; m++) {
			if (k->limit[m][A_limit_high] != INFINITY) {
				if (k->limit[m][A_limit_low] == 0.0f && k->limit[m][A_limit_high] == 360.0f) {
					correct_limit_circular(&k->xap[m], k->limit[m][A_limit_low], k->limit[m][A_limit_high]);
				}
				else {
					correct_limit(&k->xap[m], k->limit[m][A_limit_low], k->limit[m][A_limit_high]);
				}
			}
		}
	}

	// new state covariance
	if (k->refresh_rq) {
		update_kalman_cov_6dof(k);
	}

	// update update number
	k->Nupdate += 1;

	// Debugging
	double timer_diff = clock_stop(timer_start);
	//deb_disk = deb_disk_save;

};

void update_kalman_gain_6dof(struct kal6s * k) {

	// update apriori state error covariance estimate
	// first summand
	ar_m2d(k->A_J, k->P, k->M, k->M, k->M, k->tmp1_2d);
	//write_txt_debug2d(k->tmp1_2d, k->M, k->M, 0);

	ar_tp2d(k->A_J, k->M, k->M, k->tmp2_2d);
	//write_txt_debug2d(k->tmp2_2d, k->M, k->M, 0);

	ar_m2d(k->tmp1_2d, k->tmp2_2d, k->M, k->M, k->M, k->tmp3_2d);
	//write_txt_debug2d(k->tmp3_2d, k->M, k->M, 0);

	// second summand
	ar_m2d(k->W, k->Q, k->M, k->M, k->M, k->tmp1_2d);
	//write_txt_debug2d(k->tmp1_2d, k->M, k->M, 0);

	ar_tp2d(k->W, k->M, k->M, k->tmp2_2d);
	//write_txt_debug2d(k->tmp2_2d, k->M, k->M, 0);

	ar_m2d(k->tmp1_2d, k->tmp2_2d, k->M, k->M, k->M, k->tmp4_2d);
	//write_txt_debug2d(k->tmp4_2d, k->M, k->M, 0);

	// update
	ar_s2d(k->tmp3_2d, k->tmp4_2d, k->M, k->M, k->Pap);
	//write_txt_debug2d(k->Pap, k->M, k->M, 0);



	// update Kalman gain divisor
	// first summand
	ar_m2d(k->C_J, k->Pap, k->N, k->M, k->M, k->tmp1_2d);
	//write_txt_debug2d(k->tmp1_2d, k->N, k->M, 0);

	ar_tp2d(k->C_J, k->N, k->M, k->tmp2_2d);
	//write_txt_debug2d(k->tmp2_2d, k->M, k->N, 0);

	ar_m2d(k->tmp1_2d, k->tmp2_2d, k->N, k->M, k->N, k->tmp3_2d);
	//write_txt_debug2d(k->tmp3_2d, k->N, k->N, 0);
	
	// second summand
	ar_m2d(k->V, k->R, k->N, k->N, k->N, k->tmp1_2d);
	//write_txt_debug2d(k->tmp1_2d, k->N, k->N, 0);

	ar_tp2d(k->V, k->N, k->N, k->tmp2_2d);
	//write_txt_debug2d(k->tmp2_2d, k->N, k->N, 0);

	ar_m2d(k->tmp1_2d, k->tmp2_2d, k->N, k->N, k->N, k->tmp4_2d);
	//write_txt_debug2d(k->tmp4_2d, k->N, k->N, 0);

	// update
	ar_s2d(k->tmp3_2d, k->tmp4_2d, k->N, k->N, k->divisor);
	//write_txt_debug2d(k->divisor, k->N, k->N, 0);



	// update Kalman gain numerator
	ar_tp2d(k->C_J, k->N, k->M, k->tmp1_2d);
	//write_txt_debug2d(k->tmp1_2d, k->M, k->N, 0);

	ar_m2d(k->Pap, k->tmp1_2d, k->M, k->M, k->N, k->numerator);
	//write_txt_debug2d(k->numerator, k->M, k->N, 0);



	// invert Kalman gain divisor
	ar_inv2d(k->divisor, k->N, k->tmp1_2d);
	//write_txt_debug2d(k->tmp1_2d, k->N, k->N, 0);



	// update Kalman gain
	ar_m2d(k->numerator, k->tmp1_2d, k->M, k->N, k->N, k->K);
	//write_txt_debug2d(k->K, k->M, k->N, 0);

}
void update_kalman_cov_6dof(struct kal6s * k) {
	
	// update aposteriori state error covariance estimate
	// second term
	ar_m2d(k->K, k->C_J, k->M, k->N, k->M, k->tmp1_2d);
	//write_txt_debug2d(k->tmp1_2d, k->M, k->M, 0);

	ar_neg2d(k->tmp1_2d, k->M, k->M, k->tmp2_2d);
	//write_txt_debug2d(k->tmp2_2d, k->M, k->M, 0);

	// difference term
	ar_s2d(k->eyearray, k->tmp2_2d, k->M, k->M, k->tmp3_2d); // P_new
	//write_txt_debug2d(k->tmp3_2d, k->M, k->M, 0); // P_new

	// multiply
	ar_m2d(k->tmp3_2d, k->Pap, k->M, k->M, k->M, k->P);
	//write_txt_debug2d(k->P, k->M, k->M, 0);
}