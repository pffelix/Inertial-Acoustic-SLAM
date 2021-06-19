// particle.h: Particle filter
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once
#include "params.h"
#include "statistics.h"

#ifdef __cplusplus
extern "C" {
#endif

	//////////////////////////////////////////
	// external particle slam functions (public)
	//////////////////////////////////////////

	// initialize particle slam
	void init_particleslam_6dof(struct par6s* p, bool imu_on, bool track_first, long Npar, long Nobs, long Nobs_skip, int Nvarbuffer, float** pos_init, float obs_rq, float** obs_r, int par_priors, float par_rq, float** par_r, float T, bool refresh_rq, int Ninitsteps, float* weightpower, bool lanerrormedian, struct fras* f, struct heads* h, long* update_nr);

	// assign new observed(acoustic) landmark and (imu) input to particle filter
	// obs_new: new observations for Nobs landmarks (Mframes) and A_Ndof dimensions -> A_ref observation.
	// obs_new_var: noise variance [0...1] of new observation for Nobs landmarks (Mframes) and A_Ndof dimensions -> A_ref observation.
	// u_new: new (imu) input for A_Ndof dimensions -> A_ref, A_vel, A_acc observation. Neglected, if u_new is NULL.
	void assign_particleslam_6dof(struct par6s* p, float*** obs_new, float** u_new);

	// update particle filter
	void update_particleslam_6dof(struct par6s* p);


	//////////////////////////////////////////
	// internal particle slam functions (private)
	//////////////////////////////////////////

	// predict new particle positions by distributing via kalman filtered observations of landmarks and input (Fast Slam 2.0 methodology)
	void predict_particleslam_6dof(struct par6s* p, int d);

	// update the weights for each particle based on the likelihood of the observed landmarks
	void evaluate_particleslam_6dof(struct par6s* p, int d);

	// resample from the updated set of particles to form the new set of particles.
	void resample_particleslam_6dof(struct par6s* p, int d);

	// integrate value x once with delta T
	void integrate_once(float* x, float* T, float* y);

	// integrate value x twice with delta T
	void integrate_twice(float* x, float* T, float* y);

	// differentiate value x once with delta T
	void differentiate_once(float* x, float* T, float* y);

	// differentiate value x twice with delta T
	void differentiate_twice(float* x, float* T, float* y);

	// potentiate weight
	void weight_power(float* x, float power);


	// inverse weight [0...1] -> [inf...1]
	void weight_inverse(float* x);

	// flip weight from [0...1] -> [1...0]
	void weight_1_0_flip(float* x);

	// logarithmize weight [0...1] -> [-inf...0], [1...inf] -> [0...inf_slow]
	void weight_log(float* x);

	// minus logarithmize weight [0...1] -> [inf...0], [1...inf] -> [0...-inf_slow]
	void weight_logminus(float* x);

	// exponentialize weight [0...1] -> [1...e], [1...inf] -> [e...inf_fast]
	void weight_exp(float* x);

	// potentiate weight_array
	void ar_weight_power(float* x, int N, float power);


	//////////////////////////////////////////
	// multithreading subroutines (private)
	//////////////////////////////////////////
	void update_particleslam_6dof_multithread(void *arg);


#ifdef __cplusplus
};
#endif