// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "particleslam.h"


void init_particleslam_6dof(struct par6s* p, bool imu_on, bool track_first, long Npar, long Nobs,  long Nobs_skip, int Nvarbuffer, float** pos_init, float obs_rq, float** obs_r, int par_priors, float par_rq, float** par_r, float T, bool refresh_rq, int Ninitsteps, float* weightpower, bool lanerrormedian, struct fras* f, struct heads* h, long* update_nr) {
	
	// initialize temporary variables
	int i, m, d, t, t2;
	float*** init_limit;
	float par_rq_i;
	struct kals* init_obs_kal;
	struct kals* init_par_kal;

	// initialize settings

	// activate imu (or full acoustic tracking)
	p->imu_on = imu_on;

	
	// track position relative to first sound arrival
	p->track_first = track_first;

	// number of particles
	p->Npar = Npar;

	// number of observations
	p->Nobs = Nobs;

	// number of skipped first observations
	p->Nobs_skip = Nobs_skip;

	// timestep between 2 updates
	p->T = T;

	// update number
	p->update_nr = update_nr;

	// landmark - observation error metric
	p->lanerrormedian = lanerrormedian;

	// observation damping constant (how exponentially strongly is a fluctuating observation damped)
	p->weightpower = ar_declare(A_weight_N);
	for (d = 0; d < A_weight_N; d++) {
		p->weightpower[d] = weightpower[d];
	}

	// particle prior type
	p->par_priors = par_priors;

	// maximum likelihood position
	p->pos = ar_zeros2d(A_Ndof, A_Ndofderivative);
	if (p->pos) {
		if (pos_init) {
			for (d = 0; d < A_Ndof; d++) {
				for (t = 0; t < A_Ndofderivative; t++) {
					p->pos[d][t] = pos_init[d][t];
				}
			}
		}
		else {
			p->pos[A_dim[A_el]][A_ref] = 90.0f; // set elevation to 0 angle
		}
	}
	
	// initialize limits and pre-compute kalman gain and covariance (possible as we are using non-extended kalman filters)
	//////////////////////////////////////////////////////////////////////////////////
	init_limit = ar_value3d(A_Ndof, A_Nderivative, A_limit_N, INFINITY);
	init_obs_kal = (struct kals *)malloc(A_Ndof * sizeof(struct kals));
	init_par_kal = (struct kals *)malloc(A_Ndof * sizeof(struct kals));


	for (d = 0; d < A_Ndof; d++) {
		// get limit
		//if (A_dim_r[d] == A_az) {
		init_limit[d][A_ref][A_limit_low] = A_limit[d][A_limit_low];
		init_limit[d][A_ref][A_limit_high] = A_limit[d][A_limit_high];
		//}

		// pre-compute observation kalman gain and covariance
		init_kalman_1dof(&init_obs_kal[d], obs_rq, obs_r[A_dim_r[d]], p->T, init_limit[d], refresh_rq, Ninitsteps);

		// pre-compute general particle kalman gain and covariance
		if (p->par_priors == A_parprior_kalsingle) {
			init_kalman_1dof(&init_par_kal[d], par_rq, par_r[A_dim_r[d]], p->T, init_limit[d], refresh_rq, Ninitsteps);
		}
		if (p->par_priors == A_parprior_kalmulti) {
			init_kalman_1dof(&init_par_kal[d], par_rq, par_r[A_dim_r[d]], p->T / (float)Npar, init_limit[d], refresh_rq, Ninitsteps);
		} 
		if (p->par_priors == A_parprior_median || p->par_priors == A_parprior_medianabs) {
			init_kalman_1dof(&init_par_kal[d], par_rq, par_r[A_dim_r[d]], p->T, init_limit[d], refresh_rq, Ninitsteps);
		}
	}

	// particles
	//////////////////////////////////////////////////////////////////////////////////
	p->par = (struct parparticle *)malloc(sizeof(struct parparticle) * Npar);
	if (p->par) {
		for (i = 0; i < Npar; i++) {
			// initialize memory of each particle 
			p->par[i].pos = (struct kals *)malloc(sizeof(struct kals) * A_Ndof);
			p->par[i].kal = (struct kals **)malloc(sizeof(struct kals*) * A_Ndof);
			if (p->par[i].kal) {
				for (d = 0; d < A_Ndof; d++) {
					p->par[i].kal[d] = (struct kals*)malloc(sizeof(struct kals) * p->Nobs);
				}
			}
		}
		
		// initialize kalman filters for each dimension of each particle
		for (d = 0; d < A_Ndof; d++) {
			for (i = 0; i < Npar; i++) {
				// pre-compute particle dependent kalman gain and covariance
				if (p->par_priors == A_parprior_median || p->par_priors == A_parprior_medianabs) {
					// initialize particle position kalman filter
					par_rq_i = par_rq * powf(10000.0f, (float)(i - Npar / 2)); // different observation smoothing capabilty
					init_kalman_1dof(&p->par[i].pos[d], par_rq_i, par_r[A_dim_r[d]], p->T, init_limit[d], refresh_rq, Ninitsteps);
				}
				// use pre-computed particle dependent kalman gain and covariance
				else {
					// initialize particle position kalman filter
					init_kalman_1dof(&p->par[i].pos[d], par_rq, par_r[A_dim_r[d]], p->T, init_limit[d], refresh_rq, 0);
					// assign pre-computed kalman gain and covariance
					for (t = 0; t < A_Nderivative; t++) {
						for (t2 = 0; t2 < A_Nderivative; t2++) {
							p->par[i].pos[d].K[t][t2] = init_par_kal[d].K[t][t2];
							p->par[i].pos[d].P[t][t2] = init_par_kal[d].P[t][t2];
						}
					}
				}
				// initialize first position
				if (pos_init) {
					for (t = 0; t < A_Ndofderivative; t++) {
						p->par[i].pos[d].x[t] = pos_init[d][t];
					}
				}else {
					if (A_dim_r[d] == A_el) {
						p->par[i].pos[d].x[A_ref] = 90.0f; // set elevation to 0 angle
						p->par[i].pos[d].y[A_ref] = 90.0f; // set elevation to 0 angle

					}
				}

				for (m = 0; m < p->Nobs; m++) {
					if (p->par[i].kal[d]) {
						// initialize landmark observation position kalman filter
						init_kalman_1dof(&p->par[i].kal[d][m], obs_rq, obs_r[A_dim_r[d]], p->T, init_limit[d], refresh_rq, 0);
						// get pre-computed kalman gain and covariance
						for (t = 0; t < A_Nderivative; t++) {
							for (t2 = 0; t2 < A_Nderivative; t2++) {
								p->par[i].kal[d][m].K[t][t2] = init_obs_kal[d].K[t][t2];
								p->par[i].kal[d][m].P[t][t2] = init_obs_kal[d].P[t][t2];
							}
						}
					}
				}
				
			}
		}
	}

	// (acoustic) landmark observations
	//////////////////////////////////////////////////////////////////////////////////
	p->obs = (struct parobservation *)malloc(sizeof(struct parobservation) * Nobs);
	if (p->obs) {
		for (m = 0; m < p->Nobs; m++) {
			// initialize memory of each observation 
			p->obs[m].x_p = ar_zeros2d(A_Ndof, A_Ndofderivative);
			p->obs[m].weight = ar_value(A_Ndof, 1.0f);
			p->obs[m].kal = (struct kals *)malloc(sizeof(struct kals) * A_Ndof);
		}

		// initialize kalman filter for each dimension of each observation
		for (d = 0; d < A_Ndof; d++) {
			for (m = 0; m < p->Nobs; m++) {
				if (p->obs[m].kal) {
					// initialize kalman filter
					init_kalman_1dof(&p->obs[m].kal[d], obs_rq, obs_r[A_dim_r[d]], p->T, init_limit[d], refresh_rq, 0);

					// assign pre-computed kalman gain and covariance
					for (t = 0; t < A_Nderivative; t++) {
						for (t2 = 0; t2 < A_Nderivative; t2++) {
							p->obs[m].kal[d].K[t][t2] = init_obs_kal[d].K[t][t2];
							p->obs[m].kal[d].P[t][t2] = init_obs_kal[d].P[t][t2];
						}
					}
				}
			}
		}
	}

	// global weight of landmark observations
	p->obs_weight = ar_zeros(Nobs);

	// probability or cumulative density function for likelihood of landmark observations
	p->obs_pdf = ar_zeros2d(A_Ndof, Nobs);
	p->obs_cdf = ar_zeros2d(A_Ndof, Nobs);

	// probability or cumulative density function for likelihood of particles
	p->par_pdf = ar_zeros2d(A_Ndof, Npar);
	p->par_cdf = ar_zeros2d(A_Ndof, Npar);



	// variance buffer and its length for variance calculation from past kalman states
	p->Nvarbuffer = Nvarbuffer;
	for (d = 0; d < A_Ndof; d++) {
		for (m = 0; m < p->Nobs; m++) {
			p->obs[m].varbuffer = ar_declare2d(A_Ndof, p->Nvarbuffer + 3);
		}
	}

	// frame struct
	p->fra = f;

	// head struct
	p->head = h;

	// assignment between observations and landmarks
	p->lan = ar_lineari(0, 1, p->Nobs);

	// temporary
	// particle array: number
	p->tmp_par_i = ar_valuei(Npar, 0);
	// particle array: amplitude
	p->tmp_par_amp = ar_zeros(Npar);
	// particle array: buffers
	p->tmp_par_amp_buffer_left = ar_zeros(Npar);
	p->tmp_par_amp_buffer_right = ar_zeros(Npar);

	// frame array: number
	p->tmp_obs_m = ar_valuei(Nobs, 0);
	// frame array: amplitude
	p->tmp_obs_amp = ar_zeros(Nobs);
	// frame array: buffers
	p->tmp_obs_amp_buffer_left = ar_zeros(Nobs);
	p->tmp_obs_amp_buffer_right = ar_zeros(Nobs);

	// free memory: de-initialize limits and precomputed kalman gain and covariance
	ar_free3d(init_limit, A_Ndof, A_Nderivative, A_limit_N);
	free(init_obs_kal);
	free(init_par_kal);
}


void assign_particleslam_6dof(struct par6s* p, float*** obs_new, float** u_new) {
	// init temporary variables
	int i, m, d, t;
	float mean;
	float weight_energy;
	float weight_peak;
	float weight_sparsity;
	float weight_stability;
	float weight_global;
	float weight_elevation;
	float weight_early;

	// if first assignment
	if (update_first) {
		for (d = 0; d < A_Ndof; d++) {
			// init observations
			for (m = 0; m < p->Nobs; m++) {
				// set differential observations to 0
				obs_new[m][d][A_vel] = 0.0f;

				// init running variance
				if (A_dim_r[d] == A_x || A_dim_r[d] == A_y || A_dim_r[d] == A_z) {
					sample_mean_var_moving_init(obs_new[m][d][A_vel], p->obs[m].varbuffer[d], p->Nvarbuffer);
				}
				if (A_dim_r[d] == A_az) {
					sample_mean_var_circular_moving_init(obs_new[m][d][A_ref], p->obs[m].varbuffer[d], p->Nvarbuffer, A_limitstep[A_dim_r[d]]);
				}
				if (A_dim_r[d] == A_el) {
					sample_mean_var_moving_init(obs_new[m][d][A_ref], p->obs[m].varbuffer[d], p->Nvarbuffer);
				}
			}

			// init particles
			//for (i = 0; i < p->Npar; i++) {
				//standard_distribution(1, 0.0f, 0.1f, p->par[i].pos[d].x[A_ref]);
			//}
		}

		// init each particle (to position of first sound arrival)
		if (p->track_first) {
			for (d = 0; d < A_Ndof; d++) {
				for (i = 0; i < p->Npar; i++) {
					p->par[i].pos[d].x[A_ref] = obs_new[p->fra->m_energyfirst][d][A_ref];
				}
			}
		}
	}

	// if landmark observation is new: assign initial state such that no jump occurs 
	////////////////////////////////////////////////////////////////////
	for (d = 0; d < A_Ndof; d++) {
		for (m = 0; m < p->Nobs; m++) {
			if (p->fra->m_new[m] || update_nr < 2) {
				for (t = 0; t < A_Ndofderivative; t++) {
					// init observation
					//if (!((t == A_vel) && (A_dim_r[d] == A_az || A_dim_r[d] == A_el))) { // get rid of double cross-correlation
						p->obs[m].kal[d].x[t] = obs_new[m][d][t];

						// init each particle observation
						for (i = 0; i < p->Npar; i++) {
							p->par[i].kal[d][m].x[t] = obs_new[m][d][t];
						}
					//}
				}
			}
		}
	}


	// get normalized (running) variance of observation y over past Nvarbuffer steps
	////////////////////////////////////////////////////////////////////
	for (m = 0; m < p->Nobs; m++) {
		if (p->fra->m[m]) {
			weight_stability = 1.0f;
			for (d = 0; d < A_Ndof; d++) {
				// get normalized linear running variance for differential observation (A_vel better as than limit is known)
				if (A_dim_r[d] == A_x || A_dim_r[d] == A_y || A_dim_r[d] == A_z) {
					sample_mean_var_moving(obs_new[m][d][A_vel], p->obs[m].varbuffer[d], p->Nvarbuffer, &mean, &p->obs[m].weight[d]); // weight not circular
					sample_var_normalize(&p->obs[m].weight[d], A_limitstep[A_dim_r[d]]); // [0(good)...1(bad)] 
					//if (p->obs[m].weight[d] > 1.0f) {
						//console_log(PRIO_WARNING, "x,y,z variance limit bigger than 1");
					//}
				}
				// get circular variance  for reference observation (no running update yet, but dependend on varbuffer_kalx shift)
				if (A_dim_r[d] == A_az) {
					sample_mean_var_circular_moving(obs_new[m][d][A_ref], p->obs[m].varbuffer[d], p->Nvarbuffer, A_limitstep[A_dim_r[d]], &mean, &p->obs[m].weight[d]); // weight not circular
				}
				// get normalized linear running variance for reference observation
				if (A_dim_r[d] == A_el) {
					sample_mean_var_moving(obs_new[m][d][A_ref], p->obs[m].varbuffer[d], p->Nvarbuffer, &mean, &p->obs[m].weight[d]); // weight not circular
					sample_var_normalize(&p->obs[m].weight[d], A_limitstep[A_dim_r[d]]); // [0(good)...1(bad)] 

				}
				// flip and time fluctuation variance
				weight_1_0_flip(&p->obs[m].weight[d]); // [1(good)...0(bad)]

				// Add to global frame weight (new)
				//weight_power(&p->obs[m].weight[d], p->weightpower[d]);
				//weight_stability *= p->obs[m].weight[d];

			}
			// collect weights
			weight_energy = p->fra->m_weight_energy[m];
			weight_peak = p->fra->m_weight_corpeak[m];
			weight_sparsity = p->head->pos_var[m][A_dim[A_az]][A_ref];
			weight_1_0_flip(&weight_sparsity);
			weight_stability = p->obs[m].weight[A_dim[A_az]];
			weight_elevation = p->fra->m_weight_elevation[m];
			weight_early = m < p->Nobs_skip ? 0.0f : p->fra->m_weight_early[m];

			// set importance of weights
			weight_power(&weight_energy, p->weightpower[A_weight_energy]);
			weight_power(&weight_peak, p->weightpower[A_weight_peak]);
			weight_power(&weight_sparsity, p->weightpower[A_weight_sparsity]);
			weight_power(&weight_stability, p->weightpower[A_weight_stability]);
			weight_power(&weight_elevation, p->weightpower[A_weight_elevation]);
			weight_power(&weight_early, p->weightpower[A_weight_early]);

			// finish global frame weight
			weight_global = weight_energy * weight_peak * weight_sparsity * weight_early;
			weight_power(&weight_global, p->weightpower[A_weight_global]);

			if (m == 11 || m == 12 || m == 13 || m == 14 || m == 15 || m == 16 || m == 23 || m == 26 || m == 27 || m == 28 || m == 29) {
				weight_global = 0.0f;
			}
			if ( m == 19 || m == 20 || m == 21  || m == 24 || m == 25) {
				weight_global = 1.0f;
				//weight_global = p->fra->m_weight_energy[m];
			}
			if (m == 21 || m == 22) {
				weight_global = 0.0f;
				//weight_global = p->fra->m_weight_energy[m];	
			}
			// assign global frame weight
			p->obs_weight[m] = weight_global;

			

		}
	}

	// add external (imu) input to each particle kalman filter
	////////////////////////////////////////////////////////////////////
	if (p->imu_on && u_new) {
		for (d = 0; d < A_Ndof; d++) {
			for (i = 0; i < p->Npar; i++) {
				if (A_dim_r[d] == A_x) {
					p->par[i].pos[d].u[A_acc] = u_new[d][A_acc];
					// sine terms are necessary when sensor is mounted on head and changes its coordinate system with rotational movement
					p->par[i].pos[d].u[A_acc] *= sinf(deg_to_rad(p->pos[A_dim[A_az]][A_ref])) * sinf(deg_to_rad(p->pos[A_dim[A_el]][A_ref]));
				}
				if (A_dim_r[d] == A_y) {
					p->par[i].pos[d].u[A_acc] = u_new[d][A_acc];
					p->par[i].pos[d].u[A_acc] *= cosf(deg_to_rad(p->pos[A_dim[A_az]][A_ref])) * sinf(deg_to_rad(p->pos[A_dim[A_el]][A_ref]));
				}
				if (A_dim_r[d] == A_z) {
					p->par[i].pos[d].u[A_acc] = u_new[d][A_acc];
					p->par[i].pos[d].u[A_acc] *= cosf(deg_to_rad(p->pos[A_dim[A_el]][A_ref]));
				}
				if (A_dim_r[d] == A_az || A_dim_r[d] == A_el) {
					if (A_dim_r[d] == A_az) {
						p->par[i].pos[d].u[A_ref] = -u_new[d][A_vel];
					}
				}
			}
		}
	}

	// add new observations to smoothing kalman filter
	////////////////////////////////////////////////////////////////////
	for (d = 0; d < A_Ndof; d++) {
		for (m = 0; m < p->Nobs; m++) {
			if (p->fra->m[m]) {
				for (t = 0; t < A_Ndofderivative; t++) {
					// store past kalman observation
					p->obs[m].x_p[d][t] = p->obs[m].kal[d].x[t];

					// assign new kalman observation (after init loop has passed)
					//if (p->update_nr != 0 && t != A_vel) {
					//if (!((t == A_vel) && (A_dim_r[d] == A_az || A_dim_r[d] == A_el))) { // get rid of double cross-correlation
					p->obs[m].kal[d].y[t] = obs_new[m][d][t];
					
					// convert radius observation in x,y,z observation
					if (t == A_vel) {
						switch (A_dim_r[d]) {
							case A_x:
								p->obs[m].kal[d].y[t] *= (sinf(deg_to_rad(p->obs[m].kal[A_dim[A_az]].x[A_ref])) * sinf(deg_to_rad(p->obs[m].kal[A_dim[A_el]].x[A_ref]))); break;
							case A_y:
								p->obs[m].kal[d].y[t] *= (cosf(deg_to_rad(p->obs[m].kal[A_dim[A_az]].x[A_ref])) * sinf(deg_to_rad(p->obs[m].kal[A_dim[A_el]].x[A_ref]))); break;
							case A_z:
								p->obs[m].kal[d].y[t] *= (cosf(deg_to_rad(p->obs[m].kal[A_dim[A_el]].x[A_ref]))); break;
						}
					}
					//}
				}
				// update smoothing kalman filter of each observation and dimension
				update_kalman_1dof(&p->obs[m].kal[d]);
			}
		}
	}

	// add new observations to landmarks
	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	// search for landmarks fitting to observation
	bool dynamic_landmark = false;
	int Nframesearch = 1;
	if (dynamic_landmark && !update_first){
		for (m = 0; m < p->Nobs; m++) {
			// guess what landmark is connected to what observation
			// looking at similar change in azimuth position at connected frames
			float e = 0.0f;
			float e_best = INFINITY;
			for (int lan_m = p->lan[m] - Nframesearch; lan_m <= p->lan[m] + 2 * Nframesearch + 1; lan_m++) { // lan_m, obs_m are switched
				if (lan_m > 0 && lan_m < p->Nobs) {
					e = p->obs[lan_m].kal[A_dim[A_az]].x[A_ref]   - (p->par[A_obsx].kal[A_dim[A_az]][m].y[A_ref] + u_new[A_dim[A_az]][A_vel]); // new obs - (old obs + imu change) 
					//e = p->par[i].kal[A_dim[A_az]][m].tmp1_1d[0] - p->par[A_obsx].kal[A_dim[A_az]][obs_m].y[0];
					e *= e;
					if (e < e_best) {
						e_best = e;
						p->lan[m] = lan_m;
					}
				}
			}
		}
	}

	// assign landmarks to particles
	for (d = 0; d < A_Ndof; d++) {
		for (m = 0; m < p->Nobs; m++) {
			if (p->fra->m[m]) {


				// assign observation to particles (from observation)
				for (t = 0; t < A_Ndofderivative; t++) {
					for (i = 0; i < p->Npar; i++) {
						// assign current observation to particle filter landmark
						if (i == A_obsx) {
							p->par[i].kal[d][m].y[t] = p->obs[p->lan[m]].kal[d].x[t];
						}
						else {
							p->par[i].kal[d][m].y[t] = p->par[A_obsx].kal[d][m].y[t];
						}
					}
				}
			}
		}
	}
}

void update_particleslam_6dof(struct par6s* p) {

	// debugging
	//bool deb_disk_save = deb_disk;
	//deb_disk = deb_particleslam ? true : false;


	// update particle for each dimension
	loop_update_thread_or_serial(pool, update_particleslam_6dof_multithread, A_Ndof, par6_pool_on, true); // correctly par6s p should be also given as input parameter
	p->update_nr += 1;

	// debugging
	deb_particleslam = true;
	if (deb_particleslam) {
		if (deb_parmultiobs){
			char loopnr[path_N];
			sprintf(loopnr, "%d", update_nr);
			write_particleslam_debug_multithread(p, loopnr, "obskalx", "dof", p->Nobs, A_Ndof, A_Ndofderivative, 1, deb_pool_on);
			write_particleslam_debug_multithread(p, loopnr, "parpos", "dof", p->Npar, A_Ndof, A_Ndofderivative, 1, deb_pool_on);
			write_particleslam_debug_multithread(p, loopnr, "parpdfcdf", "dof", 1, A_Ndof, p->Npar, 1, deb_pool_on);
		}
		else {
			write_particleslam_debug_multithread(p, "obs", "kalx_dof", "kalx_der", p->Nobs, A_Ndof, A_Ndofderivative, 1, deb_pool_on);
			write_particleslam_debug_multithread(p, "par", "pos_dof", "pos_der", p->Npar, A_Ndof, A_Ndofderivative, 1, deb_pool_on);
			write_particleslam_debug_multithread(p, "", "dof", "par_pdfcdf", 1, A_Ndof, p->Npar, 1, deb_pool_on);
		}
		write_particleslam_debug_multithread(p, "obs", "kaly_dof", "kaly_der", p->Nobs, A_Ndof, A_Ndofderivative, 1, deb_pool_on);
		write_particleslam_debug_multithread(p, "", "obs", "dof_weight", 1, p->Nobs, A_Ndof, 1, deb_pool_on);
		write_particleslam_debug_multithread(p, "", "dof", "obs_pdfcdf", 1, A_Ndof, p->Nobs, 1, deb_pool_on);
		//write_particleslam_debug_multithread(p, "par", "dof", "obs_kalx_ref", p->Npar, A_Ndof, p->Nobs, 1, deb_pool_on);
		write_particleslam_debug_multithread(p, "", "obs", "weight", 1, p->Nobs, A_Ndof, 1, deb_pool_on);


		int sampleN = 1;
		int sampleNdiff  = 1;
		if (deb_par6_probxyzsph == A_debprob_sph) {
				sampleN = cir->prob_N[A_az];
				sampleNdiff = cir->probdiff_N[A_az];
		}
		if(deb_par6_probxyzsph == A_debprob_xyzsph) {
				sampleN = ar_maxi(cir->prob_N, A_Ndof);
				sampleNdiff = ar_maxi(cir->probdiff_N, A_Ndof);
		}
		//write_particleslam_debug_multithread(p, "dof", "frame", "prob", A_Ndof, frame_M, sampleN, 1, deb_pool_on);
		//write_particleslam_debug_multithread(p, "dof", "frame", "probdiff", A_Ndof, frame_M, sampleNdiff, 1, deb_pool_on);
	}

	//deb_disk = deb_disk_save;
}

void update_particleslam_6dof_multithread(void* arg) {
	// get argument: dimension (optimal faster in case of more than Ndof CPU cores would be to include also particle)
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	int* arg_ptr = arg;
	int d = *arg_ptr;

	// calculate
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// debugging
	long timer_start;
	double timer_diff;

	// update
	timer_start = clock_tick();
	predict_particleslam_6dof(par6, d);
	timer_diff = clock_stop(timer_start);
	timer_start = clock_tick();
	evaluate_particleslam_6dof(par6, d);
	timer_diff = clock_stop(timer_start);
	timer_start = clock_tick();
	resample_particleslam_6dof(par6, d);
	timer_diff = clock_stop(timer_start);
}

void predict_particleslam_6dof(struct par6s* p, int d) {
	// debuging
	//long timer_start = clock_tick();

	//write_particleslam_debug(p, "obs", "kalx_dof", "kalx_der", p->Nobs, A_Ndof, A_Ndofderivative, 1);
	//write_particleslam_debug(p, "obs", "kaly_dof", "kaly_der", p->Nobs, A_Ndof, A_Ndofderivative, 1);
	//double timer_diff = clock_stop(timer_start);

	// init
	int i, m;
	float weight;
	int par_pos_change_likely_m;
	float par_pos_change_likely;
	float Npar_f;
	float osb_x_diff;
	weight = 0.0f;
	par_pos_change_likely_m = 0;
	par_pos_change_likely = 0.0f;



	// predict x, y, z, azimuth, elevation
	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	if (A_dim_r[d] != A_rd) {

		if (A_dim_r[d] == A_az) {
			int aaaaaaaa = 0;
		}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////
		// prior update (observation && input): derive cumulative density function describing weight for possible changes in particle position
		///////////////////////////////////////////////////////////////////////////////////////////////////////////
		// assign weigth and values of probability density function (pdf)
		for (m = 0; m < p->Nobs; m++) {
			if (p->fra->m[m]) {
				// Weight: Energy strength of landmark observation and time variation
				weight = p->obs_weight[p->lan[m]]; // new (before d)
				p->obs_pdf[d][m] = weight; // pdf is attached to all observations m and dimensions d: p->obs[m].kal[d].x[A_vel];
			}
			else {
				p->obs_pdf[d][m] = 0.0f;
			}
		}

		// derive observation cumulative density function (cdf) from pdf
		pdf_to_cdf(p->obs_pdf[d], p->Nobs, p->obs_cdf[d]);

		///////////////////////////////////////////////////////////////////////////////////////////////////////////
		// landmark estimate (particle): sample for each particle position new change in position and predict from it new observation positions 
		///////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (d == 3) {
			int aaaaa = 0;
		}
		// get most likely changes from prior cdf to derive possible new particle position change index
		ar_Nmaxi(p->obs_pdf[d], p->Nobs, p->tmp_par_i, p->tmp_par_amp, p->Npar);

		// Prepare selected particle prior
		if (p->par_priors == A_parprior_median || p->par_priors == A_parprior_medianabs) {
			// observation
			for (i = 0; i < p->Npar; i++) {
				par_pos_change_likely_m = p->tmp_par_i[i];
				p->tmp_par_amp[i] = p->par[A_obsx].kal[d][par_pos_change_likely_m].y[A_vel];
				p->tmp_par_amp[i] = A_parprior_medianabs ? fabsf(p->tmp_par_amp[i]) : p->tmp_par_amp[i];
			}
			sample_median(p->tmp_par_amp, p->Npar, 0, p->tmp_par_amp_buffer_left, p->tmp_par_amp_buffer_right, &weight);
		}
		// for every particle
		for (i = 0; i < p->Npar; i++) {
			// sample from prior cdf to derive possible new particle position change index
			//sample_index_from_cdf(p->obs_cdf[d], p->Nobs, &par_pos_change_likely_m);


			// Get selected particle prior
			if (p->par_priors == A_parprior_kalsingle) {
				// observation
				if (p->track_first) {
					p->par[i].pos[d].y[A_ref] = p->par[A_obsx].kal[d][fra->m_energyfirst].y[A_ref];
				}
				else {
					par_pos_change_likely_m = p->tmp_par_i[i];
					p->par[i].pos[d].y[A_vel] = p->par[A_obsx].kal[d][par_pos_change_likely_m].y[A_vel];  // same as p->obs[par_pos_change_likely_m].kal[d].x[A_vel]	
					if (p->imu_on) {
						p->par[i].pos[d].y[A_vel] = p->par[i].pos[d].y[A_vel] / 10.0f;  // same as p->obs[par_pos_change_likely_m].kal[d].x[A_vel]	
					}
				}
				// update particle
				update_kalman_1dof(&p->par[i].pos[d]);
			}
			if (p->par_priors == A_parprior_kalmulti) {
				// input (correct)
				Npar_f = (float)p->Npar;
				p->par[i].pos[d].u[A_ref] /= Npar_f;
				p->par[i].pos[d].u[A_acc] /= Npar_f;
				// observation
				p->par[i].pos[d].x[A_acc] = 0.0f;
				par_pos_change_likely_m = p->tmp_par_i[p->Npar - 1 - i];
				p->par[i].pos[d].y[A_vel] = p->par[A_obsx].kal[d][par_pos_change_likely_m].y[A_vel];
				if (p->track_first) {
					osb_x_diff = (p->par[A_obsx].kal[d][fra->m_energyfirst].y[A_ref] - p->obs[m].x_p[d][A_ref]) / Npar_f;
					p->par[i].pos[d].y[A_ref] = p->obs[m].x_p[d][A_ref] + osb_x_diff;
				}
				// update particle
				update_kalman_1dof(&p->par[i].pos[d]);
			}
			if (p->par_priors == A_parprior_median || p->par_priors == A_parprior_medianabs) {
				// observation
				par_pos_change_likely_m = p->tmp_par_i[i];
				p->par[i].pos[d].y[A_vel] = weight;  // same as p->obs[par_pos_change_likely_m].kal[d].x[A_vel]
				if (p->track_first) {
					p->par[i].pos[d].y[A_ref] = p->par[A_obsx].kal[d][fra->m_energyfirst].y[A_ref];
				}
				// update particle
				update_kalman_1dof(&p->par[i].pos[d]);
			}

			// display selected frames
			if (d == 0) {
				console_logii(PRIO_PARTICLE, "Frame particle %i: %i\n", i, par_pos_change_likely_m + 1);
			}
		}

		// propagate kalman filter of each landmark about particle prior probablity
		for (i = 0; i < p->Npar; i++) {
			for (m = 0; m < p->Nobs; m++) {
				if (p->fra->m[m]) {
					// add prior to landmark change (from suggested particle position change)
					p->par[i].kal[d][m].u[A_ref] = p->par[i].pos[d].x[A_vel];

					// predict new landmark position
					update_kalman_1dof(&p->par[i].kal[d][m]);

					if (isnan(p->par[i].kal[d][m].x[0])) {
						int aaa = 0;
					}
				}
			}
		}
	}
	else {
		// predict wall distance rd
		///////////////////////////////////////////////////////////////////////////////////////////////////////////
		for (m = 0; m < p->Nobs; m++) {
			if (p->fra->m[m]) {
				bool later_reflection = false;
				if (later_reflection) {
					par_pos_change_likely = tanf(deg_to_rad(p->obs[m].kal[A_dim[A_az]].x[A_ref])) / tanf(deg_to_rad(p->obs[m].x_p[A_dim[A_az]][A_ref]));
					p->obs[m].kal[d].x[A_ref] = sqrtf(powf(p->pos[A_dim[A_x]][A_vel] / (par_pos_change_likely - 1.0f), 2.0f) + powf(p->pos[A_dim[A_y]][A_vel] / (1.0f / par_pos_change_likely - 1.0f), 2.0f));
				}
				else {
					// use existing assignment from first reflection tracking technique
				}
				update_kalman_1dof(&p->obs[m].kal[d]);
			}
		}
	}

	// debuging
	//write_wav_debug(p->obs_pdfcdf, A_Ndof, p->Nobs, 1.0f, "./obs_cdf.wav", "", 0);
	//write_particleslam_debug(p, "", "dof", "obs_pdfcdf", 1, A_Ndof, p->Nobs, 1);

}


void evaluate_particleslam_6dof(struct par6s* p, int d) {
	// debuging

	// init
	int i, m, t;
	float par_pos_most_likely;
	long par_pos_most_likely_i;
	float error;
	float weight;
	float pos_estimate;
	float pos_obs;
	pos_estimate = 0.0f;
	error = 0.0f;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	// likelihood (particle & observation): estimate and evaluate particle position likelihood by comparing to all landmark observations
	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (i = 0; i < p->Npar; i++) {
		weight = 0.0f;
		for (m = 0; m < p->Nobs; m++) {
			if (p->fra->m_on[m]) {
				// predict landmark position from particle position (using linear camera model)
				switch (A_dim_r[d]) {
					case A_x:
						pos_estimate = p->par[i].kal[d][m].x[A_vel];
						pos_obs = p->par[A_obsx].kal[d][m].y[A_vel];
						error = (pos_estimate - pos_obs); break; 
					case A_y:
						pos_estimate = p->par[i].kal[d][m].x[A_vel];
						pos_obs = p->par[A_obsx].kal[d][m].y[A_vel];
						error = (pos_estimate - pos_obs); break; 
					case A_z:
						pos_estimate = p->par[i].kal[d][m].x[A_vel];
						pos_obs = p->par[A_obsx].kal[d][m].y[A_vel];
						error = (pos_estimate - pos_obs); break; 
					case A_az:
						pos_estimate = p->par[i].kal[d][m].x[A_ref];
						pos_obs = p->obs[m].kal[d].x[A_ref];
						error = (pos_estimate - pos_obs); 
						angdiff_to_circular360(&error); break;
					case A_el:
						pos_estimate = p->par[i].kal[d][m].x[A_ref];
						pos_obs = p->obs[m].kal[d].x[A_ref];
						error = (pos_estimate - pos_obs); break;
					case A_rd:
						error = 0.0f; break;
				}
				// evaluate predicted landmark position by comparing with observed landmark position
				//error = error / A_limitstep[A_dim_r[d]]; // division not necessary but normalizes
				if (p->lanerrormedian) {
					p->tmp_obs_amp[m] = error * error;
				}
				else {
					weight += error * error * p->obs_weight[p->lan[m]];
				}
			}
			else {
				if (p->lanerrormedian) {
					p->tmp_obs_amp[m] = 0.0f;
				}
			}

			// assign weight (all weights in range 0...1)
			////////////////////////////////////////////////////////////////////

			////////////////////////////////////////////////////////////////////

		}
		if (d == 3) {
			int aaaaa = 0;
		}
		// finish median landmark - observation error metric
		if (p->lanerrormedian) {
			sample_median(p->tmp_obs_amp, p->Npar, 0, p->tmp_obs_amp_buffer_left, p->tmp_obs_amp_buffer_right, &weight);
		}


		// add error metric to particle pdf describing likelihood of each particle occurance
		if (weight != 0.0f && !(fpclassify(weight) == FP_SUBNORMAL)) {
			weight_inverse(&weight);
		}

		if (isnan(weight)) {
			int aaaa = 0;
		}
		if (weight < 0.0f) {
			int aaa = 0;
		}
		p->par_pdf[d][i] = weight;

		// limit derived particle position to coordinate boundaries
		//switch (A_dim_r[d]) {
			//case A_az: correct_limit_circular(&p->par[i].pos[d].x[A_ref], A_limit[A_dim_r[d]][A_limit_low], A_limit[A_dim_r[d]][A_limit_high]); break; already done by kalman filter
			//case A_el: correct_limit(&p->par[i].pos[d].x[A_ref], A_limit[A_dim_r[d]][A_limit_low], A_limit[A_dim_r[d]][A_limit_high]); break;
		//}
	}

	// normalize particle pdf and amplify differences
	weight = ar_sum(p->par_pdf[d], p->Npar);
	if (weight != 0.0f && !(fpclassify(weight) == FP_SUBNORMAL)) {
		ar_divide(p->par_pdf[d], p->Npar, weight); // [1(good)...0(bad)]
	}
	ar_weight_power(p->par_pdf[d], p->Npar, 10.0f);


	// Derive most likely particle position for the dimension and set as the most likely new particle position usable for next dimension iteration
	ar_max(p->par_pdf[d], p->Npar, &par_pos_most_likely_i, &par_pos_most_likely);
	for (t = 0; t < A_Ndofderivative; t++) {
		if (d == 0 && t == 0) {
			console_logi(PRIO_PARTICLE, "Most likely particle: %i\n", par_pos_most_likely_i + 1);
		}
		p->pos[d][t] = p->par[par_pos_most_likely_i].pos[d].x[t];
	}

	// Debug
	for (i = 0; i < p->Npar; i++) {
		if (p->par_pdf[d][i] < 0.0f) {
			int bbb = 0;
		}
		if(isnan(p->par_pdf[d][i])) {
			int bb = 0;
		}
	}

}


void resample_particleslam_6dof(struct par6s* p, int d) {
	// init
	int i, m;
	int par_pos_likely_i;


	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	// particle: sample from cdf and resample most likely particle positions as initial position for next algorithm update
	///////////////////////////////////////////////////////////////////////////////////////////////////////////

	// derive particle cdf from pdf to sample from it
	pdf_to_cdf(p->par_pdf[d], p->Npar, p->par_cdf[d]);

	// resample particles
	for (i = 0; i < p->Npar; i++) {
		// sample new particle from particle cdf
		sample_index_from_cdf(p->par_cdf[d], p->Npar, &par_pos_likely_i);

		// copy relevant memory
		// particle position
		memcpy(p->par[i].pos[d].x, p->par[par_pos_likely_i].pos[d].x, A_Ndofderivative * sizeof(float));
		// state of the landmark kalman filters (other variables identical as R_inf = const)
		for (m = 0; m < p->Nobs; m++) {
			memcpy(p->par[i].kal[d][m].x, p->par[par_pos_likely_i].kal[d][m].x, A_Ndofderivative * sizeof(float));
		}
	}
	
	// debuging
	//write_particleslam_debug(p, "", "dof", "par_pdfcdf", 1, A_Ndof, p->Npar, 1);
	//write_particleslam_debug(p, "par", "pos_dof", "pos_der", p->Npar, A_Ndof, A_Ndofderivative, 1);

}

void integrate_once(float *x, float* T, float *y) {
	*y = *T * *x;

}
void integrate_twice(float *x, float* T, float *y) {
	*y = 2.0f / (*T * *T) * *x;

}

void differentiate_once(float *x, float* T, float *y) {
	*y = *T * *x;

}
void differentiate_twice(float *x, float* T, float *y) {
	*y = 2.0f / (*T * *T) * *x;

}

void weight_power(float* x, float power) {
	*x = powf(*x, power);
}

void weight_inverse(float *x) {
	*x = 1.0f / *x;
}

void weight_1_0_flip(float *x) {
	*x = 1.0f - *x;
}

void weight_log(float* x) {
	*x = logf(*x);
}

void weight_logminus(float* x) {
	*x = -logf(*x);
}

void weight_exp(float* x) {
	*x = expf(*x);
}

void ar_weight_power(float* x, int N, float power) {
	for (int n = 0; n < N; n++) {
		x[n] = powf(x[n], power);
	}
}


