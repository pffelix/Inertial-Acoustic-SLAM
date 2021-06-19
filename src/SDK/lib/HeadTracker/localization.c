// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "localization.h"

void init_localization(struct locs* loc, float xi1, float xi2, float xi3, float xi4, float yi1, float yi2, float yi3, float yi4, float zi1, float zi2, float zi3, float zi4, float zero_offset, bool td_absolute) {
		
		// pre-set variables

		// number of channels
		loc->Nch = 4;
		// number of dimensions
		loc->Ndim = 3;
		// number of solutions
		loc->Nsol = 2;
		// zero offset
		loc->zero_offset = zero_offset;
		// use absolute time difference between microphones instead of +/- signed time difference
		loc->td_absolute = td_absolute;

		// declare arrays
		// (unknown) position of head (2 solutions)
		loc->xyz2sol = ar_zeros2d(loc->Nsol, loc->Nch);
		// (unknown) position of head
		loc->xyz = ar_zeros(loc->Nch);
		// (known) position of microphones: i, j, k, l
		loc->xyzi = ar_declare2d(loc->Nch, loc->Ndim);
		// (known) squared position of microphones: i, j, k, l
		loc->xyzi2 = ar_zeros2d(loc->Nch, loc->Ndim);
		// (known) difference in position of microphones: x_ji, x_ki, x_jk, x_lk
		loc->xyzji = ar_zeros2d(loc->Nch, loc->Ndim);
		// (known) time of arrival difference expressed in metre between microphones multiplied by sound speed: R_ij, R_ik, R_kj, R_kl
		loc->Rij = ar_zeros(loc->Nch); 
		// (known) squared time of arrival difference expressed in metre between microphones multiplied by sound speed: R_ij, R_ik, R_kj, R_kl
		loc->Rij2 = ar_zeros(loc->Nch); 
		// (calculated) time of arrival prediction error between microphones multiplied by sound speed: R_i, R_j, R_k, R_l
		loc->error = ar_zeros(loc->Nch); 

		// pre-set arrays
		loc->xyzi[0][0] = xi1;
		loc->xyzi[1][0] = xi2;
		loc->xyzi[2][0] = xi3;
		loc->xyzi[3][0] = xi4;
		loc->xyzi[0][1] = yi1;
		loc->xyzi[1][1] = yi2;
		loc->xyzi[2][1] = yi3;
		loc->xyzi[3][1] = yi4;
		loc->xyzi[0][2] = zi1;
		loc->xyzi[1][2] = zi2;
		loc->xyzi[2][2] = zi3;
		loc->xyzi[3][2] = zi4;

		for (int dim = 0; dim < loc->Ndim; dim++) {
		loc->xyzji[0][dim] = loc->xyzi[1][dim] - loc->xyzi[0][dim]; // x_ji
		loc->xyzji[1][dim] = loc->xyzi[2][dim] - loc->xyzi[0][dim]; // x_ki
		loc->xyzji[2][dim] = loc->xyzi[1][dim] - loc->xyzi[2][dim]; // x_jk
		loc->xyzji[3][dim] = loc->xyzi[3][dim] - loc->xyzi[2][dim]; // x_lk
		}

		for (int ch = 0; ch < loc->Nch; ch++) {
			for (int dim = 0; dim < loc->Ndim; dim++) {
				loc->xyzi2[ch][dim] = loc->xyzi[ch][dim] * loc->xyzi[ch][dim];
			}
		}
};

void update_localization(struct locs * loc){
	// uses as reference the equations of "Sreeram Potluri (2002) - Hyperbolic position location estimator with TDOAs from four stations" 
	// temporary values
	loc->A = 0.0f;
	loc->B = 0.0f;
	loc->C = 0.0f;
	loc->D = 0.0f;
	loc->E = 0.0f;
	loc->F = 0.0f;
	loc->G = 0.0f;
	loc->H = 0.0f;
	loc->I = 0.0f;
	loc->J = 0.0f;
	loc->K = 0.0f;
	loc->L = 0.0f;
	loc->M = 0.0f;
	loc->N = 0.0f;
	loc->O = 0.0f;
	loc->tmp = 0.0f;
	int ch;
	int sol;

    // calculate

    // R_ij, R_ik, R_kj, R_kl and its square
	for (ch = 0; ch < loc->Nch; ch++) {
		//->Rij[ch] = loc->tdmetre[ch]; // absolute time delay tdmetre value is also possible
		if (loc->td_absolute) {
			loc->Rij[ch] = fabsf(loc->Rij[ch]);
		}
		loc->Rij2[ch] = loc->Rij[ch] * loc->Rij[ch];
	}

	// add zero_offset if sound is coming from 0 or 180 degree azimuth angle to avoid null division
    loc->tmp = loc->Rij[0] * loc->xyzji[1][1] - loc->Rij[1] * loc->xyzji[0][1];
	if (loc->tmp == 0.0f) {
		loc->tmp = loc->zero_offset;
		for (ch = 0; ch < loc->Nch; ch++) {
			loc->Rij[ch] = loc->Rij[ch] + loc->zero_offset;
		}
	}

	// A, B, C
    loc->A = (loc->Rij[1] * loc->xyzji[0][0] - loc->Rij[0] * loc->xyzji[1][0]) / loc->tmp;
    loc->B = (loc->Rij[1] * loc->xyzji[0][2] - loc->Rij[0] * loc->xyzji[1][2]) / loc->tmp;
    loc->C = (loc->Rij[1] * (loc->Rij2[0] + loc->xyzi2[0][0] - loc->xyzi2[1][0] + loc->xyzi2[0][1] - loc->xyzi2[1][1] + loc->xyzi2[0][2] - loc->xyzi2[1][2]) - loc->Rij[0] * (loc->Rij2[1] + loc->xyzi2[0][0] - loc->xyzi2[2][0] + loc->xyzi2[0][1] - loc->xyzi2[2][1] + loc->xyzi2[0][2] - loc->xyzi2[2][2])) / (2.0f * loc->tmp);

	// add zero_offset if sound is coming from 0 or 180 degree azimuth angle to avoid null division
    loc->tmp = loc->Rij[2] * loc->xyzji[3][1] - loc->Rij[3] * loc->xyzji[2][1];
	if (loc->tmp == 0.0f) {
		loc->tmp = loc->zero_offset;
		for (ch = 0; ch < loc->Nch; ch++) {
			loc->Rij[ch] += loc->zero_offset;
		}
	}

	// D, E, F
	loc->D = (loc->Rij[3] * loc->xyzji[2][0] - loc->Rij[2] * loc->xyzji[3][0]) / loc->tmp;
    loc->E = (loc->Rij[3] * loc->xyzji[2][2] - loc->Rij[2] * loc->xyzji[3][2]) / loc->tmp;
    loc->F = (loc->Rij[3] * (loc->Rij2[2] + loc->xyzi2[2][0] - loc->xyzi2[1][0] + loc->xyzi2[2][1] - loc->xyzi2[1][1] + loc->xyzi2[2][2] - loc->xyzi2[1][2]) - loc->Rij[2] * (loc->Rij2[3] + loc->xyzi2[2][0] - loc->xyzi2[3][0] + loc->xyzi2[2][1] - loc->xyzi2[3][1] + loc->xyzi2[2][2] - loc->xyzi2[3][2])) / (2.0f * loc->tmp);

	// G, H
    loc->G = (loc->E - loc->B) / (loc->A - loc->D);  // rounding errors can occur because of float
    loc->H = (loc->F - loc->C) / (loc->A - loc->D); // rounding errors can occur because of float

	// I, J
    loc->I = loc->A * loc->G + loc->B;
    loc->J = loc->A * loc->H + loc->C;

	// K, L
    loc->K = loc->Rij2[1] + loc->xyzi2[0][0] - loc->xyzi2[2][0] + loc->xyzi2[0][1] - loc->xyzi2[2][1] + loc->xyzi2[0][2] - loc->xyzi2[2][2] + 2.0f * loc->xyzji[1][0] * loc->H + 2.0f * loc->xyzji[1][1] * loc->J;
    loc->L = 2.0f * (loc->xyzji[1][0] * loc->G + loc->xyzji[1][1] * loc->I + 2.0f * loc->xyzji[1][2]);

	// M, N, O
    loc->M = 4.0f * loc->Rij2[1] * (loc->G * loc->G + loc->I * loc->I + 1.0f) - loc->L * loc->L; 
    loc->N = 8.0f * loc->Rij2[1] * (loc->G * (loc->xyzi[0][0] - loc->H) + loc->I * (loc->xyzi[0][1] - loc->J) + loc->xyzi[0][2]) + 2.0f * loc->L * loc->K;
    loc->O = 4.0f * loc->Rij2[1] * ((loc->xyzi[0][0] - loc->H) * (loc->xyzi[0][0] - loc->H) + (loc->xyzi[0][1] - loc->J) * (loc->xyzi[0][1] - loc->J) + loc->xyzi2[0][2]) - loc->K * loc->K;

	// z coordinate of sound source and its second solution
	// second part of quadratic formula
	loc->tmp = (loc->N / (2.0f * loc->M)) * (loc->N / (2.0f * loc->M)) - loc->O / loc->M;
	if (loc->tmp >= 0.0f) {
		loc->tmp = sqrtf(loc->tmp);
	}
	else {
        console_logf(PRIO_LOCALIZATIONDETAIL, "3D localization: Z dimension is complex with imaginary part sqrt of: %4.4f\n", loc->tmp);
		loc->tmp = 0.0f;
	}
    loc->xyz2sol[0][2] = loc->tmp;
    loc->xyz2sol[1][2] = -loc->tmp; 
	// first part of quadratic formula
	loc->tmp = loc->N / (2.0f * loc->M);
	loc->xyz2sol[0][2] += loc->tmp;
	loc->xyz2sol[1][2] += loc->tmp;

	// x and y coordinate of sound source and its second solution
	for (sol = 0; sol < loc->Nsol; sol++) {
		loc->xyz2sol[sol][0] = loc->G * loc->xyz2sol[sol][2] + loc->H; // relevant is only H, as G = 0 if z=0 of microphones
		loc->xyz2sol[sol][1] = loc->I * loc->xyz2sol[sol][2] + loc->J; // relevant is only J, asIG = 0 if z=0 of microphones
	}
    
    // estimation error
	for (ch = 0; ch < loc->Nch; ch++) {
		//loc->error[ch] = Ri[ch] - ((loc->xyzi[ch][0] - loc->xyz[0]) ^ 2 + (loc->xyzi[ch][1] - loc->xyz[1]) ^ 2 + (loc->xyzi[ch][2] - loc->xyz[2]) ^ 2) ^ 0.5;
	}
};
