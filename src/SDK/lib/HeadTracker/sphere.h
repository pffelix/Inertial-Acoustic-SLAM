// sphere.h: Spherical sound incidence estimation
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once
//#include "debugging.h"
#include "params.h"
#include "head.h"
#include "rotation.h"

#ifdef __cplusplus
extern "C" {
#endif
	// distance between 2 points in cartesian coordinates
	void distance(float* pc1, float* pc2, float* dist);

	// distance between 2 points in cartesian coordinates in milliseconds
	void distance_in_ms(float* pc1, float* pc2, float sos, float* dist_ms);

	// get azimuth and elevation bin where sphere surface is maximal and the attached amplitude
	void get_sph_max_n(float** sph_x, int* az_n_max, int* el_n_max, float* max_amp, int Naz, int Nel);

	// set azimuth and elevation bin where sphere surface is maximal to certain amplitude value
	void set_sph_max_n(float** sph_x_read, float** sph_x_set, int Naz, int Nel, float value);

	// get azimuth and elevation angles where sphere surface is maximal out of Nmax maximum angles and the variance between it
	void get_sph_max_az_el(float** sph_x, float* az_max, float* el_max, float* az_var, float* el_var, int ang_stepsize, int Naz, int Nel, int Nmax);

	// reset energy in  sound energy in total sphere to value
	void reset_sph(float** sph_x, long az_N, long el_N, float value);

	// calculate the mapping between cross-correlation bin and sphere angle and the and rate of occurance of each mapped angle
	void get_map_tdn_ang(long cor_N, long cor_fs, long cor_Nlim, long Nang, long* map_tdn_ang, long* w_Ntdn_ang, int Ntd_ang, bool reset);
	
	// selects orthogonal or free version of add_td_energy_to_sph()
	void add_td_energy_to_sph_select(float* cor, long cor_N, long cor_fs, long cor_Nlim, float** sph_x, float** sph_xtmp, long*** map_tdn_sph, long* map_tdn_ang, long* w_Ntdn_ang, long* w_Ntd_ang_sol, long** w_Nsphn, int Naz, int Nel, int tdn_ang_limit, int tdn_ang_max, bool orthogonal, bool cor_positive);

	// add energy at certain time difference to a orthogonal sphere of sound energy incidence (cor_Nlim must be minimum 90 bins to cover all angles of incidence)
	// positive_cor: only add cross-correlation bins with postive value to sphere (recommended for removing ambiguity for microphones between lamdba/2 and lamdba wavelength separated, but not very largely or shortly separated)
	void add_td_energy_to_sph(float* cor, long cor_N, long cor_fs, long cor_Nlim, float** sph_x, float** sph_xtmp, long*** map_tdn_sph, long* map_tdn_ang, long* w_Ntdn_ang, long* w_Ntd_ang_sol, long** w_Nsphn, int Naz, int Nel, int tdn_ang_limit, int tdn_ang_max, bool cor_positive);

	// add energy at certain time difference to a orthogonal sphere of sound energy incidence (cor_Nlim must be minimum 90 bins to cover all angles of incidence)
	// positive_cor: only add cross-correlation bins with postive value to sphere (recommended for removing ambiguity for microphones between lamdba/2 and lamdba wavelength separated, but not very largely or shortly separated)
	// in this code version microphones have to be orthogonal
	void add_td_energy_to_sph_orthogonal(float* cor, long cor_N, long cor_fs, long cor_Nlim, float** sph_x, float** sph_xtmp, long*** map_tdn_sph, long* map_tdn_ang, long* w_Ntdn_ang, long* w_Ntd_ang_sol, long** w_Nsphn, int Naz, int Nel, int tdn_ang_limit, int tdn_ang_max, bool cor_positive);

	// add map angle resulting from time delay of sound incidence given for a cross-correlation between 2 microphones to a spherical angle of sound incidence in a sphere of sound incidence
	// in this code version microphones do not have to be orthogonal
	void map_td_to_sph(float* mic_loc1, float* mic_norm1, float* mic_loc2, float* mic_norm2, float radius, int ang_stepsize, int Naz, int Nel, long*** map_tdn_sph, long* w_Ntd_ang_sol, long** w_Nsphn);

	// add map angle resulting from time delay of sound incidence given for a cross-correlation between 2 microphones to a spherical angle of sound incidence in a sphere of sound incidence
	// in this code version microphones have to be orthogonal
	void map_td_to_sph_orthogonal(float* mic_loc1, float* mic_norm1, float* mic_loc2, float* mic_norm2, float radius, int ang_stepsize, int Naz, int Nel, long*** map_tdn_sph, long* w_Ntd_ang_sol, long** w_Nsphn);

#ifdef __cplusplus
};
#endif