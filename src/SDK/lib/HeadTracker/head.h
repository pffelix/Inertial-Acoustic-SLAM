// head.h: Head orientation calculation from features
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once
#include "debugging.h"

#ifdef __cplusplus
extern "C" {
#endif

	// calculate azimuth head orientation from time difference in seconds
	void td_to_ang(float td, float td_maximal_msec, float *ang);

	// calculate azimuth head orientation from level difference in dB
	void ld_to_ang(float ld, float ld_maximal_db, float *ang);

	// calculate differential angle from two angles
	void ang_to_angdiff(float ang1, float ang2, float* angdiff);

	// calculate differential circular around 360 degree angle from differential angle
	void angdiff_to_circular360(float* angdiff);

	// calculate 
	void ang_to_tdn(float ang, long N_lim, long* tdn);

	// spherical coordinate system to interaural-polar coordinate system (sph_az=[0(front),359], sph_el=(90(top),-90(bottom)]
	void ang_sph_to_ipc(float sph_az, float sph_el, float* ipc_az, float* ipc_el);

	// interaural-polar coordinate system to spherical coordinate system (ipc_az=[-90(left),90(right)], ipc_el=(-90(bottom),270(back to azimuth)]
	void ang_ipc_to_sph(float ipc_az, float ipc_el, float* sph_az, float* sph_el);

	// convert arcus-tangens atan2() azimuth ouput range [90...0,0...-90,-90...-180,180...90] to azimuth [0...360] degree
	void atan2_to_az(float* az);

	//  convert iso azimuth angle [90...360...90] to azimuth angle [0..360]
	void az90_to_az(float* az);

	// convert azimuth angle [0..360] to iso azimuth angle [90...360...90]
	void az_to_az90(float* az);

	// convert azimuth angle [0..360] to azimuth angle signed [0...180,-180...0]
	void az_to_azsigned(float* deg);

	// convert azimuth angle signed [0...180,-180...0] to azimuth angle [0..360]
	void azsigned_to_az(float* deg);

	// convert elevation angle [0..180] to elevation angle signed [90...0,0...-90]
	void el_to_elsigned(float* ang_el);

	// convert elevation angle signed [90...0,0...-90] to elevation angle [0..180]
	void elsigned_to_el(float* ang_el);

	// convert elevation bin [0..x] to elevation angle signed [-90...0,0...90]
	void elbin_to_elsigned(long bin_el, float* ang_el, int sph_ang_stepsize);

	// convert yaw [0...-180,180...0] to azimuth [0...360] 
	void yaw_to_az(float yaw, float* ang_az);

	// convert pitch [-90...90] to elevation [0...180] 
	void pitch_to_el(float pitch, float* ang_el);

	// convert yaw [0...360], pitch [90...-90] coordinate unit vector to spherical angle [0...360,0...180] (ignoring roll information) 
	void yawpitch_to_sph_slow(float yaw, float pitch, float* az, float* el);

	// convert cartesian coordinates to spherical coordinates [0...360, 0...180]
	void cart_to_sph(float x, float y, float z, float* az, float* el, float* rd);

	// convert spherical coordinates [0...360, 0...180] to cartesian coordinates
	void sph_to_cart(float az, float el, float rd, float* x, float* y, float* z);

#ifdef __cplusplus
};
#endif