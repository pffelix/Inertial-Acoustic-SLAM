// params.h: Fixed parameters for algorithm.
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once
#include <string.h>
#include "debugging.h"
#include <stdbool.h>
#include <math.h>

// define core program framework parameters
#define FRAMEWORK_WINDOWSUSED 1 // Windows system is used (1) or Linux / Apple / Android used (0), has impact on selected clock timer functions in debugging.c, more precision in windows
#define FRAMEWORK_OFFLINEREAD 1 // read sensor data from disk (1) or read real-time sensor data (1)
#define FRAMEWORK_OFFLINESAVE 0 // save sensor data to disk (1) or do not save sensor data to disk (0)
#define FRAMEWORK_OFFLINEREADSAVE 0 // read and save modified sensor data to disk (1)
#define FRAMEWORK_IRONLY 0 // calculate only impulse response as fast as possible

// define constants
#define PI 3.14159265358979323846F
#define PI2 6.28318530717958647692F
#define PI_2 1.57079632679489661923
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
#define M_PIf 3.14159265358979323846F
#define DEG_TO_RAD 0.01745329251994329577F
#define RAD_TO_DEG 57.29577951308232087685F
#define path_N 512 // length of file system path character arrays
#define char_N 50 // length of standard char array
#define charlong_N 100 // length of long char array
#define int_N 50
// length of standard int array
//#define  REALSIZE     4                                  /* complex number data type bits in units of byte */
#define pwrtwo(x) (1 << (x))

#define PRIO_HIGH pwrtwo(0) // Logging base types
#define PRIO_NORMAL pwrtwo(1)

#define PRIO_LOW pwrtwo(2)
#define PRIO_TIMER pwrtwo(3)
#define PRIO_WARNING pwrtwo(4)
#define PRIO_INFO pwrtwo(5)
#define PRIO_STRUCTURE pwrtwo(6)
#define PRIO_STATE pwrtwo(7)
#define PRIO_TIMERDETAIL pwrtwo(8)
#define PRIO_ERROR pwrtwo(9)
#define PRIO_FFTTIMER pwrtwo(10)
#define PRIO_FFTTIMERDETAIL pwrtwo(11)
#define PRIO_LOCALIZATION pwrtwo(12)
#define PRIO_LOCALIZATIONDETAIL pwrtwo(13)
#define PRIO_BEAM pwrtwo(14)
#define PRIO_BEAMDETAIL pwrtwo(15)
#define PRIO_WARNINGDEACTIVATED pwrtwo(16)
#define PRIO_SYNC pwrtwo(17)
#define PRIO_MAIN pwrtwo(18)
#define PRIO_IMU pwrtwo(19)
#define PRIO_NGIMU pwrtwo(20)
#define PRIO_LITTLEWARNING pwrtwo(21)
#define PRIO_PARTICLE pwrtwo(22)
#define PRIO_LATENCY pwrtwo(23)

// define what logging base types will be print in console
//#define PRIO_LOG (PRIO_HIGH | PRIO_NORMAL | PRIO_TIMER | PRIO_WARNING | PRIO_INFO | PRIO_STRUCTURE | PRIO_STATE | PRIO_TIMERDETAIL | PRIO_FFTTIMER | PRIO_LOCALIZATION |PRIO_BEAMDETAIL)
#define PRIO_LOG (PRIO_LATENCY) //PRIO_SYNC | PRIO_MAIN | PRIO_WARNING | PRIO_PARTICLE)

// define tracking types
#define Mixed 0 // using acoustics head tracker, imu head tracker and accelerometer
#define Acoustic 1 // using acoustic head tracker
#define Imu 2 // using imu head tracker

// define GUI plot types
#define GUI_Imu 0 // position from imu estimation (only head orientation)
#define GUI_Slam 1 // position from slam estimation (head orientation and x,y position)
#define GUI_Truth_is 2 // position from vicon ground truth data of impulse source (source orientation and x,y position)
#define GUI_Truth_rec 3 // position from vicon ground truth data of recording receiver (head orientation and x,y position)

// define GUI size
#define GUI_x_pixel 500 // pixel in x-direction
#define GUI_y_pixel 950 // pixel in y-direction

// define vicon objects
#define VIC_is 0 // object number of sound impulse source markers
#define VIC_rec 1 // object number of sound recording receiver markers

// define assignments
// on/off
#define A_on 1
#define A_off 0

// asignment of recording channel name to physical input and recording channel number
#define A_l 0 // left
#define A_l2 1 // left 2
#define A_r 2 // right
#define A_r2 3 // right 2
#define A_rf 4 // front
#define A_rf2 5 // front 2

// asignment of recording channel name to virtual recording channel number
#define A_beam_lc 6 // left front (virtual beamformed)
#define A_beam_lg 7 // right front (virtual beamformed)
#define A_beam_rf 8 // left back (virtual beamformed)
#define A_beam_rb 9 // right back (virtual beamformed)
#define A_beam_rr 10 // left back (virtual beamformed)
#define A_beam_rl 11 // right back (virtual beamformed)

// asignment of audio input channel name to physical input channel number
#define A_sound 6 // input channel of sound that is binauralized: 6 analog, 10 digital

// asignment of output channel name to physical output channel number
#define A_bisoundl 0 // binauralized sound left ear
#define A_bisoundr 1 // binauralized sound right ear
#define A_is 2 // mono impulse

// asignment of cross-channel name to cross channel number
#define A_lr 0 // left-right
//#define A_l2r2 1 // left 2 - right 2
#define A_ll2 1 // left - left 2
#define A_rr2 2 // right -right 2
//#define A_lr2 4 // left - right 2
//#define A_l2r 5 // left 2 - right
#define A_ff2 3 // front - front 2

// assignment of front and back channels
//#define A_f 0 // front
//#define A_b 1 // back

// assignment of cartesian and angular position coordinates
#define A_x 0 // x
#define A_y 1 // y
#define A_z 2 // z
#define A_az 3 // azimuth
#define A_el 4 // elevation
#define A_rd 5 // radius

// assignment of number of standard elements to position coordinates
#define A_Naz 360 // azimuth
#define A_Nel 180 // elevation

// positional coordinates
#define A_Ndim 6 // number of dimensions used in head/location tracker: 6: x,y,z,az,el, 2: az,el
#define A_Ndimxyz 3 // number of dimensions cartesian coordinates
#define A_Ndimsph 3 // number of dimensions spherical coordinates
#define A_Ndimyawpitchroll 3 // number of dimensions yaw-pitch-roll coordinates

// head tracker
#define A_Ndof 6 // number of dimensions observed in head/location tracker: 6: x,y,z,az,el, 2: az,el
#define A_Ndofderivative 3 // number of derivatives observed in head/location tracker: 1: A_ref 2: A_ref, A_diff, 3: A_ref, A_diff, A_acc
#define A_Ntoa 2 // number of dimensions observed in time or arrival: 

// algorithm
#define A_Nalgo 1 // number of dimensions of algorithm state variables
#define A_rate_ms 0 // update rate in ms (ms required as otherwise sampling rate at debugging 0 Hz)

// assignment of spherical coordinates
#define A_sph_az 0 // azimuth (phi)
#define A_sph_el 1 // elevation (theta)
#define A_sph_rd 2 // radius (r)

// assignament of derivates
#define A_ref 0 // absolute coordinate (reference)
#define A_vel 1 // 1st derivative (velocity)
#define A_acc 2 // 2nd derivative (acceleration)
#define A_Npos 3 // positional coordinate (x,y,z)
#define A_Nposang 6 // positional coordinate (x,y,z,az,el,rd)
#define A_Nderivative 3 // number of derivatives positional coordinate (ref,vel,acc)
#define A_Nang 3 // number of angles positional coordinate (az,el,rd or yaw, pitch, roll)

// assignment of microphone setups
#define A_xno 0 // delay scanned in left-right direction, focus no
#define A_xf 1 // delay scanned in left-right direction, focus front
#define A_xb 2 // delay scanned in left-right direction, focus back
#define A_xc 3 // delay scanned in left-right direction, focus ceil
#define A_xg 4 // delay scanned in left-right direction, focus ground
#define A_yno 5 // delay scanned in front-back direction, focus no
#define A_yl 6 // delay scanned in front-back direction, focus left
#define A_yr 7 // delay scanned in front-back direction, focus right
#define A_yc 8 // delay scanned in front-back direction, focus ceil
#define A_yg 9 // delay scanned in front-back direction, focus ground
#define A_zno 10 // delay scanned in ceil-ground direction, focus no
#define A_zl 11 // delay scanned in ceil-ground direction, focus left
#define A_zr 12 // delay scanned in ceil-ground direction, focus right
#define A_zf 13 // delay scanned in ceil-ground direction, focus front
#define A_zb 14 // delay scanned in ceil-ground direction, focus back

// assignemnt of toa
#define A_toaframe 0
#define A_toaspeaker 1

// kalman filter assignment
#define Xtrans 0
#define Xrot 1
#define Xrotmap 2
#define Ytrans 0
#define Yrotimu 1
#define Yrotacu 2

// kalman filter deactive observation 
#define A_ron 1.0f / (1000000.0f)
#define A_roff (1000000.0f)
#define A_roff2 (A_roff * A_roff)
#define A_roff3 (A_roff * A_roff * A_roff)

// particle filter
#define A_obsx 0 // particle 0 temporary stores in its y landmark observation x state

//  limits
#define A_limit_low 0
#define A_limit_high 1
#define A_limit_N 2

// probability debug 
#define A_debprob_off 0
#define A_debprob_sph 1
#define A_debprob_xyzsph 2

// frame weights
#define A_weight_N 7
#define A_weight_energy 0
#define A_weight_peak 1
#define A_weight_sparsity 2
#define A_weight_stability 3
#define A_weight_elevation 4
#define A_weight_early 5
#define A_weight_global 6

// particle prior
#define A_parprior_kalsingle 0
#define A_parprior_kalmulti 1
#define A_parprior_median 2
#define A_parprior_medianabs 3

// kalman mode
#define A_mode_ref 0 
#define A_mode_vel 1
#define A_mode_refvel 2


// enums for filter library
enum TPassTypeName { LPF, HPF, BPF, NOTCH };
enum TWindowType {
	wtNONE, wtKAISER, wtSINC, wtHANNING, wtHAMMING, wtBLACKMAN,
	wtFLATTOP, wtBLACKMAN_HARRIS, wtBLACKMAN_NUTTALL, wtNUTTALL,
	wtKAISER_BESSEL, wtTRAPEZOID, wtGAUSS, wtSINE, wtTEST
};

#ifdef __cplusplus
extern "C" {
#endif


	// variables
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// general setup
	extern long fs; // sampling rate, change in sampling rate requires 2 times restart of program
	extern float update_ms; // timer interval for Head Tracking Update calculation in ms
	extern float sos; // speed of sound in metre / s
	extern float T; // time in sec between two updates 

	// offline read in of sensor data
	extern char offline_path[path_N]; // path where offline data is read from
	extern bool offline_rec; // activate offline saving of impulse recording data
	extern bool offline_ir; // activate offline saving of impulse response data
	extern bool offline_head; // activate offline saving of head position data
	extern bool offline_imu; // activate offline saving of imu position data
	extern bool offline_vicis; // activate offline saving of vicon ground truth loudspeaker position data
	extern bool offline_vicrec; // activate offline saving of vicon ground truth head position data
	extern bool offline_weight; // activate offline saving of frame weight data
	extern bool offline_toa; // activate offline saving of time of arrival data
	extern bool offline_parml; // activate offline saving of most likely particle (slam) position
	extern bool offline_algo; // activate offline saving of program data

	// routing between internal channels
	extern int R_rec[int_N]; // routing from phyiscal channel number to internal recording channel number
	
	// routing between internal cross-channels
	extern int R_cross[int_N][2]; // routing between channels and cross channels (e.g. which channels are correlated with each other)

	// binauralization
	extern bool binaural_on;

	// impulse
	extern float is_ms; // pulse playback_duration in ms
	extern char is_type[char_N]; // pulse type, "linear", "exponential"
	extern double is_amp; // amplitude of impulse
	extern long is_f1; // impulse lower limit frequency in Hz
	extern long is_f2; // impulse upper limit frequency in Hz
	extern char is_fadein_window[char_N]; // use fade-in window for impulse edeges to make it silent: "none", "linear", "hann", "blackman"
	extern double is_fadein_percent; // duration of fade in window in percent (expressed in decimal float);
	extern char is_fadeout_window[char_N]; // use fade-out window for impulse edeges to make it silent: "none", "linear", "hann", "blackman"
	extern double is_fadeout_percent; // duration of fade out window in percent (expressed in decimal float);
	extern bool is_unfade; // unfade fade in and fade out of recording for impulse response convolution
	extern bool is_fade_inverse; // fade in and out inverse impulse, when impulse is faded in and out

	// inverse impulse
	extern char isinv_type[char_N]; // inverse pulse type, "analytical", "kirkeby"

	// beamforming
	extern bool beam_on; // beamforming activation
	extern int beam_Nch; // number of used beamforming arrays
	extern int beam_Nmic; // number of microphones used in one beamforming array
	extern int beam_Nang; // number of angles where the beamforming arrays are steered to 
	extern float beam_ang[int_N]; // steering angle where the beamforming arrays are focused on


	// channels (physical input)
	extern int ch_Nmic; // number of input channels for microphone recordings
	extern int ch_Nsound; // number of input channels for sound that is binauralized

	// channels (physical output)
	extern int ch_Nis; // number of output channels for impulse playback
	extern int ch_Nbisound;// number of output channels used for binauralized sound

	// channels (internal)
    extern int ch_Nrec; // number of recording channels
	extern int ch_Nrecbeam; // number of recording channels inclusive virtually beamformed recording channels
	extern int ch_Ncross; // number of internal channels containing signals between the recording channels
	extern int ch_Ncrossbeam; // number of internal channels containing signals between the recording channels inclusive virtually beamformed channels


	// microphones
	extern float mic_interdistance; // distance between 2 microphones used for beamforming (distance between open holes)
	extern float mic_loc[int_N][3]; // position of microphones in cartesian coordinates in metre centered around head
	extern float mic_norm[int_N][3]; // normal (focusing direction) of microphones in cartesian corredinates in metre centered around head
	
	// recording buffer
	float recbuffer_ms; // recording buffer duration in ms

	// rapid impulse response measurement technique;
	extern bool rapid_on; // use rapid impulse response measurement technique
	extern bool rapid_snrfull; // use full SNR of rapid impulse response measurement technique (otherwise only half sided spectrum)
	extern float rapid_edge_ms; // rapid impulse edge duration in ms
	extern bool rapid_edge_on; // clean rapid impulse response edges
	extern char rapid_fade_window[char_N]; // use fade-in / fade-out window for rapid impulse recording to optimize frequency response: "none", "linear", "hann"
	extern float rapid_fade_percent; // duration of fade-in / fade-out window for rapid impulse recording in percent (expressed in decimal float);
	
	// recording
	extern float rec_ms; // impulse recording duration in ms
	extern bool rec_edge_on; // clean impulse response edges
	extern float rec_edge_ms; // impulse edge duration in ms



	// upsampling
	extern bool up_fft; // do upsampling via fft
	extern int up_N; // impulse response upsampling factor

	// synchronization
	extern bool sync_on; // synchronize clock of playback device and recording device (needed if not the same device)
	extern bool sync_update; // update synchronization in every loop or only at program start (problem smartphone has a slightly different clock -> signal moves when false (clock compensation has to be applied), but jumps when true as synchronization not fine enough)
	extern float sync_clock; // clock compensation samples/s: shift impulse response in each update about subsample dimension x (+ shift to right, - shift to left)
	
	// impulse response frames
	extern float frame_ms; // length of each frame window in milliseconds
	extern float frame_overlap; // overlap in percent of frame length (written as decimal)
	extern long frame_Mstart; // frame number to start with
	extern long frame_M; // number of frames (to be screened for sound energy from impulse response)
	extern long frame_Mon; // number of selected frames (to be considered from impulse response)
	extern bool frame_window; // window signal frames at edges with hann window
	extern bool frame_normalize; // normalize each frame window, 
	extern bool frame_energy; // use sound energy instead of sound pressure for cross-correlation
	extern int frame_reference; // reference frame used for calculting statics (framework has to be expanded if z microphone is other head side)

	// impulse response frame weights
	extern float weight_el_diff; // weight for deviation from horizontal plane

	// impulse response full
	extern bool ir_multichannelnormalize; // activate multichannel normalization before start sample is searched (takes 2ms)
	extern bool ir_startinit; // init start sample once at beginning, by looking for threshold at program start (deactivate if movement towards sound source occurs)
	extern bool ir_startupdate; // get start sample dynamically, by looking for threshold at every head tracker update (deactivate if particle slam)
	extern bool ir_startlatency; // init start sample once at beginning, by correcting hardware latency at program start
	extern float ir_startthreshold; // pressure threshold for first start sample, in percent (written as decimal) of normalized 0..1 gain, recommended is 0.01 
	extern float ir_startmsoffset; // start offset in ms, additional to found first start sample offset at ir_startthreshold
	extern float ir_startdynamicsearchms; // search start dynamical in an area of given ms around peak of signal (faster, but might be unreliable if no direct sound, deactivate with 0.0f)
	extern float ir_energythreshold; // energy threshold per sample of impulse response for signal being detected as active, in percent (written as decimal) of normalized 0..1 gain, recommended is 0.01 
	extern bool ir_sparse_on; // sparsify impulse response peaks
	extern float ir_latency_ms; // latency of audio device resulting in delay in impulse response (can be measured by looking where peak is when microphone is next to loudspeaker, in optimal case peak is at rec_ms / 1000.0f * fs)
	
								// impulse response in frames

	// impulse response in frames and upsampled

	// time differences
	extern bool td_on; // activate time difference calculation
	extern float td_itd_maxms; // maximal interaural time difference in ms
	extern float td_atd_maxms; // maximal aural time difference in ms (timer difference between 2 micorphones at one ear)
	extern float td_std_maxms; // maximal source time difference in ms (time difference between impulse source and one ear)
	
	// level differences
	extern bool ld_on; // activate level difference calculation
	extern bool ld_merge_td; // merge level difference into cross-correlation time difference estimation approach using a window around level difference estimation angle
	extern bool ld_merge_window_cor; // merge with correlation signal or gaussian window
	extern int ld_merge_window_corin; // correlation signal type where level difference is taken from
	extern int ld_merge_window_corout; // correlation signal type where window level difference is merged into
	extern float ld_merge_window_Nang; // length of window (left / right) of estimated angle from level difference in degrees (one side length)
	extern float ld_merge_window_strength; // strength of window (left / right) of estimated angle from level difference in degrees (high -> sharper window)
	extern char ld_merge_window_type[char_N]; // window type around around estimated angle from level difference in degrees
	extern float ld_ild_maxdb; // maximal ild in db

	// correlation
	extern char cor_shift[char_N]; // "no", "middle", "delay": set first correlation sample to middle of signal or delayN sample to capture more sampels in correlation (exclude signal edges in crosscorrelation) 
	extern char cor_type[char_N]; // type of crosscorrelation: "cor": unweighted cross-correlation, "cor-phat": phase transformed generalized cross-correlation, "cor-phatlim": phase transformed generalized cross-correlation weighted only for specified bandwidth
	extern long cor_phatlim_f1; // lower limit of PHAT cross-correlation band containing signal energy
	extern long cor_phatlim_f2;// upper limit of PHAT cross-correlation band containing signal energy
	extern bool cor_normalized; // perform normalized cross-correlation

	// upsampled correlation between 2 microphones between 2 microphones

	// upsampled double cross-correlation between 2 microphones between 2 microphones
	extern bool cor2_on; // activate

	// circle
	extern bool cir_on; // activate incidence energy circle calculation for each frame
	extern bool cir_flip_x_plane; // flip axis of x correlations signal
	extern bool cir_flip_y_plane; // flip axis of y correlations signal
	extern bool cir_flip_z_plane; // flip axis of z correlations signal
	extern int cir_ang_stepsize; // step size between 2 angles of circle in degree
	extern bool cir_diffmax_instead_of_diffmean; // ML likelihood from differential correlation instead of probabilistic mean from differential correlation
	extern bool cir_delta_instead_of_diff; // difference from two correlations instead of differential correlation
	extern bool cir_binarymax_instead_of_y; // azimuth angle from x pair and maximum of y pair, instead of full y pair distribution
	
	// upsampled cross-correlation between 1 microphone at 2 time points
	extern bool corm_on;  // activate

	// sphere
	extern bool sph_on; // activate incidence energy sphere calculation for each frame
	extern bool sph_glob_on; // activate global sphere summing all frame spheres
	extern int sph_limit; // number of (one-sideded) angles around correlation maxium that should be calculated: maxium 90, minimum 0
	extern float sph_radius; // radius of sphere
	extern int sph_ang_stepsize; // step size between 2 angles of sphere in degree
	extern int sph_Ndim; // number of dimensions of sphere
	extern bool sph_orthogonal; // user older older version of sphere calculation, only orthogonal microphones, numerical inaccuracy
	extern int sph_Nmax; // number of maximums of sphere used for maximum and variance calculation

	// features
	extern bool fe_energy_on; // activate energy features
	extern bool fe_tdld_on; // activate time and energy difference features
	extern int fe_NVar; // number of observations included in running variance of the change in head position prediction of each frame
	
	// localization
	extern bool loc_on; // activate reflection localization estimation
	extern float loc_zero_offset; // when azimuth incidence angle of wave is exactly 0.0 apply denoted offset to divisor to remove symetry / zero devision
	extern bool loc_td_absolute; // use absolute time difference between microphones instead of +/- signed time difference

	// kalman filter 1 DOF
	extern bool kal_on; // activate 1 dof kalman filter
	extern bool kal_refresh_rq; // update rq ratio
	extern int kal_Ninitsteps; // pre-initialize kalman gain and covariance using Nsteps convergence (necessary if kal_refresh_rq = false)
	extern float kal_rq; // ratio between noise covariance of observation R vs. noise covariance of kinematics motion model Q (high -> kinematics motion model is considered stronger than observation)
	extern float kal_r[A_Nderivative]; // noise covariance of absolute, velocity and acceleration observation (e.g. one value can be set to 1 and the other relative to it as ratio) //before: R[0][0] = R[0][0]*rabsvel and R[1][1]= 1, ratio between noise covariance of absolute head orientation observation vs. noise covariance of head orientation velocity observation (high -> head orientation velocity observation is considered stronger than absolute head orientation observation)

	// kalman filter 6 dof
	extern bool kal6_on; // activate 6 dof kalman filter
	extern bool kal6_refresh_rq; // update rq ratio
	extern int kal6_Ninitsteps; // pre-initialize kalman gain and covariance using Nsteps convergence (necessary if kal_refresh_rq = false)
	extern float kal6_rq[3];  // ratio between translation, rotation, and mapping noise covariance of observation R vs. noise covariance of kinematics motion model Q (high -> kinematics motion model is considered stronger than observation)
	extern float kal6_r[A_Nderivative]; // noise covariance of absolute, velocity and acceleration observation (e.g. one value can be set to 1 and the other relative to it as ratio) // before inverse:  ratio between noise covariance of absolute head orientation observation vs. noise covariance of head orientation velocity / acceleration observation (high -> head orientation velocity / acceleration observation is considered stronger than absolute head orientation observation)

	// particle filter 6 dof
	extern bool par6_on; // activate 6 dof particle filter
	extern bool par6_pool_on; // activate multithreading for particle filter
	extern bool par6_imu_on; // activate imu as input (otherwise full acoustic tracking)
	extern bool par6_track_first; // track position position relative to first sound arrival (otherwise ignoring first sound arrival position)
	extern long par6_Nobs; // number of landmark observations
	extern long par6_Nobs_skip; // number of skipped first landmark observations
	extern long par6_Npar; // number of motion particles
	extern int par6_Nvarbuffer; // number of elements in variance buffer
	extern bool par6_refresh_rq; // kalman filters: update rq ratio
	extern int par6_Ninitsteps; // kalman filters: pre-initialize kalman gain and covariance using Nsteps convergence (necessary if kal_refresh_rq = false)
	extern bool par6_lanerrormedian; // landmark - observation error metric: median or expectation value
	extern int par6_obs_mode; // observation kalman filters: select reference or velocity features
	extern float par6_obs_rq; // observation kalman filters: how important is model relative to observation: ratio between noise covariance of landmark observation R vs. noise covariance of landmarks kinematics motion model Q (high -> kinematics motion model is considered stronger than observation)
	extern float** par6_obs_r; // observation kalman filters: noise covariance of absolute, velocity and acceleration landmark observation (e.g. one value can be set to 1 and the other relative to it as ratio, high -> less importance of observation)
	extern int par6_par_priors; // type of particle prior
	extern float par6_par_rq; // particle kalman filters: how important is imu vs acoustic prior
	extern float** par6_par_r; // particle kalman filters: how important is direct sound reference vs reflection based differential acoustic prior
	extern float par6_weightpower[A_weight_N]; // observation frame selection weight parameters (how exponentially strongly is a fluctuating observation damped)

	// fast fourier transformation
	extern char fft_library[char_N]; // selected FFT library: "native",  "intelmkl" (20 x speedup compared to native, JUCE Kiss FFT is used when JUCE_DSP_INTEL_MKL is disabled in Projucer (then 2x speedup compared to native))

	// debugging
	extern char deb_path_text[path_N]; // path where debug file is written too as txt file
	extern char deb_path_wav[path_N]; // path where debug file is written too as byte wav file (faster)
	extern char deb_multichannel_folder[path_N]; // folder where multichannel debug files are written too
	extern bool deb_on; // general: should any debug function run (makes program slow)
	extern bool deb_pool_on; // run debugging over multithreading pool (0 latency)
	extern bool deb_disk; // should debug writing to disk run
	extern bool deb_stopimpulse; // should impulse playback being stopped while head tracker calculation runs
	extern bool deb_console; // should console (only windows) be enabled that shows printf stdout written with console_log() 
	extern bool deb_particleslam; // should particle slam write to disk
	extern bool deb_timer; // should time measurements be enabled
	extern bool deb_parmultiobs; // should debuggin of observations be written for every update in new file
	extern int deb_Nmax; // number of structs used for debugging
	extern int deb_par6_probxyzsph; // 0: off, 1: sph, 2: xyz and sph (slow)

	// multithreading pool
	extern bool pool_on; // activate multithreading pool for faster computation (cross-platform Windows, Unix)
	extern int pool_Nthread; // number of threads assigend to pool executed at the same time (recommended: number of cpu cores)


	// initialize parameters
	void init_params();

#ifdef __cplusplus
};
#endif