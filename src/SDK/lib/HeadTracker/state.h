// state.h: Variable program configuration derived from params.h.
// Copyright 2020, Felix Pfreundtner, All rights reserved.

// ignore warnings
#pragma once
#pragma warning( disable : 4115 4459 4028 )


// datatype definitions:
// type irfs[isinv_N][y];
//unsigned (*arr)[m] = malloc(sizeof(unsigned[n][m]));
//free(arr);
//#include <vld.h>
#include "params.h"
#include "NgimuMain.h"
#include "array.h"
#include "measure.h"
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "impulse.h"
#include "debugging.h"
#include "JuceCWrapper.h"
#include "localization.h"
#include "kalman.h"
#include "fft.h"
#include "sphere.h"
#include "particleslam.h"
#include "threadmain.h"
#include "circle.h"
#include "frame.h"
#include "inertial.h"
#include "ViconCWrapper.h"

#ifdef __cplusplus
extern "C" {
#endif
	// definitions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	#define UNUSED(x) (void)(x)


	// "preprocessor" arrays
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// positional coordinates
	extern int A_dim[6]; // routing dim to dof (A_Ndim)
	extern int A_dim_r[6]; // inverse routing dof to dim (A_Ndim)

	// limit
	extern float A_limit[6][2]; // limit of coordinate boundaries (A_Ndim) (A_limit_N)

	// limit maximal change between 2 update steps of algorithm
	extern float A_limitstep[6];

	// variable definitions:
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	

	// general setup
	extern long fsup; // upsampled sampling rate

	// measurement

	// microphones
	extern int * mic_routing; // channel routing between recorded micorphone channels and input signals

	// impulse
	extern long is_N;  // length of impulse in samples
	//extern long is_f;

	// inverse impulse
	extern long isinv_N; // length of inverse impulse in samples

	// beamforming

	// recording buffer
	extern long recbuffer_N; // length of impulse recording buffer in samples

	// rapid impulse response measurement technique
	extern long rapid_n; // current sample of rapid impulse response measurement measurement
	extern long rapid_edge_N; // length of rapid impulse recording edge in samples 

	// recording
	extern long rec_N; // length of impulse recording in samples 
	extern long rec_edge_N; // length of impulse recording edge in samples 
	
	// synchronization
	extern long sync_Ndelay; // sample number where recording starts
	extern long sync_Nplay; // detected impulse playback position to align recording buffer with beginning of playback
	extern long sync_N; // synchronization buffer sample position
	extern long sync_Noffset; // detected synchronization offset
	extern long sync_Noffset_p; // detected past synchronization offset
	extern long sync_Ntrail;  // detected synchronization start (and number of bins in trailing signal part)
	extern long sync_Nlead;  // detected synchronization number of bins in leading signal part
	extern float sync_clocknow; // not yet (+) or too much (-) shifted subsample clock remainder

	// impulse response full
	extern long ir_N; // length of impulse response in samples
	extern long ir_Nstartdynamicsearch; // search start dynamical in an area of given samples around peak of signal (faster)
	extern long ir_Nstartoffset; // offset in samples for impulse response start point
	extern long ir_Nstart; // sample number of first sample of impule response with enough energy (from binaural left and right ear impulse response recording)
	extern long ir_Nlatency; // latency of audio device resulting in delay in impulse response 
	extern long ir_Nstart0; // sample number where impule response frame extraction starts
	extern long ir_Nstartframe; // sample number where frame selection starts;

							 // impulse response frames
	extern long frame_N; // length of frame in samples
	extern long frame_Noverlap; // overlap in samples between frames
	extern long frame_Nup; // length of upsampled frame in samples

	// impulse response in frames
	extern long irf_N; // impulse response length in frame in samples
	//extern long irf_Nf; // impulse response frame length in samples

	// impulse response in frames and upsampled
	extern long irfup_N; // upsampled impulse response length in samples
	//extern long irfup_Nf; // upsampled impulse response frame length in samples

	// time differences
	extern long td_itd_N; // maximal interaural time difference in samples
	extern long td_itd_Nup; // maximal interaural time difference in upsampled samples
	extern long td_atd_N; // maximal aural time difference in samples
	extern long td_atd_Nup; // maximal aural time difference in upsampled samples
	extern long td_std_N; // maximal source time difference in samples
	extern long td_std_Nup; // maximal source time difference in upsampled samples

	// correlation (from fft method, full correlation)
	extern long corf_N; // correlation length in samples

	// upsampled correlation between 2 microphones  (from fft method, full correlation)
	extern long corfup_N; // length in samples of upsampled correlation between 2 microphones
	extern long *corfup_Nlim; // pointer to delay limits in bins of upsampled correlation


	// upsampled double cross-correlation between 2 microphones (from fft method, full correlation)
	extern long corf2up_N; // length in samples of upsampled double cross-correlation between 2 microphones
	extern long *corf2up_Nlim; // pointer to delay limits in bins of upsampled double cross-correlation between 2 microphones

	// upsampled cross-correlation between 1 microphone at 2 time points (from fft method, full correlation)
	extern long corfmup_N; //  length in samples of upsampled double cross-correlation between 1 microphone at 2 time points
	extern long corfmup_Nlim; // pointer to delay limits in bins of upsampled cross-correlation between 1 microphone at 2 time points


	// correlation upsampled  (from time method, only up to Nlim delay correlation)
	extern long corfuplim_N;
	extern long corfuplim_Nlim;

	// level differences
	extern long ld_merge_window_N; // total length of window around (left and right of) estimated angle from level difference in bins

	// circle
	extern long cir_Nframe; // number of generated circles
	extern int cir_Naz; // number of azimuth angle bins
	extern int cir_Nel; // number of elevation angle bins

	// sphere
	extern long sph_Nframe; // number of generated spheres
	extern int sph_Naz; // number of azimuth angle bins
	extern int sph_Nel; // number of elevation angle bins
	extern int sph_Nrd; // number of radius bins
	extern int sph_Ntd_ang; // number of time delay angles
	extern int sph_Ntd_ang_sol; // number of ambiguous solutions for each time delay angle

	// localization
	extern int loc_Ndim; // number of dimensions localization algorithm tracks

	// kalman filter

	// kalman filter 6 dof

	// particle filter 6 dof

	// debugging

	// fast fourier transformation
	extern long fft_Ntmpir; // maximal length of any zeropadded signal used for impulse response FFT transform (needed for temporary fft arrays, next power of 2 to maximal time domain signal length, array with double that size is declared to allow expansion to complex values in frequency domain)
	extern long fft_Ntmpirf; // maximal length of any zeropadded signal used for impulse reponse frame FFT transform (needed for temporary fft arrays, next power of 2 to maximal time domain signal length, array with double that size is declared to allow expansion to complex values in frequency domain)
	extern long fft_NISinverse; // length of zeropadded inserve impulse signal
	
	// temporary arrays
	extern float* tmp_rec_hann; // array with hann window of length rec_edge_N
	extern float* tmp_rapid_hann; // array with hann window of length rapid_edge_N
	extern float* tmp_frame_hann; // array with hann window of length frame_N
	extern float* tmp_ld_kaiser; // array with kaiser window of length ld_window_length_ang
	extern float* tmp_frame_upweight; // array with Nup weighting values for frame linear upsampling
	
	// locks
	extern bool lock_rec; // lock for recording buffer
	extern bool lock_sync; // lock for synchronization variables
	extern bool lock_head; // lock for head variables

	// flags

	// update
	extern long update_nr; // number of update
	extern bool update_first; // initialization flag
	extern long update_Tstart; // time at last update step
	extern float update_Tdiff; // measured time between 2 update steps in seconds
	extern float update_Tdiffmax; // maximal possible measured time between 2 update steps in seconds
	extern float* update_algo;// summarized algorithm state variables

	// Type structs
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// structure for single complex number, real and imaginary parts
	struct cpx { 
		float real; 
		float imag;
	};         /* for complex number */

	// structure for array of complex numbers, real and imaginary parts
	struct cpxs { 
		float* real; 
		float* imag;
	};         /* for complex number */
	
	// struct including a probability or cumulative density function
	struct pdfcdf {
		float value; // value (x)
		float weight; // weight (y, probability)
		int index; // index of value (x bin number)
	};

	// Temporary structs

	// Multithreading pool
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pool with threads
	tpool_t* pool;

	// Fixed signal structs
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// struct including the impulse shape
	struct iss
	{
		float* send;
		float* inverse;
	};
	extern struct iss *is;

	// struct including the rapid impulse response measurement recording buffer stream
	struct recbuffers
	{
		float** x;
	};
	extern struct recbuffers *recbuffer;
	
	// struct including the final recording stream
	struct recs
	{
		float** x;
	};
	extern struct recs *rec;




	// struct including the generated full impulse response
	struct irs
	{
		float** x;

	};
	extern struct irs *ir;


	// struct including the filter signals
	// struct including the fast fourier transformation signals
	struct ffts {
		// fixed
		float* ISinverse;

		// temporary
		float* Xtmp;
		float* Htmp;
		float* Ytmp;
		float* zeros;
		struct cpx *NATIVEtmp;

	};
	extern struct ffts* fftir;
	extern struct ffts* fftirf;

	// struct including frame parameters converting ir to ir frame based
	struct fras {
		// frame_M elements
		bool* m; // frames currently selected
		bool* m_p; // frames before on
		bool* m_new; // frames just got on (subset of m)
		bool* m_on; // frames continuously on since at least 2 updates (other subset of m)
		int m_Non; // number of frames continuously on since at least 2 updates 
		long* m_Npeak; // peak sample number of frame m
		float* m_weight_energy; // relative amount of energy in frame
		float* m_weight_corpeak; // relative incidence direction peak amplitude (estimated via cross-correlation)
		float* m_weight_elevation; // vertical incidence direction (estimated via cross-correlation)
		float* m_weight_early; // relative frame number
		long m_first; // first frame number (direct sound)
		long m_energyfirst; // first energy frame number (direct sound)
		long m_energymax; // maximal energy frame number
		//float* m_sparsity; // relative amount of neigbouring active frames

		// frame_Mon elements
		float* on_weight_energy; // energy in active frames
		int* on_m; // frame number m of active frames

		// temp
		bool* tmp_false; // array with false values

	};
	extern struct fras *fra;


	// struct including the generated cutted impulse response in frames
	struct irfs
	{
		float*** x;
	};
	extern struct irfs *irf;
	extern struct irfs *irf_p;

	// struct including the generated cutted and upsampled impulse response in frames
	struct irfups
	{
		float*** x;
	};
	extern struct irfups *irfup;
	extern struct irfups *irfup_p;

	// struct including the correlation
	struct corfs
	{
		float*** x;
	};
	extern struct corfs *corf;

	// struct including the upsampled correlation in each frame between 2 microphones
	struct corfups
	{
		float*** x;
	};
	extern struct corfups *corfup;
	extern struct corfups *corfup_p;

	// struct including the upsampled double correlation in each frame between 2 microphones
	struct corf2ups
	{
		float*** x;
	};
	extern struct corf2ups *corf2up;

	// struct including the upsampled correlation in each frame between 1 microphone at 2 time points 
	struct corfmups
	{
		float*** x;
	};
	extern struct corfmups *corfmup;


	// struct including the upsampled correlation from FFT
	struct corfuplims
	{
		float*** x;
	};
	extern struct corfuplims *corfuplim;

	// struct including the circle
	struct cirs {
		float*** prob; // probability for each dof and frame and its elements
		float** prob_norm; // norm of probability for each dof and frame
		float*** prob_p; // past probability for each dof and frame and its elements
		float** prob_norm_p; // norm of past probability for each dof and frame
		int* prob_N; // number elements in probability for each dof
		int* prob_map_xyz; // mapping between x,y,z probability elements and (mono mic) correlation
		int** prob_map_az; // mapping between azimuth probability elements and (x pair,y pair) correlation
		int* prob_map_el; // mapping between elevation probability elements and (z pair) correlation
		float* scale; // maximal value of each dof elements
		float*** probdiff; // change in probability for each dof and frame and its elements
		float** probdiff_norm; // norm of change in probability for each dof and frame
		int* probdiff_N; // number elements in probability for each dof
	};
	extern struct cirs *cir;

	// struct including the sphere of sound incidence
	struct sphs {
		float*** x; // incidence energy in each spherical angle at each frame for each frame (default = 1.0f)
		float*** xtmp; // temporary array for incidence energy in each spherical angle at each frame for each frame (default = 0.0f)
		long**** map_tdn_sph; // mapping between 1D angle reslting from time delay and 3D spherical coordinate at each cross-correlation microphone pair
		long** map_tdn_ang; // mapping between angle [-90...90] degree and cross-correlation time delay bin 
		long** w_Ntdn_ang; // the number of cross-correlation time delay bins assigned to each incidence angle set at each cross-correlation microphone pair with certain length and delay limits
		long** w_Ntd_ang_sol; // the number of (ambigious) incidence angles assigned to each each incidence angle set resulting from the time delay at each cross-correlation microphone pair (number varies with orthogonal and free setup as spherical coordinate system doesn't allow to formulate circles with integers around non z-plane, otherwise always 360 / ang_stepsize, orthogonal setup also misses some values because of numerical accuracy)
		long*** w_Nsphn; // the number of occurence of each (ambigious) incidence angle at each cross-correlation microphone pair (number varies with orthogonal and free setup as spherical coordinate system doesn't allow to formulate circles with integers around non z-plane, otherwise always 360 / ang_stepsize, orthogonal setup also misses some values because of numerical accuracy)

	};
	extern struct sphs *sph;

	// struct including the features
	struct fes
	{
		// time differences in each frame from correlation between 2 microphones
		float** td;
		float** td_amp;
		float** td_ang;
		float** td_angdiff;
		float* td_angdiffabsaverage;

		// time differences in each frame from double correlation between 2 microphones
		float** td2;
		float** td2_amp;
		float** td2_ang;
		float** td2_angdiff;
		float* td2_angdiffabsaverage;

		// time differences in each frame from correlation between 1 microphone at 2 time points 
		float** tdm;
		float** tdm_amp;

		// distance in each frame between 1 microphone at 2 time points 
		float** tdm_distdiff;
		float* tdm_distdiffabsaverage;

		// level differences in each frame
		float** ld;
		float** ld_ang;
		float** ld_angdiff;
		float* ld_angdiffabsaverage;

		// localization angle in each frame (1 or 2 solutions)
		float** loc_ang;
		float*** loc_ang2sol;

		// energy
		float** energy;
		float* energy_mean;
		float energy_global;

		// beamforming
		float** beam_gain;

		// aggregated
		float* angdiffabsaverage;
		float* distdiffabsaverage;

		// running variance
		float** sdbuffer_az;
		float** sdbuffer_el;
		float* sd_az;
		float* sd_el;
		
	};
	extern struct fes *fe;
	extern struct fes *fe_p;
	
	// struct including the head position
	struct heads
	{
		float itd_diffang; // change in interaul time difference angle, depreciated
		float ild_diffang; // change in interaul level difference angle, depreciated

		float*** pos; // head position at each reflection frame in A_Ndof dimensions and A_Ndofderivative in degree
		float*** pos_var;  // head position variance at each reflection frame in A_Ndof dimensions and A_Ndofderivative in degree
		float** pos_p; // head position in A_Ndof dimensions at each reflection frame in degree
		float** posipc; // head position in A_Ndof dimensions at each reflection frame in ipc coordinates, depreciated
		float** toa; // time of arrival at each reflection frame in A_toa dimensions
		float md; // microphone difference between two updates in metre

	};
	extern struct heads *head;

	// struct including the imu data
	struct imus
	{
		// position
		float** pos;
		// past position
		float** pos_p;

		// set (constant) timestep between 2 updates
		float T;
		// measured timestep between 2 updates
		float* update_Tdiff;

	};
	extern struct imus* imu;

	// struct including the vicon data
	struct vics
	{
		// position of impulse source
		float** pos_is;
		// past position of impulse source
		float** pos_is_p;

		// position of recording receiver
		float** pos_rec;
		// past position of recording receiver
		float** pos_rec_p;

		// set (constant) timestep between 2 updates
		float T;
		// measured timestep between 2 updates
		float* update_Tdiff;

	};
	extern struct vics* vic;

	// struct including the triangulization - localization state
	struct locs 
	{
		// number of channels
		int Nch;
		// number of dimensions
		int Ndim;
		// number of solutions
		int Nsol;
		// zero offset
		float zero_offset;
		// use absolute time difference between microphones instead of +/- signed time difference
		bool td_absolute;
		// (unknown) position of head (2 solutions)
		float** xyz2sol;
		// (unknown) position of head
		float* xyz;
		// (known) position of microphones: i, j, k, l
		float** xyzi;
		// (known) squared position of microphones: i, j, k, l
		float** xyzi2;
		// (known) difference in position of microphones: x_ji, x_ki, x_jk, x_lk
		float** xyzji;
		// (known) time of arrival difference expressed in metre between microphones: R_ij, R_ik, R_kj, R_kl
		float* Rij;
		// (known) squared time of arrival difference expressed in metre between microphones: R_ij, R_ik, R_kj, R_kl
		float* Rij2;
		// (calculated) time of arrival prediction error between microphones: R_i, R_j, R_k, R_l
		float* error;
		// temporary values
		float A;
		float B;
		float C;
		float D;
		float E;
		float F;
		float G;
		float H;
		float I;
		float J;
		float K;
		float L;
		float M;
		float N;
		float O;
		float tmp;
	};
	extern struct locs *loc;

	// struct including the 1dof kalman filter state
	struct kals 
	{
		// new observation
		float* y; // y_new
		// number of states
		int M;
		// number of observations
		int N;
		// state matrix
		float **A;
		// state matrix jacobian
		float **A_J;
		// state error jacobian
		float **W;
		// observation matrix
		float **C;
		// observation matrix jacobian
		float **C_J;
		// observation error jacobian
		float **V;
		// kalman gain
		float **K;
		// states
		//float *x_new;
		float *x;
		// states apriori
		//float *xap_new;
		float *xap;
		// state input
		float* u;
		// state error
		//float *e_new;
		float *e;
		// initial error covariance estimate
		float P_n;
		// error covariance
		//float **P_new;
		float **P;
		// error covariance
		float **Pap;
		// Q / R ratio
		float R_m;
		float Q_m;
		// Process noise covariance
		float **Q;
		// initial observation noise covariance
		float **R0;
		// observation noise covariance
		float **R;
		// state limit
		float** limit;
		// is limit active
		bool limit_on;
		// update R and Q dependent matrices (full kalman filter, but slower)
		bool refresh_rq;
		// update number
		long Nupdate;

		// temporary variables
		float **divisor;
		float **numerator;
		float **eyearray;
		float* tmp1_1d;
		float** tmp1_2d; 
		float** tmp2_2d;
		float** tmp3_2d; 
		float** tmp4_2d; 

	};
	extern struct kals* kalaz;
	extern struct kals* kalel;

	// struct including the 6 dof kalman filter state
	struct kal6s 
	{
		// new observation
		float* y; // y_new
		// number of states
		int M;
		// number of observations
		int N;
		// state matrix
		float **A;
		// state matrix jacobian
		float **A_J;
		// state error jacobian
		float **W;
		// observation matrix
		float **C;
		// observation matrix jacobian
		float **C_J;
		// observation error jacobian
		float **V;
		// kalman gain
		float **K;
		// states
		//float *x_new;
		float *x;
		// states apriori
		//float *xap_new;
		float *xap;
		// state input
		float* u;
		// state error
		//float *e_new;
		float *e;
		// initial error covariance estimate
		float P_n;
		// error covariance
		//float **P_new;
		float **P;
		// error covariance
		float **Pap;
		// Q / R ratio
		float R_m;
		float Q_m;
		// Process noise covariance
		float **Q;
		// initial observation noise covariance
		float **R0;
		// observation noise covariance
		float **R;
		// state limit
		float** limit;
		// is limit active
		bool limit_on;
		// update R and Q dependent matrices (full kalman filter, but slower)
		bool refresh_rq;
		// update number
		long Nupdate;

		// temporary variables
		float **divisor;
		float **numerator;
		float **eyearray;
		float* tmp1_1d;
		float** tmp1_2d; 
		float** tmp2_2d;
		float** tmp3_2d; 
		float** tmp4_2d; 

	};
	extern struct kal6s* kal6;

	// struct including particle of particle filter
	struct parparticle {
		struct kals* pos; // kalman state transition model estimating this particle position, velocity and acceleration for each dimension
		struct kals** kal; // kalman state transition model estimating each observation position estimated from this particle, velocity and acceleration for each dimension
	};

	// struct including observation of particle filter
	struct parobservation {
		struct kals* kal; // kalman state transition model estimating the observation position, velocity and acceleration for each dimension
		float** x_p; // past position of the landmark state for each dimension and derivative
		float* weight; // a priori weighting factor of the observation for each dimension with range [0...1]
		float** varbuffer; // buffer storing the past Nvarbuffer observations y for each dimension

	};

	// struct including the 6 dof particle filter state
	struct par6s
	{
		// activate imu (or full acoustic tracking)
		bool imu_on;
		// track position relative to first sound arrival
		bool track_first;
		// number of particles
		long Npar;
		// number of observations
		long Nobs;
		// number of skipped first observations
		long Nobs_skip;
		// set (constant) timestep between 2 updates
		float T;
		// update number of head tracker
		long* update_nr;
		// particles

		struct parparticle* par;
		// (acoustic) landmark observations
		struct parobservation* obs;
		// (imu) input
		float** u;
		// maximum likelihood position for each dof and derivative
		float** pos;

		// landmark - observation error metric
		bool lanerrormedian;

		// observation damping constant
		float* weightpower;

		// particle prior type
		int par_priors;

		// most likely frame weights

		// global weight of landmark observations
		float* obs_weight;
		// probability density function for likelihood of each dimension of landmark observations
		float** obs_pdf;
		// cumulative density function for likelihood of each dimension of landmark observations
		float** obs_cdf;
		// probability density function for likelihood of particles of each dimension
		float** par_cdf;
		// cumulative density function for likelihood of particles of each dimension
		float** par_pdf;


		// assignment between observations and landmarks (every value is observation that is assigend to landmark index)
		int* lan;

		// number of elements in variance buffer
		int Nvarbuffer;

		// frame struct
		struct fras* fra;

		// head struct
		struct heads* head;

		// temporary
		// particle array: number
		int* tmp_par_i;
		// particle array: amplitude
		float* tmp_par_amp;
		// particle array: buffer left
		float* tmp_par_amp_buffer_left;
		// particle array: buffer right
		float* tmp_par_amp_buffer_right;
		// observation array: number
		int* tmp_obs_m;
		// observation array: amplitude
		float* tmp_obs_amp;
		// observation array: buffer left
		float* tmp_obs_amp_buffer_left;
		// observation array: buffer right
		float* tmp_obs_amp_buffer_right;

	};

	extern struct par6s *par6;



	// struct including debugging writing information for the disk
	struct debs
	{	
		int N; // number of debugging struct
		bool open; // is debugging struct currently not used
		struct par6s* p; 
		char channel[50]; // char_N
		char frame[50]; // char_N
		char sample[50]; // char_N
		int Nchannels;
		int Nframes;
		int Nsamples;
		int flag;
		
		float*** x3d;
		float** x2d;
		char filepath[512]; // path_N
		char metadata[512]; // path_N
		float sampling_rate;
		
	};
	extern struct debs **deb;


	// variables

	// general

	// impulse response frames



	// init JUCE C Wrapper
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void init_juce_c_wrapper();

	// initialize temporary structs
	
	// Threading
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	tpool_t* init_state_pools();

	// initialize fixed signal structs
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	struct iss* init_state_iss();
	struct recbuffers* init_state_recbuffers();
	struct recs* init_state_recs();
	struct irs* init_state_irs();
	struct ffts* init_state_ffts(long Nmax);
	struct ffts* init_state_pool_ffts(char* fft_type); // activate if multithreading is used, also requires to add fft[m] instead of fft in loops 
	struct fras* init_state_fras();
	struct irfs* init_state_irfs();
	struct irfups* init_state_irfups();
	struct corfs* init_state_corfs();
	struct corfups* init_state_corfups();
	struct corf2ups* init_state_corf2ups();
	struct corfmups* init_state_corfmups();
	struct corfuplims* init_state_corfuplims();
	struct cirs* init_state_cirs();
	struct sphs* init_state_sphs();
	struct fes* init_state_fes();
	struct heads* init_state_heads();
	struct imus* init_state_imus();
	struct vics* init_state_vics();
	struct locs* init_state_locs();
	struct kals* init_state_kals();
	struct kal6s* init_state_kal6s();
	struct par6s* init_state_par6s();
	struct debs** init_state_debs();


	// initialize standard struct pointers (depreciated)
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// standard initialization float pointer array capturing signals for all microphone channels
	float** standard_2d_ptr_channel_float_init(float* x_l, float* x_r, float* x_l2, float* x_r2, float* x_s, float* x_lf, float* x_rf, float* x_lb, float* x_rb, float** x_ptr, int ch_N);

	// standard initialization float pointer array capturing signals for all microphone channels and frames
	float *** standard_3d_ptr_channel_float_init(float ** x_l, float **x_r, float **x_l2, float **x_r2, float **x_s, float **x_lf, float ** x_rf, float **x_lb, float ** x_rb, float ***x_ptr, int ch_N);

	// standard initialization float pointer array capturing crosschannel signals for all microphone cross-channels
	float** standard_2d_ptr_crosschannel_float_init(float* x_lr, float* x_l2r2, float* x_ll2, float* x_rr2, float* x_lr2, float* x_l2r, float* x_ls, float* x_l2s, float* x_rs, float* x_r2s, float* x_lfrf, float* x_lbrb, float** x_ptr, int ch_N);

	// standard initialization float pointer array capturing crosschannel signals for all microphone cross-channels and frames
	float*** standard_3d_ptr_crosschannel_float_init(float** x_lr, float** x_l2r2, float** x_ll2, float** x_rr2, float** x_lr2, float** x_l2r, float** x_ls, float** x_l2s, float** x_rs, float** x_r2s, float** x_lfrf, float** x_lbrb, float*** x_ptr, int ch_N);

	// standard initialization long pointer array capturing signals for all microphone cross-channels
	long** standard_2d_ptr_crosschannel_long_init(long* x_lr, long* x_l2r2, long* x_ll2, long* x_rr2, long* x_lr2, long* x_l2r, long* x_ls, long* x_l2s, long* x_rs, long* x_r2s, long* x_lfrf, long* x_lbrb, long** x_ptr, int ch_N);

	// initialize temporary arrays
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	float* init_tmp_hann(int Nwindow);
	float* init_tmp_kaiser(int Nwindow, float Strenghtwindow);
	float* init_tmp_upweight(int Nup);

	// initialize state at program start
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void init_state();

	// initialize state in each program loop
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void update_state();

	// update impulse inverse array (called if rapid impulse response measurement technique is selected)
	void rapid_update_is(); 

	// update impulse recording array (called if rapid impulse response measurement technique is selected)
	void rapid_update_rec();  

	#ifdef __cplusplus
	};
	#endif