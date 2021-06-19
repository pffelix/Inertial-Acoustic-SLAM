// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "state.h"

// Declarations
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Threading
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
tpool_t* pool;

// signal structs
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct recbuffers *recbuffer;
struct recs *rec;
struct ffts* fftir;
struct ffts* fftirf;
struct iss *is;
struct irs *ir;
struct fras *fra;
struct irfs *irf;
struct irfs *irf_p;
struct irfups *irfup;
struct irfups *irfup_p;
struct corfs *corf;
struct corfups *corfup;
struct corfups *corfup_p;
struct corf2ups *corf2up;
struct corfmups *corfmup;
struct corfuplims *corfuplim;
struct cirs* cir;
struct sphs* sph;
struct fes *fe;
struct fes *fe_p;
struct heads *head;
struct imus* imu;
struct vics* vic;
struct locs* loc;
struct kals* kalaz;
struct kals* kalel;
struct kal6s* kal6;
struct par6s* par6;
struct debs** deb;

// "preprocessor" arrays
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// positional coordinates

// routing
int A_dim[6];
int A_dim_r[6];

// limit
float A_limit[6][2];

// Limit maximal change between 2 update steps of algorithm
float A_limitstep[6];

// variables
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// general

// impulse response frames

// initialization flag
//bool initialized = false;



// Variables

// general setup
long fsup;


// microphones

// impulse
long is_N;
long is_pos;

// inverse impulse
long isinv_N;

// beamforming

// recording buffer
long recbuffer_N;

// rapid impulse response measurement technique
long rapid_n;
long rapid_edge_N;


// recording
long rec_N;
long rec_edge_N;


// synchronization
long sync_Ndelay;
long sync_Nplay;
long sync_N;
long sync_Noffset;
long sync_Noffset_p;
long sync_Ntrail;
long sync_Nlead;
float sync_clocknow;


// impulse response full
long ir_N;
long ir_Nstartdynamicsearch;
long ir_Nstartoffset;
long ir_Nstart;
long ir_Nlatency;
long ir_Nstart0;
long ir_Nstartframe;

// impulse response frames
long frame_N;
long frame_Noverlap;
long frame_Nup;

// impulse response in frames
long irf_N;
//long irf_Nf;

// impulse response in frames and upsampled
long irfup_N;
//long irfup_Nf;

// time differences
long td_itd_N;
long td_itd_Nup;
long td_atd_N;
long td_atd_Nup;
long td_std_N;
long td_std_Nup;

// correlation (from fft method, full signal)
long corf_N;

// upsampled correlation between 2 microphones (from fft method, full correlation)
long corfup_N;
long *corfup_Nlim;

// upsampled double cross-correlation between 2 microphones (from fft method, full correlation)
long corf2up_N;
long *corf2up_Nlim;

// upsampled cross-correlation between 1 microphone at 2 time points (from fft method, full correlation)
long corfmup_N;
long corfmup_Nlim;

// upsampled correlation between 2 microphones (from time method, only up to Nlim delay correlation)
long corfuplim_Nlim;
long corfuplim_N;

// level differences
long ld_merge_window_N;

// circle
long cir_Nframe;
int cir_Naz;
int cir_Nel;

// sphere
long sph_Nframe;
int sph_Naz;
int sph_Nel;
int sph_Nrd;
int sph_Ntd_ang;
int sph_Ntd_ang_sol;

// localization
int loc_Ndim;

// kalman filter

// kalman filter 6 dof

// particle filter 6 dof

// debugging

// temporary struct variables

// fast fourier transformation
long fft_Ntmpir;
long fft_Ntmpirf;
long fft_NISinverse;

// temporary arrays
float* tmp_rec_hann;
float* tmp_rapid_hann;
float* tmp_frame_hann;
float* tmp_ld_kaiser;
float* tmp_frame_upweight;

// locks
bool lock_rec;
bool lock_sync;
bool lock_head;

// flags

// update
long update_nr;
bool update_first;
long update_Tstart;
float update_Tdiff;
float update_Tdiffmax;
float* update_algo;


// intialization function at startup
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void init_state() {
	
	// init "preprocessor" arrays
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// positional coordinates

	// routing
	if (A_Ndof == 6) {
		A_dim[A_x] = 0;
		A_dim[A_y] = 1;
		A_dim[A_z] = 2;
		A_dim[A_az] = 3;
		A_dim[A_el] = 4;
		A_dim[A_rd] = 5;

		A_dim_r[0] = A_x;
		A_dim_r[1] = A_y;
		A_dim_r[2] = A_z;
		A_dim_r[3] = A_az;		
		A_dim_r[4] = A_el;
		A_dim_r[5] = A_rd;

	}
	if (A_Ndof == 2) {
		A_dim[A_az] = 0;
		A_dim[A_el] = 1;

		A_dim_r[0] = A_az;
		A_dim_r[1] = A_el;

	}

	// limit
	A_limit[A_x][A_limit_low] = -INFINITY; // m
	A_limit[A_y][A_limit_low] = -INFINITY; // m
	A_limit[A_z][A_limit_low] = -INFINITY; // m
	A_limit[A_az][A_limit_low] = 0.0f; // degree
	A_limit[A_el][A_limit_low] = 0.0f; // degree
	A_limit[A_rd][A_limit_low] = -INFINITY; // m

	A_limit[A_x][A_limit_high] = INFINITY; // m
	A_limit[A_y][A_limit_high] = INFINITY; // m
	A_limit[A_z][A_limit_high] = INFINITY; // m
	A_limit[A_az][A_limit_high] = A_Naz; // degree
	A_limit[A_el][A_limit_high] = A_Nel; // degree
	A_limit[A_rd][A_limit_high] = INFINITY; // m

	// Limit maximal change between 2 update steps of algorithm
	A_limitstep[A_x] = 1.0f; // m
	A_limitstep[A_y] = 1.0f; // m
	A_limitstep[A_z] = 1.0f; // m
	A_limitstep[A_az] = A_Naz; // degree
	A_limitstep[A_el] = A_Nel; // degree
	A_limitstep[A_rd] = 1.0f; // m


	// init variables
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// general
	fsup = fs * up_N;
	/*
	// interface
	strcpy(interface_path_params, "../../unity/head_tracker/info");
	strcpy(interface_path_rec_l, "../../unity/head_tracker/rec_l");
	strcpy(interface_path_rec_r, "../../unity/head_tracker/rec_r");
	*/


	// impulse
	is_N = msec_to_N(is_ms, fs);
	is_pos = 0; // dynamic

	// inverse impulse
	isinv_N = !strncmp(isinv_type, "kirkeby", 7) ? next_power_of_2(is_N) : is_N;
	
	// microphone
	//mic_routing
	// beamforming

	// recording buffer
	recbuffer_N = msec_to_N(recbuffer_ms, fs);
	
	// rapid impulse response measurement technique
	rapid_n = 0;
	rapid_edge_N = msec_to_N(rapid_edge_ms, fs);


	// recording
	rec_N = msec_to_N(rec_ms, fs);
	rec_edge_N = msec_to_N(rec_edge_ms, fs);



	// synchronization
	sync_Ndelay = 0; // dynamic
	sync_Nplay = 0; // dynamic
	sync_N = 0; // dynamic
	sync_Noffset = 0; // dynamic
	sync_Noffset_p = 0; // dynamic
	sync_Ntrail = 0; // dynamic
	sync_Nlead = rec_N; // dynamic
	sync_clocknow = 0.0f; // dynamic

	// impulse response full
	ir_N = (rec_N + isinv_N - 1 );
	ir_Nstartdynamicsearch = msec_to_N(ir_startdynamicsearchms, fs);
	ir_Nstartoffset = msec_to_N(ir_startmsoffset, fs);
	ir_Nstart = is_N + ir_Nstartoffset; // dynamic
	ir_Nlatency = msec_to_N(ir_latency_ms, fs);
 	ir_Nstart0 = ir_startlatency ? ir_Nstart + ir_Nlatency : ir_Nstart;
	ir_Nstartframe = ir_Nstart0; // dynamic

	// impulse response frames
	frame_N = msec_to_N(frame_ms, fs);
	frame_Noverlap = (long)(frame_overlap * (float)frame_N);
	frame_Nup = up_N *frame_N - (up_N - 1);

	// impulse response in frames
	irf_N = frame_N;
	//irf_Nf = frame_N;

	// impulse response in frames and upsampled
	irfup_N = frame_Nup;
	//irfup_Nf = up_N * irf_Nf - (up_N - 1);


	// time differences
	td_itd_N = (long)(td_itd_maxms / 1000.0f * fs);
	td_itd_Nup = (long)(td_itd_maxms / 1000.0f * fsup);
	td_atd_N = (long)(td_atd_maxms / 1000.0f * fs);
	td_atd_Nup = (long)(td_atd_maxms / 1000.0f * fsup);
	td_std_N = (long)(td_std_maxms / 1000.0f * fs);
	td_std_Nup = (long)(td_std_maxms / 1000.0f * fsup);


	// correlation  (from fft method, full correlation)
	corf_N = 2 * frame_N - 1; // corf_N = 2 * frame_N - 1;

	// upsampled correlation between 2 microphones (from fft method, full correlation)
	corfup_N = up_fft ? (2 * frame_N - 1) * up_N : 2 * frame_Nup - 1; // up_fft ? (2 * corf_N) * up_N : 2 * frame_Nup - 1
	corfup_Nlim = ar_zerosl(ch_Ncrossbeam);
	corfup_Nlim[A_lr] = td_itd_Nup;
    corfup_Nlim[A_ll2] = td_atd_Nup;
    corfup_Nlim[A_rr2] = td_atd_Nup;
	corfup_Nlim[A_ff2] = td_atd_Nup;
    //corfup_Nlim[A_l2r2] = td_itd_Nup;
	//corfup_Nlim[A_lr2] = td_itd_Nup;
	//corfup_Nlim[A_l2r] = td_itd_Nup;

	// upsampled double cross-correlation between 2 microphones (from fft method, full correlation)
	corf2up_N = 2 * corfup_N - 1;
	corf2up_Nlim = ar_zerosl(ch_Ncrossbeam);
    corf2up_Nlim[A_lr] = td_itd_Nup;
    corf2up_Nlim[A_ll2] = td_atd_Nup;
    corf2up_Nlim[A_rr2] = td_atd_Nup;
	corf2up_Nlim[A_ff2] = td_atd_Nup;
    //corf2up_Nlim[A_l2r2] = td_itd_Nup;
	//corf2up_Nlim[A_lr2] = td_itd_Nup;
	//corf2up_Nlim[A_l2r] = td_itd_Nup;
	
   // upsampled cross-correlation between 1 microphone at 2 time points (from fft method, full correlation)
	corfmup_N = up_fft ? (2 * frame_N - 1) * up_N : 2 * frame_Nup - 1; // 2 * frame_Nup - 1
	corfmup_Nlim = (corfmup_N - 1) / 2; // frame_Nup - 1
	A_limitstep[A_x] = N_to_m(corfmup_Nlim, fsup, sos);
	A_limitstep[A_y] = N_to_m(corfmup_Nlim, fsup, sos);
	A_limitstep[A_z] = N_to_m(corfmup_Nlim, fsup, sos);

	// upsampled correlation between 2 microphones (from time method, only up to Nlim delay correlation)
	corfuplim_N = 2 * td_itd_Nup + 1;
	corfuplim_Nlim = td_itd_Nup;

	// level differences
	ang_to_tdn(ld_merge_window_Nang, corfup_Nlim[ld_merge_window_corout], &ld_merge_window_N);

	// circle
	cir_Nframe = frame_M;
	cir_Naz = 360 / cir_ang_stepsize;
	cir_Nel = 180 / cir_ang_stepsize + 1;

	// sphere
	sph_Nframe = frame_M  + sph_glob_on * 1;
	sph_Naz = 360 / sph_ang_stepsize;
	sph_Nel = 180 / sph_ang_stepsize + 1;
	sph_Nrd = 1;
	sph_Ntd_ang = 2 * 90 + 1;
	sph_Ntd_ang_sol = 360;

	// localization
	loc_Ndim = 3;
	
	// kalman filter

	// kalman filter 6 dof

	// particle filter 6 dof

	// temporary struct variables


	// fast fourier transformation
	fft_Ntmpir = next_power_of_2(ir_N); // might also be now 4 * td_atd_Nup (but smaller than corf2up_N)
	fft_Ntmpirf = next_power_of_2((4 * td_atd_Nup > corf2up_N) ? 4 * td_atd_Nup : corf2up_N); // might also be now 4 * td_atd_Nup (but smaller than corf2up_N)
	fft_NISinverse = next_power_of_2(ir_N);

	// debugging


	// temporary arrays
	tmp_rec_hann = init_tmp_hann(2 * rec_edge_N);
	tmp_rapid_hann = init_tmp_hann(2 * rapid_edge_N);
	tmp_frame_hann = init_tmp_hann(frame_N);
	tmp_ld_kaiser = init_tmp_kaiser(ld_merge_window_N, ld_merge_window_strength);
	tmp_frame_upweight = init_tmp_upweight(up_N);


	// locks
	lock_rec = false;
	lock_sync = false;
	lock_head = false;

	// init JUCE C Wrapper
	init_juce_c_wrapper();

	
	// init temporary structs

	// init multithreading pool
	pool = init_state_pools();

	// init fixed signal structs
	recbuffer = init_state_recbuffers();
	rec = init_state_recs();
	is = init_state_iss();
    ir = init_state_irs();
	fftir = init_state_pool_ffts("ir");
	fftirf = init_state_pool_ffts("irf");
	fra = init_state_fras();
	irf = init_state_irfs();
	irf_p = init_state_irfs();
	irfup = init_state_irfups();
	irfup_p = init_state_irfups();
	corf = init_state_corfs();
    corfup = init_state_corfups();
    corfup_p = init_state_corfups();
    corf2up = init_state_corf2ups();
    corfmup = init_state_corfmups();
	corfuplim = init_state_corfuplims();
	cir = init_state_cirs();
	sph = init_state_sphs();
	fe = init_state_fes();
    fe_p = init_state_fes();
	head = init_state_heads();
	loc = init_state_locs();
	imu = init_state_imus();
	vic = init_state_vics();
	kalaz = init_state_kals();
	kalel = init_state_kals();
	kal6 = init_state_kal6s();
	par6 = init_state_par6s();
	deb = init_state_debs();

    // general

    // impulse response frames

	// flags

	// update
	update_nr = 0; // dynamic
	update_first = true;
	update_Tstart = 0; // dynamic
	update_Tdiff = rapid_on ? 0.0f : rec_ms / 1000.0f; // dynamic
	update_Tdiffmax = rec_ms / 1000.0f + 0.2f; // 0.2f maximal computation time
	update_algo = ar_zeros(A_Nalgo);

};

// Functions

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// init JUCE C Wrapper
void init_juce_c_wrapper() {
	long fft_Nmax =  fft_Ntmpir > fft_Ntmpirf ? fft_Ntmpir : fft_Ntmpirf;
	int channel_Nmax = ch_Nrecbeam > ch_Ncrossbeam ? ch_Nrecbeam : ch_Ncrossbeam;
	if (!strcmp(fft_library, "intelmkl")) {
		
		// Find maximal possible zeropadded length of any fast convolution
		//long y_N = rec_N + isinv_N - 1; 

		// Find length with smallest power of 2 for faster computation of FFT
		int fftorder_Nmax = (int)ceil(log(fft_Nmax)/log(2)); 

		// init JUCE C Wrapper
		JuceCWrapperInit(fftorder_Nmax, fft_Nmax, channel_Nmax, (double)fsup);
	}
	else {
		JuceCWrapperInit(0, fft_Nmax, channel_Nmax, (double)fsup);
	}

};


// init multithreading pool
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
tpool_t* init_state_pools() {
	tpool_t* y;
	if (pool_on || deb_pool_on) {
		y = thread_init(pool_Nthread);
		if (y) {
			return y;
		}
		else {
			return NULL;
		}
	}
	return NULL;
}

// init signal structs
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct iss* init_state_iss() {
	// assign memory
	struct iss* y = (struct iss *)malloc(sizeof(struct iss));
	if (y) {
		y->send = ar_zeros(is_N);
		y->inverse = ar_zeros(isinv_N);
	}
	double * y_buffer = (double*)malloc(is_N * sizeof(double));
	//double * y_buffer_inverse = (double*)malloc(isinv_N * sizeof(double));

	// assign value

	// generate impulse and impulse impulse
	generate_impulse_double(y_buffer, is_N, fs, is_f1, is_f2, is_type, is_amp);
	// fade in and fade out impulse and inverse impulse
	if (is_fade_inverse) {
		fade_in_double(y_buffer, is_N, is_fadein_window, is_fadein_percent, false);
		fade_out_double(y_buffer, is_N, is_fadeout_window, is_fadeout_percent, false);
	}
	//write_txt_debug_double(y_buffer, is_N);

	//if (is_unfade) {
		//fade_in_double(y_buffer_inverse, isinv_N, is_fadein_window, is_fadein_percent, true);
		//fade_out_double(y_buffer_inverse, isinv_N, is_fadeout_window, is_fadeout_percent, true);
	//}

	// calculate inverse impulse
	if (y && y_buffer) { // && y_buffer_inverse
		for (long n = 0; n < is_N; n++) {
			y->send[n] = (float)y_buffer[n];
			//y->inverse[n] = (float)y_buffer[n];
		}
		//flip(y->inverse, isinv_N);
		calculate_inverse_impulse(y->send, is_N, y->inverse, isinv_N, is_type, isinv_type, fs, is_f1, is_f2, fftir, fft_library);
	}


	// fade in and fade out impulse
	if (!is_fade_inverse) {
		fade_in_double(y_buffer, is_N, is_fadein_window, is_fadein_percent, false);
		fade_out_double(y_buffer, is_N, is_fadeout_window, is_fadeout_percent, false);
		if (y && y_buffer) { // && y_buffer_inverse
			for (long n = 0; n < is_N; n++) {
				y->send[n] = (float)y_buffer[n];
				//y->inverse[n] = (float)y_buffer[n];
			}
		}
	}
	write_txt_debug(y->send,  is_N, 1);
	write_txt_debug(y->inverse,  is_N, 1);

	// free memory
	free(y_buffer);
	//free(y_buffer_inverse);

	// return
	return y;
};

struct recbuffers* init_state_recbuffers() {

	// assign memory
    struct recbuffers* y = (struct recbuffers *)malloc(sizeof(struct recbuffers));
	if (y) {
		y->x = ar_zeros2d(ch_Nrecbeam, recbuffer_N);
	}

	// return
	return y;

};


struct recs* init_state_recs() {

	// assign memory
    struct recs* y = (struct recs *)malloc(sizeof(struct recs));
	if (y) {
		y->x = ar_zeros2d(ch_Nrecbeam, rec_N);
	}

	// return
	return y;

};



struct irs* init_state_irs() {
    // assign memory
    struct irs* y = (struct irs *)malloc(sizeof(struct irs));
	if (y) {
		y->x = ar_zeros2d(ch_Nrecbeam, ir_N);
	}

	// return
	return y;
};


struct ffts* init_state_ffts(long Nmax) {
	// assign memory
    struct ffts* y = (struct ffts *)malloc(sizeof(struct ffts));
	// Temporary values
	// assign value
	if (y) {
		y->Xtmp = ar_zeros(Nmax * 2);
		y->Htmp = ar_zeros(Nmax * 2);
		y->Ytmp = ar_zeros(Nmax * 2);
		y->zeros = ar_zeros(Nmax);
		y->NATIVEtmp = (struct cpx*)malloc((Nmax * sizeof(struct cpx)));
	}

	// Fixed values
	// assign value
	if (y) {
		// ISinverse
		y->ISinverse = ar_zeros(fft_NISinverse * 2);
		if (y->ISinverse) {
			// copy signal into frequency domain array
  			memcpy(y->ISinverse, is->inverse, isinv_N * sizeof(float));
			// zeropad
			memcpy(y->ISinverse + isinv_N, y->zeros, (fft_NISinverse - isinv_N) * sizeof(float));
			// perform FFT
			fast_fft(y->ISinverse, fft_NISinverse, y, 1, fft_library);
		}
	}

	// return
	return y;
};


struct ffts* init_state_pool_ffts(char* fft_type) { // could be more memory efficient implemented by having two separate varying sized fft structs (ir convolution [Mchannel], cross-correlation [Mframes])
	// assign memory

	// number of pools used for parallel fft computation
	int fft_Ntmp_pool = 0;
	long Nmax = 0;
	if(!strcmp(fft_type, "ir")){
		Nmax = fft_Ntmpir;
		fft_Ntmp_pool = ch_Nrecbeam;
	}
	if(!strcmp(fft_type, "irf")){
		Nmax = fft_Ntmpirf;
		fft_Ntmp_pool = frame_M > ch_Ncrossbeam ? frame_M : ch_Ncrossbeam;
	}

	// arrays in single pool used for fft computation
    struct ffts* y = (struct ffts *)malloc(fft_Ntmp_pool * sizeof(struct ffts));
	if (y) {
		for (int i = 0; i < fft_Ntmp_pool; i++) {
			// Assign temporary, rewritten, arrays
			y[i].Xtmp = ar_zeros(Nmax * 2);
			y[i].Htmp = ar_zeros(Nmax * 2);
			y[i].Ytmp = ar_zeros(Nmax * 2);
			y[i].zeros = ar_zeros(Nmax);
			if (!strcmp(fft_library, "native")) {
				y[i].NATIVEtmp = (struct cpx*)malloc((Nmax * sizeof(struct cpx)));
			}
			// Assign fixed arrays
			if(!strcmp(fft_type, "ir")){
				y[i].ISinverse = ar_zeros(fft_NISinverse * 2);
				if (y[i].ISinverse) {
					// copy signal into frequency domain array
					memcpy(y[i].ISinverse, is->inverse, isinv_N * sizeof(float));
						// zeropad
						memcpy(y[i].ISinverse + isinv_N, y[i].zeros, (fft_NISinverse - isinv_N) * sizeof(float));
						// perform FFT
						fast_fft(y[i].ISinverse, fft_NISinverse, y, 1, fft_library);
				}
			}
		}
	}

	// return
	return y;
};

struct fras* init_state_fras() {

	// assign memory
    struct fras* y = (struct fras *)malloc(sizeof(struct fras));
	if (y) {
		y->m = ar_declareb(frame_M);
		y->m_p = ar_valueb(frame_M, false);
		y->m_new = ar_declareb(frame_M);
		y->m_on = ar_declareb(frame_M);
		y->m_Non = 0;
		y->m_Npeak = ar_declarel(frame_M);
		y->m_weight_energy = ar_zeros(frame_M);
		y->m_weight_corpeak = ar_zeros(frame_M);
		y->m_weight_elevation = ar_zeros(frame_M);
		y->m_weight_early = ar_zeros(frame_M);
		y->m_energyfirst = 0;
		y->m_energymax = 0;
		//y->m_sparsity = ar_value(frame_M, 0.0f);

		y->on_weight_energy = ar_declare(frame_Mon);
		y->on_m = ar_declarei(frame_Mon);

		y->tmp_false = ar_valueb(frame_M, false);

	}

	// return
	return y;

}

struct irfs* init_state_irfs() {
	// assign memory
    struct irfs* y = (struct irfs *)malloc(sizeof(struct irfs));
	if (y) {
		y->x = ar_zeros3d(ch_Nrecbeam, frame_M, irf_N);
	}

	// return
	return y;
};

struct irfups* init_state_irfups() {
	// assign memory
    struct irfups* y = (struct irfups *)malloc(sizeof(struct irfups));
	if (y) {
		y->x = ar_zeros3d(ch_Nrecbeam, frame_M, irfup_N);
	}

	// return
	return y;
};

struct corfs* init_state_corfs() {
	// assign memory
    struct corfs* y = (struct corfs *)malloc(sizeof(struct corfs));
	if (y) {
		y->x = ar_zeros3d(ch_Ncrossbeam, frame_M, corf_N);
	}

	// return
	return y;
};

struct corfups* init_state_corfups() {
	// assign memory
    struct corfups* y = (struct corfups *)malloc(sizeof(struct corfups));
	if (y) {
		y->x = ar_zeros3d(ch_Ncrossbeam, frame_M, corfup_N);
	}

	// return
	return y;
};

struct corf2ups* init_state_corf2ups() {
	// assign memory
    struct corf2ups* y = (struct corf2ups *)malloc(sizeof(struct corf2ups));
	if (y) {
		y->x = ar_zeros3d(ch_Ncrossbeam, frame_M, corf2up_N);
	}

	// return
	return y;
}

struct corfmups* init_state_corfmups() {
	// assign memory
    struct corfmups* y = (struct corfmups *)malloc(sizeof(struct corfmups));
	if (y) {
		y->x = ar_zeros3d(ch_Nrecbeam, frame_M, corfmup_N);
	}

	// return
	return y;
}

struct corfuplims* init_state_corfuplims() {
	// assign memory
	struct corfuplims* y = (struct corfuplims *)malloc(sizeof(struct corfuplims));
	if (y) {
		y->x = ar_zeros3d(ch_Ncrossbeam, frame_M, corfuplim_N);
	}
	
	// return
	return y;
};

struct cirs* init_state_cirs() {
	// assign memory
	struct cirs* y = (struct cirs *)malloc(sizeof(struct cirs));
	if (y) {
		y->prob = (float***)malloc(A_Ndof * sizeof(float**));
		y->prob_norm = ar_value2d(A_Ndof, cir_Nframe, 1.0f);
		y->prob_p = (float***)malloc(A_Ndof * sizeof(float**));
		y->prob_norm_p = ar_value2d(A_Ndof, cir_Nframe, 1.0f);
		y->prob_N = ar_declarei(A_Ndof);
		y->probdiff = (float***)malloc(A_Ndof * sizeof(float**));
		y->probdiff_N = ar_declarei(A_Ndof);
		y->probdiff_norm = ar_value2d(A_Ndof, cir_Nframe, 1.0f);
		y->scale = ar_declare(A_Ndof);

		// init absolute arrays for each dimension
		if (y->prob && y->prob_N) {
			for (int d = 0; d < A_Ndof; d++) {
				// select dof and assign dimension
				switch (A_dim_r[d]) {
					case A_x: y->prob_N[d] = corfmup_Nlim; break;
					case A_y: y->prob_N[d] =  y->prob_N[A_x]; break;
					case A_z: y->prob_N[d] =  y->prob_N[A_x]; break;
					case A_az: y->prob_map_az = init_cir_az_mapping(corfup_N, td_atd_Nup, &y->prob_N[d], cir_flip_x_plane, cir_flip_y_plane); break;	//4 * td_atd_Nup	
					case A_el: y->prob_map_el = init_cir_linear_mapping(corfup_N, td_atd_Nup, &y->prob_N[d], cir_flip_z_plane); break; //2 * td_atd_Nup + 1
					case A_rd: y->prob_N[d] = 1; break;
				}
				y->prob[d] = ar_zeros2d(cir_Nframe, y->prob_N[d]);
				y->prob_p[d] = ar_zeros2d(cir_Nframe, y->prob_N[d]);
			}
		}
		// init differential arrays for each dimension
		if (y->prob && y->probdiff && y->prob_N && y->probdiff_N) {
			for (int d = 0; d < A_Ndof; d++) {
				// select dof and assign dimension
				switch (A_dim_r[d]) {
					case A_x: y->prob_map_xyz = init_cir_linear_mapping(corfmup_N, corfmup_Nlim, &y->probdiff_N[d], false); break;
					case A_y: y->probdiff_N[d] = y->probdiff_N[A_x]; break;
					case A_z: y->probdiff_N[d] = y->probdiff_N[A_x]; break;
					case A_az: y->probdiff_N[d] = 2 * y->prob_N[d] - 1; break; // resulting from cross-correlation of y->prob with y->prob_p
					case A_el: y->probdiff_N[d] =  2 * y->prob_N[d] - 1; break; // resulting from cross-correlation of y->prob with y->prob_p
					case A_rd: y->probdiff_N[d] = 1; break;
				}
				y->probdiff[d] = ar_zeros2d(cir_Nframe, y->probdiff_N[d]);
			}
		}

		// init scale
		if (y->scale && y->prob_N) {
			for (int d = 0; d < A_Ndof; d++) {
				// select dof and assign dimension
				switch (A_dim_r[d]) {
				case A_x: y->scale[d] = (float)y->prob_N[d] / (float)fsup * sos; break;
				case A_y: y->scale[d] = y->scale[A_x]; break;
				case A_z: y->scale[d] = y->scale[A_x]; break;
				case A_az: y->scale[d] = (float)A_Naz; break;
				case A_el: y->scale[d] = (float)A_Nel ; break;
				case A_rd: ; break;
				}
				y->probdiff[d] = ar_zeros2d(cir_Nframe, y->probdiff_N[d]);
			}
		}
	}

	// return
	return y;
}

struct sphs* init_state_sphs() {
	// assign memory
    struct sphs* y = (struct sphs *)malloc(sizeof(struct sphs));
	if (y) {
		// assign memory
		y->x = ar_value3d(sph_Nframe, sph_Naz, sph_Nel, 1.0f);
		y->xtmp = ar_zeros3d(sph_Nframe, sph_Naz, sph_Nel); // sph_Nframe is needed for multithreading pool
		y->map_tdn_sph = ar_zeros4dl(ch_Ncrossbeam, sph_Ntd_ang, sph_Ntd_ang_sol, (long)sph_Ndim); // every circle cutting through a sphere has maximal 360 / sph_ang_stepsize values 
		y->map_tdn_ang = ar_zeros2dl(ch_Ncrossbeam, sph_Ntd_ang);
		y->w_Ntdn_ang = ar_zeros2dl(ch_Ncrossbeam, sph_Ntd_ang);
		y->w_Ntd_ang_sol = ar_zeros2dl(ch_Ncrossbeam, sph_Ntd_ang);
		y->w_Nsphn = ar_zeros3dl(ch_Ncrossbeam, sph_Naz, sph_Nel);

		// assign value
		long corfup_Nlim_tmp;
		for(int ch = 0; ch < ch_Ncrossbeam; ch++){
			
			// get mapping between correlation bin and spherical angle for each cross-correlation setup (not perfectly efficient if some setups have same microphone normal)
			if (sph_orthogonal) {
				map_td_to_sph_orthogonal(mic_loc[R_cross[ch][0]], mic_norm[R_cross[ch][0]], mic_loc[R_cross[ch][1]], mic_norm[R_cross[ch][1]], sph_radius, sph_ang_stepsize, sph_Naz, sph_Nel, y->map_tdn_sph[ch],  y->w_Ntd_ang_sol[ch], y->w_Nsphn[ch]);
			}
			else {
				map_td_to_sph(mic_loc[R_cross[ch][0]], mic_norm[R_cross[ch][0]], mic_loc[R_cross[ch][1]], mic_norm[R_cross[ch][1]], sph_radius, sph_ang_stepsize, sph_Naz, sph_Nel, y->map_tdn_sph[ch],  y->w_Ntd_ang_sol[ch], y->w_Nsphn[ch]);
			}

			// if left/right correlation is calculated from interaural level difference, adapt corfup_Nlim if it exceeds the one sided correlation length
			if (ld_merge_td && ch == A_lr) {
				corfup_Nlim_tmp = (corfup_Nlim[A_lr] > (corfup_N - 1) / 2) ? (corfup_N - 1) / 2 : corfup_Nlim[A_lr];
			}
			else {
				corfup_Nlim_tmp = corfup_Nlim[ch];
			}

			// get mapping between time delay bin and number of time delay bins mapped to each angle
			get_map_tdn_ang(corfup_N, fsup, corfup_Nlim_tmp, sph_Ntd_ang, y->map_tdn_ang[ch], y->w_Ntdn_ang[ch], sph_Ntd_ang, false);
		}
	}

	// return
	return y;
};

struct fes* init_state_fes() {
	// assign memory
    struct fes * y = (struct fes *)malloc(sizeof(struct fes));
	if (y) {
		// time differences in each frame from correlation between 2 microphones
		y->td = ar_zeros2d(ch_Ncrossbeam, frame_M);
		y->td_amp = ar_zeros2d(ch_Ncrossbeam, frame_M);
		y->td_ang = ar_zeros2d(ch_Ncrossbeam, frame_M);
		y->td_angdiff = ar_zeros2d(ch_Ncrossbeam, frame_M);
		y->td_angdiffabsaverage = ar_zeros(ch_Ncrossbeam);

		// time differences in each frame from double correlation between 2 microphones
		y->td2 = ar_zeros2d(ch_Ncrossbeam, frame_M);
		y->td2_amp =  ar_zeros2d(ch_Ncrossbeam, frame_M);
		y->td2_ang = ar_zeros2d(ch_Ncrossbeam, frame_M);
		y->td2_angdiff = ar_zeros2d(ch_Ncrossbeam, frame_M);
		y->td2_angdiffabsaverage = ar_zeros(ch_Ncrossbeam);

		// time differences in each frame from correlation between 1 microphone at 2 time points 
		y->tdm = ar_zeros2d(ch_Nrec, frame_M);
		y->tdm_amp = ar_zeros2d(ch_Nrec, frame_M);

		// distance in each frame between 1 microphone at 2 time points 
		y->tdm_distdiff = ar_zeros2d(ch_Nrec, frame_M);
		y->tdm_distdiffabsaverage = ar_zeros(ch_Nrec);

		// level differences
		y->ld = ar_zeros2d(ch_Ncrossbeam, frame_M);
		y->ld_ang = ar_zeros2d(ch_Ncrossbeam, frame_M);
		y->ld_angdiff = ar_zeros2d(ch_Ncrossbeam, frame_M);
		y->ld_angdiffabsaverage = ar_zeros(ch_Ncrossbeam);

		// localization angle in each frame (1 or 2 solutions)
		y->loc_ang = ar_zeros2d(frame_M, loc_Ndim);
		y->loc_ang2sol = ar_zeros3d(2, frame_M, loc_Ndim);

		// energy
		y->energy = ar_zeros2d(ch_Nrecbeam, frame_M);
		y->energy_mean = ar_zeros(ch_Nrecbeam);
		y->energy_global = 0.0f;

		// beamforming
		y->beam_gain = ar_zeros2d(beam_Nch, frame_M);

		// aggregated
		y->angdiffabsaverage = ar_zeros(ch_Ncrossbeam);
		y->distdiffabsaverage = ar_zeros(ch_Ncrossbeam);	

		// running variance
		y->sdbuffer_az = ar_zeros2d(frame_M, fe_NVar + 3);  // + 3 as 2 sums memorized and last value stores last buffer position (in float N)
		y->sdbuffer_el = ar_zeros2d(frame_M, fe_NVar + 3);  // + 3 as 2 sums memorized and last value stores last buffer position (in float N)
		y->sd_az = ar_zeros(frame_M);
		y->sd_el = ar_zeros(frame_M);

	}

	// return
	return y;
};

struct heads* init_state_heads() {

	// assign memory
    struct heads* y = (struct heads *)malloc(sizeof(struct heads));
	// assign value
	if (y) {
		y->itd_diffang= 0.0f;
		y->ild_diffang = 0.0f;

		y->pos = ar_zeros3d(frame_M, A_Ndof, A_Ndofderivative);
		y->pos_var = ar_zeros3d(frame_M, A_Ndof, A_Ndofderivative);
		y->pos_p = ar_zeros2d(frame_M, A_Ndof);
		y->posipc = ar_zeros2d(frame_M, A_Ndof);
		y->toa = ar_zeros2d(frame_M, A_Ntoa);
		for (int m = 0; m < frame_M; m++) {
			y->pos[m][A_dim[A_az]][0] = 90.0f;
			y->pos_p[m][A_dim[A_az]] = 90.0f;
		}
	}

	// return
	return y;
};



struct locs* init_state_locs() {
    // assign memory
    struct locs* y = (struct locs *)malloc(sizeof(struct locs));
	if (y) {
		init_localization(y, mic_loc[A_l][A_x], mic_loc[A_r][A_x], mic_loc[A_l2][A_x], mic_loc[A_r2][A_x], mic_loc[A_l][A_y], mic_loc[A_r][A_y], mic_loc[A_l2][A_y], mic_loc[A_r2][A_y], mic_loc[A_l][A_z], mic_loc[A_r][A_z], mic_loc[A_l2][A_z], mic_loc[A_r2][A_z], loc_zero_offset, loc_td_absolute);
	}

	// return
	return y;

}

struct imus* init_state_imus() {
    // assign memory
    struct imus* y = (struct imus *)malloc(sizeof(struct imus));
	if (y) {
		y->pos = ar_zeros2d(A_Ndof, A_Ndofderivative);
		y->pos_p = ar_zeros2d(A_Ndof, A_Ndofderivative);
		y->T = T;
		y->update_Tdiff = &update_Tdiff;
	}

	// return
	return y;

}

struct vics* init_state_vics() {
    // assign memory
    struct vics* y = (struct vics *)malloc(sizeof(struct vics));
	if (y) {
		y->pos_is = ar_zeros2d(A_Ndof, A_Ndofderivative);
		y->pos_is_p = ar_zeros2d(A_Ndof, A_Ndofderivative);
		y->pos_rec = ar_zeros2d(A_Ndof, A_Ndofderivative);
		y->pos_rec_p = ar_zeros2d(A_Ndof, A_Ndofderivative);
		y->T = T;
		y->update_Tdiff = NULL;
	}

	// return
	return y;

}
struct kals* init_state_kals() {
    // assign memory
    struct kals* y = (struct kals *)malloc(sizeof(struct kals));
	if (y){
		init_kalman_1dof(y, kal_rq, kal_r, T, NULL, kal_refresh_rq, kal_Ninitsteps);
	}

	// return
	return y;
};

struct kal6s* init_state_kal6s() {
    // assign memory
    struct kal6s* y = (struct kal6s *)malloc(sizeof(struct kal6s));
	if (y){
		init_kalman_6dof(y, kal6_rq, kal6_r, T, frame_M, NULL, kal6_refresh_rq, kal6_Ninitsteps);
	}

	// return
	return y;
};


struct par6s* init_state_par6s() {
    // assign memory
    struct par6s* y = (struct par6s *)malloc(sizeof(struct par6s));
	if (y){
		init_particleslam_6dof(y, par6_imu_on, par6_track_first, par6_Npar, par6_Nobs, par6_Nobs_skip, par6_Nvarbuffer, NULL, par6_obs_rq, par6_obs_r, par6_par_priors, par6_par_rq, par6_par_r, T, par6_refresh_rq, par6_Ninitsteps, par6_weightpower, par6_lanerrormedian, fra, head, &update_nr);
	}

	// return
	return y;
};



struct debs** init_state_debs() {
	// assign memory
    struct debs** y = (struct debs **)malloc(pool_Nthread * sizeof(struct debs*));
	if (y) {
		for (int n = 0; n < deb_Nmax; n++) {
			y[n] = (struct debs *)malloc(sizeof(struct debs));
			y[n]->N = n;
			y[n]->open = true;
		}
	}
	// return
	return y;
}

// standard inits (depreciated)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////



float ** standard_2d_ptr_channel_float_init(float * x_l, float *x_r, float *x_l2, float * x_r2, float *x_s, float *x_lf, float *x_rf, float *x_lb, float *x_rb, float **x_ptr, int ch_N) {
	x_ptr = (float **)malloc(ch_N * sizeof(float *));
	if (x_ptr) {
		x_ptr[0] = x_l;
		if (ch_N > 1){
			x_ptr[1] = x_r;
		}
		if (ch_N > 2){
			x_ptr[2] = x_l2;
		}
		if (ch_N > 3){
			x_ptr[3] = x_r2;
		}
		if (ch_N > 4){
			x_ptr[4] = x_s;
		}
		if (ch_N > 5){
			//x_ptr[5] = x_s2;
			x_ptr[5] = x_lf;
		}
		if (ch_N > 6){
			x_ptr[6] = x_rf;
		}
		if (ch_N > 7){
			x_ptr[7] = x_lb;
		}
		if (ch_N > 8){
			x_ptr[8] = x_rb;
		}
	}

	//free(x_r);
	return x_ptr;
}

float *** standard_3d_ptr_channel_float_init(float ** x_l, float **x_r, float **x_l2, float ** x_r2, float **x_s, float ** x_lf, float ** x_rf, float ** x_lb, float ** x_rb, float ***x_ptr, int ch_N) {
	x_ptr = (float ***)malloc(ch_N * sizeof(float **));
	if (x_ptr) {
		x_ptr[0] = x_l;
		if (ch_N > 1){
			x_ptr[1] = x_r;
		}
		if (ch_N > 2){
			x_ptr[2] = x_l2;
		}
		if (ch_N > 3){
			x_ptr[3] = x_r2;
		}
		if (ch_N > 4){
			x_ptr[4] = x_s;
		}
		if (ch_N > 5){
			//x_ptr[5] = x_s2;
			x_ptr[5] = x_lf;
		}
		if (ch_N > 6){
			x_ptr[6] = x_rf;
		}
		if (ch_N > 7){
			x_ptr[7] = x_lb;
		}
		if (ch_N > 8){
			x_ptr[8] = x_rb;
		}
	}

	//free(x_r);
	return x_ptr;
}

float ** standard_2d_ptr_crosschannel_float_init(float * x_lr, float *x_l2r2, float *x_ll2, float * x_rr2, float* x_lr2, float* x_l2r, float *x_ls, float *x_l2s, float *x_rs, float *x_r2s, float *x_lfrf, float *x_lbrb,  float **x_ptr, int ch_N) {
	x_ptr = (float **)malloc(ch_N * sizeof(float *));
	if (x_ptr) {
		x_ptr[0] = x_lr;
		x_ptr[1] = x_l2r2;
		x_ptr[2] = x_ll2;
		x_ptr[3] = x_rr2;
		x_ptr[4] = x_lr2;
		x_ptr[5] = x_l2r;
		x_ptr[6] = x_ls;
		x_ptr[7] = x_l2s;
		x_ptr[8] = x_rs;
		x_ptr[9] = x_r2s;
		x_ptr[10] = x_lfrf;
		x_ptr[11] = x_lbrb;
	}

	//free(x_r);
	return x_ptr;
}


float *** standard_3d_ptr_crosschannel_float_init(float ** x_lr, float **x_l2r2, float **x_ll2, float ** x_rr2, float** x_lr2, float** x_l2r, float **x_ls, float **x_l2s, float **x_rs, float **x_r2s, float **x_lfrf, float **x_lbrb, float ***x_ptr, int ch_N) {
	x_ptr = (float ***)malloc(ch_N * sizeof(float **));
	if (x_ptr) {
		x_ptr[0] = x_lr;
		x_ptr[1] = x_l2r2;
		x_ptr[2] = x_ll2;
		x_ptr[3] = x_rr2;
		x_ptr[4] = x_lr2;
		x_ptr[5] = x_l2r;
		x_ptr[6] = x_ls;
		x_ptr[7] = x_l2s;
		x_ptr[8] = x_rs;
		x_ptr[9] = x_r2s;
		x_ptr[10] = x_lfrf;
		x_ptr[11] = x_lbrb;
	}

	//free(x_r);
	return x_ptr;
}

long** standard_2d_ptr_crosschannel_long_init(long* x_lr, long* x_l2r2, long* x_ll2, long* x_rr2, long* x_lr2, long* x_l2r, long* x_ls, long* x_l2s, long* x_rs, long* x_r2s, long* x_lfrf, long* x_lbrb, long** x_ptr, int ch_N) {
	x_ptr = (long **)malloc(ch_N * sizeof(long *));
	if (x_ptr) {
		x_ptr[0] = x_lr;
		x_ptr[1] = x_l2r2;
		x_ptr[2] = x_ll2;
		x_ptr[3] = x_rr2;
		x_ptr[4] = x_lr2;
		x_ptr[5] = x_l2r;
		x_ptr[6] = x_ls;
		x_ptr[7] = x_l2s;
		x_ptr[8] = x_rs;
		x_ptr[9] = x_r2s;
		x_ptr[10] = x_lfrf;
		x_ptr[11] = x_lbrb;

	}
	return x_ptr;
}

// initialize temporary arrays
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
float* init_tmp_hann(int Nwindow) {
	float* hann;
	hann = ar_value(Nwindow, 1.0f);
	window(hann, Nwindow, "hann");
	return hann;
}

float* init_tmp_kaiser(int Nwindow, float Strenghtwindow) {
	float* kaiser;
	kaiser = kaiserf(Nwindow, false, Strenghtwindow, false);
	return kaiser;
}
float* init_tmp_upweight(int Nup) {
	float* upweight;
	upweight = ar_declare(Nup);
	for (long nup = 0; nup < Nup; nup++) {
		upweight[nup] = 1 - ((float) nup) / Nup;
	};
	return upweight;
}



// update state variables while program runs
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

void update_state() {

	// shift inverse impulse array and update rec pointer if not already done in audio buffer routine
	bool rapid_shifted = false;
	if (rapid_on && rapid_shifted) {
		rapid_update_is();
		rapid_update_rec();
	}
}

void rapid_update_is() {
	// shift inverse impulse x
	shift_circular(is->inverse, isinv_N, rapid_n);
	//write_txt_debug(is->inverse,  isinv_N, 0);

	// fade in and fade out at impulse edges
	fade_in(is->inverse, isinv_N, rapid_fade_window, rapid_fade_percent, false);
	fade_out(is->inverse, isinv_N, rapid_fade_window, rapid_fade_percent, false);
	//write_txt_debug(is->inverse,  isinv_N, 0);

	// copy into frequency domain array
	memcpy(fftir->ISinverse, is->inverse, isinv_N * sizeof(float));

	// perform FFT
	fast_fft(fftir->ISinverse, fft_NISinverse, fftir, 1, fft_library);
	//write_txt_debug(fft->ISinverse,  isinv_N, 0); // check looking
}

void rapid_update_rec() {
	long timer_start = clock_tick();
	for (int ch = 0; ch < ch_Nrec; ch++) {
		// fill in recording array with current recbuffer up to current time point rec_N
		//memcpy(rec->ptr[ch], recbuffer->ptr[ch], rapid_n * sizeof(float));
		// fill in recording with past recbuffer from values that might have already been overwritten after current time point rec_N
		//memcpy(rec->ptr[ch] + rapid_n, recbuffer_p->ptr[ch] + rapid_n, (rec_N - rapid_n) * sizeof(float));
		//memcpy(rec->ptr[ch], recbuffer_p->ptr[ch] + rapid_n, (rec_N - rapid_n) * sizeof(float));

		// fade in and fade out at impulse edges
		fade_in(rec->x[ch], rec_N, rapid_fade_window, rapid_fade_percent, false);
		fade_out(rec->x[ch], rec_N, rapid_fade_window, rapid_fade_percent, false);
		//write_txt_debug(rec->ptr[ch],  rec_N, 0);

	}
	//write_wav_debug(rec->ptr, rec_N, ch_Nrec, deb_path_wav, "", 0);
	double timer_diff = clock_stop(timer_start);
	if (((float)timer_diff / 1000.0f) > rec_ms) {
		console_log(PRIO_WARNING, "Reading values from recording buffers took longer than length of one impulse response playback iteration. Increase parameter is_ms and set it equal to rec_ms. \n");
	}
}