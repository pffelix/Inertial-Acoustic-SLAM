// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "params.h"

// variables
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// general setup
long fs;
float update_ms;
float sos;
float T;

// offline read in of sensor data
char offline_path[path_N];
bool offline_rec;
bool offline_ir;
bool offline_head;
bool offline_imu;
bool offline_vicis;
bool offline_vicrec;
bool offline_weight;
bool offline_toa;
bool offline_parml;
bool offline_algo;

// routing between internal channels
int R_rec[int_N];

// routing between internal cross-channels
int R_cross[int_N][2];

// binauralization
bool binaural_on;

// impulse
char is_type[char_N];
double is_amp;
float is_ms;
long is_f1;
long is_f2;
char is_fadein_window[char_N];
double is_fadein_percent;
char is_fadeout_window[char_N];
double is_fadeout_percent;
bool is_unfade;
bool is_fade_inverse;

// inverse impulse
char isinv_type[char_N];

// beamforming
bool beam_on;
int beam_Nch;
int beam_Nmic;
int beam_Nang;
float beam_ang[int_N];

// channels (physical input)
int ch_Nmic;
int ch_Nsound;

// channels (physical output)
int ch_Nis;
int ch_Nbisound;

// channels (internal)
int ch_Nrec; 
int ch_Nrecbeam;
int ch_Ncross;
int ch_Ncrossbeam;

// microphones
float mic_interdistance;
float mic_loc[int_N][3];
float mic_norm[int_N][3];

// recording buffer
float recbuffer_ms;

// rapid impulse response measurement technique;
bool rapid_on;
bool rapid_snrfull;
float rapid_edge_ms;
bool rapid_edge_on;
char rapid_fade_window[char_N];
float rapid_fade_percent;

// recording
float rec_ms;
bool rec_edge_on;
float rec_edge_ms;

// upsampling
bool up_fft;
int up_N;

// synchronization
bool sync_on;
bool sync_update;
float sync_clock;

// impulse response frames
float frame_ms;
float frame_overlap;
long frame_Mstart;
long frame_M;
long frame_Mon;
bool frame_window;
bool frame_normalize;
bool frame_energy;
int frame_reference;

// impulse response frame weights
float weight_el_diff = 30.0f;

// impulse response full
bool ir_multichannelnormalize;
bool ir_startinit;
bool ir_startupdate;
bool ir_startlatency;
float ir_startthreshold;
float ir_startmsoffset;
float ir_startdynamicsearchms;
float ir_energythreshold;
bool ir_sparse_on;
float ir_latency_ms;

// impulse response in frames

// impulse response in frames and upsampled

// time differences
bool td_on;
float td_itd_maxms;
float td_atd_maxms; 
float td_std_maxms;

// level differences
bool ld_on;
bool ld_merge_td;
bool ld_merge_window_cor;
int ld_merge_window_corin;
int ld_merge_window_corout;
float ld_merge_window_Nang;
float ld_merge_window_strength;
char ld_merge_window_type[char_N];
float ld_ild_maxdb;

// correlation
char cor_shift[char_N];
char cor_type[char_N];
long cor_phatlim_f1;
long cor_phatlim_f2;
bool cor_normalized;

// upsampled correlation between 2 microphones

// upsampled double cross-correlation between 2 microphones
bool cor2_on;

// circle
bool cir_on;
bool cir_flip_x_plane;
bool cir_flip_y_plane;
bool cir_flip_z_plane;
int cir_ang_stepsize;
bool cir_diffmax_instead_of_diffmean;
bool cir_delta_instead_of_diff;
bool cir_binarymax_instead_of_y;

// upsampled cross-correlation between 1 microphone at 2 time points
bool corm_on; 

// sphere
bool sph_on;
bool sph_glob_on;
int sph_limit;
float sph_radius;
int sph_ang_stepsize;
int sph_Ndim;
bool sph_orthogonal;
int sph_Nmax;

// features
bool fe_energy_on;
bool fe_tdld_on;
int fe_NVar;

// localization
bool loc_on;
float loc_zero_offset;
bool loc_td_absolute;

// kalman filter 1 DOF
bool kal_on;
bool kal_refresh_rq;
int kal_Ninitsteps;
float kal_rq;
float kal_r[A_Nderivative];

// kalman filter 6 dof
bool kal6_on;
bool kal6_refresh_rq;
int kal6_Ninitsteps;
float kal6_rq[3];
float kal6_r[A_Nderivative];

// particle filter 6 dof
bool par6_on;
bool par6_pool_on;
bool par6_imu_on;
bool par6_track_first;
long par6_Nobs;
long par6_Nobs_skip;
long par6_Npar;
int par6_Nvarbuffer;
bool par6_refresh_rq;
int par6_Ninitsteps;
bool par6_lanerrormedian;
int par6_obs_mode;
float par6_obs_rq;
float** par6_obs_r;
int par6_par_priors;
float par6_par_rq;
float** par6_par_r;
float par6_weightpower[A_weight_N];

// fast fourier transformation
char fft_library[char_N];

// debugging
char deb_path_text[path_N];
char deb_path_wav[path_N];
char deb_multichannel_folder[path_N];
bool deb_on;
bool deb_pool_on;
bool deb_disk;
bool deb_stopimpulse;
bool deb_console;
bool deb_particleslam;
bool deb_timer;
bool deb_parmultiobs;
int deb_Nmax;
int deb_par6_probxyzsph;

// multithreading pool
bool pool_on;
int pool_Nthread;

void init_params() {


	// variables
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// general
	fs = 44100; // optimal fs/2 = is_f2 (than most short impulse transient response), trick 48khz gives good tracking like pattern in impulse response -> more stable estimate
	update_ms = 1.0f;
	sos = 343.0f;
	T = 1.0f;

	// offline read in of sensor data
	strcpy(offline_path, "C:/Exchange/MasterThesis/Code/sensor/src/juce/src/AugmentedAudioRenderer/Builds/VisualStudio2019/Offline/"); // with "/" as last character
	offline_rec = FRAMEWORK_IRONLY ? false : true;
	offline_ir = FRAMEWORK_IRONLY ? true : true;
	offline_head = FRAMEWORK_IRONLY ? false : true;
	offline_imu = FRAMEWORK_IRONLY ? false : true;
	offline_vicis = FRAMEWORK_IRONLY ? true : true;
	offline_vicrec = FRAMEWORK_IRONLY ? true : true;
	offline_weight = FRAMEWORK_IRONLY ? false : true;
	offline_toa = FRAMEWORK_IRONLY ? false : true;
	offline_parml  = FRAMEWORK_IRONLY ? false : true;
	offline_algo = FRAMEWORK_IRONLY ? true : true;


	// routing between internal channels
	R_rec[0] = A_l;
	R_rec[1] = A_l2;
	R_rec[2] = A_r;
	R_rec[3] = A_r2;
	R_rec[4] = A_rf;
	R_rec[5] = A_rf2;
	R_rec[6] = A_beam_lc;
	R_rec[7] = A_beam_lg;
	R_rec[8] = A_beam_rf;
	R_rec[9] = A_beam_rb;
	R_rec[10] = A_beam_rr;
	R_rec[11] = A_beam_rl;

	// routing between internal cross-channels
	R_cross[A_lr][0] = A_l;
	R_cross[A_lr][1] = A_r;
	//R_cross[A_l2r2][0] = A_l2;
	//R_cross[A_l2r2][1] = A_r2;
	R_cross[A_ll2][0] = A_l;
	R_cross[A_ll2][1] = A_l2;
	R_cross[A_rr2][0] = A_r;
	R_cross[A_rr2][1] = A_r2;
	//R_cross[A_lr2][0] = A_l;
	//R_cross[A_lr2][1] = A_r2;
	//R_cross[A_l2r][0] = A_l2;
	//R_cross[A_l2r][1] = A_r;
	R_cross[A_ff2][0] = A_rf;
	R_cross[A_ff2][1] = A_rf2;

	// binauralization
	binaural_on = true; // activate input sound source binauralization based on head tracker information

	// impulse
	is_ms = 1000.0f; // recommended: 100ms, 85ms would be next lower order FFT (performance difference around 1-2ms, but worse results in particle filter)
	strcpy(is_type, "linear"); // recommended: linear (always ends at 0 zero crossing, when upper frequency is nyquist frequency, with this less temporal extension of impulse response than exponential sweep, who would have to be cutted manually at last 0 crossing (see paper Farina(2000))
	is_amp = 0.9f; // recommended: 0.9f (to avoid any possible hearable clipping coming from rounding errors in audio device/system)
	is_f1 = 10; // recommended: 18000, 0 Hz not possible as otherwise error in impulse shape generation
	is_f2 = 22050; // recommended: sweep up to nyquist frequency (samplingrate / 2), then shortest impulse response, but don't go to full edges of nyquist frequency, otherwise aliasing as sweep has a certain bandwith width, 1kHz distance should be fine (recommended 21000Hz (@44.1kHz), 23000Hz (@48kHz)
	strcpy(is_fadein_window, "hann"); // recommended: hann
	is_fadein_percent = 0.0f; // recommended: 0.5 (no significant impact on temporal extension of impulse response, unless no window is applied)
	strcpy(is_fadeout_window, "hann"); // recommended: hann (less hearable)
	is_fadeout_percent = 0.0f; // recommended: 0.5 (no significant impact on temporal extension of impulse response, unless no window is applied)
	is_unfade = false; // recommended: false (less hearable on edges)
	is_fade_inverse = true; // was true before

	// inverse impulse
	strcpy(isinv_type, "analytical"); // recommended: analytical, for very sparse peaks "kirkeby" (moving mean) or "kireby-bandlimit"


	// beamforming
	beam_on = false;
	beam_Nch = 3; // recommended 2
	beam_Nmic = 2; // recommended 2
	beam_Nang = 2; // recommended 2
	beam_ang[0] = 90; // recommended 90
	beam_ang[1] = -90; // recommended -90

	// channels (physical input)
	ch_Nmic = 6;
	ch_Nsound = 1;

	// channels (physical output)
	ch_Nis = 1;
	ch_Nbisound = 2;

	// channels (internal)
	ch_Nrec = ch_Nmic;
	ch_Nrecbeam = ch_Nrec + beam_on * beam_Nch * beam_Nang; // recommended: ch_Nrec + beam_Nch * beam_Nang (2 beamformed channels (l,r) for two steering angles (front, back))
	ch_Ncross = 4;
	ch_Ncrossbeam = ch_Ncross; // recommended: ch_Nrec + beam_Nch ( beamformed cross-correlation for each steering angle (front, back))

	// microphones
	mic_interdistance = 0.004f; // EM258 (outer hole): radius 0.004f (if away from head, 0.008f if close to head but unstable);

	mic_loc[A_l][A_x] = -0.08f;
	mic_loc[A_l2][A_x] = -0.08f;
	mic_loc[A_r][A_x] = 0.08f;
	mic_loc[A_r2][A_x] = 0.08f;
	mic_loc[A_rf][A_x] = 0.07f;
	mic_loc[A_rf2][A_x] = 0.07f - mic_interdistance;

	mic_loc[A_l][A_y] = 0.0f;
	mic_loc[A_l2][A_y] = 0.0f; // -0.018f
	mic_loc[A_r][A_y] = 0.0f;
	mic_loc[A_r2][A_y] = 0.0f - mic_interdistance;// -0.018f
	mic_loc[A_rf][A_y] = 0.008f;
	mic_loc[A_rf2][A_y] = 0.008f;

	mic_loc[A_l][A_z] = 0.0f;
	mic_loc[A_l2][A_z] = 0.0f - mic_interdistance;
	mic_loc[A_r][A_z] = 0.0f;
	mic_loc[A_r2][A_z] = 0.0f;
	mic_loc[A_rf][A_z] = 0.00f;
	mic_loc[A_rf2][A_z] = 0.00f;

	mic_norm[A_l][A_x] = -1.0f;
	mic_norm[A_l2][A_x] = -1.0f;
	mic_norm[A_r][A_x] = 1.0f;
	mic_norm[A_r2][A_x] = 1.0f;
	mic_norm[A_rf][A_x] = 0.0f;
	mic_norm[A_rf2][A_x] = 0.0f;

	mic_norm[A_l][A_y] = 0.0f;
	mic_norm[A_l2][A_y] = 0.0f;
	mic_norm[A_r][A_y] = 0.0f;
	mic_norm[A_r2][A_y] = 0.0f;
	mic_norm[A_rf][A_y] = 1.0f;
	mic_norm[A_rf2][A_y] = 1.0f;

	mic_norm[A_l][A_z] = 0.0f;
	mic_norm[A_l2][A_z] = 0.0f;
	mic_norm[A_r][A_z] = 0.0f;
	mic_norm[A_r2][A_z] = 0.0f;
	mic_norm[A_rf][A_z] = 0.0f;
	mic_norm[A_rf2][A_z] = 0.0f;

	// recording buffer
	recbuffer_ms = 100.0f;

	// rapid impulse response measurement technique;
	rapid_on = (recbuffer_ms == is_ms) ? true : false; // (rec_ms == is_ms) ? true : false; // recommended: (rec_ms == is_ms) ? true : false;
	rapid_snrfull = true;
	rapid_edge_ms = 10.0f;
	rapid_edge_on = false;
	strcpy(rapid_fade_window, "linear"); 
	rapid_fade_percent = 0.1f;

	// recording
	rec_ms = (rapid_on && rapid_snrfull) ? recbuffer_ms * 2: recbuffer_ms; // conventional impulse response measurement technique: rec_ms = 2 * is_ms, if rec_ms = is_ms -> rapid impulse response measurement technique is selected
	rec_edge_on = true;
	rec_edge_ms = 10.0f;

	// recording buffer
	//recbuffer_ms = rec_ms;

	// upsampling
	up_fft = false;
	up_N = next_power_of_2(32); // next_power_of_2(32); // if up_fft = true -> must be power of 2 resulting from fft radix-2 algorithm
	
	// synchronization
	sync_on = false;
	sync_update = false; // false (better for particle slam)
	sync_clock = 1.15f; // iphone 6s and Focusrite Scarlet 18i8: 1.15f (+ if impulse response moves to start, else -)

	// impulse response full
	ir_multichannelnormalize = false;
	ir_startinit = false;
	ir_startupdate = false; 
	ir_startlatency = true; // recommended: true (os otherwise useless frames before latency threshold)
	ir_startthreshold = sync_on? 0.1f : 0.01f; // reccomended: sync_on ? 0.1f : 0.01f, but smartphone synchronization has more noise -> incrase threshold
	ir_startmsoffset = sync_on ? 0.0f : 0.0f; // recommended: 2.0f (transient time impulse)
	ir_startdynamicsearchms = -0.0f; // recommended: 0.0f (off) or -20.0f (on)
	ir_energythreshold = 0.03f;
	ir_sparse_on = false;
	ir_latency_ms = sync_on ? 0.0f : 11.05f; // focusrite to UMS-1 = 7.01 (peak: 10.01) @ 256 samples buffer size (192khz), 8.05 (peak 11.05) @ 512 samples buffer size (192khz)


    // impulse response frames
	frame_ms = 2.6f; // recommended: 1.2f (minimum 0.1f), (@192khz: 1ms: 192 samples (next fft order: 512 (2.6ms), 256 (1.3ms) or 128 (0.6ms)))
	frame_overlap = 0.75f; // recommended: 0.75f
	frame_Mstart = 0; // recommended: 0
	frame_M = 30; // recommended: 20
	frame_M = frame_Mstart == 0 ? frame_M : frame_M - frame_Mstart;
	frame_Mon = 30; // recommended: 10, very small does not work as than always switch between frames
	frame_window = true; // recommended: true
	frame_normalize = false; // recommended: false
	frame_energy = false; // recommneded: false
	frame_reference = A_r;

	// impulse response frame weights
	weight_el_diff = 30.0f;
	
	// impulse response in frames

	// impulse response in frames and upsampled

    // time differences
	td_on = true;
	td_itd_maxms = 0.7f;
	td_atd_maxms = m_to_msec(mic_interdistance, sos); // distance_in_ms(mic_loc[A_l], mic_loc[A_l2], sos, &td_atd_maxms);
	td_std_maxms = 10.0f;

    // level differences
	ld_on = true;
	ld_merge_td = true;
	ld_merge_window_cor = false;
	ld_merge_window_corin = A_lr;
	ld_merge_window_corout = A_ff2;
	ld_merge_window_Nang = 45.0f; // recommended 45.0f (in angle degrees)
	ld_merge_window_strength = 15.0f; // recommended 15.0f, if < 8.0f not going down to 0, if around 100.0f maximal sharp (then control via Nang necessary)
	strcpy(ld_merge_window_type, "kaiser"); 
    ld_ild_maxdb = 25.0f;

	// correlation
	strcpy(cor_shift, "no");
	strcpy(cor_type, "cor"); // recommended: "cor-phatlim"
	cor_phatlim_f1 = is_f1; // recommended: is_f1
	cor_phatlim_f2 = is_f1 + 1000; // recommended: is_f2 (when not full nyquist range)
	cor_normalized = true; // recommend: true

	// upsampled correlation between 2 microphones

	// upsampled double cross-correlation between 2 microphones
	cor2_on = false;

	// circle
	cir_on = true;
	cir_flip_x_plane = true;
	cir_flip_y_plane = true;
	cir_flip_z_plane = false;
	cir_ang_stepsize = 1;
	cir_diffmax_instead_of_diffmean = true;
	cir_delta_instead_of_diff = true;
	cir_binarymax_instead_of_y = rapid_on ? false : false;

	// upsampled cross-correlation between 1 microphone at 2 time points
	corm_on = cir_delta_instead_of_diff ? false : true;

	// sphere
	sph_on = false;
	sph_glob_on = true;
	sph_limit = 90;
	sph_radius = 1.0f;
	sph_ang_stepsize = 1;
	sph_Ndim = 3;
	sph_orthogonal = false;  // recommended: "false"
	sph_Nmax = 5;

	// features
	fe_energy_on = false;
	fe_tdld_on = false;
	fe_NVar = 10;

	// localization
	loc_on = false;
	loc_zero_offset = 0.0001f;
	loc_td_absolute = false;

	// kalman filter 1 DOF
	kal_on = false;
	kal_refresh_rq = true;
	kal_Ninitsteps = 0;
	kal_rq = 10.0f; // recommended: 10 (might be now inverse)
	kal_r[A_ref] = 1.0f/100000.0f; // before: kal_rabsvel = 1/100000 (higher focus absolute); no this would be (1/1000, 1, 1)
	kal_r[A_vel] = 1.0f;
	kal_r[A_acc] = 1.0f;

	// kalman filter 6 dof
	kal6_on = false;
	kal6_refresh_rq = true;
	kal6_Ninitsteps = 0;
	kal6_rq[Xtrans] = 10; 
	kal6_rq[Xrot] = 10; 
	kal6_rq[Xrotmap] = 10; 
	kal6_r[A_ref] = 1.0f/1000.0f;
	kal6_r[A_vel] = 1.0f;
	kal6_r[A_acc] = 1.0f;

	// particle filter 6 dof
	par6_on = true;
	par6_pool_on = false;
	par6_imu_on = false;
	par6_track_first = false;
	par6_Nobs = frame_M; // but only frame_Mon selected
	par6_Nobs_skip = par6_track_first ? 0 : 12;
	par6_Npar = (4 < frame_Mon) ? 4: frame_Mon; // recommended: not more than number of landmark observations (par6_Nobs)
	par6_Nvarbuffer = 200;
	par6_refresh_rq = false;
	par6_Ninitsteps = 1000;
	par6_lanerrormedian = true;
	par6_obs_mode = A_mode_refvel;
	par6_obs_rq = 1000000000000.0f; // recommended: 1000000000000.0f, how important is model relative to observation (R = 1.0f standard, high more model than observation)
	par6_obs_r = ar_declare2d(A_Ndim, A_Nderivative);
	par6_obs_r[A_x][A_ref] = par6_obs_mode == A_mode_ref || par6_obs_mode == A_mode_refvel ? 1.0f : A_roff3; // how important is differential observation (high value less importance)
	par6_obs_r[A_x][A_vel] = par6_obs_mode == A_mode_vel || par6_obs_mode == A_mode_refvel ? 1.0f : A_roff3;
	par6_obs_r[A_x][A_acc] = A_roff3;
	par6_obs_r[A_y][A_ref] = par6_obs_mode == A_mode_ref || par6_obs_mode == A_mode_refvel ? 1.0f : A_roff3; // how important is differential observation (high value less importance)
	par6_obs_r[A_y][A_vel] = par6_obs_mode == A_mode_vel || par6_obs_mode == A_mode_refvel ? 1.0f : A_roff3;
	par6_obs_r[A_y][A_acc] = A_roff3;
	par6_obs_r[A_z][A_ref] = par6_obs_mode == A_mode_ref || par6_obs_mode == A_mode_refvel ? 1.0f : A_roff3; // how important is differential observation (high value less importance)
	par6_obs_r[A_z][A_vel] = par6_obs_mode == A_mode_vel || par6_obs_mode == A_mode_refvel ? 1.0f : A_roff3;
	par6_obs_r[A_z][A_acc] = A_roff3;
	par6_obs_r[A_az][A_ref] = par6_obs_mode == A_mode_ref || par6_obs_mode == A_mode_refvel ? 1.0f : A_roff3; // how important is differential observation (high value less importance)
	par6_obs_r[A_az][A_vel] = par6_obs_mode == A_mode_vel || par6_obs_mode == A_mode_refvel ? 1.0f : A_roff3;
	par6_obs_r[A_az][A_acc] = A_roff3; // A_ron high: stabilize acceleration to 0 (strong rotation absolute currently, that is also why differential input currently not working)
	par6_obs_r[A_el][A_ref] = par6_obs_mode == A_mode_ref || par6_obs_mode == A_mode_refvel ? 1.0f : A_roff3; // how important is differential observation (high value less importance)
	par6_obs_r[A_el][A_vel] = par6_obs_mode == A_mode_vel || par6_obs_mode == A_mode_refvel ? 1.0f : A_roff3;
	par6_obs_r[A_el][A_acc] = A_roff3;
	par6_obs_r[A_rd][A_ref] = A_roff3; // how important is differential observation (high value less importance)
	par6_obs_r[A_rd][A_vel] = A_roff3;
	par6_obs_r[A_rd][A_acc] = A_roff3;
	par6_par_priors =  par6_track_first? A_parprior_kalsingle : A_parprior_kalsingle; // merge multiple landmarks for prior estimation instead of using only a single landmark
	par6_par_rq = 1.0f; // 0.0000001f how important is acoustic particle prior observation vs intertial motion model (high value less importance)
	par6_par_r = ar_declare2d(A_Ndim, A_Nderivative);
	par6_par_r[A_x][A_ref] = par6_track_first ? 1.0f : A_roff2; // how important is differential observation (high value less importance)
	par6_par_r[A_x][A_vel] = 1.0f;
	par6_par_r[A_x][A_acc] = A_roff2;
	par6_par_r[A_y][A_ref] = par6_track_first ? 1.0f : A_roff2; // how important is differential observation (high value less importance)
	par6_par_r[A_y][A_vel] = 1.0f;
	par6_par_r[A_y][A_acc] = A_roff2;
	par6_par_r[A_z][A_ref] = par6_track_first ? 1.0f : A_roff2; // how important is differential observation (high value less importance)
	par6_par_r[A_z][A_vel] = 1.0f;
	par6_par_r[A_z][A_acc] = A_roff2;
	par6_par_r[A_az][A_ref] = par6_track_first ? 1.0f : A_roff2; // how important is differential observation (high value less importance)
	par6_par_r[A_az][A_vel] = 1.0f;
	par6_par_r[A_az][A_acc] = A_roff2; // A_ron high: stabilize acceleration to 0 (strong rotation absolute currently, that is also why differential input currently not working)
	par6_par_r[A_el][A_ref] = par6_track_first ? 1.0f : A_roff2; // how important is differential observation (high value less importance)
	par6_par_r[A_el][A_vel] = 1.0f;
	par6_par_r[A_el][A_acc] = A_roff2;
	par6_par_r[A_rd][A_ref] = A_roff2; // how important is differential observation (high value less importance)
	par6_par_r[A_rd][A_vel] = A_roff2;
	par6_par_r[A_rd][A_acc] = A_roff2 ;
	par6_weightpower[A_weight_energy] = 0.0f; // 0.2f
	par6_weightpower[A_weight_peak] = 0.0f; // 1.0f
	par6_weightpower[A_weight_sparsity] = 0.0f; // 1.0f
	par6_weightpower[A_weight_stability] = 0.0f; // 15.0f
	par6_weightpower[A_weight_elevation] = 0.0f; // 1.0f
	par6_weightpower[A_weight_early] = 1.0f; // 3.0f
	par6_weightpower[A_weight_global] = 1.0f; // 1.0f

	//par6_weightpower[A_el] = 20.0f; // old:az 15.0f, el 20.0f (own dimension), xyz 100.0f, rd A_off
	
	// fast fourier transformation
	strcpy(fft_library, "intelmkl");

	// debugging
	strcpy(deb_path_text, "./debug.txt"); 
	strcpy(deb_path_wav, "./debug.wav"); 
	strcpy(deb_multichannel_folder, "C:/Exchange/MasterThesis/Code/analysis/irs/realtime/5_room/2_rec_"); 
	deb_on = true;
	deb_pool_on = true;
	deb_disk = true;
	deb_stopimpulse = true; // does not hold when rapid impulse response measurement mode is on
	deb_console = true;
	deb_particleslam = true;
	deb_timer = true;
	deb_parmultiobs = true;
	deb_Nmax = 20;
	deb_par6_probxyzsph = A_debprob_sph; // 

	// multithreading pool
	pool_on = true;
	pool_Nthread = (frame_M > (ch_Ncrossbeam + ch_Nrec)) ? frame_M + deb_Nmax + A_Ndof : (ch_Ncrossbeam + ch_Nrec) + deb_Nmax + A_Ndof; // assuming par6_Npar < frame_M
};
