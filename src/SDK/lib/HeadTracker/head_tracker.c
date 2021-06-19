// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "head_tracker.h"

//#include <algorithm>
//#include <dirfent.h>
//#include <jni.h>
//#include <android/log.h>

#ifdef __cplusplus
extern "C" {
#endif

	// external functions

	void init_head_tracker() {
		// add console for debugging (only windows)
		console_add();

		// initialize
		init_head_tracker_params();
		init_head_tracker_state();
	}



	// initialization functions

	void init_head_tracker_params() {
		// initialize
		init_params();
	}

	void init_head_tracker_state() {
		// initialize
		init_state();
	}

	// internal head tracker functions

	void update_head_tracker_state() {
		// initialize 
		long timer_start;
		double timer_diff;

		// dynamically update program state
		update_state();

		// read from imu sensor
		if (!FRAMEWORK_OFFLINEREADSAVE) {
			get_imu_pos(imu);
		}
		// read from vicon sensor
		if (!FRAMEWORK_OFFLINEREADSAVE) {
			get_vicon_pos(vic->pos_is, vic->pos_is_p, VIC_is, A_Ndof, &A_dim, A_Nderivative, vic->update_Tdiff);
			get_vicon_pos(vic->pos_rec, vic->pos_rec_p, VIC_rec, A_Ndof, &A_dim, A_Nderivative, vic->update_Tdiff);
		}
		// start reading from acoustic recording
		ask_lock(&lock_rec);

		// recording (rec) to clean recording (rec)
		timer_start = clock_tick();
		if (!FRAMEWORK_OFFLINEREADSAVE) {
			rec_clean();
		}
		timer_diff = clock_stop(timer_start);
		console_logd(PRIO_TIMER, "sync rec in: %4.4f ms\n", timer_diff);

		// save and read offline files
		if (FRAMEWORK_OFFLINESAVE) {
			save_head_tracker_disk();
		}
		if (FRAMEWORK_OFFLINEREAD) {
			read_head_tracker_disk();
		}
		if (FRAMEWORK_OFFLINEREADSAVE) {
			if (update_nr != 0) {
				// write back modified data to data read from past loop
				update_nr = update_nr - 1;
				save_head_tracker_disk();
				update_nr = update_nr + 1;
			}
			// read new update position
			read_head_tracker_disk();

		}

		// recording (rec) to synchronized recording (rec)
		timer_start = clock_tick();
		rec_sync();
		timer_diff = clock_stop(timer_start);
		console_logd(PRIO_TIMER, "sync rec in: %4.4f ms\n", timer_diff);

		// recording (rec) to impulse response (ir)
		timer_start = clock_tick();
		rec_to_ir();
		timer_diff = clock_stop(timer_start);
		console_logd(PRIO_TIMER, "rec to ir in: %4.4f ms\n", timer_diff);
		
		// finish reading from acoustic recording
		close_lock(&lock_rec);
		if (FRAMEWORK_IRONLY) {
			// finish update loop
			finish_update();
			return;
		}

		// impulse response (ir) to sparse impulse response (ir)
		timer_start = clock_tick();
		ir_to_irsparse();
		timer_diff = clock_stop(timer_start);
		console_logd(PRIO_TIMER, "ir to sparseir in: %4.4f ms\n", timer_diff);
		

		// calculate features

		// impulse response (ir) to impulse response frames (irf)
		timer_start = clock_tick();
		irsparse_to_irf();
		timer_diff = clock_stop(timer_start);
		console_logd(PRIO_TIMER, "ir to irf in: %4.4f ms\n", timer_diff);

		// impulse response frames (irf) to features (fe)
		timer_start = clock_tick();
		irf_to_fe();
		timer_diff = clock_stop(timer_start);
		console_logd(PRIO_TIMER, "irf to fe in: %4.4f ms\n", timer_diff);

		// features (fe) to slam (slam)
		timer_start = clock_tick();
		fe_to_slam();
		timer_diff = clock_stop(timer_start);
		console_logd(PRIO_TIMER, "fe to kal in: %4.4f ms\n", timer_diff);

		// slam (slam) to head position (pos)
		ask_lock(&lock_head);
		timer_start = clock_tick();
		slam_to_pos();
		timer_diff = clock_stop(timer_start);
		console_logd(PRIO_TIMER, "kal to head in: %4.4f ms\n", timer_diff);
		close_lock(&lock_head);

		// save features for next update routine
		timer_start = clock_tick();
		save_state();
		timer_diff = clock_stop(timer_start);
		console_logd(PRIO_TIMER, "save state in: %4.4f ms\n", timer_diff);

		// finish update loop
		finish_update();
		console_logi(PRIO_MAIN, "first frame %i\n", fra->m_energyfirst);
	};

	void rec_clean(){
		// initialize
		long timer_start;
		double timer_diff;
		int ch, n;
		long rec_edge_N2m1;
		long rec_Nm1;
		long rapid_edge_Nstart;


		// clean impulse response edges
		if (rec_edge_on) {

			// apply fade in/out window 
			rec_edge_N2m1 = 2 * rec_edge_N - 1;
			rec_Nm1 = rec_N - 1;
			for (ch = 0; ch < ch_Nrec; ch++) {
				for (n = 0; n < rec_edge_N; n++) {
					// begin of recording
					rec->x[ch][n] *= tmp_rec_hann[n]; 
					// end of recording
					rec->x[ch][rec_Nm1 - n] *= tmp_rec_hann[rec_edge_N2m1 - n]; 
				}
			}

			// apply window to buffer edge in rapid impulse response measurement
			if (rapid_on && rapid_edge_on) {
				rapid_edge_Nstart = rapid_n - rapid_edge_N; // guaranteed by measurement implementation to be larger than 0
				for (ch = 0; ch < ch_Nrec; ch++) {
					for (n = 0; n < rapid_edge_N; n++) {
						// begin of new buffer 
						rec->x[ch][rapid_edge_Nstart + n] *= tmp_rapid_hann[rapid_edge_N + n]; 
						// end of old buffer
						rec->x[ch][rapid_n + n] *= tmp_rapid_hann[n];
					}
				}
			}
		}
	}

	void rec_sync() {
		// initialize
		long timer_start;
		double timer_diff;
		long sync_Noffset_l_r;
		long irsync_N;
		//sync_on = false;
		if (sync_on) {

			// detect playback and delay position to sync recording buffer to playback start (changes rec array)
			if (sync_update || update_first) {
				// use old impulse position estimate to shift left and right recording signal
				shift_circular(rec->x[A_l], recbuffer_N, -  sync_Noffset_p);
				shift_circular(rec->x[A_r], recbuffer_N, -  sync_Noffset_p);
				//write_txt_debug(rec->x[A_r], recbuffer_N, 1);
				// derive new impulse position estimate with matched filter for recording signal
				irsync_N = (recbuffer_N + isinv_N - 1);
				convolve_rec_to_ir(rec->x[A_l], recbuffer_N, is->inverse, isinv_N, ir->x[A_l], irsync_N, fftir, fft_library);
				convolve_rec_to_ir(rec->x[A_r], recbuffer_N, is->inverse, isinv_N, ir->x[A_r], irsync_N, fftir, fft_library);
				//write_txt_debug(ir->x[A_r], irsync_N, 1);
				
				if (rapid_on) {
					// calculate new playback position estimate
					get_binaural_playback_position(ir->x[A_l], ir->x[A_r], irsync_N, isinv_N, isinv_N, &sync_Ndelay, &sync_Nplay);
					sync_Noffset = modulus(sync_Nplay + sync_Ndelay, recbuffer_N);
					console_logl(PRIO_SYNC, "peak estimate offset: %ld \n", modulus_diff(sync_Noffset, isinv_N));
				}
				else {
					// calculate new delay position estimate
					get_binaural_delay_position(ir->x[A_l], ir->x[A_r], irsync_N, isinv_N, recbuffer_N, isinv_N, &sync_Ndelay);
					sync_Noffset = modulus(sync_Nplay + sync_Ndelay, recbuffer_N);
					console_logl(PRIO_SYNC, "peak estimate offset: %ld \n", modulus_diff(sync_Noffset, isinv_N));
				}

				//write_txt_debug(rec->x[A_l], recbuffer_N, 1);
				//write_txt_debug(ir->x[A_l], ir_N, 0);

				// shift recordings with new impulse position estimate
				sync_Noffset_l_r = sync_Noffset;
				sync_Noffset = modulus(sync_Noffset + sync_Noffset_p, recbuffer_N);
				for (int ch = 0; ch < ch_Nrec; ch++) {
					if (ch == A_l || ch == A_r) {
						// shift recordings used for new impulse position estimation (sync_Noffset_p shift already included)
						shift_circular(rec->x[A_l], recbuffer_N, -sync_Noffset_l_r);
						shift_circular(rec->x[A_r], recbuffer_N, -sync_Noffset_l_r);
					}
					else {
						// shift all other recordings with new incremental impulse position estimate
						shift_circular(rec->x[ch], recbuffer_N, -sync_Noffset);
					}
				}

				// save incremental impulse position estimate
				sync_Noffset_p = sync_Noffset;
			}
			else {
				// use old impulse position estimate to shift recording signal
				for (int ch = 0; ch < ch_Nrec; ch++) {
					shift_circular(rec->x[ch], recbuffer_N, -  sync_Noffset_p);
				}
			}

			// rapid full snr technique: copy shifted recording buffer to full recording
			if (rapid_on && rapid_snrfull) {
				for (int ch = 0; ch < ch_Nrec; ch++) {
					memcpy(rec->x[ch] + recbuffer_N, rec->x[ch], recbuffer_N * sizeof(float));
				}
			}

			// incrementally synchronize recording and playback clock missmatch
			sync_Noffset_p -= (long)roundf(sync_clocknow);
			sync_clocknow += sync_clock * update_Tdiff - roundf(sync_clocknow);

		}

	}

	void rec_to_ir() {
		// initialize
		long timer_start;
		double timer_diff;

		// initialize

		// calculate

		// beamform microphone signals
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		timer_start = clock_tick();
		if (beam_on) {
			for (int ch = 0; ch < beam_Nch; ch++) {
				for (int ang = 0; ang < beam_Nang; ang++) {
					rec_to_recbeam(rec->x[2 * ch], rec->x[2 * ch + 1], rec_N, beam_ang[ang], mic_loc[A_r2][A_x] - mic_loc[A_r][A_x], sos, (float)fs, rec->x[ch_Nrec + 2 * ch + ang]); // distance between any microphone pair always mic_loc[A_l2][A_y] - mic_loc[A_l][A_y] 
				}
			}
		}
		timer_diff = clock_stop(timer_start);


        // calculate impulse for each recording channel
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////

		timer_start = clock_tick();
		loop_update_thread_or_serial(pool, rec_to_ir_multithread, ch_Nrecbeam, pool_on, true);
		timer_diff = clock_stop(timer_start);
	}

	void rec_to_ir_multithread(void* arg) {
		// get argument: recording channel
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		int* arg_ptr = arg;
		int ch = *arg_ptr;

		// convole impulse recording with inverse impulse to get room impulse response recording
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//for (int ch = 0; ch < ch_Nrecbeam; ch++) {
			convolve_rec_to_ir(rec->x[ch], rec_N, is->inverse, isinv_N, ir->x[ch], ir_N, &fftir[ch], fft_library);
			// filter impulse recording
			//lowpass(ir->x[ch], ir_N, wtKAISER, 51, 100.0f, 23000, fs);
			//convolve_rec_to_ir(is->send, is_N, is->inverse, isinv_N, ir->x[ch], ir_N, fft, fft_library);
		//}

	}

	void ir_to_irsparse() {
		// initialize

		//long Nvalue = 91;
		//long YN = (2 * Nvalue - 1) * up_N + 10;
		//convolve_fft(ar_value(Nvalue, 1.0f), Nvalue, ar_value(Nvalue, 1.0f), Nvalue, ir->x[0], YN, 0, fft, fft_library, "");
		//write_txt_debug(ir->x[0], YN, 1);

		if (ir_sparse_on) {
			// sparsify impulse response 
			for (int ch = 0; ch < ch_Nrecbeam; ch++) {
				long Ndirfilter = 100;
				long Nirframe = 10000;
				long Ndirinvfilter = next_power_of_2(Ndirfilter);
				float* dirinvfilter = ar_declare(Ndirinvfilter);
				inverse_kirkeby(ir->x[ch] + ir_Nstart, Ndirfilter, dirinvfilter, Ndirinvfilter, "linear", "kirkeby-bandlimit", fs, is_f1, is_f2, fftir, fft_library);
				convolve_fft(ir->x[ch] + ir_Nstart, Nirframe, dirinvfilter, Ndirinvfilter, ir->x[ch], Nirframe + Ndirinvfilter - 1, 0, fftir, fft_library, "");
				//write_txt_debug(dirinvfilter, Ndirinvfilter, 0);

				//crosscorrelate_fft(ir->x[ch] + ir_Nstart, Nirframe, ir->x[ch] + ir_Nstart, Ndirfilter, ir->x[ch], Nirframe + Ndirfilter - 1, 1, fft, fft_library, cor_type, cor_phatlim_f1, cor_phatlim_f2, fs, false);
			}
		}
	}

	void irsparse_to_irf() {
		// initialize
		long timer_start;
		double timer_diff;

		// multichannel normalize impulse response (slow)
		if (ir_multichannelnormalize) {
			timer_start = clock_tick();
			multichannel_normalize(ir->x, ir_N, ch_Nrecbeam);
			timer_diff = clock_stop(timer_start);
		}
		//write_txt_debug(ir->x[0], ir_N, 1);

		// get impulse response start sample position
		timer_start = clock_tick();
		if (sync_on) {
			// get first sample with signal energy
			get_binaural_start_sample(ir->x[A_l], ir->x[A_r], ir_N, ir_Nstart0, ir_Nstartdynamicsearch, ir_startthreshold, ir_Nstartoffset, &ir_Nstart); // might be faster implemented by adding start position x_N_search_start that is at is_N (but not reliable in case of sync)
			//if (update_first) {
				//check_start_sample(ir_N, frame_M, frame_N, frame_Noverlap, &ir_Nstart);
				//ir_Nstart0 = ir_Nstart; // always active
			//}
			//ir_Nstart = isinv_N; // +2 * ir_Nstartoffset;
			// standard frame start
			if (update_first) {
				ir_Nstartframe = ir_Nstart0;
				//ir_Nstartframe += ir_Nstartoffset;
			}
			// dynamic frame start
			if (ir_startupdate || (ir_startinit && update_first)) {
				check_start_sample(ir_N, frame_M, frame_N, frame_Noverlap, &ir_Nstart);
				ir_Nstartframe = ir_Nstart; // only active with option
				//ir_Nstartframe += ir_Nstartoffset;
			}
		}
		else{
			// get first sample with signal energy
			get_binaural_start_sample(ir->x[A_l], ir->x[A_r], ir_N, ir_Nstart0, ir_Nstartdynamicsearch, ir_startthreshold, ir_Nstartoffset, &ir_Nstart); // might be faster implemented by adding start position x_N_search_start that is at is_N (but not reliable in case of sync)
			// set start point of first frame
			
			// standard frame start
			if (update_first) {
				ir_Nstartframe = ir_Nstart0;
				//ir_Nstartframe += ir_Nstartoffset;
			}
			// dynamic frame start
			if (ir_startupdate || (ir_startinit && update_first)) {
				check_start_sample(ir_N, frame_M, frame_N, frame_Noverlap, &ir_Nstart);
				ir_Nstartframe = ir_Nstart; // only active with option
				ir_Nstartframe += ir_Nstartoffset;
			}
		}
		timer_diff = clock_stop(timer_start);

		// extract frames from full impulse response
		timer_start = clock_tick();
		loop_update_thread_or_serial(pool, extract_frames_multithread, ch_Nrecbeam, pool_on, true); // statistics are framebased not multichannel
		timer_diff = clock_stop(timer_start);



		// debugging
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		// Debug


		//write_txt_debug(rec->x[0], rec_N,0);
		//write_txt_debug(is->send, isinv_N,0);
		//write_txt_debug(is->inverse, isinv_N,0);
		//write_txt_debug(rec->x[0], rec_N,0);
		//write_wav_debug_multithread(&is->inverse, 1, is_N, fs, deb_path_wav, "", 1, deb_pool_on);
		//write_wav_frame_debug_multithread(irf->x, ch_Nrecbeam, frame_M, irf_N, (float)fs, deb_path_wav, "", 1, deb_pool_on);
		//write_wav_debug_multithread(ir->x, ch_Nrec, ir_N, (float)fs, deb_path_wav, "", 2, deb_pool_on);
		//write_wav_debug_multithread(rec->x, ch_Nrecbeam, rec_N, (float)fs, deb_path_wav, "", 1, deb_pool_on);
		//write_wav_debug_multithread(ir->x, ch_Nrec, isinv_N + isinv_N - 1, (float)fs, deb_path_wav, "", 1, deb_pool_on);
		write_wav_debug_multithread(ir->x, ch_Nrecbeam, ir_N, (float)fs, deb_path_wav, "", 1, deb_pool_on);

	};

	void extract_frames_multithread(void* arg) {
		// get argument: frame number
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		int* arg_ptr = arg;
		int ch = *arg_ptr;

		// calculate
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		extract_frames(fra, ir->x[ch], ir_N, tmp_frame_hann, frame_N, frame_M, frame_Mon, frame_Mstart, ir_Nstartframe, ir_Nstart, frame_Noverlap, frame_window, frame_normalize, ch, frame_reference, irf->x[ch]);

	}

	void irf_to_fe() {


        // calculate features for each impulse response frame
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		loop_update_thread_or_serial(pool, irf_to_fe_multithread, frame_M, pool_on, true);


		// aggegrate channel based features to frame based features 
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		tdamp_to_frame_corpeak_normalize(fra, frame_M);

		// wall position estimate (is not integratable in multithreading)
		long timer_start = clock_tick();
		if (A_Ndof == 6) {
			for (int m = 0; m < frame_M; m++) { // has to wait for finish of first frame calculation
				if (fra->m[m]) {
					Nstart_to_frame_xyz(fra, head, par6, fra->m_Npeak[m], ir_Nstart0, ir_Nstart, ir_Nlatency, sync_on, m, fs, sos); // ir_Nstart + (frame_N - frame_Noverlap) * m
				}
			}
		}

		// difference features (has to come after wall position estimate)
		if (cir_delta_instead_of_diff && !update_first) { // 
			for (int d = 0; d < A_Ndof; d++) {
				for (int m = 0; m < frame_M; m++) {
					ang_to_angdiff(head->pos_p[m][d], head->pos[m][d][A_ref], &head->pos[m][d][A_vel]);
					if (d == A_dim_r[A_az]) {
						float aaaaaa = head->pos[m][d][A_vel];
						int bbb = 0;
					}
				}

			}
		}
		double timer_diff = clock_stop(timer_start);
		//Nstart_to_frame_xyz(fra, head, par6, ir_Nstart, ir_Nstart0, fra->m_energyfirst, fs, sos);
		
		// aggegrate frame based features to channel based features 
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		// mean absolute differential time, level and distance differences
		if (fe_tdld_on) {
			for (int ch = 0; ch < ch_Ncrossbeam; ch++) {
				ar_sumabsaverage(fe->td_angdiff[ch], (long)frame_M, &fe->td_angdiffabsaverage[ch]);
				ar_sumabsaverage(fe->ld_angdiff[ch], (long)frame_M, &fe->ld_angdiffabsaverage[ch]);
				ar_sumabsaverage(fe->tdm_distdiff[ch], (long)frame_M, &fe->tdm_distdiffabsaverage[ch]);
			}

			// mean energy
			for (int ch = 0; ch < ch_Nrecbeam; ch++) {
				ar_sumabsaverage(fe->energy[ch], (long)frame_M, &fe->energy_mean[ch]);
			}
		}

		// aggregate channel based features to global output feature 
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////

		// get differential angle from left and right channel mean absolute differential time and level differences
		if (fe_tdld_on) {
			bool run_level_difference_diff_feature = false;
			if (run_level_difference_diff_feature) {
				ar_smean(fe->td_angdiffabsaverage, fe->ld_angdiffabsaverage, (long)ch_Ncrossbeam, fe->angdiffabsaverage);
			}
			else {
				ar_smean(fe->td_angdiffabsaverage, fe->td_angdiffabsaverage, (long)ch_Ncrossbeam, fe->angdiffabsaverage);
			}
		}
		if (corm_on) {
			// get distance change from differential single microphone differences
			ar_smean(fe->tdm_distdiffabsaverage, fe->tdm_distdiffabsaverage, (long)ch_Ncrossbeam, fe->distdiffabsaverage);
		}
		if (fe_energy_on) {
			// get average global energy in each channel
			ar_sumabsaverage(fe->energy_mean, (long)ch_Ncrossbeam, &fe->energy_global);
		}

		// debugging
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//float** sph0 = ar_declare2d(sph_Naz, sph_Nel);
		//for (int nn = 0; nn < sph_Naz; nn++) {
			//for (int mm = 0; mm < sph_Nel; mm++) {
				//sph0[nn][mm] = (float)sph->x[0][nn][mm];
			//}
		//}
		//write_txt_debug2d(sph->x[0], sph_Naz, sph_Nel, 0);
		//write_txt_debug2d(sph->x[0], sph_Naz, sph_Nel, 0);
		//write_txt_debug2d(sph->w_Nsphn[0], sph_Naz, sph_Nel, 0);
		//write_wav_frame_debug_multithread(irfup->x, ch_Nrecbeam, frame_M, irfup_N, (float)fsup, deb_path_wav, "", 1, deb_pool_on);
		//write_wav_debug_multithread(cir->prob[A_dim[A_az]], cir_Nframe, cir->prob_N[A_dim[A_az]], A_az, deb_path_wav, "", 2, deb_pool_on);
		//write_wav_debug_multithread(cir->prob[A_dim[A_el]], cir_Nframe, cir->prob_N[A_dim[A_el]], A_el, deb_path_wav, "", 2, deb_pool_on);
		//write_wav_debug_multithread(cir->probdiff[A_dim[A_az]], cir_Nframe, cir->probdiff_N[A_dim[A_az]], A_az, deb_path_wav, "", 2, deb_pool_on);
		//write_wav_debug_multithread(cir->probdiff[A_dim[A_el]], cir_Nframe, cir->probdiff_N[A_dim[A_el]], A_el, deb_path_wav, "", 2, deb_pool_on);
		//write_wav_frame_debug_multithread(sph->x, sph_Nframe, sph_Naz, sph_Nel, (float)fsup, deb_path_wav, "", 2, deb_pool_on);
		//write_wav_frame_debug_multithread(corfup->x, ch_Ncrossbeam, frame_M, corfup_N, (float)fsup, deb_path_wav, "", 1, deb_pool_on);		
		//write_wav_frame_debug_multithread(corf2up->x, ch_Ncrossbeam, frame_M, corf2up_N, (float)fsup, deb_path_wav, "", 2, deb_pool_on);
		//write_wav_frame_debug_multithread(corfmup->x, ch_Nrecbeam, frame_M, corfmup_N, (float)fsup, deb_path_wav, "", 2, deb_pool_on);
		//write_wav_debug_multithread(fe->td, ch_Ncrossbeam, frame_M, (float)fs, deb_path_wav, "", 1000, deb_pool_on);
        //console_logf(PRIO_INFO, "Head orientation change ITD: %4.10f\n", fe->itd_diffang);
        //console_logf(PRIO_INFO, "Head orientation change ILD: %4.10f\n", fe->ild_diffang);
        //console_logf(PRIO_INFO, "head orientation change: %3.1f degree\n", fe->diffang);


    };

	void irf_to_fe_multithread(void* arg) {
		// get argument: frame number
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		int* arg_ptr = arg;
		int m = *arg_ptr;


		// calculate
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////

		// if frame m is active
		if(fra->m[m]){

			// initialize
			long timer_start;
			double timer_diff;

			// impulse response

			// caculate upsampled impulse response frames
			if (!up_fft) {
				timer_start = clock_tick();
				for (int ch = 0; ch < ch_Nrecbeam; ch++) {
					linupsample(irf->x[ch][m], irf_N, tmp_frame_upweight, up_N, irfup->x[ch][m]);
				}
				timer_diff = clock_stop(timer_start);
				console_logd(PRIO_TIMERDETAIL, "upsample frame: %4.4f ms\n", timer_diff);
			}
			// square impulse response to get energy impulse response
			if (frame_energy) {
				for (int ch = 0; ch < ch_Nrecbeam; ch++) {
					square(irfup->x[ch][m], irfup_N);
				}
			}

			// inter-microphone time differences
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////

			// calculate upsampled cross-correlation between two microphones
			if (td_on) {
				timer_start = clock_tick();
				for (int ch = 0; ch < ch_Ncrossbeam; ch++) {
					if (up_fft) {
						crosscorrelate_fft(irf->x[R_cross[ch][0]][m], irf_N, irf->x[R_cross[ch][1]][m], irf_N, corfup->x[ch][m], corfup_N, &fftirf[m], fft_library, cor_type, cor_phatlim_f1, cor_phatlim_f2, fsup, cor_normalized);
					}
					else {
						crosscorrelate_fft(irfup->x[R_cross[ch][0]][m], irfup_N, irfup->x[R_cross[ch][1]][m], irfup_N, corfup->x[ch][m], corfup_N, &fftirf[m], fft_library, cor_type, cor_phatlim_f1, cor_phatlim_f2, fsup, cor_normalized);
					}
					corf_to_tdmax(corfup->x[ch][m], corfup_N, corfup_Nlim[ch], fsup, &fe->td[ch][m], &fe->td_amp[ch][m]);
					td_to_ang(fe->td[ch][m], (float)corfup_Nlim[ch] / (float)fsup, &fe->td_ang[ch][m]);
					ang_to_angdiff(fe_p->td_ang[ch][m], fe->td_ang[ch][m], &fe->td_angdiff[ch][m]);
				}
				if (fe_tdld_on) {
					for (int ch = 0; ch < ch_Ncrossbeam; ch++) {
						ang_to_angdiff(fe_p->td_ang[ch][m], fe->td_ang[ch][m], &fe->td_angdiff[ch][m]);
					}
				}
				timer_diff = clock_stop(timer_start);
				console_logd(PRIO_TIMERDETAIL, "correlate frame: %4.4f ms\n", timer_diff);
			}

			// calculate upsampled normalized double cross-correlation between two microphones
			if (cor2_on) {
				timer_start = clock_tick();
				for (int ch = 0; ch < ch_Ncrossbeam; ch++) {
					//mono_normalize(corfup->x[ch][m], corfup_N);
					crosscorrelate_fft(corfup->x[ch][m], corfup_N, corfup_p->x[ch][m], corfup_N, corf2up->x[ch][m], corf2up_N, &fftirf[m], fft_library, "cor", cor_phatlim_f1, cor_phatlim_f2, fsup, cor_normalized);
					corf_to_tdmax(corf2up->x[ch][m], corf2up_N, corf2up_Nlim[ch], fsup, &fe->td2[ch][m], &fe->td2_amp[ch][m]);
					td_to_ang(fe->td2[ch][m], (float)corf2up_Nlim[ch] / fsup, &fe->td2_ang[ch][m]);
					ang_to_angdiff(fe_p->td2_ang[ch][m], fe->td2_ang[ch][m], &fe->td_angdiff[ch][m]);
				}
				if (fe_tdld_on) {
					for (int ch = 0; ch < ch_Ncrossbeam; ch++) {
						ang_to_angdiff(fe_p->td2_ang[ch][m], fe->td2_ang[ch][m], &fe->td_angdiff[ch][m]);
					}
				}
				timer_diff = clock_stop(timer_start);
				console_logd(PRIO_TIMERDETAIL, "double correlate frame: %4.4f ms\n", timer_diff);
			}

			// mono-microphone time differences
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////
			if (corm_on) {
				// calculate upsample cross-correlation between single microphone at 2 time points
				timer_start = clock_tick();
				for (int ch = 0; ch < ch_Nrecbeam; ch++) {
					if (up_fft) {
						crosscorrelate_fft(irf->x[ch][m], irf_N, irf_p->x[ch][m], irf_N, corfmup->x[ch][m], corfmup_N, &fftirf[m], fft_library, "cor", cor_phatlim_f1, cor_phatlim_f2, fsup, cor_normalized);
					}
					else {
						crosscorrelate_fft(irfup->x[ch][m], irfup_N, irfup_p->x[ch][m], irfup_N, corfmup->x[ch][m], corfmup_N, &fftirf[m], fft_library, "cor", cor_phatlim_f1, cor_phatlim_f2, fsup, cor_normalized);
					}
					corf_to_tdmax(corfmup->x[ch][m], corfmup_N, corfmup_Nlim - 1, fsup, &fe->tdm[ch][m], &fe->tdm_amp[ch][m]);
					sec_to_m(fe->tdm[ch][m], sos, &fe->tdm_distdiff[ch][m]);
				}
				timer_diff = clock_stop(timer_start);
				console_logd(PRIO_TIMERDETAIL, "double correlate frame: %4.4f ms\n", timer_diff);
			}



			// level differences
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// calculate level difference features
			if (ld_on) {
				timer_start = clock_tick();
				for (int ch = 0; ch < ch_Ncrossbeam; ch++) {
					irf_to_ld(irf->x[R_cross[ch][0]][m], irf->x[R_cross[ch][1]][m], irf_N, &fe->ld[ch][m]);
					ld_to_ang(fe->ld[ch][m], ld_ild_maxdb, &fe->ld_ang[ch][m]);
				}
				if (fe_tdld_on) {
					for (int ch = 0; ch < ch_Ncrossbeam; ch++) {
						ang_to_angdiff(fe_p->ld_ang[ch][m], fe->ld_ang[ch][m], &fe->ld_angdiff[ch][m]);
					}
				}
				timer_diff = clock_stop(timer_start);

				// merge level differences with time differences
				if (ld_merge_td) {
					timer_start = clock_tick();
					long corfup_Nlim_save = corfup_Nlim[ld_merge_window_corout];
					long correlation_limit_critical = (corfup_N - 1) / 2;

					// set new correlation limit if existing frame has been shrinked smaller than initial ITD correlation limit
					if (corfup_Nlim[ld_merge_window_corout] > correlation_limit_critical) {
						// ar_set(corfup->x[A_lr][m], corfup_N, 1.0f); // why? sets td correlation to 1, but actually not used anymore
						corfup_Nlim[ld_merge_window_corout] = correlation_limit_critical;
						//get_w_Ntdn_ang(corfup_N, fsup, corfup_Nlim[A_lr], sph_Ntd_ang, sph->w_Ntdn_ang[A_lr], true);
					}
					// calculate a synthetic normalized cross-correlation from level differences
					ang_windowed_in_cor(corfup->x[ld_merge_window_corout][m], corfup_N, corfup_Nlim[ld_merge_window_corout], fe->ld_ang[ld_merge_window_corin][m], tmp_ld_kaiser, ld_merge_window_N, ld_merge_window_cor);
					//mono_normalize(corfup->x[ld_merge_window_corout][m], corfup_N);
					//write_txt_debug(corfup->x[ld_merge_window_corout][m], corfup_N, false);

					// adapt features
					fe->td_ang[ld_merge_window_corout][m] = fe->ld_ang[ld_merge_window_corout][m];

					// reset correlation limit (might lead to error as in add_td_energy_to_sph we have the wrong reseted correlation limit for merge ild)
					corfup_Nlim[ld_merge_window_corout] = corfup_Nlim_save;
					timer_diff = clock_stop(timer_start);
				}

				// merge level differences with Gaussian


			}

			// circle
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////
			if (cir_on) {
				timer_start = clock_tick();
				add_cir_ref(cir, corfup->x[A_ff2][m], corfup->x[A_rr2][m], corfup->x[A_ll2][m], m, fe->td_ang[A_rr2][m], cir_binarymax_instead_of_y, cir_flip_y_plane);
				add_cir_diff(cir, corfmup->x[A_l][m], corfmup->x[A_l][m], corfmup->x[A_l][m], &fftirf[m], fft_library, m, cir_delta_instead_of_diff, cir_delta_instead_of_diff);
				get_cir_head_pos(cir, head, m, cir_diffmax_instead_of_diffmean, cir_delta_instead_of_diff);
				timer_diff = clock_stop(timer_start);
				//write_txt_debug2d(head->pos[0], frame_M, A_Ndof, 0);
				//write_txt_debug2d(head->pos_var[0], frame_M, A_Ndof, 0);
			}

			// sphere
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////
			if (sph_on) {
				timer_start = clock_tick();
				// add energy in time delay to sphere
				//add_td_energy_to_sph_select(corfup->x[A_lr][m], corfup_N, fsup, corfup_Nlim[A_lr], sph->x[m], sph->xtmp[m], sph->map_tdn_sph[A_lr],  sph->map_tdn_ang[A_ll2], sph->w_Ntdn_ang[A_lr], sph->w_Ntd_ang_sol[A_lr], sph->w_Nsphn[A_lr], sph_Naz, sph_Nel, sph_limit, (int)roundf(fe->td_ang[A_lr][m]), sph_orthogonal, false);
				add_td_energy_to_sph_select(corfup->x[A_ll2][m], corfup_N, fsup, corfup_Nlim[A_ll2], sph->x[m], sph->xtmp[m], sph->map_tdn_sph[A_ll2], sph->map_tdn_ang[A_ll2], sph->w_Ntdn_ang[A_ll2], sph->w_Ntd_ang_sol[A_ll2], sph->w_Nsphn[A_ll2], sph_Naz, sph_Nel, sph_limit, (int)roundf(fe->td_ang[A_ll2][m]), sph_orthogonal, true);
				add_td_energy_to_sph_select(corfup->x[A_rr2][m], corfup_N, fsup, corfup_Nlim[A_rr2], sph->x[m], sph->xtmp[m], sph->map_tdn_sph[A_rr2], sph->map_tdn_ang[A_ll2], sph->w_Ntdn_ang[A_rr2], sph->w_Ntd_ang_sol[A_rr2], sph->w_Nsphn[A_rr2], sph_Naz, sph_Nel, sph_limit, (int)roundf(fe->td_ang[A_rr2][m]), sph_orthogonal, true);
				add_td_energy_to_sph_select(corfup->x[A_ff2][m], corfup_N, fsup, corfup_Nlim[A_ff2], sph->x[m], sph->xtmp[m], sph->map_tdn_sph[A_ff2], sph->map_tdn_ang[A_ll2], sph->w_Ntdn_ang[A_ff2], sph->w_Ntd_ang_sol[A_ff2], sph->w_Nsphn[A_ff2], sph_Naz, sph_Nel, sph_limit, (int)roundf(fe->td_ang[A_ff2][m]), sph_orthogonal, true);
				// get maximum angle of sphere
				get_sph_max_az_el(sph->x[m], &head->pos[m][A_dim[A_az]][A_ref], &head->pos[m][A_dim[A_el]][A_ref], &head->pos_var[m][A_dim[A_az]][A_ref], &head->pos_var[m][A_dim[A_el]][A_ref], sph_ang_stepsize, sph_Naz, sph_Nel, sph_Nmax);
				timer_diff = clock_stop(timer_start);

				// get global sphere
				if (sph_glob_on) {
					//float value = (float)(frame_M - m); // frame number
					//float value = -1/logf(fe->td_amp[A_ll2][m] * fe->td_amp[A_rr2][m] * fe->td_amp[A_ff2][m]) + 1; // inverse correlation quality
					//float value = fe->td_amp[A_ll2][m] * fe->td_amp[A_rr2][m] * fe->td_amp[A_ff2][m]; // correlation quality
					//value = value * 1.0f/(logf(sqrtf((head->ang_az_p[m] - head->ang_az[m])*(head->ang_az_p[m] - head->ang_az[m]) + (head->ang_el_p[m] - head->ang_el[m])*(head->ang_el_p[m] - head->ang_el[m])) + 2.0f)); // inverse change of maximum position
					sample_sd_moving(head->pos[m][A_dim[A_az]][A_ref], fe->sdbuffer_az[m], fe_NVar, &fe->sd_az[m]);
					sample_sd_moving(head->pos[m][A_dim[A_el]][A_ref], fe->sdbuffer_el[m], fe_NVar, &fe->sd_el[m]);
					float value = 1.0f / fe->sd_az[m] * fe->sd_el[m]; // inverse standard deviation of maximum position
					value = isinf(value) ? 1.0f : value;
					set_sph_max_n(sph->x[m], sph->x[frame_M], sph_Naz, sph_Nel, value);
				}
				console_logd(PRIO_TIMERDETAIL, "spherical calculation frame: %4.4f ms\n", timer_diff);

			}

			// energy metrics
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////
			if (fe_energy_on) {
				// calculate average energy in each channel
				for (int ch = 0; ch < ch_Nrecbeam; ch++) {
					get_average_energy_per_sample(irf->x[ch][m], irf_N, &fe->energy[ch][m]);
				}

				// caculate gain in the beamformer microphone channels (compared to single microphone channel)
				for (int ch = 0; ch < beam_Nch; ch++) {
					get_array_gain(irf->x[ch_Nrec + ch][m], irf->x[ch_Nrec + ch + beam_Nang][m], irf_N, &fe->beam_gain[ch][m]);
				}
			}

			// frame metrics
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////
			tdamp_to_frame_corpeak(fra, fe->td_amp, ch_Ncrossbeam, m); // currently framebased not multichannel
			el_to_frame_el(fra, head, m);
			// debugging
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////

			// beamformer gain
			if (beam_on) {
				for (int ch = 0; ch < beam_Nch; ch++) {
					console_logs(PRIO_BEAMDETAIL, "%s mic, ", (ch == 0) ? "left" : "right");
					console_logi(PRIO_BEAMDETAIL, "frame %i ", m);
					console_logd(PRIO_BEAMDETAIL, "sound pressure gain: %4.1f dB\n", fe->beam_gain[ch][m] * 2);
				}
			}
		}
	}

	void fe_to_slam() {
		// init
		float lr_cm;
		//float l2r2_cm;
		float ll2_cm;
		float rr2_cm;
		//float lr2_cm;
		//float l2r_cm;

		// calculate

		// update hyperbolic localization algorithm
		if(loc_on) {
			for (int m = 0; m < frame_M; m++) {
				lr_cm = sec_to_cm(fe->td[A_lr][m], sos);
				//l2r2_cm = sec_to_cm(fe->td[A_l2r2][m], sos);
				ll2_cm = sec_to_cm(fe->td[A_ll2][m], sos);
				rr2_cm = sec_to_cm(fe->td[A_rr2][m], sos);
				//lr2_cm = sec_to_cm(fe->td[A_lr2][m], sos);
				//l2r_cm = sec_to_cm(fe->td[A_l2r][m], sos);
				sec_to_m(fe->td[A_lr][m], sos, &loc->Rij[0]);  // ti(1) - ti(2) //fe->td_lr[m]; //  -0.1600 // 0.00029102855f -> 0.7286, 0.72921, 1.03003
				sec_to_m(fe->td[A_ll2][m], sos, &loc->Rij[1]); // ti(1) - ti(3) //fe->td_ll2[m]; // -0.0001760701083244332 // 0.00001400752f 
				//sec_to_m(fe->td[A_l2r][m], sos, &loc->Rij[2]); // ti(3) - ti(2) //fe->td_l2r[m]; // -0.159823929891676// 0.00027702102f
				//sec_to_m(fe->td[A_l2r2][m], sos, &loc->Rij[3]); // ti(3) - ti(4); // -0.15973919476455 // 0.00029202379f 
				update_localization(loc);
				//float *a = loc->xyz2sol[0];
				memcpy(fe->loc_ang[m], loc->xyz2sol[0], loc_Ndim * sizeof(*loc->xyz2sol[0]));
				memcpy(fe->loc_ang2sol[0][m], loc->xyz2sol[0], loc_Ndim * sizeof(*loc->xyz2sol[0]));
				memcpy(fe->loc_ang2sol[1][m], loc->xyz2sol[1], loc_Ndim * sizeof(*loc->xyz2sol[1]));
				console_logi(PRIO_LOCALIZATION, "frame %i ", m);
				console_logf3d(PRIO_LOCALIZATION, "3D localization: x = %4.4f, y = %4.4f, z = %4.4f\n", fe->loc_ang[m][0], fe->loc_ang[m][1], fe->loc_ang[m][2]);
			}
		}

		// correct absolute 2 DOF acoustic head orientation observation
		if (kal_on || kal6_on || par6_on) {
			for (int m = 0; m < frame_M; m++) {
				head->pos[m][A_dim[A_az]][A_ref] = isinf(head->pos[m][A_dim[A_az]][A_ref]) || isnan(head->pos[m][A_dim[A_az]][A_ref]) ? 0.0f : head->pos[m][A_dim[A_az]][A_ref];
				head->pos[m][A_dim[A_el]][A_ref] = isinf(head->pos[m][A_dim[A_el]][A_ref]) || isnan(head->pos[m][A_dim[A_el]][A_ref]) ? 0.0f : head->pos[m][A_dim[A_el]][A_ref];
				//ang_sph_to_ipc(head->ang_az[m], head->ang_el[m], &head->angipc_az[m], &head->angipc_el[m]);
			}
		}
		// get differential mono-microphone acoustic head orientation observation


		// get absolute and differential inter-aural acoustic head orientation observation
		//float y0 = (fe->td_ang[A_lr][0] + fe->td_ang[A_lr][0] + fe->td_ang[A_lr][0] + fe->td_ang[A_lr][0]) / 4.0f;
		//float y1 = (float)copysignl((double)((fabsf(fe->td_angdiff[A_lr][0]) + fabsf(fe->td_angdiff[A_lr][0]))/2.0f), (float)(fe->td_ang[A_lr][0] - fe_p->td_ang[A_lr][0])); 
		//float y2 = 0.0f;



	}


	void slam_to_pos() {

		// init

		// update kalman acoustic 1 DOF direct sound filter (azimuth)
		if (kal_on) {
			kalaz->y[0] = head->pos[0][A_dim[A_az]][A_ref];
			kalaz->y[1] = 0.0f;
			kalaz->y[2] = 0.0f;
			update_kalman_1dof(kalaz);
		}
		// update kalman acoustic 1 DOF direct sound filter (elevation)
		if (kal_on) {
			kalel->y[0] = head->pos[0][A_dim[A_el]][A_ref];
			kalel->y[1] = 0.0f;
			kalel->y[2] = 0.0f;
			update_kalman_1dof(kalel);
		}

		// update kalman acoustic/inertial 6 DOF filter (x, y, z, yaw, pitch, roll)
		if (kal6_on) {
			update_kalman_6dof(kal6);
		}

		// update particle slam filter
		if (par6_on) {
			long timer_start = clock_tick();
			assign_particleslam_6dof(par6, head->pos, imu->pos);
			update_particleslam_6dof(par6);
			double timer_diff = clock_stop(timer_start);
			console_logf(PRIO_MAIN, "Slam update rate: %3.3f ms\n", timer_diff);
		}
	};

	void save_state() {

		// save selected state for next update routine
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		for (int ch = 0; ch < ch_Nrec; ch++) {
			if (corm_on) {
				for (int m = 0; m < frame_M; m++) {
					if (up_fft) {
						memcpy(irf_p->x[ch][m], irf->x[ch][m], irf_N * sizeof(float));
					}
					else {
						memcpy(irfup_p->x[ch][m], irfup->x[ch][m], irfup_N * sizeof(float));
					}
				}
			}
		}
		for (int ch = 0; ch < ch_Ncrossbeam; ch++) {
			if (fe_tdld_on) {
				memcpy(fe_p->td_ang[ch], fe->td_ang[ch], frame_M * sizeof(float));
				memcpy(fe_p->ld_ang[ch], fe->ld_ang[ch], frame_M * sizeof(float));
			}
			if (cor2_on) {
				memcpy(fe_p->td2_ang[ch], fe->td2_ang[ch], frame_M * sizeof(float));
				for (int m = 0; m < frame_M; m++) {
					memcpy(corfup_p->x[ch][m], corfup->x[ch][m], corfup_N * sizeof(float));
				}
			}
			if (cir_on) {
				for (int d = 0; d < A_Ndof; d++) {
					for (int m = 0; m < frame_M; m++) {
						memcpy(cir->prob_p[d][m], cir->prob[d][m], cir->prob_N[d] * sizeof(float));
					}
					memcpy(cir->prob_norm_p[d], cir->prob_norm[d], cir_Nframe * sizeof(float));
				}
			}
		}

		for (int m = 0; m < frame_M; m++) {
			for (int d = 0; d < A_Ndof; d++) {
				head->pos_p[m][d] = head->pos[m][d][A_ref];
			}
		}

		// reset energy in sphere to 0
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (sph_on) {
			float reset_value = sph_orthogonal ? 1.0f : 1.0f;
			for (int m = 0; m < frame_M; m++) {
				reset_sph(sph->x[m], sph_Naz, sph_Nel, reset_value);
			}
			if (sph_glob_on) {
				reset_sph(sph->x[frame_M], sph_Naz, sph_Nel, 0.0f);
			}
		}
	};

	void finish_update() {
		// update statistics
		update_nr += 1;
		update_first = false;

		// update timer telling how long update took (maximal close to particle slam filter)
		timerdiff_update(&update_Tstart, &update_Tdiff, update_Tdiffmax);
		update_algo[A_rate_ms] = update_Tdiff * 1000.0f;

		// symbolize new iteration
		console_log(PRIO_STRUCTURE, "---------------------------------------------\n");
	}





#ifdef __cplusplus
};
#endif