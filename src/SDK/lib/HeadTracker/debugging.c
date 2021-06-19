// Copyright 2020, Felix Pfreundtner, All rights reserved.
#include "debugging.h"

void write_txt(float* x, long N, char* filepath){
	if (deb_on && deb_disk) {
		//long timer_start = clock_tick();

		// init
		char path[path_N];
		strcpy(path, filepath);

		// write
		FILE* fp = fopen(path, "w+");
		if (fp) {
			for (long n = 0; n < N; n++) {
				fprintf(fp, "%4.10f\n", x[n]);
			}
			fclose(fp);
		}
		//double timer_diff = clock_stop(timer_start);
	}
};

void write_txt2d(float* x, long N, char* filepath, bool newfile){
	if (deb_on && deb_disk) {
		//long timer_start = clock_tick();

		// init
		char path[path_N];
		strcpy(path, filepath);

		// write
		FILE* fp;
		if (newfile) {
			fp = fopen(path, "w+");
		}
		else {
			fp = fopen(path, "a");
		}

		if (fp) {
			for (long n = 0; n < N; n++) {
				fprintf(fp, "%4.10f ", x[n]);
			}
			fprintf(fp, "\n");
			fclose(fp);
		}
		//double timer_diff = clock_stop(timer_start);

	}
};



void write_txt_complex(struct cpx *x, long N, char* filepath){
	if (deb_on && deb_disk) {
		//long timer_start = clock_tick();

		// init
		char path[path_N];
		strcpy(path, filepath);

		// write
		FILE* fp = fopen(path, "w+");
		if (fp) {
			for (long n = 0; n < N; n++) {
				fprintf(fp, "%4.10f %4.10f\n", x[n].real, x[n].imag);
				// init
			}
			fclose(fp);
		}
		//double timer_diff = clock_stop(timer_start);

	}
};


void write_txt_double(double* x, long N, char* filepath){
	if (deb_on && deb_disk) {
		// init
		char path[path_N];
		strcpy(path, filepath);

		// write
		FILE* fp = fopen(path, "w+");
		if (fp) {
			for (long n = 0; n < N; n++) {
				fprintf(fp, "%4.10f\n", (float)x[n]);
			}
			fclose(fp);
		}
	}
};

void write_txt_long(long* x, long N, char* filepath){
	if (deb_on && deb_disk) {
		// init
		char path[path_N];
		strcpy(path, filepath);

		// write
		FILE* fp = fopen(path, "w+");
		if (fp) {
			for (long n = 0; n < N; n++) {
				fprintf(fp, "%4.10f\n", (float)x[n]);
			}
			fclose(fp);
		}
	}
};


void write_txt_debug(float* x, long N, bool normalize){
	if (deb_on && deb_disk) {
		if (normalize) {
			float *x_norm = (float*)malloc(N * sizeof(float));
			if (x_norm) {
				ar_norm(x, N, x_norm);
				write_txt(x_norm, N, deb_path_text);
				free(x_norm);
			}
		}
		else {
			write_txt(x, N, deb_path_text);
		}
	}
};

void write_txt_debug2dmultifile(float** x, long M, long N, bool normalize){
	if (deb_on && deb_disk) {
		for (long m = 0; m < M; m++) {
			// set filename based on channel information M
			char filename[20];
			sprintf(filename, "%d", m + 1);

			// set output path
			size_t len_folder = strlen(deb_multichannel_folder);
			size_t len_filename = strlen(filename);

			char* path_m = malloc(len_folder + len_filename + 1); /* one for extra char, one for trailing zero */
			if (path_m) {
				strcpy(path_m, deb_multichannel_folder);
				strcpy(path_m + len_folder, filename);
				//char control_path[135];
				//strcpy(control_path, path);

				// write file
				if (normalize) {
					float* x_norm = (float*)malloc(N * sizeof(float));
					if (x_norm) {
						ar_norm(x[m], N, x_norm);
						write_txt(x_norm, N, path_m);
						free(x_norm);
					}
				}
				else {
					write_txt(x[m], N, path_m);
				}
				free(path_m);
			}
		}
	}
};


void write_txt_debug2d(float** x, long M, long N, bool normalize){
	if (deb_on && deb_disk) {
		for (long m = 0; m < M; m++) {
			//char control_path[135];
			//strcpy(control_path, path);
			bool newfile = false;
			if (m == 0) {
				newfile = true;
			}
			// write file
			if (normalize) {
				float *x_norm = (float*)malloc(N * sizeof(float));
				if (x_norm) {
					ar_norm(x[m], N, x_norm);
					write_txt2d(x_norm, N, deb_path_text, newfile);
					free(x_norm);
				}
			}
			else {
				write_txt2d(x[m], N, deb_path_text, newfile);
			}
		}
	}
};

void write_txt_debug_stereo(float* x_l, float *x_r, long N, bool normalize){
	if (deb_on && deb_disk) {
		if (normalize) {
			binaural_normalize(x_l, x_r, N);
		}
		//long timer_start = clock_tick();
		struct cpx* x = (struct cpx*)malloc(N * sizeof(struct cpx));
		if (x) {
			for (long n = 0; n < N; n++) {
				x[n].real = x_l[n];
				x[n].imag = x_r[n];
			}
		}
		//double timer_diff = clock_stop(timer_start);

		write_txt_debug_complex(x, N, 0);
		free(x);
	}
};

void write_txt_debug_complex(struct cpx *x, long N, bool normalize){
	if (deb_on && deb_disk) {
		if (normalize) {
			struct cpx *x_norm = (struct cpx*)malloc(N * sizeof(struct cpx));
			if (x_norm) {
				memcpy(x_norm, x, N * sizeof(struct cpx));
				cpx_normalize(x_norm, N);
				write_txt_complex(x_norm, N, deb_path_text);
				free(x_norm);
			}
		}
		else {
			write_txt_complex(x, N, deb_path_text);
		}
	}
};



void write_txt_debug_double(double* x, long N){
	if (deb_on && deb_disk) {
		write_txt_double(x, N, deb_path_text);
	}
};

void write_txt_debug_long(long* x, long N){
	if (deb_on && deb_disk) {
		write_txt_long(x, N, deb_path_text);
	}
};

void write_wav_debug(float** x, int Nchannels, long Nsamples, float sampling_rate, const char* filepath, const char* metadata, long flag) {
	if (deb_on && deb_disk) {
		if (flag != 0) {
				float **x_norm = (float**)malloc(Nchannels * sizeof(float*));
				if (x_norm) {
					for (int m = 0; m < Nchannels; m++) {
						x_norm[m] = (float*)malloc(Nsamples * sizeof(float));
						if (x_norm[m]) {
							memcpy(x_norm[m], x[m], Nsamples * sizeof(float));
							if (flag == 2) {
								ar_norm(x[m], Nsamples, x_norm[m]);
							}
							if (flag >= 10) {
								ar_mvalue(x[m], Nsamples, (float)flag, x_norm[m]);
							}
						}
					}
					if (flag == 1) {
						multichannel_normalize(x_norm, Nsamples, Nchannels);
					}
					if (flag == 4) {
						sampling_rate = multichannel_normalize_norm(x_norm, Nsamples, Nchannels);
						sampling_rate = sampling_rate == 0.0f ? 1.0f : sampling_rate;
					}
					JuceCWrapperWAVWrite(x_norm, Nchannels, Nsamples, filepath, metadata, (double)sampling_rate);
					for (int m = 0; m < Nchannels; m++) {
						if (x_norm[m]) {
							free(x_norm[m]);
						}
					}
					free(x_norm);
				}
		}
		else {
			JuceCWrapperWAVWrite(x, Nchannels, Nsamples, filepath, metadata, (double)sampling_rate);
		}
	}
}

void write_wav_frame_debug(float*** x, int Nchannels, int Nframes, long Nsamples, float sampling_rate, const char* filepath, const char* metadata, long flag) {
	int ch, m; // channel, frame
	long s, n, nshift; // sample, 1d frame-sample (new), shift of 1d frame-sample
	if (deb_on && deb_disk) {
		float** xnew = (float**)malloc(Nchannels * sizeof(float*));
		if (xnew) {
			// restructure array
			long xnew_Nsamples = Nframes * Nsamples;
			for (ch = 0; ch < Nchannels; ch++) {
				xnew[ch] = (float*)malloc(xnew_Nsamples * sizeof(float));
				if (xnew[ch]) {
					for (m = 0; m < Nframes; m++) {
						nshift = Nsamples * m;
						for (s = 0; s < Nsamples; s++) {
							n = nshift + s;
							xnew[ch][n] = x[ch][m][s];
						}
					}
				}
			}

			// flag
			if (flag == 3) {
				for (ch = 0; ch < Nchannels; ch++) {
					if (xnew[ch]) {
						for (m = 0; m < Nframes; m++) {
							mono_normalize(xnew[ch] + Nsamples * m, Nsamples);
						}
					}
				}
				flag = 0;
			}

			// write wav
			write_wav_debug(xnew, Nchannels, xnew_Nsamples, sampling_rate, filepath, metadata, flag);

			// free memory
			for (ch = 0; ch < Nchannels; ch++) {
				if (xnew[ch]) {
					free(xnew[ch]);
				}
			}

			free(xnew);
		}
	}

};

void write_particleslam_debug(struct par6s* p, const char* channel,  const char* frame, const char* sample, int Nchannels, int Nframes, int Nsamples, int flag) {
	if (deb_on && deb_disk) {
		// read
		
		// init
		///////////////////////////////////

		// init variables
		int ch, m; // channel, frame
		long s, n, nshift; // sample, 1d frame-sample (new), shift of 1d frame-sample
		long xnew_Nsamples = Nframes * Nsamples;
		float x_abs;
		float* x_info;
		bool dim1d = (Nchannels == 1 && Nframes == 1) ? true : false;
		bool dim2d = (Nchannels == 1 && Nframes != 1) ? true : false;
		bool dim3d = (Nchannels != 1 && Nframes != 1 && Nsamples != 1) ? true : false;
		int Ninfo = 4;
		char* filepath_info;
		float x_abs_max = 0.0f;
		bool info_txt = flag == 1 ? true : false;
		int Nfilepathparts = info_txt ? 5 : 4;
		size_t id = 0;
		n = 0;
		nshift = 0;
		const char** filepathparts = (const char**)malloc(sizeof(const char*) * Nfilepathparts);
		filepathparts[0] = channel;
		filepathparts[1] = frame;
		filepathparts[2] = sample;
		filepathparts[3] = ".wav";
		char* filepath = malloc(strlen(channel) + strlen(frame) + strlen(sample) + strlen(".wav") + Ninfo - 1 + 1);
		float** xnew = ar_declare2d(Nchannels, xnew_Nsamples);


		// get debugging ids
		///////////////////////////////////

		
		// select channel type
		if (!strcmp(channel, "obs")) {
			// select frame type
			if (!strcmp(frame, "kalx_dof")) {
				// select sample type
				if (!strcmp(sample, "kalx_der")) {
					id = 1;
				}
			}
		}

		// select channel type
		if (!strcmp(channel, "obs")) {
			// select frame type
			if (!strcmp(frame, "kaly_dof")) {
				// select sample type
				if (!strcmp(sample, "kaly_der")) {
					id = 2;
				}
			}
		}
		// select channel type
		if (!strcmp(channel, "")) {
			// select frame type
			if (!strcmp(frame, "obs")) {
				// select sample type
				if (!strcmp(sample, "dof_weight")) {
					id = 3;
				}
			}
		}

		// select channel type
		if (!strcmp(channel, "")) {
			// select frame type
			if (!strcmp(frame, "dof")) {
				// select sample type
				if (!strcmp(sample, "obs_pdfcdf")) {
					id = 4;
				}
			}
		}

		// select channel type
		if (!strcmp(channel, "")) {
			// select frame type
			if (!strcmp(frame, "dof")) {
				// select sample type
				if (!strcmp(sample, "par_pdfcdf")) {
					id = 5;
				}
			}
		}

		// select channel type
		if (!strcmp(channel, "par")) {
			// select frame type
			if (!strcmp(frame, "pos_dof")) {
				// select sample type
				if (!strcmp(sample, "pos_der")) {
					id = 6;
				}
			}
		}

		// select channel type
		if (!strcmp(channel, "par")) {
			// select frame type
			if (!strcmp(frame, "dof")) {
				// select sample type
				if (!strcmp(sample, "obs_kalx_ref")) {
					id = 7;
				}
			}
		}

		// select channel type
		if (!strcmp(channel, "dof")) {
			// select frame type
			if (!strcmp(frame, "frame")) {
				// select sample type
				if (!strcmp(sample, "prob")) {
					id = 8;
				}
			}
		}

		// select channel type
		if (!strcmp(channel, "dof")) {
			// select frame type
			if (!strcmp(frame, "frame")) {
				// select sample type
				if (!strcmp(sample, "probdiff")) {
					id = 9;
				}
			}
		}

		// select channel type
		if (!strcmp(channel, "")) {
			// select frame type
			if (!strcmp(frame, "obs")) {
				// select sample type
				if (!strcmp(sample, "weight")) {
					id = 10;
				}
			}
		}
		// select frame type
		if (!strcmp(frame, "obskalx")) {
			// select sample type
			if (!strcmp(sample, "dof")) {
				id = 11;
			}
		}
		if (!strcmp(frame, "parpos")) {
			// select sample type
			if (!strcmp(sample, "dof")) {
				id = 12;
			}
		}		
		if (!strcmp(frame, "parpdfcdf")) {
			// select sample type
			if (!strcmp(sample, "dof")) {
				id = 13;
			}
		}


		// get data from structs based in debugging id
		///////////////////////////////////
		for (ch = 0; ch < Nchannels; ch++) {
			for (m = 0; m < Nframes; m++) {
				if (dim2d ||dim3d) {
					nshift = Nsamples * m;
				}
				for (s = 0; s < Nsamples; s++) {
					if (dim1d) {
						n = s;
					}
					if (dim2d ||dim3d) {
						n = nshift + s;
					}
					// select and assign value
					switch (id) {
						case 1: xnew[ch][n] = p->obs[ch].kal[m].x[s]; break;
						case 2: xnew[ch][n] = p->obs[ch].kal[m].y[s]; break;
						case 3: xnew[ch][n] = p->obs[m].weight[s]; break;
						case 4: xnew[ch][n] = ch == 0 ? p->obs_pdf[m][s] : p->obs_cdf[m][s] ; break;
						case 5: xnew[ch][n] = ch == 0 ? p->par_pdf[m][s] : p->par_cdf[m][s] ; break;
						case 6: xnew[ch][n] = p->par[ch].pos[m].x[s]; break;
						case 7: xnew[ch][n] = p->par[ch].kal[m][s].x[A_ref]; break;
						case 8: xnew[ch][n] = s < cir->prob_N[ch] ? cir->prob[ch][m][s] : 0.0f; break;
						case 9: xnew[ch][n] = s < cir->probdiff_N[ch] ? cir->probdiff[ch][m][s] : 0.0f; break;
						case 10: xnew[ch][n] = p->obs_weight[m]; break;
						case 11: xnew[ch][n] = p->obs[ch].kal[m].x[s]; break;
						case 12: xnew[ch][n] = p->par[ch].pos[m].x[s]; break;
						case 13: xnew[ch][n] = ch == 0 ? p->par_pdf[m][s] : p->par_cdf[m][s] ; break;
					}
					//if (is_unregular_float(&xnew[ch][n], true, false)) {
						//int aaa = 0;
					//}
					// get maximum
					x_abs = myAbs(xnew[ch][n]);
					if (x_abs > x_abs_max) {
						x_abs_max = x_abs;
					}
				}
			}
		}
		if (x_abs_max > 1.0f) {
			// normalize data to fit in wav file format
			ar_divide2d(xnew, Nchannels, xnew_Nsamples, x_abs_max);
		}
		else {
			// do not write gain factor to output, as no normalization occured
			x_abs_max = 1.0f;
		}
		if (x_abs_max == 0.0f) {
			x_abs_max = 1.0f;
		}

		// set filepath
		char_concatenatemulti(filepathparts, Nfilepathparts - 1, "-", filepath);

		// write wav byte data
		write_wav_debug(xnew, Nchannels, xnew_Nsamples, roundf(x_abs_max), filepath, "", 0);
		
		// write txt info
		if (info_txt) {
			// init
			Ninfo = 4;
			filepath_info = malloc(strlen(channel) + strlen(frame) + strlen(sample) + strlen("-info") + strlen(".txt") + Ninfo + 1);
			x_info = ar_declare(Ninfo);

			// get data
			x_info[0] = (float)Nchannels;
			x_info[1] = (float)Nframes;
			x_info[2] = (float)Nsamples;
			x_info[3] = x_abs_max;

			// set filepath
			filepathparts[3] = "info";
			filepathparts[4] = ".txt";
			char_concatenatemulti(filepathparts, Nfilepathparts, "-", filepath_info);

			// write info txt
			write_txt(x_info, Ninfo, filepath_info);

			// free memory
			free(filepath_info);
			free(x_info);
		}

		
		// free memory
		ar_free2d(xnew, Nchannels, xnew_Nsamples);
		free(filepath);
		free((char**)filepathparts); // (no free on filepathparts 2d array content as the content are pointers towards a local char variable which get's automatically freed at end of function)
	}
}

void write_wav_debug_multithread(float** x, int Nchannels, long Nsamples, float sampling_rate, const char* filepath, const char* metadata, long flag, bool multithread_on) {
	struct debs* d;

	if (deb_on && deb_disk) {
		// get free debugging struct
		d = get_deb_struct(deb, deb_Nmax);

		// write parameters to debugging struct
		d->x2d = x;
		d->Nchannels = Nchannels;
		d->Nsamples = Nsamples;
		d->sampling_rate = sampling_rate;
		strcpy(d->filepath, filepath);
		strcpy(d->metadata, metadata);
		d->flag = flag;

		// debug single threaded or multithreaded
		single_update_thread_or_serial(pool, write_wav_debug_struct, d, multithread_on, false);
	}
}

void write_wav_frame_debug_multithread(float*** x, int Nchannels, int Nframes, long Nsamples, float sampling_rate, const char* filepath, const char* metadata, long flag, bool multithread_on) {
	struct debs* d;

	if (deb_on && deb_disk) {
		// get free debugging struct
		d = get_deb_struct(deb, deb_Nmax);

		// write parameters to debugging struct
		d->x3d = x;
		d->Nchannels = Nchannels;
		d->Nframes = Nframes;
		d->Nsamples = Nsamples;
		d->sampling_rate = sampling_rate;
		strcpy(d->filepath, filepath);
		strcpy(d->metadata, metadata);
		d->flag = flag;

		// debug single threaded or multithreaded
		single_update_thread_or_serial(pool, write_wav_frame_debug_struct, d, multithread_on, false);
	}

}

void write_particleslam_debug_multithread(struct par6s* p, const char* channel, const char* frame, const char* sample, int Nchannels, int Nframes, int Nsamples, int flag, bool multithread_on) {
	struct debs* d;

	if (deb_on && deb_disk) {
		// get free debugging struct
		d = get_deb_struct(deb, deb_Nmax);

		// write parameters to debugging struct
		d->p = p;
		strcpy(d->channel, channel);
		strcpy(d->frame, frame);
		strcpy(d->sample, sample);
		d->Nchannels = Nchannels;
		d->Nframes = Nframes;
		d->Nsamples = Nsamples;
		d->flag = flag;

		// debug single threaded or multithreaded
		single_update_thread_or_serial(pool, write_particleslam_debug_struct, d, multithread_on, false);
	}

}


void write_wav_debug_struct(struct debs* d) {
	if (deb_on && deb_disk) {

		// call conventional debug function
		write_wav_debug(d->x2d, d->Nchannels, d->Nsamples, d->sampling_rate, d->filepath, d->metadata, d->flag);

		// leave debugging struct
		leave_deb_struct(d);
	}
}

void write_wav_frame_debug_struct(struct debs* d) {
	if (deb_on && deb_disk) {

		// call conventional debug function
		write_wav_frame_debug(d->x3d, d->Nchannels, d->Nframes, d->Nsamples, d->sampling_rate, d->filepath, d->metadata, d->flag);

		// leave debugging struct
		leave_deb_struct(d);
	}
}

void write_particleslam_debug_struct(struct debs* d) {
	if (deb_on && deb_disk) {

		// call conventional debug function
		write_particleslam_debug(d->p, d->channel, d->frame, d->sample, d->Nchannels, d->Nframes, d->Nsamples, d->flag);

		// leave debugging struct
		leave_deb_struct(d);
	}
}

struct debs* get_deb_struct(struct debs** dd, int deb_Nmax) {
	struct debs* d;
	int n;

	for (n = 0; n < deb_Nmax; n++) {
		if (dd[n]->open) {
			dd[n]->open = false;
			break;
		}
	}
	d = dd[n];
	return d;
}

void leave_deb_struct(struct debs* d) {
	d->open = true;
}

long clock_tick() {
	long timer = 0l;
	if (deb_timer) {
		timer = timer_tick();
	}

	return timer;

};

double clock_stop(long start) {
	double timer = 0.0;
	if (deb_timer) {
		timer = timer_stop(start);
	}
	return timer;

};


double clock_diff(long start, long stop) {
	double time_diff_ms = 0.0;
	if (deb_timer) {
		time_diff_ms = timer_diff(start, stop);
	}
	return time_diff_ms;
};

void console_add() {
	if (deb_on && deb_console) {
		#if(FRAMEWORK_WINDOWSUSED == 1)
				// add console for debugging
				FILE* pConsole;
				AllocConsole();
				pConsole = freopen("CONOUT$", "wb", stdout); // _s, &pConsole

		#endif
	}
}


void console_logdyn(int priority, const char *format, ...){
	if (deb_on && deb_console) {

		if (priority & PRIO_LOG) {
			va_list args;
			va_start(args, format);
			if (priority & PRIO_TIMERDETAIL) {
				printf("      ");
				vprintf(format, args);
			}
			else {
				vprintf(format, args);
			}
			va_end(args);
		}
	}
}

void console_logd(int priority, const char *format, double arg){
	if (deb_on && deb_console) {
		if (priority & PRIO_LOG) {
			if (priority & PRIO_TIMERDETAIL) {
				printf("      ");
				printf(format, arg);
			}
			else {
				printf(format, arg);
			}
		}
	}
}

void console_logf(int priority, const char *format, float arg){
	if (deb_on && deb_console) {
		if (priority & PRIO_LOG) {
			if (priority & PRIO_TIMERDETAIL) {
				printf("      ");
				printf(format, arg);
			}
			else {
				printf(format, arg);
			}
		}
	}
}

void console_logf3d(int priority, const char *format, float arg1, float arg2, float arg3){
	if (deb_on && deb_console) {
		if (priority & PRIO_LOG) {
			if (priority & PRIO_TIMERDETAIL) {
				printf("      ");
				printf(format, arg1, arg2, arg3);
			}
			else {
				printf(format, arg1, arg2, arg3);
			}
		}
	}
}
void console_logl(int priority, const char *format, long arg){
	if (deb_on && deb_console) {
		if (priority & PRIO_LOG) {
			if (priority & PRIO_TIMERDETAIL) {
				printf("      ");
				printf(format, arg);
			}
			else {
				printf(format, arg);
			}
		}
	}
}


void console_logi(int priority, const char *format, int arg){
	if (deb_on && deb_console) {
		if (priority & PRIO_LOG) {
			if (priority & PRIO_TIMERDETAIL) {
				printf("      ");
				printf(format, arg);
			}
			else {
				printf(format, arg);
			}
		}
	}
}

void console_logii(int priority, const char *format, int arg1, int arg2){
	if (deb_on && deb_console) {
		if (priority & PRIO_LOG) {
			if (priority & PRIO_TIMERDETAIL) {
				printf("      ");
				printf(format, arg1, arg2);
			}
			else {
				printf(format, arg1, arg2);
			}
		}
	}
}

void console_logs(int priority, const char *format, char * arg){
	if (deb_on && deb_console) {
		if (priority & PRIO_LOG) {
			if (priority & PRIO_TIMERDETAIL) {
				printf("      ");
				printf(format, arg);
			}
			else {
				printf(format, arg);
			}
		}
	}
}

void console_log(int priority, const char *format){
	if (deb_on && deb_console) {
		if (priority & PRIO_LOG) {
			if (priority & PRIO_TIMERDETAIL) {
				printf("      ");
				printf(format);
			}
			else {
				printf(format);
			}
		}
	}
}

bool is_unregular_float(float* x, bool DEN, bool zero){
	bool unregular = false;

	unregular = unregular || isnan(*x);
	unregular = unregular || isinf(*x);
	if (DEN) {
		unregular = unregular || (fpclassify(*x) == FP_SUBNORMAL);
	}
	if (zero) {
		unregular = unregular || (*x == 0.0f);
	}

	return unregular;
}