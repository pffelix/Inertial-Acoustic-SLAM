// Copyright 2020, Felix Pfreundtner, All rights reserved.
#include "offline.h"

void save_head_tracker_disk() {
	// initialize
		char* filepath;
		int Nfilepathparts;
		char** filepathparts;
		char update_nr_char[char_N];
		float amp_max;
		long normalize_flag;

		// run
		Nfilepathparts= 4;
		filepathparts = (char**)malloc(sizeof(char*) * Nfilepathparts);
		sprintf(update_nr_char, "%d", update_nr);
		normalize_flag = 4;
		if (filepathparts) {
			// construct file path
			filepathparts[0] = offline_path;
			filepathparts[1] = update_nr_char;
			filepathparts[3] = ".wav";
			filepath = (char*)malloc(sizeof(char) * (strlen(filepathparts[0]) + strlen(filepathparts[1]) + 3 + strlen(filepathparts[3]) + Nfilepathparts - 1 + 2));
			
			// save acoustic recording as wav file to file path
			if (offline_rec) {
				filepathparts[2] = "rec";
				char_concatenatemulti(filepathparts, Nfilepathparts, "_", filepath);
				write_wav_debug_multithread(rec->x, ch_Nrecbeam, rec_N, (float)fs, filepath, "", 0, deb_pool_on);
			}

			// save acoustic impulse response as wav file to file path
			if (offline_ir) {
				filepathparts[2] = "ir";
				char_concatenatemulti(filepathparts, Nfilepathparts, "_", filepath);
				write_wav_debug_multithread(ir->x, ch_Nrecbeam, ir_N, (float)fs, filepath, "", 1, deb_pool_on);
			}

			// save head position as wav file to file path
			if (offline_head) {
				filepathparts[2] = "head";
				char_concatenatemulti(filepathparts, Nfilepathparts, "_", filepath);
				write_wav_frame_debug_multithread(head->pos, frame_M, A_Ndof, A_Ndofderivative, 0.0f, filepath, "", normalize_flag, deb_pool_on);
			}

			// save imu sensor as wav file to file path
			if (offline_imu) {
				filepathparts[2] = "imu";
				char_concatenatemulti(filepathparts, Nfilepathparts, "_", filepath);
				write_wav_debug_multithread(imu->pos, A_Ndof, A_Ndofderivative, 0.0f, filepath, "", normalize_flag, deb_pool_on);
			}

			// save vicon impulse source position as wav file to file path
			if (offline_vicis) {
				filepathparts[2] = "vicis";
				char_concatenatemulti(filepathparts, Nfilepathparts, "_", filepath);
				write_wav_debug_multithread(vic->pos_is, A_Ndof, A_Ndofderivative, 0.0f, filepath, "", normalize_flag, deb_pool_on);
			}

			// save vicon recording receiver position as wav file to file path
			if (offline_vicis) {
				filepathparts[2] = "vicrec";
				char_concatenatemulti(filepathparts, Nfilepathparts, "_", filepath);
				write_wav_debug_multithread(vic->pos_rec, A_Ndof, A_Ndofderivative, 0.0f, filepath, "", normalize_flag, deb_pool_on);
			}

			// save frame weight as wav file to file path
			if (offline_weight) {
				filepathparts[2] = "weight";
				char_concatenatemulti(filepathparts, Nfilepathparts, "_", filepath);
				write_wav_debug_multithread(&par6->obs_weight, 1, frame_M, 1.0f, filepath, "", 0, deb_pool_on);
				
			}
			// save frame toa as wav file to file path
			if (offline_toa) {
				filepathparts[2] = "toa";
				char_concatenatemulti(filepathparts, Nfilepathparts, "_", filepath);
				write_wav_debug_multithread(head->toa, frame_M, A_Ntoa, 0.0f, filepath, "", normalize_flag, deb_pool_on);
			}

			// save most likely particle position
			if (offline_parml) {
				filepathparts[2] = "parml";
				char_concatenatemulti(filepathparts, Nfilepathparts, "_", filepath);
				write_wav_debug_multithread(par6->pos, A_Ndof, A_Ndofderivative, 0.0f, filepath, "", normalize_flag, deb_pool_on);
			}

			// save algorithm state as wav file to file path
			if (offline_algo) {
				filepathparts[2] = "algo";
				char_concatenatemulti(filepathparts, Nfilepathparts, "_", filepath);
				write_wav_debug_multithread(&update_algo, 1, A_Nalgo, 0.0f, filepath, "", normalize_flag, deb_pool_on);
			}


			// free memory
			free(filepath);
			free(filepathparts);
		}
}

void read_head_tracker_disk(){
		// initialize
		char* filepath;
		int Nfilepathparts;
		char** filepathparts;
		char update_nr_char[char_N];
		float amp_max;

		// run
		Nfilepathparts= 4;
		filepathparts = (char**)malloc(sizeof(char*) * Nfilepathparts);
		sprintf(update_nr_char, "%d", update_nr);
		if (filepathparts) {
			// construct file path
			filepathparts[0] = offline_path;
			filepathparts[1] = update_nr_char;
			filepathparts[3] = ".wav";
			filepath = (char*)malloc(sizeof(char) * (strlen(filepathparts[0]) + strlen(filepathparts[1]) + 3 + strlen(filepathparts[3]) + Nfilepathparts - 1 + 1 + 1));
			
			// read acoustic recording as wav file from file path
			filepathparts[2] = "rec";
			char_concatenatemulti(filepathparts, Nfilepathparts, "_", filepath);
			rec->x[0][0] = 0.0f;
			JuceCWrapperWAVRead(filepath, rec->x, &amp_max); // amp_max never bigger than 1.0f

			// read imu sensor as wav file from file path
			filepathparts[2] = "imu";
			char_concatenatemulti(filepathparts, Nfilepathparts, "_", filepath);
			JuceCWrapperWAVRead(filepath, imu->pos, &amp_max);
			ar_multiply2d(imu->pos, A_Ndof, A_Ndofderivative, amp_max);

			// read vicon impulse source position as wav file from file path
			filepathparts[2] = "vicis";
			char_concatenatemulti(filepathparts, Nfilepathparts, "_", filepath);
			JuceCWrapperWAVRead(filepath, vic->pos_is, &amp_max);
			ar_multiply2d(vic->pos_is, A_Ndof, A_Ndofderivative, amp_max);

			// read vicon recording receiver position as wav file from file path
			filepathparts[2] = "vicrec";
			char_concatenatemulti(filepathparts, Nfilepathparts, "_", filepath);
			JuceCWrapperWAVRead(filepath, vic->pos_rec, &amp_max);
			ar_multiply2d(vic->pos_rec, A_Ndof, A_Ndofderivative, amp_max);

			// free memory
			free(filepath);
			free(filepathparts); // 1 is unused
		}
}


void read_head_tracker_disk_old() {
	// add console for debugging
	//FILE * pConsole;
	//AllocConsole();
    //pConsole = freopen("CONOUT$", "wb", stdout); // _s, &pConsole

    // read in recording
	long rec_Nloc;
	long *rec_N_ptr = &rec_Nloc;
	char interface_path_params[path_N] = "./info"; // "/sdcard/HeadTracker/interface/info"; // "E:/Eigene Dateien/Eigene Dokumente/Polybox/Startups/Media VR/Programming/ARFoundation/AugmentedAudio/src/unity/head_tracker/interface/info";
	char interface_path_rec_l[path_N] = "./rec_l"; // "/sdcard/HeadTracker/interface/rec_l"; // "E:/Eigene Dateien/Eigene Dokumente/Polybox/Startups/Media VR/Programming/ARFoundation/AugmentedAudio/src/unity/head_tracker/interface/realtime/5_room/1000_rec_l";
	char interface_path_rec_r[path_N] = "./rec_r"; // "/sdcard/HeadTracker/interface/rec_r"; // "E:/Eigene Dateien/Eigene Dokumente/Polybox/Startups/Media VR/Programming/ARFoundation/AugmentedAudio/src/unity/head_tracker/interface/realtime/5_room/1000_rec_r";
	read_interface_disk_params_old(rec_N_ptr, interface_path_params);
	float *rec_l_disc = ar_declare(rec_Nloc);
	float *rec_r_disc = ar_declare(rec_Nloc);
	read_interface_disk_rec_old(rec_l_disc, rec_r_disc, interface_path_rec_l, interface_path_rec_r);

	// track head
	long rec_channels = 2;
	float ** rec_ptr = (float**)malloc(sizeof(float*) * rec_channels);
	if (rec_ptr) {
		rec_ptr[0] = rec_l_disc;
		rec_ptr[1] = rec_r_disc;
	}

	// update params
	ch_Nrec = rec_channels;
	rec_N = rec_Nloc;
	//update_head_tracker_state();

};


void read_interface_disk_params_old(long *rec_N_ptr, char *interface_path_params) {
	// init
    char path[path_N];
	char str[path_N];
	FILE *fp;
	void *rec_lm;
	float *rec_rm;
	int pos = 0;
    strcpy(path, interface_path_params);


	// disk input: read file from path
    fp = fopen(path, "r");
    if (fp == NULL){
        console_logs(PRIO_INFO, "Could not open interface file %s\n", path);
        // exit(0);
    }
	else {
		// File was opened, filepoint can be used to read the stream.
		strncpy(str, "", path_N);
		while (fgets(str, path_N, fp) != NULL) {
			//while ((fscanf(fp, "%[^\n]", str)) != EOF){
				//fgetc(fp);
			if (pos == 0) {
				*rec_N_ptr = (long)strtol(str, NULL, 0);
                console_logs(PRIO_INFO, "Opened interface file %s\n", path);
                console_logs(PRIO_INFO, "buffer size: %s\n", str);
            };
			if (pos == 1) {
				rec_lm = (float *)str;
			};
			if (pos == 2) {
				rec_rm = (void *)str;
			};
			strncpy(str, "", path_N);
			pos += 1;
		}
		fclose(fp);
    }

};

void read_interface_disk_rec_old(float* rec_l, float *rec_r, char *interface_path_rec_l, char *interface_path_rec_r) {

	// init
    char path_l[path_N];
    char path_r[path_N];
	char str[path_N];
	FILE *fp;
	int pos = 0;
	char * line;
	size_t len;
	char chunk[128];
    strcpy(path_l, interface_path_rec_l);
    strcpy(path_r, interface_path_rec_r);


	// disk input: read left and right recording from path
	// left
	line = NULL;
	len = 0;
	pos = 0;

    fp = fopen(path_l, "r");
    if (fp == NULL){
        console_logs(PRIO_WARNING, "Could not open interface file %s\n", path_l);
        exit(0);
    }
	else {
		// File was opened, filepoint can be used to read the stream.
		strncpy(str, "", path_N);
		while (fgets(chunk, sizeof(chunk), fp) != NULL) {
			rec_l[pos] = (float)atof(chunk);
			pos += 1;
		}
        console_logs(PRIO_INFO, "Opened interface file %s\n", path_l);
    }
	fclose(fp);

	// right
	line = NULL;
	len = 0;
	pos = 0;

    fp = fopen(path_r, "r");
    if (fp == NULL){
        console_logs(PRIO_WARNING, "Could not open interface file %s\n", path_r);
        exit(0);
    }
	else {
		// File was opened, filepoint can be used to read the stream.
        strncpy(str, "", path_N);
		while (fgets(chunk, sizeof(chunk), fp) != NULL) {
			rec_r[pos] = (float)atof(chunk);
			pos += 1;
		}
		console_logs(PRIO_INFO,  "Opened interface file %s\n", path_r);
		fclose(fp);
    }
};



