// threadmain.h: Main function for threading pool framework using pthread
// Modified from: https://nachtimwald.com/2019/04/12/thread-pool-in-c/
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once
#include <stdlib.h>
#include <stdio.h>
#include "threadpool.h"
#ifdef __linux__ 
#include <pthread.h>
#include <unistd.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif
	static const size_t num_threads = 4;
	static const size_t num_items   = 100;

	//////////////////////////////////////////
	// worker that is executed as thread
	void workerexample(void* arg);

	// framework that starts num_items workers on num_threads 
	int threadexample(int argc, char** argv);
	
	// initializes pool with Npool threads
	tpool_t* thread_init(int Npool);

	// run function f over thread pool tm or in serial dependend on variable multithreading
	void single_update_thread_or_serial(tpool_t* tm, void (*f)(void), void* arg, bool multithread_on, bool wait_finish);

	// run function with thread pool
	void single_update_thread(tpool_t* tm, void (*f)(void), void* arg, bool wait_finish);

	// run function f over Nitems in loop over thread pool tm or in serial dependend on variable multithreading
	void loop_update_thread_or_serial(tpool_t* tm, void (*f)(void), int Nitem, bool multithread_on, bool wait_finish);

	// run function with thread pool by iterating over Nitems in loop 
	void loop_update_thread(tpool_t* tm, void (*f)(void), int Nitem, bool wait_finish);

	// destorys thread pool
	void thread_finish(tpool_t* tm);

	#ifdef _WIN32
	void usleep(__int64 usec);
	#endif

#ifdef __cplusplus
};
#endif

