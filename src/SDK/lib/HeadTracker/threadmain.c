// Copyright 2020, Felix Pfreundtner, All rights reserved.
#include "threadmain.h"

void workerexample(void *arg)
{
    int *val = arg;
    int  old = *val;

    *val += 1000;
    //printf("tid=%p, old=%d, val=%d\n", pthread_self(), old, *val);

    if (*val%2)
        usleep(100000);
}

int threadexample(int argc, char **argv)
{
    tpool_t *tm;
    int     *vals;
    size_t   i;

    tm   = tpool_create(num_threads);
    vals = calloc(num_items, sizeof(*vals));

    for (i=0; i<num_items; i++) {
		if (vals) {
			vals[i] = (int)i;
			tpool_add_work(tm, workerexample, vals + i);
		}
    }

    tpool_wait(tm);

    for (i=0; i<num_items; i++) {
		if (vals) {
			printf("%d\n", vals[i]);
		}
    }

    free(vals);
    tpool_destroy(tm);
    return 0;
}

tpool_t* thread_init(int Npool) {
	tpool_t *tm;
    tm = tpool_create(Npool);
	return tm;
}


void single_update_thread_or_serial(tpool_t* tm, void (*f)(void), void* arg, bool multithread_on, bool wait_finish) {
	if (multithread_on) {
		single_update_thread(tm, f, arg, wait_finish);
	}
	else {
		f(arg);
	}
}

void single_update_thread(tpool_t *tm, void (*f)(void), void* arg,  bool wait_finish){
	// add work to pool
	tpool_add_work(tm, f, arg);

	// wait until pool finishes its work
	if (wait_finish) {
		tpool_wait(tm);
	}
}

void loop_update_thread_or_serial(tpool_t *tm, void (*f)(void), int Nitem, bool multithread_on, bool wait_finish) {
	if (multithread_on) {
		loop_update_thread(tm, f, Nitem, wait_finish);
	}
	else {
		for (int n = 0; n < Nitem; n++) {
			f(&n);
		}
	}
}

void loop_update_thread(tpool_t *tm, void (*f)(void), int Nitem, bool wait_finish) {
	// add work to pool
	size_t n;
	int *vals;
    vals = calloc(Nitem, sizeof(*vals));

	for (n = 0; n < Nitem; n++) {
		if (vals) {
			vals[n] = (int)n;
		}
		tpool_add_work(tm, f, vals + n);
	}
	// wait until pool finishes its work
	if (wait_finish) {
		tpool_wait(tm);
	}

}
void thread_finish(tpool_t *tm) {
    tpool_destroy(tm);
}


#ifdef _WIN32
void usleep(__int64 usec) 
{ 
    HANDLE timer; 
    LARGE_INTEGER ft; 

    ft.QuadPart = -(10*usec); // Convert to 100 nanosecond interval, negative value indicates relative time

    timer = CreateWaitableTimer(NULL, TRUE, NULL); 
	SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
	WaitForSingleObject(timer, INFINITE);
	CloseHandle(timer);
}
#endif