// threadcrossplatform.h: Wraper to run threads in windows and Posix with pthread.
// modified from: https://nachtimwald.com/2019/04/05/cross-platform-thread-wrapper/
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once

#ifdef _WIN32
# include <stdbool.h>
# include <windows.h>
#else
# include <pthread.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

	//////////////////////////////////////////
	#ifdef _WIN32
	typedef CRITICAL_SECTION pthread_mutex_t;
	typedef void pthread_mutexattr_t;
	typedef void pthread_condattr_t;
	typedef void pthread_rwlockattr_t;
	typedef void pthread_attr_t; // added
	typedef HANDLE pthread_t;
	typedef CONDITION_VARIABLE pthread_cond_t;

	typedef struct {
		SRWLOCK lock;
		bool    exclusive;
	} pthread_rwlock_t;

	struct timespecthread {
		long tv_sec;
		long tv_nsec;
	};

	#endif

	#ifdef _WIN32
	int pthread_create(pthread_t *thread, pthread_attr_t *attr, void *(*start_routine)(void *), void *arg);
	int pthread_join(pthread_t thread, void **value_ptr);
	int pthread_detach(pthread_t);

	int pthread_mutex_init(pthread_mutex_t *mutex, pthread_mutexattr_t *attr);
	int pthread_mutex_destroy(pthread_mutex_t *mutex);
	int pthread_mutex_lock(pthread_mutex_t *mutex);
	int pthread_mutex_unlock(pthread_mutex_t *mutex);

	int pthread_cond_init(pthread_cond_t *cond, pthread_condattr_t *attr);
	int pthread_cond_destroy(pthread_cond_t *cond);
	int pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex);
	int pthread_cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *mutex, const struct timespecthread *abstime);
	int pthread_cond_signal(pthread_cond_t *cond);
	int pthread_cond_broadcast(pthread_cond_t *cond);

	int pthread_rwlock_init(pthread_rwlock_t *rwlock, const pthread_rwlockattr_t *attr);
	int pthread_rwlock_destroy(pthread_rwlock_t *rwlock);
	int pthread_rwlock_rdlock(pthread_rwlock_t *rwlock);
	int pthread_rwlock_tryrdlock(pthread_rwlock_t *rwlock);
	int pthread_rwlock_wrlock(pthread_rwlock_t *rwlock);
	int pthread_rwlock_trywrlock(pthread_rwlock_t  *rwlock);
	int pthread_rwlock_unlock(pthread_rwlock_t *rwlock);

	static DWORD timespec_to_ms(const struct timespecthread* abstime); // added

	#endif

	unsigned int pcthread_get_num_procs();
	void ms_to_timespec(struct timespecthread *ts, unsigned int ms);



#ifdef __cplusplus
};
#endif
