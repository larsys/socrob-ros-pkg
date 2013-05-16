#ifdef WIN32
#include <windows.h>
#else
#include <time.h>
#endif

#include "Timer.h"

struct Timer_Impl
{
#ifdef WIN32
	LARGE_INTEGER freq;
	LARGE_INTEGER tv;
#else
	struct timespec tv;
#endif
};

Timer::Timer()
{
	_pimpl = new Timer_Impl;
	Timer_Impl *impl = static_cast<Timer_Impl *>(_pimpl);

#ifdef WIN32
	QueryPerformanceFrequency(&impl->freq);
	QueryPerformanceCounter(&impl->tv);
#else
	clock_gettime(CLOCK_REALTIME, &impl->tv);
#endif
}

Timer::~Timer()
{
	delete static_cast<Timer_Impl *>(_pimpl);
	_pimpl = NULL;
}

#ifndef WIN32
static timespec timespecDiff(timespec start, timespec end)
{
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}
#endif

double Timer::getElapsedTime()
{
	Timer_Impl *impl = static_cast<Timer_Impl *>(_pimpl);

#ifdef WIN32
	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);

	double ret = double(now.QuadPart - impl->tv.QuadPart) / impl->freq.QuadPart;

	impl->tv.QuadPart = now.QuadPart;
	return ret;
#else
	struct timespec now;
	clock_gettime(CLOCK_REALTIME, &now);
	timespec d = timespecDiff(impl->tv, now);
	impl->tv = now;
	return d.tv_sec + (d.tv_nsec / 1000000000.0);
#endif
}
