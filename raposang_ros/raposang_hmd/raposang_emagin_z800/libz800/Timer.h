#pragma once

class Timer
{
public:
	Timer();
	virtual ~Timer();

	/**
	 * Retrieves the elapsed time since the last call in seconds.
	 */
	double getElapsedTime();

protected:
	void *_pimpl;
};
