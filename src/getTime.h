#pragma once

#include <chrono>
#include <ctime>

class Timer
{
public:
	time_t begin;
	Timer()
	{
		begin = clock();
	}
	double elapsed()
	{
		time_t end = clock();
		double Times = double(end - begin) / CLOCKS_PER_SEC;
		return Times;
	}
};