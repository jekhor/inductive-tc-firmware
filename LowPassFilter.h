#ifndef __LOWPASSFILTER_H__
#define __LOWPASSFILTER_H__

#include <Arduino.h>

class LowPassFilter {
//#define LP_Na	2047
//#define LP_Nb	1
//#define LP_K	11
#define LP_Order 20

	public:
	LowPassFilter(unsigned char k, unsigned long int initY=0);

	void input(unsigned long int x);
	unsigned long int output();
	void setOutput(unsigned long int y);
	void setK(unsigned char K);

	private:
	unsigned long long int Y;
	unsigned char mK;
	uint16_t mNa;
};

#endif
