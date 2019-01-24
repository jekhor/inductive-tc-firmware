#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(unsigned char K, unsigned long int initY) {
	setK(K);

	Y = (unsigned long long)initY << LP_Order;
}

void LowPassFilter::setK(unsigned char K)
{
	mK = K;
	mNa = (1 << K) - 1;
}

void LowPassFilter::setOutput(unsigned long int y) {
	Y = (unsigned long long)y << LP_Order;
}

void LowPassFilter::input(unsigned long int x) {
	Y = (mNa * Y + ((unsigned long long)x << LP_Order)) >> mK;
}

unsigned long int LowPassFilter::output() {
	return Y >> LP_Order;
}


