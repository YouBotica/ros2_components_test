#ifndef DETECTOR_H
#define DETECTOR_H

#include <stdint.h>

#define _USE_MATH_DEFINES
#include <math.h>

const double rad_to_counts = static_cast<double>(1 << 15) / M_PI;

inline uint32_t radians_to_counts(float rad)
{
	// Assumes rad \in [-PI, PI).  For now simply add PI until we know the final
	// detector configuration. TODO: Implement detector conversion.
	return static_cast<uint32_t>((rad + M_PI) * rad_to_counts);
}

inline float counts_to_radians(uint32_t counts) { return static_cast<float>(static_cast<double>(counts) / rad_to_counts - M_PI); }

#endif /*DETECTOR_H*/
