#ifndef CALIBRATION_PARSER__H
#define CALIBRATION_PARSER__H

#include "resim.hpp"
#include "schemas/detectorcalibration.capnp.h"
#include "schemas/roic705.capnp.h"

#include <memory>

struct calibration_struct;

namespace ReSim
{

class CalibrationParser
{
public:
	CalibrationParser(const CalibrationParser &other);
	CalibrationParser(const Schemas::Calibration::Reader &calibration_message);

	CalibrationParser &operator=(const CalibrationParser &other);

	~CalibrationParser();

	const calibration_struct &getCalibration() const;

	void setCalibration(const calibration_struct &cal);

	std::unique_ptr<calibration_struct> calibration;

private:
	void Parse(const Schemas::Calibration::Reader &calibration_message);
	void AssertCompatible(const Schemas::Calibration::Reader &calibration_message);
};
} // namespace ReSim

#endif /*CALIBRATION_PARSER__H*/
