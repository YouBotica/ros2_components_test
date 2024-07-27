// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_MATH_WEIGHTED_HARMONIC_MEAN_H
#define LUM_COMMON_MATH_WEIGHTED_HARMONIC_MEAN_H

#include <limits>

namespace lum {
namespace common {
namespace math {

/// @brief Class for Weighted Harmonic mean calculations
class WeightedHarmonicMeanCalculator
{
public:
  /// @brief Method to add new value the calculator with weight
  /// @param [in] value Value to be added for harmoni mean calculation
  /// @param [in] weight Weight of the value added (defaults to 1.0)
  void addNewValue(const double value, const double weight = 1.0)
  {

    if (value <= std::numeric_limits<double>::epsilon())
    {
      // skipping this value
      return;
    }
    if (weight <= std::numeric_limits<double>::epsilon())
    {
      // skipping this value
      return;
    }

    running_weighted_sum_ += weight / value;

    weight_sum_ += weight;

    ++num_values_;
  }

  /// @brief Method to get current harmonic mean
  /// @return returns the result as a pair of [validity of harmonic mean, harmonic mean]
  std::pair<bool, double> getHarmonicMean() const
  {
    if (canCalculateHarmonicMean())
    {
      return std::make_pair(true, weight_sum_ / running_weighted_sum_);
    }
    return std::make_pair(false, 0.0);
  }

  /// @brief Method to get the number of values in the Harmonic mean calculator
  /// @return returns the number of values added
  std::size_t size() const { return num_values_; }

  /// @brief Method to reset internal variables in the class
  void reset()
  {
    running_weighted_sum_ = 0.0;
    weight_sum_ = 0.0;
    num_values_ = 0UL;
  }

private:
  /// @brief Method to check if harmonic mean can be calculated
  /// @return returns true if harmonic mean can be calculated
  bool canCalculateHarmonicMean() const
  {
    return running_weighted_sum_ > std::numeric_limits<double>::epsilon();
  }

  double running_weighted_sum_{0.0}; ///< running sum of (weight / value)
  double weight_sum_{0.0};           ///< running sum of weights
  std::size_t num_values_{0};        ///< Number of values added
};

} // namespace math
} // namespace common
} // namespace lum

#endif
