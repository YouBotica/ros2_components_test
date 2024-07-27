// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_MATH_WEIGHTED_ARITHMETIC_MEAN_H
#define LUM_COMMON_MATH_WEIGHTED_ARITHMETIC_MEAN_H

#include <limits>
#include <tuple>

namespace lum {
namespace common {
namespace math {

/// @brief Class to find weighted arithmetic mean
/// @code
/// // Initialize mean calculator
/// WeightedArithmeticMean calculator;
/// calculator.addNewValue(value1, weight1);
/// calculator.addNewValue(value2, weight2);
/// const auto validity_and_mean = calculator.getMean();
/// const auto validity = validity_and_mean.first;
/// const auto mean = validity_and_mean.second;
/// @endcode
class WeightedArithmeticMean
{
public:
  /// @brief Struct to store output
  struct Output
  {
    double mean{0.0};     ///< mean of the samples
    double variance{0.0}; ///< variance of the samples
    bool validity{false}; ///< validity of the result
  };
  /// @brief Method to add new value with its weight to the calculator
  /// @param [in] value Value to be added
  /// @param [in] weight Weight of the value to be added
  void addNewValue(const double value, const double weight)
  {
    if (weight < std::numeric_limits<double>::epsilon())
    {
      // do not add value if its weight is zero
      return;
    }

    running_sum_ += value * weight;
    running_sum_of_squares_ += value * value * weight;
    weight_sum_ += weight;
    ++num_values_;
  }

  /// @brief Method to get the mean and variance at current instant
  /// @return returns a struct with validity, actual mean and sample variance
  Output getMeanAndVariance() const
  {
    Output output;
    if (canCalculateMean())
    {
      const double mean = running_sum_ / weight_sum_;
      const double mean_of_squares = running_sum_of_squares_ / weight_sum_;
      // sample variance is calculated of the formula
      // Var(X) = Mean(X^2) - (Mean(X))^2
      const double variance = mean_of_squares - mean * mean;
      output.mean = mean;
      output.variance = variance;
      output.validity = true;
    }
    return output;
  }

  /// @brief Method to get the number of values currently under consideration of arithmetic mean
  /// @return Returns number of values
  std::size_t size() const { return num_values_; }

  /// @brief Method to reset calculator
  void reset()
  {
    running_sum_ = 0.0;
    weight_sum_ = 0.0;
    num_values_ = 0U;
    running_sum_of_squares_ = 0.0;
  }

private:
  /// @brief Method to check if mean can be calculated
  /// @return returns true if mean is valid and can be calculated
  bool canCalculateMean() const { return weight_sum_ > std::numeric_limits<double>::epsilon(); }

  double running_sum_{0.0}; ///< to store running sum of  (weight * value)

  double running_sum_of_squares_{0.0}; ///< to store sum of (weight * value * value)

  double weight_sum_{0.0}; ///< to store running sum of weight

  std::size_t num_values_{0}; ///< Number of values in the calculator
};

} // namespace math
} // namespace common
} // namespace lum

#endif
