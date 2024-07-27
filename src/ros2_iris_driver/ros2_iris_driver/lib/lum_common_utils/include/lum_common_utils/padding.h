// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_UTILS_PADDING_H
#define LUM_COMMON_UTILS_PADDING_H

#include <vector>

namespace lum {
namespace common {
namespace utils {
namespace padding {

/// @brief Default amount of padding.
///
/// Chosen to be bigger than connectivity check (3) and divisible by 4 (performance).
static constexpr int DEFAULT_PADDING = 4;

/// @brief Converts padded vector to an OpenCV matrix.
///
/// The vector contains data padded on the right, on the bottom, and flattened.
///
/// @param M one of cv::Mat* types.
/// @param PADDING amount of vertical and horizontal padding.
/// @param padded [in] padded data.
/// @param unpadded [out] unpadded data.  unpadded must be pre-allocated.
template <class M, class T, int PADDING = DEFAULT_PADDING>
void unpad(std::vector<T> const& padded, M* unpadded)
{
  for (int i = 0; i < unpadded->rows; ++i)
  {
    for (int j = 0; j < unpadded->cols; ++j)
    {
      unpadded->template at<T>(i, j) = padded[i * (unpadded->cols + PADDING) + j];
    }
  }
}

/// @brief Converts an OpenCV matrix to the padded vector.
///
/// The vector will contain data padded on the right, on the bottom, and flattened.
///
/// @param M one of cv::Mat* types.
/// @param PADDING amount of vertical and horizontal padding.
/// @param unpadded [in] unpadded data.
/// @param pad_value [in] padded areas will be filled with this value.
/// @returns padded data.
template <class M, class T, int PADDING = DEFAULT_PADDING>
std::vector<T> pad(M const& unpadded, T pad_value)
{
  int padded_rows = unpadded.rows + PADDING;
  int padded_cols = unpadded.cols + PADDING;
  std::vector<T> retval(padded_rows * padded_cols, pad_value);

  for (int i = 0; i < unpadded.rows; ++i)
  {
    for (int j = 0; j < unpadded.cols; ++j)
    {
      retval[i * (unpadded.cols + PADDING) + j] = unpadded.template at<T>(i, j);
    }
  }

  return retval;
}

} // namespace padding
} // namespace utils
} // namespace common
} // namespace lum

#endif // LUM_COMMON_UTILS_PADDING_H
