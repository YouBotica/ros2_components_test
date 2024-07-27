// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_I_TOPIC_H
#define LUM_COMMON_SWC_I_TOPIC_H

#include <type_traits>
#include <typeindex>

namespace lum {
namespace common {
namespace swc {

/// @brief Interface to provide a named identifier to a communication channel
class ITopic
{
public:
  ///  @brief The unique topic identifier
  using Id = std::type_index;

  ITopic() = default;
  virtual ~ITopic() = default;

  ITopic(const ITopic&) = delete;
  ITopic& operator=(const ITopic&) & = delete;
  ITopic(ITopic&&) = delete;
  ITopic& operator=(ITopic&&) & = delete;

  /// @brief Get the unique identifier for this topic
  /// @return Process-wide unique identifier
  virtual Id getId() const = 0;
};

/// @brief Get the identifier of a concrete topic class
/// @return Process-wide unique identifier
template <typename Topic>
inline ITopic::Id getTopicIdentifier()
{
  static_assert(std::is_final<Topic>::value, "Topic must be a final class");
  return std::type_index{typeid(Topic)};
}

} // namespace swc
} // namespace common
} // namespace lum

#endif
