// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_I_MESSAGE_H
#define LUM_COMMON_SWC_I_MESSAGE_H

#include <memory>

namespace lum {
namespace common {
namespace swc {

/// @brief Communication message interface
class IMessage
{
public:
  /// @brief A handle to the message which maintains the ownership
  using Handle = std::unique_ptr<IMessage>;

  IMessage() = default;
  virtual ~IMessage() = default;

  IMessage(const IMessage&) = delete;
  IMessage& operator=(const IMessage&) & = delete;
  IMessage(IMessage&&) = delete;
  IMessage& operator=(IMessage&&) & = delete;

  /// @brief Deep copy of self
  /// @return Handle to the copied message
  virtual Handle clone() const = 0;
};

} // namespace swc
} // namespace common
} // namespace lum

#endif
