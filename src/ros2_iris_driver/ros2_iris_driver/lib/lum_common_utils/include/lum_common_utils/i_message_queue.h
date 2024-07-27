// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_UTILS_I_MESSAGE_QUEUE_H
#define LUM_COMMON_UTILS_I_MESSAGE_QUEUE_H

namespace lum {
namespace common {
namespace utils {

// A thread-safe message queue
class IMessageQueue
{
public:
  IMessageQueue() = default;
  virtual ~IMessageQueue() = default;

  IMessageQueue(const IMessageQueue&) = delete;
  IMessageQueue(IMessageQueue&&) = delete;

  IMessageQueue& operator=(const IMessageQueue&) & = delete;
  IMessageQueue& operator=(IMessageQueue&&) & = delete;
};

} // namespace utils
} // namespace common
} // namespace lum

#endif // LUM_COMMON_UTILS_I_MESSAGE_QUEUE_H
