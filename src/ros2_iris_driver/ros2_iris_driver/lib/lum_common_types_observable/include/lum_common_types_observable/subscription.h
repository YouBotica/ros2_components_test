// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_OBSERVABLE_SUBSCRIPTION_H
#define LUM_COMMON_TYPES_OBSERVABLE_SUBSCRIPTION_H

#include <functional>
#include <memory>
#include <utility>

namespace lum {
namespace common {
namespace types {
namespace observable {

/// @brief class to hold a function callback and enable calling it with any updated value
template <typename... T>
class Subscription
{
public:
  /// @brief function to subscribe to a callback
  explicit Subscription(std::function<void(T...)> callback) : callback_(std::move(callback)) {}

  /// @brief function to enable calling it with any updated value
  void newValueRecieved(T... value)
  {
    // std::functions can be empty
    if (callback_)
    {
      callback_(value...);
    }
  }

private:
  /// @brief function to hold a callback
  std::function<void(T...)> callback_;
};

/// @brief user-facing handle to a subscription.  This is a wrapper class to hide internal
/// Subscription callback tracking.
template <typename... T>
class SubscriptionHandle
{
public:
  /// @brief constructor of a user-facing handle of a subscription
  SubscriptionHandle() = default;
  /// @brief user-facing handle of a subscription
  explicit SubscriptionHandle(std::shared_ptr<Subscription<T...>> subscription)
      : subscription_(std::move(subscription))
  {
  }

  /// @brief is this handle tied to an observable, or is it orphaned
  /// @throw
  /// @return true if this handle was created with an observable
  inline bool isValid() const { return subscription_ != nullptr; }

  /// @brief force this handle to become invalid.
  /// @throw
  inline void unsubscribe() { subscription_ = nullptr; }

private:
  /// @brief pointer to handle of a subscription
  std::shared_ptr<Subscription<T...>> subscription_;
};

} // namespace observable
} // namespace types
} // namespace common
} // namespace lum

#endif // LUM_COMMON_TYPES_OBSERVABLE_SUBSCRIPTION_H
