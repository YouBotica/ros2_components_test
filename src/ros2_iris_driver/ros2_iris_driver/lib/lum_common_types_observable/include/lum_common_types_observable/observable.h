// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_OBSERVABLE_OBSERVABLE_H
#define LUM_COMMON_TYPES_OBSERVABLE_OBSERVABLE_H

#include <functional>
#include <list>
#include <memory>
#include <mutex>

#include <lum_common_types_observable/subscription.h>

namespace lum {
namespace common {
namespace types {
namespace observable {

/// @brief An Rx Observable style class to notify subscribers when a new value is available
template <typename... T>
class Observable
{
public:
  Observable() = default;

  virtual ~Observable() = default;                          // destructor
  Observable(const Observable&) = default;                  // copy constructor
  Observable(Observable&&) noexcept = default;              // move constructor
  Observable& operator=(const Observable&) & = default;     // copy assign
  Observable& operator=(Observable&&) & noexcept = default; // move assign

  /// @brief subscribe to receive new values
  /// @param [in] callback handler function for the new value
  /// @returns subscription handle.  Automatically unsubscribes when it is destroyed or leaves scope
  virtual SubscriptionHandle<T...> subscribe(const std::function<void(T...)>& callback)
  {
    // if the functor is empty, return an empty handle
    if (!callback)
    {
      return {};
    }
    const std::shared_ptr<Subscription<T...>> subscription =
      std::make_shared<Subscription<T...>>(callback);
    const std::weak_ptr<Subscription<T...>> subscription_weak_ptr = subscription;
    std::lock_guard<std::mutex> guard(subscriber_lock_);
    subscriptions_.push_back(subscription_weak_ptr);
    return SubscriptionHandle<T...>(subscription);
  }

  /// @brief notify all subscribers that a new value is available
  /// @param [in] new_value the new value to pass to subscribers
  virtual void updateValue(T... new_values)
  {
    std::lock_guard<std::mutex> guard(subscriber_lock_);
    auto list_iter = subscriptions_.begin();
    while (list_iter != subscriptions_.end())
    {
      auto shared = list_iter->lock();
      if (shared) // Subscription is still alive, notify the subscription of new value
      {
        shared->newValueRecieved(new_values...);
        ++list_iter;
      }
      else // This subscription was cleaned up, remove from list
      {
        list_iter = subscriptions_.erase(list_iter);
      }
    }
  }

  std::size_t numSubscribers() const
  {
    std::lock_guard<std::mutex> guard(subscriber_lock_);
    return subscriptions_.size();
  }

private:
  mutable std::list<std::weak_ptr<Subscription<T...>>> subscriptions_; ///< list of subscribers
  mutable std::mutex subscriber_lock_; ///< mutex for subscriber list safety
};

} // namespace observable
} // namespace types
} // namespace common
} // namespace lum

#endif // LUM_COMMON_TYPES_OBSERVABLE_OBSERVABLE_H
