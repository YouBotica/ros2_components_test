// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_OBSERVABLE_OBSERVABLE_VALUE_H
#define LUM_COMMON_TYPES_OBSERVABLE_OBSERVABLE_VALUE_H

#include <functional>
#include <memory>
#include <mutex>
#include <type_traits>
#include <utility>

#include <lum_common_types_observable/observable.h>
#include <lum_common_types_observable/subscription.h>

namespace lum {
namespace common {
namespace types {
namespace observable {

/// @brief An Rx Behavior-Subject style class that holds a value, and notifies subscribers when it
/// changes
template <typename T>
class ObservableValue final : public Observable<T>
{
public:
  using storable_type = typename std::remove_cv<typename std::remove_reference<T>::type>::type;

  /// @brief construct a new ObservableValue, passing in the initial state
  explicit ObservableValue(T value) : Observable<T>(), value_(std::move(value)){};

  ~ObservableValue() override = default;
  ObservableValue(const ObservableValue&) = default;                  // copy constructor
  ObservableValue(ObservableValue&&) noexcept = default;              // move constructor
  ObservableValue& operator=(const ObservableValue&) & = default;     // copy assign
  ObservableValue& operator=(ObservableValue&&) & noexcept = default; // move assign

  /// @brief subscribe to changes to the held value, callback is called immediately with the current
  /// value
  /// @param [in] callback the handler function for the current value and all updates
  /// @returns a subscriber handle that automatically unsubscribes when it is destroyed or leaves
  /// scope
  SubscriptionHandle<T> subscribe(const std::function<void(T)>& callback) override
  {
    // if the functor is empty, return an empty handle
    if (!callback)
    {
      return {};
    }
    {
      std::lock_guard<std::mutex> guard(value_lock_);
      callback(value_);
    }
    return Observable<T>::subscribe(callback);
  }

  /// @brief update the held value, and notify any subscribers of the new value
  /// @param [in] new_value the new value
  void updateValue(T new_value) override
  {
    {
      std::lock_guard<std::mutex> guard(value_lock_);
      value_ = new_value;
    }
    Observable<T>::updateValue(new_value);
  }

  /// @brief utility method to get currently held state without subscribing
  const storable_type& peek() const
  {
    std::lock_guard<std::mutex> guard(value_lock_);
    return value_;
  }

private:
  mutable std::mutex value_lock_; ///< mutex to protect internal value
  storable_type value_;           ///< the stateful internal value
};
} // namespace observable
} // namespace types
} // namespace common
} // namespace lum

#endif // LUM_COMMON_TYPES_OBSERVABLE_OBSERVABLE_VALUE_H
