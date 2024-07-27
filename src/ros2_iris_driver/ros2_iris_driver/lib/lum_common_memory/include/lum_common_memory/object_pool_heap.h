// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_MEMORY_OBJECT_POOL_HEAP_H
#define LUM_COMMON_MEMORY_OBJECT_POOL_HEAP_H

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>

#include <lum_common_memory/object_pool.h>

namespace lum {
namespace common {
namespace memory {
namespace object_pool {

namespace detail {

/// @brief Smart Pooled Object Reference, implements IPooledObject
///
/// A smart reference object is retrieved from a SmartPool, and contains a reusable object.  You
/// do not reuse this object directly, but treat it as disposable.  Once you destroy your
/// reference to it, it will check itsself back into the SmartPool for further reuse
///
/// This is placed inside of a SmartReference<> instance to mask the use of replaceable intermediate
/// types.
///
/// @tparam POOLED_TYPE the object type that is held by the object pool
template <typename PooledType>
class SmartReferenceHeap : public IPooledObject<PooledType>
{
public:
  /// @brief construct a smart pooled object reference.  This is meant to be created inside of a
  /// Smart Pool
  /// @param checkin [in] weak pointer to the checkin function of the originating pool
  /// @param deallocate [in] a shared pointer to the pool deallocator
  /// @param reference [in] the pooled object
  SmartReferenceHeap(std::weak_ptr<std::function<void(std::shared_ptr<PooledType>&)>> checkin,
                     std::shared_ptr<std::function<void(std::shared_ptr<PooledType>&)>> deallocate,
                     std::shared_ptr<PooledType> reference)
      : checkin_(std::move(checkin)), deallocate_(std::move(deallocate)), ref_(std::move(reference))
  {
  }

  /// @brief destructor.  If the pool exists, the checkin function is called.  If it does not,
  /// then the pooled object is destroyed
  ~SmartReferenceHeap() override
  {
    auto checkin = checkin_.lock();
    if (checkin)
    {
      // if the pool exists, check back in
      (*checkin)(ref_);
    }
    else
    {
      // the object is done, deallocate it
      (*deallocate_)(ref_);
    }
  }

  SmartReferenceHeap(const SmartReferenceHeap&) = delete;
  SmartReferenceHeap& operator=(const SmartReferenceHeap&) = delete;
  SmartReferenceHeap(SmartReferenceHeap&& other) = delete;
  SmartReferenceHeap& operator=(SmartReferenceHeap&&) = delete;

  /// @brief accessor to handle the pooled object
  /// @return a shared pointer to the pooled object
  PooledType& fetch() override { return *ref_; }

private:
  std::weak_ptr<std::function<void(std::shared_ptr<PooledType>&)>>
    checkin_; ///< function to return ref_ to the pool
  std::shared_ptr<std::function<void(std::shared_ptr<PooledType>&)>>
    deallocate_;                    ///< destroy ref if pool is destroyed
  std::shared_ptr<PooledType> ref_; ///< the pooled object
};

} // namespace detail

/// @brief a smart object pool using heap allocation
///
/// An object pool allows objects to be checked out of, and into, the pool for object reuse.  This
/// class wraps the objects into s Smart Ref, so that the objects will check their backing back
/// into the pool when they are destroyed.  This removes the need for downstream processing to be
/// pool-aware
///
/// @note If a smart reference is held after the pool is destroyed, it will handle its own
/// lifecycle, but will not be returned to the pool.
///
/// @code
/// // a trivially constructed pooled object
/// SmartPoolHeap<std::array<char, 5>> pool;
///
/// // initialize the pool (allocating memory)
/// pool.init(10);
///
/// auto object = pool.checkout();
///
/// // call a function against the backed object
/// object->fill(0);
///
/// // check the object back into the pool
/// object = nullptr;
/// @endcode
///
/// @code
/// // custom alloc, reset and deallocator functions
/// SmartPoolHeap<TYPE> pool(
///  [](std::shared_ptr<TYPE> checked_in_object) { // reset object for reuse here },
///  [](std::shared_ptr<TYPE> checked_in_object) { // destroy object here },
/// );
///
/// // initialize the pool (allocating memory)
/// pool.init(10,
///   [] () -> std::shared_ptr<TYPE> { return ... allocate object here }
/// );
///
/// auto object = pool.checkout();
/// @endcode
template <typename PooledType>
class SmartPoolHeap : public ISmartPool<PooledType>
{
private:
  std::function<void(std::shared_ptr<PooledType>&)>
    reset_object_handler_; ///< function called on objects to reset them
  std::shared_ptr<std::function<void(std::shared_ptr<PooledType>&)>>
    deallocator_handler_; ///< pointer to function to destroy objects
  std::shared_ptr<std::function<void(std::shared_ptr<PooledType>&)>>
    checkin_handler_; ///< pointer to function to return objects to the pool

  std::queue<std::shared_ptr<PooledType>> object_pool_; ///< pool of available objects
  std::mutex pool_mutex_;                               ///< pool lock
  std::condition_variable checkin_condition_; ///< signal for proceesses waiting for a checkin
  bool initialized_{};                        ///< if we have already initialized
  const std::chrono::milliseconds DEFAULT_WAIT_TIMEOUT{100};

  /// @brief reset object and return it to the pool
  /// @param object [in] object getting returned
  void checkIn(std::shared_ptr<PooledType> object)
  {
    reset_object_handler_(object);
    std::lock_guard<std::mutex> lock(pool_mutex_);
    object_pool_.push(std::move(object));
    checkin_condition_.notify_one();
  }

  /// @brief utility function to pop the front and wrap it into a smart reference
  /// @return wrapped instance
  SmartReference<PooledType> wrapFront()
  {
    // pool_mutex_ needs to be locked before calling this
    auto ref_ptr = std::make_shared<detail::SmartReferenceHeap<PooledType>>(
      checkin_handler_, deallocator_handler_, object_pool_.front());
    object_pool_.pop();
    return SmartReference<PooledType>(ref_ptr);
  }

public:
  /// @brief create a new smart pool
  /// @param reset [in] the function to reset objects as they are checked back in
  /// @param deallocator [in] the function to destroy objects in the pool
  /// @note the allocator function is only called in the constructor, likewise the deallocate
  /// function is called during or after pool destruction, depending on the object lifecycle
  explicit SmartPoolHeap(
    std::function<void(std::shared_ptr<PooledType>&)> reset = [](std::shared_ptr<PooledType>&) {},
    std::function<void(std::shared_ptr<PooledType>&)> deallocator =
      [](std::shared_ptr<PooledType>&) {})
      : reset_object_handler_(std::move(reset))
  {
    // create checkin smart pointer
    checkin_handler_ = std::make_shared<std::function<void(std::shared_ptr<PooledType>&)>>(
      [this](std::shared_ptr<PooledType>& object) { checkIn(object); });

    // validate reset_object_handler_ exists and is callable
    if (reset_object_handler_ == nullptr)
    {
      reset_object_handler_ = [](std::shared_ptr<PooledType>&) {};
    }

    // validate and capture the deallocator
    if (deallocator)
    {
      deallocator_handler_ =
        std::make_shared<std::function<void(std::shared_ptr<PooledType>&)>>(std::move(deallocator));
    }
    else
    {
      deallocator_handler_ =
        std::make_shared<std::function<void(std::shared_ptr<PooledType>&)>>([](auto) {});
    }
  }

  ~SmartPoolHeap() override = default;

  /// @brief initialize the pool using a provided capacity and allocator function
  /// @note only call initialize once, subsequent calls automatically fail
  /// @param capacity [in] the capacity of the pool
  /// @param allocator [in] function to create shared pointers to objects.  Default is trivial
  /// make_shared<POOLED_TYPE>
  /// @return true if initialization was successful
  bool
  initialize(std::size_t capacity,
             std::function<std::shared_ptr<PooledType>()> allocator = std::make_shared<PooledType>)
  {
    std::lock_guard<std::mutex> lock(pool_mutex_);

    // we are already initialized
    if (initialized_)
    {
      return false;
    }

    // we have a bad allocator function
    if (allocator == nullptr)
    {
      // ERROR log "Bad allocator function provided to ObjectPoolHeap::initialize(), cannot
      // initialize"
      return false;
    }

    for (std::size_t i{0}; i < capacity; ++i)
    {
      auto object = allocator();
      reset_object_handler_(object);
      object_pool_.push(std::move(object));
    }
    initialized_ = true;
    return true;
  }

  /// @brief retrieve an object from the pool
  /// @return a ref on success, or nullptr if the pool is exhausted
  SmartReference<PooledType> checkout() override
  {
    std::lock_guard<std::mutex> lock(pool_mutex_);
    if (!initialized_)
    {
      // ERROR log "ObjectPoolHeap has not been initialized"
      return {};
    }

    if (object_pool_.empty())
    {
      return {};
    }

    return wrapFront();
  }

  /// @brief blocking method to wait for an object to be checked in.  This can be called at any
  /// time, but should be used instead of polling when the pool is exhausted
  /// @param timeout [in] amount of time to wait before returning nullptr.  default 100ms
  /// @return a ref on success, or nullptr testable on failure
  SmartReference<PooledType> waitForCheckout(std::chrono::nanoseconds timeout) override
  {
    std::unique_lock<std::mutex> lock(pool_mutex_);
    if (!initialized_)
    {
      // ERROR log "ObjectPoolHeap has not been initialized, returning null object"
      return {};
    }

    // try to checkout first in case we arent exhausted anymore
    if (!object_pool_.empty())
    {
      return wrapFront();
    }

    // pool is exhausted, subscribe to updates
    checkin_condition_.wait_for(lock, timeout, [this]() { return !object_pool_.empty(); });
    if (object_pool_.empty())
    {
      return {};
    }
    return wrapFront();
  }

  SmartReference<PooledType> waitForCheckout() override
  {
    return waitForCheckout(DEFAULT_WAIT_TIMEOUT);
  }

  SmartPoolHeap(const SmartPoolHeap&) = delete;
  SmartPoolHeap& operator=(const SmartPoolHeap&) = delete;
  SmartPoolHeap(SmartPoolHeap&&) = delete;
  SmartPoolHeap& operator=(SmartPoolHeap&&) = delete;
};

} // namespace object_pool
} // namespace memory
} // namespace common
} // namespace lum

#endif
