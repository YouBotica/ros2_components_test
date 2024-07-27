// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_MEMORY_OBJECT_POOL_H
#define LUM_COMMON_MEMORY_OBJECT_POOL_H

#include <chrono>
#include <memory>

namespace lum {
namespace common {
namespace memory {
namespace object_pool {

/// @brief Smart Pooled Object Reference
///
/// A smart reference object is retrieved from a SmartPool, and contains a reusable object.  You do
/// not reuse this object directly, but treat it as disposable.  Once you destroy your reference to
/// it, it will check itself back into the SmartPool for further reuse
///
/// @note this interface is meant to be between the SmartReference and the ISmartPool implementation
///
/// @tparam PooledType the object type that is held by the object pool
template <typename PooledType>
class IPooledObject
{
public:
  /// @brief accessor to handle the pooled object
  /// @return a shared pointer to the pooled object
  virtual PooledType& fetch() = 0;

  IPooledObject() = default;
  virtual ~IPooledObject() = default;
  IPooledObject(const IPooledObject&) = delete;
  IPooledObject(IPooledObject&&) = delete;
  IPooledObject& operator=(const IPooledObject&) & = delete;
  IPooledObject& operator=(IPooledObject&&) & = delete;
};

/// @brief a polymorphic PooledObject holder
///
/// The PooledObject holds the implementation for the pool check-in and perhaps implementation
/// specific details.  This is a convenience object for access
///
/// @code
/// // given an ISmartPool implementer
/// ISmartPool<TYPE>* pool;
///
/// // checkout an object from the pool
/// auto ref = pool->checkout();
///
/// if ( ref == nullptr )
/// {
///   // the checkout failed, the pool is empty
/// }
///
/// // use the pooled object
///
/// // return the object by letting it leave scope, or setting to nullptr
/// ref = nullptr;
/// @endcode
///
/// @tparam the type of object held by the originating pool
template <typename PooledType>
class SmartReference
{
public:
  SmartReference() = default;

  /// @brief Copy constructor
  SmartReference(const SmartReference& other) : ref_(other.ref_) {}

  explicit SmartReference(std::shared_ptr<IPooledObject<PooledType>> ref) : ref_(std::move(ref)) {}

  ~SmartReference() = default;
  /// @brief Assignment operator
  SmartReference<PooledType>& operator=(const SmartReference& other)
  {
    if (this != &other)
    {
      ref_ = other.ref_;
    }
    return *this;
  }
  /// @brief Assignment operator specialized for nullptr assignment
  SmartReference<PooledType>& operator=(std::nullptr_t)
  {
    ref_ = nullptr;
    return *this;
  }

  /// @brief Move Constructor
  SmartReference(SmartReference&& other) noexcept = default;

  /// @brief Move assignment operator
  SmartReference& operator=(SmartReference&&) noexcept = default;

  PooledType* operator->() { return &ref_->fetch(); }
  const PooledType* operator->() const { return &ref_->fetch(); }
  PooledType* operator*() { return &ref_->fetch(); }
  const PooledType* operator*() const { return &ref_->fetch(); }

  /// @brief allow bool conversion for simplified valid checks ie if(SmartReference)
  explicit operator bool() const { return ref_ != nullptr; }

  std::shared_ptr<IPooledObject<PooledType>> get() { return ref_; }
  std::shared_ptr<IPooledObject<PooledType>> get() const { return ref_; }

private:
  std::shared_ptr<IPooledObject<PooledType>> ref_;
};

template <typename T>
bool operator==(const SmartReference<T>& lhs, std::nullptr_t)
{
  return lhs.get() == nullptr;
}

template <typename T>
bool operator!=(const SmartReference<T>& lhs, std::nullptr_t)
{
  return lhs.get() != nullptr;
}

template <typename T>
bool operator==(const SmartReference<T>& lhs, const SmartReference<T>& rhs)
{
  return lhs.get() == rhs.get();
}

template <typename T>
bool operator!=(const SmartReference<T>& lhs, const SmartReference<T>& rhs)
{
  return !(lhs.get() == rhs.get());
}

/// @brief a simple, degenerate pooled object that does not do any lifecycling
///
/// This class is used to wrap a reusable or single-use object into an interface that is compatible
/// with ObjectPooling interfaces
template <typename PooledType>
class SimplePooledObject : public IPooledObject<PooledType>
{
public:
  explicit SimplePooledObject(std::shared_ptr<PooledType> ref) : ref_(std::move(ref)) {}

  ~SimplePooledObject() override = default;
  SimplePooledObject(const SimplePooledObject&) = delete;
  SimplePooledObject(SimplePooledObject&&) = delete;
  SimplePooledObject& operator=(const SimplePooledObject&) & = delete;
  SimplePooledObject& operator=(SimplePooledObject&&) & = delete;

  /// @brief accessor to handle the pooled object
  /// @return a shared pointer to the pooled object
  PooledType& fetch() override { return *ref_; }

  /// @brief helper to create pseudo-pooled simple objects
  /// @param[in] backing_structure a shared ptr to the object being wrapped
  /// @return a SmartReference that agrees with pooling interfaces
  static SmartReference<PooledType> wrap(std::shared_ptr<PooledType> backing_structure)
  {
    // pseudo buffer pooling
    auto pooled_object =
      std::make_shared<SimplePooledObject<PooledType>>(std::move(backing_structure));

    // create smart reference
    SmartReference<PooledType> argument(pooled_object);

    return argument;
  }

private:
  std::shared_ptr<PooledType> ref_; ///< pooled object reference
};

/// @brief Interface class for a smart object pool
///
/// Instead of holding a single instance, or allocating multiple objects, this class enables the use
/// of objects that wrap a specific type, can be checked out, and check themselves back in when the
/// wrapper is destroyed.  This class does not allocate more or resize when empty.  Proper use would
/// be for the pool size to be large enough to not become exhausted before your pipeline starts
/// returning objects to the pool
///
/// @code
/// // trivially constructable type
/// // provided an implementation instance exists
/// ISmartPool<TYPE>* pool;
///
/// // get a smart ref to the object
/// auto ref = pool->checkout();
///
/// // get a smart ref, blocking until one is available (up to timeout value, default 1ms)
/// auto block_ref = pool->waitForCheckout();
///
/// @endcode
///
/// @tparam PooledType the object to hold in the pool
template <typename PooledType>
class ISmartPool
{
public:
  /// @brief retrieve an object from the pool
  /// @return a ref on success, or nullptr testable ref if the pool is exhausted
  virtual SmartReference<PooledType> checkout() = 0;

  /// @brief blocking method to wait for an object to be checked in.  This can be called at any
  /// time, but should be used instead of polling when the pool is exhausted
  /// @param timeout [in] amount of time to wait before returning nullptr
  /// @note this can wait forever if objects are never returned to the pool.  In practice, your
  /// pools should be large enough to not run out, and all objects should be returned to the pool
  /// @return a ref on success
  virtual SmartReference<PooledType> waitForCheckout(std::chrono::nanoseconds timeout) = 0;

  /// @brief blocking method to wait for an object to be checked in.  This can be called at any
  /// time, but should be used instead of polling when the pool is exhausted.  It will wait for up
  /// to 100ms
  virtual SmartReference<PooledType> waitForCheckout() = 0;

  /// @brief default destructor
  virtual ~ISmartPool() = default;
  ISmartPool() = default;
  ISmartPool(const ISmartPool&) = delete;
  ISmartPool(ISmartPool&&) = delete;
  ISmartPool& operator=(const ISmartPool&) & = delete;
  ISmartPool& operator=(ISmartPool&&) & = delete;
};

} // namespace object_pool
} // namespace memory
} // namespace common
} // namespace lum

#endif
