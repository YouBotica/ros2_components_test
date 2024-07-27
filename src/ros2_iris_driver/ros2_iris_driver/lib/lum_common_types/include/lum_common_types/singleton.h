// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_SINGLETON_H
#define LUM_COMMON_TYPES_SINGLETON_H

#ifdef _MSC_VER
#define FORCE_C_CALLING_CONVENTION __cdecl
#else
#define FORCE_C_CALLING_CONVENTION
#endif

#include <cassert>
#include <cstdlib>
#include <mutex>

namespace lum {
namespace common {
namespace types {

///  @brief Implementation of the CreationPolicy used by Singleton
///  Creates objects using malloc and placement new
template <class T>
class CreateUsingMalloc
{
public:
  static void* allocate(const std::size_t size) noexcept
  {
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory,cppcoreguidelines-no-malloc,hicpp-no-malloc)
    return std::malloc(size);
  }

  static void deallocate(void* const ptr) noexcept
  {
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory,cppcoreguidelines-no-malloc,hicpp-no-malloc)
    std::free(ptr);
  }

  static T* create() noexcept(false)
  {
    T* memory = static_cast<T*>(allocate(sizeof(T)));
    if (memory == nullptr)
    {
      return nullptr; // GCOV_EXCL_LINE:0001
    }
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    return new (memory) T;
  }

  static void destroy(T* const p) noexcept(false)
  {
    p->~T();
    deallocate(p);
  }
};

///  @brief Implementation of the CreationPolicy used by Singleton
///  Creates objects using a straight call to the new operator
template <class T>
class CreateUsingNew
{
public:
  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
  static T* create() noexcept(false) { return new T; }

  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
  static void destroy(T* const p) noexcept(false) { delete p; }
};

///  @brief Implementation of the CreationPolicy used by Singleton
///  Creates objects using a custom allocator.
///  Usage: e.g. CreateUsing<std::allocator>::Allocator
template <template <class> class Alloc>
struct CreateUsing
{

  /// @brief a struct to define a custom allocator
  template <class T>
  class Allocator
  {
  public:
    /// @brief allocator instantiation
    static Alloc<T> allocator_;

    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    static T* create() { return new (allocator_.allocate(1U)) T; }

    /// @brief instance deallocation
    static void destroy(T* const p)
    {
      allocator_.destroy(p);
      allocator_.deallocate(p, 1U);
    }
  };
};

/// @brief handler function to be called on normal program termination
using AtExitFn = void(FORCE_C_CALLING_CONVENTION*)();

///  @brief Implementation of the LifetimePolicy used by Singleton
///  Schedules an object's destruction as per C++ rules
///  Forwards to std::atexit (which is thread-safe)
template <class T>
class DefaultLifetime
{
public:
  /// @brief schedule destruction singleton instance
  /// @param[in] func_ptr pointer to the exit function handler
  static void scheduleDestruction(AtExitFn const func_ptr) { std::ignore = std::atexit(func_ptr); }

  static void onDeadReference()
  {
    const std::logic_error message{"Dead Reference Detected"};
    throw std::logic_error{message};
  }
};

///  @brief Implementation of the LifetimePolicy used by Singleton
///  Schedules an object's destruction as per C++ rules
///  Forwards to std::atexit (which is thread-safe)
/// Ignores a dead reference which leads to resurrecting the object by the next getInstance() call
template <class T>
class ResurrectLifetime
{
public:
  /// @brief schedule destruction singleton instance
  /// @param[in] func_ptr pointer to the exit function handler
  static void scheduleDestruction(AtExitFn const func_ptr) { std::ignore = std::atexit(func_ptr); }

  static void onDeadReference() {}
};

///  @brief Implementation of the LifetimePolicy used by Singleton
///  Does not schedule object destruction and thus "leaks" the object
template <class T>
class LeakLifetime
{
public:
  static void scheduleDestruction(AtExitFn const) {}

  static void onDeadReference() {}
};

///  @brief Provides Singleton behavior for a type T
///  To protect that type from spurious instantiations,
///  you have to protect it yourself.
///
///  \tparam T The class to turn into a singleton
///  \tparam CreationPolicy Creation policy, default: CreateUsingNew
///  \tparam LifetimePolicy Lifetime policy, default: DefaultLifetime,
template <typename T,
          template <class> class CreationPolicy = CreateUsingNew,
          template <class> class LifetimePolicy = DefaultLifetime>
class Singleton
{
public:
  /// @brief Type of the singleton object
  using ObjectType = T;

  ///  @brief get instance to singleton object
  ///  @return Returns a reference to singleton object
  static T& getInstance()
  {
    if (nullptr == instance_)
    {
      makeInstance();
    }
    return *instance_;
  }

  // Protection
  Singleton() = delete;
  ~Singleton() = default;

  Singleton(const Singleton&) = delete;
  Singleton(Singleton&&) = delete;
  Singleton& operator=(const Singleton&) & = delete;
  Singleton& operator=(Singleton&&) & = delete;

private:
  // Helpers
  static void makeInstance()
  {
    static std::mutex mutex{};
    const std::lock_guard<std::mutex> guard{mutex};

    if (nullptr == instance_)
    {
      if (destroyed_)
      {
        destroyed_ = false;
        LifetimePolicy<T>::onDeadReference();
      }
      instance_ = CreationPolicy<T>::create();
      LifetimePolicy<T>::scheduleDestruction(&destroy);
    }
  }

  static void FORCE_C_CALLING_CONVENTION destroy()
  {
    // https://reviews.llvm.org/D31130
    // NOLINTNEXTLINE(hicpp-no-array-decay,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    assert(!destroyed_);
    CreationPolicy<T>::destroy(instance_);
    instance_ = nullptr;
    destroyed_ = true;
  }

  // Data
  static T* instance_;
  static bool destroyed_;
};

template <class T, template <class> class C, template <class> class L>
T* Singleton<T, C, L>::instance_{nullptr};

template <class T, template <class> class C, template <class> class L>
bool Singleton<T, C, L>::destroyed_{false};

} // namespace types
} // namespace common
} // namespace lum

#endif
