// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_MEMORY_ARENA_H
#define LUM_COMMON_MEMORY_ARENA_H

#include <array>
#include <functional>
#include <list>
#include <mutex>
#include <vector>

#include <lum_common_memory/i_arena_allocator.h>

namespace lum {
namespace common {
namespace memory {

/// @brief simple class to handle holding pointers, and checking them in and out of a pool in a
/// thread-safe way
class PointerManager
{
  std::list<void*> temp_;
  std::list<void*> free_;
  mutable std::mutex pool_mutex_;

public:
  void initialize(std::list<void*> initial_values)
  {
    const std::lock_guard<std::mutex> lock(pool_mutex_);
    free_ = std::move(initial_values);
    temp_.clear();
  }

  void* checkout()
  {
    const std::lock_guard<std::mutex> lock(pool_mutex_);
    if (free_.empty())
    {
      return nullptr;
    }
    const auto it = free_.cbegin();
    temp_.splice(temp_.cend(), free_, free_.cbegin());
    return *it;
  }

  void checkin(void* const address)
  {
    const std::lock_guard<std::mutex> lock(pool_mutex_);
    if (this->temp_.empty())
    {
      return;
    }
    // take the begin from in use and store the returned ptr
    const auto it = this->temp_.begin();
    *it = address;
    free_.splice(free_.cend(), temp_, temp_.cbegin());
  }

  bool empty() const
  {
    const std::lock_guard<std::mutex> lock(pool_mutex_);
    return free_.empty();
  }

  std::size_t getSize() const
  {
    const std::lock_guard<std::mutex> lock(pool_mutex_);
    return free_.size();
  }
};

///////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief a dynamic-sized wrapper around a PointerManager
template <class T>
class DynamicMemoryPoolCore
{
  static constexpr std::size_t ALIGN_TO{sizeof(std::uint32_t)};
  static constexpr std::size_t ALIGNED_SIZE{sizeof(T) + (ALIGN_TO - (sizeof(T) % ALIGN_TO))};
  static_assert(std::is_default_constructible<T>::value,
                "Type needs to be default constructible but is not.");

  std::vector<std::uint8_t> buffer_;
  PointerManager manager_;

public:
  explicit DynamicMemoryPoolCore(std::size_t num_objects)
  {
    const auto byte_size = ALIGNED_SIZE * num_objects;
    buffer_.resize(byte_size);
    std::list<void*> initial_values;
    for (std::size_t i{0U}; i < buffer_.size(); i += ALIGNED_SIZE)
    {
      initial_values.push_back(static_cast<void*>(&buffer_[i]));
    }

    manager_.initialize(std::move(initial_values));
  }

  bool empty() const { return manager_.empty(); }
  std::size_t getSize() const { return manager_.getSize(); }
  void checkin(T* address) { manager_.checkin(address); }
  T* checkout()
  {
    if (auto pooled_address = manager_.checkout())
    {
      return new (pooled_address) T; // NOLINT(cppcoreguidelines-owning-memory)
    }
    return nullptr;
  }
};

///////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief a template-sized wrapper around a PointerManager
template <class T, std::size_t NUM_OBJECTS>
class StaticMemoryPoolCore
{
  static constexpr std::size_t ALIGN_TO{sizeof(std::uint32_t)};
  static constexpr std::size_t ALIGNED_SIZE{sizeof(T) + (ALIGN_TO - (sizeof(T) % ALIGN_TO))};
  static_assert(std::is_default_constructible<T>::value,
                "Type needs to be default constructible but is not.");

  std::array<std::uint8_t, ALIGNED_SIZE * NUM_OBJECTS> buffer_;
  PointerManager manager_;

public:
  StaticMemoryPoolCore()
  {
    std::list<void*> initial_values;
    for (std::size_t i{0U}; i < buffer_.size(); i += ALIGNED_SIZE)
    {
      initial_values.push_back(static_cast<void*>(&buffer_[i]));
    }

    manager_.initialize(std::move(initial_values));
  }

  std::size_t getSize() const { return manager_.getSize(); }
  bool empty() const { return manager_.empty(); }
  void checkin(T* address) { manager_.checkin(address); }
  T* checkout()
  {
    if (auto pooled_address = manager_.checkout())
    {
      return new (pooled_address) T; // NOLINT(cppcoreguidelines-owning-memory)
    }
    return nullptr;
  }
};

///////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief an IArenaAllocator implementation that is sized at runtime
template <class T>
class DynamicArena : public IArenaAllocator<T>
{
public:
  explicit DynamicArena(std::size_t pool_capacity)
      : core_{std::make_shared<DynamicMemoryPoolCore<T>>(pool_capacity)}
  {
    // copy in shared pointer, so memory is not destroyed while buffers are in use
    deleter_ = [core{this->core_}](T* ptr) {
      if (ptr)
      {
        core->checkin(ptr);
      }
    };
  }

  std::size_t getSize() const override { return core_->getSize(); }

  std::unique_ptr<T, std::function<void(T*)>> checkout() override
  {
    return {core_->checkout(), deleter_};
  }

private:
  std::shared_ptr<DynamicMemoryPoolCore<T>> core_{};
  std::function<void(T* ptr)> deleter_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief an IArenaAllocator implementation that is sized at compile-time
template <class T, std::size_t NUM_OBJECTS>
class Arena : public IArenaAllocator<T>
{
public:
  Arena()
  {
    // copy in shared pointer, so memory is not destroyed while buffers are in use
    deleter_ = [core{this->core_}](T* ptr) {
      if (ptr)
      {
        core->checkin(ptr);
      }
    };
  }

  std::size_t getSize() const override { return core_->getSize(); }

  std::unique_ptr<T, std::function<void(T*)>> checkout() override
  {
    return {core_->checkout(), deleter_};
  }

private:
  std::shared_ptr<StaticMemoryPoolCore<T, NUM_OBJECTS>> core_{
    std::make_shared<StaticMemoryPoolCore<T, NUM_OBJECTS>>()};
  std::function<void(T* ptr)> deleter_;
};

} // namespace memory
} // namespace common
} // namespace lum

#endif
