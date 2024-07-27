// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_MEMORY_I_ARENA_ALLOCATOR_H
#define LUM_COMMON_MEMORY_I_ARENA_ALLOCATOR_H

#include <memory>

namespace lum {
namespace common {
namespace memory {

template <class T>
class IArenaAllocator
{
public:
  // Polymorphic base class boilerplate
  virtual ~IArenaAllocator() = default;
  IArenaAllocator() = default;
  IArenaAllocator(const IArenaAllocator&) = delete;
  IArenaAllocator(IArenaAllocator&&) = delete;
  void operator=(const IArenaAllocator&) & = delete;
  void operator=(IArenaAllocator&&) & = delete;

  virtual std::size_t getSize() const = 0;

  virtual std::unique_ptr<T, std::function<void(T*)>> checkout() = 0;
};

} // namespace memory
} // namespace common
} // namespace lum

#endif
