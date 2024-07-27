// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_RUNTIME_ALLOCATOR_H
#define LUM_COMMON_SWC_RUNTIME_ALLOCATOR_H

#include <cstddef>
#include <memory>
#include <vector>

namespace lum {
namespace common {
namespace swc {

class Allocator;

/// @brief Manages set of sized Allocators.
///
/// Designed to be memory resource for individual SWCs. It takes a max object size beyond which
/// allocations are forwarded to the global operators. The memory is allocated in bands of discrete
/// sizes. All allocation requests for a certain size range are routed to the corresponding band.
/// Each band is operated by one fixed size allocator. The constructor argument for the band width
/// together with the page size controls how many bands and thus fixed size allocators are created.
/// The number of bands is constant throughout the lifetime of RuntimeAllocator.
class RuntimeAllocator final
{
public:
  /// @brief Memory statistics
  struct Stats final
  {
    /// Number of bytes allocated from system heap memory.
    std::size_t bytes_allocated{0U};
    /// Number of bytes made available.
    std::size_t bytes_provided{0U};
  };

  /// @brief Create RuntimeAllocator and initialize all fixed size band Allocators.
  /// @param[in] page_size Number of bytes in a page of memory. Must be at least @a max_object_size.
  /// @param[in] max_object_size Max number of bytes which this may allocate per allocation request.
  /// @param[in] size_band_width Number of bytes for each sized allocation band.
  /// @throw RuntimeAllocatorException if @a page_size or @a size_band_width is zero, or @a
  /// max_object_size does not fit in @a page_size.
  /// @throw AllocatorException if underlying heap allocator cannot be configured correctly
  RuntimeAllocator(std::size_t page_size,
                   std::size_t max_object_size,
                   std::size_t size_band_width) noexcept(false);

  /// @brief Releases all Allocators and thus all memory.
  /// All pointers returned from allocate() become invalid.
  ~RuntimeAllocator() noexcept;

  RuntimeAllocator(const RuntimeAllocator&) = delete;
  RuntimeAllocator(RuntimeAllocator&&) = delete;
  RuntimeAllocator& operator=(const RuntimeAllocator&) & = delete;
  RuntimeAllocator& operator=(RuntimeAllocator&&) & = delete;

  /// @brief Allocate a block of memory of requested size.
  ///
  /// Complexity is typically O(1), worst-case O(num_chunks_of_this_byte_size). It never throws.
  /// @param[in] num_bytes Size in bytes to allocate.
  /// @return nullptr if nothing can be allocated, else pointer to an available block of memory.
  void* allocate(std::size_t num_bytes) noexcept;

  /// @brief Allocate a block of memory of requested size.
  ///
  /// Complexity is typically O(1), worst-case O(num_chunks_of_this_byte_size).
  /// @param[in] num_bytes Size in bytes to allocate.
  /// @return Pointer to an available block of memory.
  /// @throw RuntimeAllocatorOutOfMemoryException if nothing can be allocated.
  void* allocateThrows(std::size_t num_bytes) noexcept(false);

  /// @brief Deallocate a block of memory at a given place and of a specific size.
  ///
  /// Complexity is typically O(1), worst-case O(num_chunks_of_this_byte_size). It never throws.
  /// @param[in] p Pointer to memory block to deallocate.
  /// @param[in] num_bytes Size in bytes to deallocate.
  void deallocate(void* p, std::size_t num_bytes) noexcept;

  /// @brief Deallocate a block of memory at a given place but of unknown size.
  ///
  /// Complexity is O(num_allocators * num_chunks_in_all_allocators). It never throws.
  /// @param[in] p Pointer to memory block to deallocate.
  void deallocate(void* p) noexcept;

  /// @brief Get max object size handled by this class.
  /// @return Number of bytes.
  std::size_t getMaxObjectSize() const noexcept { return max_object_size_; }

  /// @brief Get number of bytes each size allocation band covers.
  /// @return Number of bytes.
  std::size_t getSizeBandWidth() const noexcept { return size_band_width_; }

  /// @brief Check that there are no corruptions.
  ///
  /// Complexity is O(num_allocators * num_chunks_in_all_allocators * num_blocks_in_all_chunks),
  /// errors are reported to syslog.
  /// @return True if there are no corruptions, false otherwise.
  bool isSane() const noexcept;

  /// @brief Get current memory statistics.
  /// @return Statistics as of now.
  Stats getCurrentMemUsage() const noexcept;

  /// @brief Get peak memory statistics.
  ///
  /// These are the maximum values occurred since the last call to this function, ie. calling this
  /// function resets the peak to current values.
  /// @return Peak statistics.
  Stats getPeakMemUsage() noexcept;

private:
  // Array of fixed size band allocators.
  std::vector<std::unique_ptr<Allocator>> pool_;

  // Largest object size in bytes supported by allocators.
  const std::size_t max_object_size_;

  // Size of allocation bands in bytes.
  const std::size_t size_band_width_;

  // Max allocated bytes since last call.
  Stats peak_mem_usage_;
};

} // namespace swc
} // namespace common
} // namespace lum
#endif
