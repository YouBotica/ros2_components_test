// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_INTERNAL_DISJOINT_SET_H
#define LUM_COMMON_TYPES_INTERNAL_DISJOINT_SET_H

#include <algorithm>
#include <numeric>
#include <vector>

#include <lum_common_casts/lum_casts.h>

namespace lum {
namespace common {
namespace types {

template <typename T>
class DisjointSet
{
public:
  /// @brief DisjointSet constructor
  /// @param [in] number_of_total_elements is the number of elements total in the sets
  explicit DisjointSet(std::size_t number_of_total_elements)
      : rank_(number_of_total_elements, 0), parent_(number_of_total_elements)
  {
    std::iota(parent_.begin(), parent_.end(), 0);
  }

  /// @brief takes the set element id x and find the element id that is
  /// representative of the set.
  /// @param [in] x is the id of the element whose set representative is to be
  /// found
  /// @return returns the id of the set represenatative
  T doFind(T x)
  {
    // save the original value
    T original_x = x;

    // climb up the tree until we find the set representative
    while (parent_[x] != x)
    {
      x = parent_[x];
    }

    // climb up again and compress the nodes until we find finish
    while (parent_[original_x] != original_x)
    {
      T temp_x = parent_[original_x];
      parent_[original_x] = x;
      original_x = temp_x;
    }

    return x;
  }

  /// @brief unites the two sets that have x and y into a single set
  /// @param [in] x and y are the element ids
  /// @return void
  void doUnion(T x, T y)
  {
    // Find current sets of x and y
    T xset = doFind(x);
    T yset = doFind(y);

    // Process only if they are in different sets
    if (xset != yset)
    {
      // Put smaller ranked item under bigger ranked item if ranks are
      // different
      if (rank_[xset] < rank_[yset])
      {
        parent_[xset] = yset;
      }
      else if (rank_[xset] > rank_[yset])
      {
        parent_[yset] = xset;
      }

      // If ranks are same, then increment
      // rank_.
      else
      {
        parent_[yset] = xset;
        ++rank_[xset];
      }
    }
  }

  /// @brief resets data srtuctures to original state;  all elements are disconnected.
  void reset()
  {
    std::fill(rank_.begin(), rank_.end(), 0);
    std::iota(parent_.begin(), parent_.end(), 0);
  }

private:
  std::vector<T> rank_;
  std::vector<T> parent_;
};

/// @brief Pointer-based implementation of disjoint set.
///
/// See Wikipedia: https://en.wikipedia.org/wiki/Disjoint_sets
/// The implementation needs formal deviation to be AUTOSAR-compliant.
///
/// Example code:
/// @code
/// DisjointSetP ds(3);  // Creating a disjoint set with 3 elements
/// ds.unite(0, 1);  // uniting #0 and #1,
/// ds.unite(1, 2);  // and #1 and #2.
/// assert(ds.find(0) == ds.find(2));  // Now #0 and #2 are united.
/// @endcode
///
/// There are two ways of identifying elements and sets:
/// a) by index (an unsigned value, here size_t)
/// b) by "ID".  This method is generally faster.
class DisjointSetP final
{
public:
  /// @brief element/set identifier.
  using ID = void**;
  using iterator = ID;
  using const_iterator = void* const*;

  DisjointSetP() = default;

  /// @param [in] number_of_elements the number of elements in the sets
  explicit DisjointSetP(std::size_t number_of_elements)
      : parents_(number_of_elements)
      , begin_(lum::common::casts::lum_reinterpret_cast<ID>(&parents_[0]))
      , end_(begin_ + number_of_elements)
  {
    reset();
  }

  /// @brief finds the root id of the set containing id.
  /// @param [in] id the id of the element.
  /// @return the id of the set represenatative, the root id.
  static ID findID(ID id)
  {
    ID temp_id = id;
    while (*temp_id != temp_id)
    {
      temp_id = lum::common::casts::lum_reinterpret_cast<ID>(*temp_id);
    }
    *id = temp_id;
    return temp_id;
  }

  /// @brief finds the root id of the set containing index.
  /// @param [in] index of the element, not the id (!).
  /// @return the id of the set represenatative, the root id.
  ID findID(std::size_t index) const { return findID(begin_ + index); }

  /// @brief finds the root index of the set containing index.
  /// @param [in] index of the element, not the id (!).
  /// @return the index of the set represenatative, the root index.
  std::size_t find(std::size_t index) const { return findID(index) - begin_; }

  /// @brief unites the two sets that contain x and y into a single set.
  /// @param [in] x element index.
  /// @param [in] y element index.
  void unite(std::size_t x, std::size_t y) const { unite(findID(x), findID(y)); }

  /// @brief unites the two sets with roots (!) x and y into a single set.
  /// Guarantees parent of x unchanged.  Requires *y == y and *x == x.
  /// @param [in] x root of one set.
  /// @param [in] y root of another set.
  static void unite(ID x, ID y) { *y = x; }

  /// @brief resets data srtuctures to original state;  all elements are disconnected.
  void reset() { std::iota(parents_.begin(), parents_.end(), begin_); }

  /// @brief resets data srtuctures to original state;  all elements are disconnected.
  void reset(std::size_t number_of_elements)
  {
    parents_.resize(number_of_elements);
    reset();
    begin_ = lum::common::casts::lum_reinterpret_cast<ID>(&parents_[0]);
    end_ = begin_ + number_of_elements;
  }

  /// @brief Resizes the data structure without resetting
  /// @param [in] number_of_elemets New number of elements
  void resize(std::size_t number_of_elements)
  {
    parents_.resize(number_of_elements);
    begin_ = lum::common::casts::lum_reinterpret_cast<ID>(&parents_[0]);
    end_ = begin_ + number_of_elements;
  }

  // Iterator methods.
  iterator begin() { return begin_; }
  iterator end() { return end_; }
  const_iterator begin() const { return begin_; }
  const_iterator end() const { return end_; }

private:
  std::vector<ID> parents_{};
  ID begin_{nullptr};
  ID end_{nullptr};
};

} // namespace types
} // namespace common
} // namespace lum

#endif // LUM_COMMON_TYPES_INTERNAL_DISJOINT_SET_H
