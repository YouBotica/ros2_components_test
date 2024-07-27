#ifndef BE64TOH_ITERATOR
#define BE64TOH_ITERATOR

#if defined(_WIN32)
#include <winsock2.h>
#else
#include <arpa/inet.h> // htonl, ntohl
#endif

#include <cstdint>
#include <memory> // std::uintptr_t

namespace lum
{
namespace util
{
inline auto convert_big_to_little_endian(const std::uint32_t val) { return ntohl(val); }
inline auto convert_little_to_big_endian(const std::uint32_t val) { return htonl(val); }
} // namespace util
} // namespace lum

/**
 * @brief class which endian swaps 64-bit words on the fly
 *
 * uint32_T is either `uint32_t` or `const uint32_t` to create const or non-const iterators
 */
template <typename uint32_T,
          typename std::enable_if<std::is_same<uint32_t, typename std::remove_const<uint32_T>::type>::value, int>::type = 0>
class be64toh_iterator
{
	// since we don't necessarily know the alignment of the starting pointer we
	// have to keep track of the first pointer and check the alignment relative to
	// that
	uint32_t const *start_ptr_; // don't modify after construction, except for from copy assignment
	uint32_T *      ptr_;

	be64toh_iterator(uint32_t const *start_ptr, uint32_T *ptr) : start_ptr_{start_ptr}, ptr_{ptr} {}

public:
	// iterator traits
	using difference_type   = long;
	using value_type        = uint32_t;
	using pointer           = const uint32_t *;
	using reference         = const uint32_t &;
	using iterator_category = std::random_access_iterator_tag;

	be64toh_iterator(uint32_T *ptr) : start_ptr_{ptr}, ptr_{ptr} {}

	~be64toh_iterator() = default;
	be64toh_iterator(const be64toh_iterator &other) : start_ptr_{other.start_ptr_}, ptr_{other.ptr_} {}
	be64toh_iterator &operator=(const be64toh_iterator &other) = default;

	uint32_t const *get_start_ptr() const { return start_ptr_; }

	/**
	 * @brief Make non-const iterators convertable to const iterators
	 */
	template <class Q = uint32_T>
	typename std::enable_if<std::is_same<Q, typename std::remove_const<Q>::type>::value, be64toh_iterator<const uint32_t>>::type
	operator()() const
	{
		return be64toh_iterator<const uint32_t>(ptr_);
	}

	be64toh_iterator &operator++()
	{
		ptr_++;
		return *this;
	}
	be64toh_iterator operator++(int)
	{
		auto retval = *this;
		++(*this);
		return retval;
	}

	inline bool operator==(const be64toh_iterator &other) const { return ptr_ == other.ptr_; }
	inline bool operator!=(const be64toh_iterator &other) const { return !(*this == other); }

	inline bool operator<(const be64toh_iterator &rhs) { return ptr_ < rhs.ptr_; }
	inline bool operator>(const be64toh_iterator &rhs) { return ptr_ > rhs.ptr_; }
	inline bool operator<=(const be64toh_iterator &rhs) { return ptr_ <= rhs.ptr_; }
	inline bool operator>=(const be64toh_iterator &rhs) { return ptr_ >= rhs.ptr_; }

	inline difference_type  operator-(const be64toh_iterator &rhs) const { return ptr_ - rhs.ptr_; }
	inline be64toh_iterator operator+(difference_type rhs) const { return be64toh_iterator{start_ptr_, ptr_ + rhs}; }
	inline be64toh_iterator operator-(difference_type rhs) const { return be64toh_iterator{start_ptr_, ptr_ - rhs}; }

	inline be64toh_iterator &operator+=(difference_type n)
	{
		ptr_ += n;
		return *this;
	}
	inline be64toh_iterator &operator-=(difference_type n)
	{
		ptr_ -= n;
		return *this;
	}

	inline uint32_t operator*() const
	{
		// Find out of this is an even or odd word relative to the first one. We
		// need to get the opposite word in the 64-bit group.
		const bool            even = !((ptr_ - start_ptr_) % 2);
		uint32_t const *const word = ptr_ + (even ? +1 : -1);
		return lum::util::convert_big_to_little_endian(*word);
	}

	/**
	 * @brief a set function for non-const iterators
	 */
	template <class Q = uint32_T>
	typename std::enable_if<std::is_same<Q, typename std::remove_const<Q>::type>::value, void>::type set(const uint32_t newval)
	{
		// Find out of this is an even or odd word relative to the first one. We
		// need to get the opposite word in the 64-bit group.
		const bool      even = !((ptr_ - start_ptr_) % 2);
		uint32_t *const word = ptr_ + (even ? +1 : -1);

		*word = lum::util::convert_little_to_big_endian(newval);
	}

	inline uint32_t operator[](size_t idx) const { return *be64toh_iterator{start_ptr_, ptr_ + idx}; }
};

typedef be64toh_iterator<uint32_t>       be64toh_iterator_nonconst;
typedef be64toh_iterator<const uint32_t> be64toh_iterator_const;

#endif // BE64TOH_ITERATOR
