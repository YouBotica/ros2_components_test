#ifndef LIBRESIM__H
#define LIBRESIM__H

#include "schemas/compressed.capnp.h"
#include "schemas/detector.capnp.h"
#include "schemas/detectorcalibration.capnp.h"
#include "schemas/pointcloud.capnp.h"
#include "schemas/roic705.capnp.h"
#include "schemas/semver.capnp.h"
#include "schemas/tdc_metadata.capnp.h"

#include "resim_error_mode.hpp"

#include <type_traits>

/**
 * LibReSim provides methods to convert between two stages of the LiDAR
 * Algorithms Pipeline. To convert between two types, instantiate the LibReSim
 * struct with the appropriate From/To Types. For the sake of performance,
 * choose the From/To Types to do the most computation possible in a single
 * step. While chaining conversions from `LibReSim<A, B> -> LibReSim<B, C>`
 * should be identical to `LibReSim<A, C>`, the chained version may perform
 * significantly less well due to buffering and conversions between the steps.
 */

namespace ReSim
{

/**
 * @brief The bare minimum conversions to be supported by a LibReSim
 */
template <typename FromType, typename ToType> struct LibReSimInterface
{
	LibReSimInterface() : ERROR_MODE_{PRINT} {};
	LibReSimInterface(ReSimErrorMode error_mode) : ERROR_MODE_{error_mode} {};

	virtual ~LibReSimInterface()                                                        = default;
	virtual void Process(const typename FromType::Reader &, typename ToType::Builder &) = 0;

	const ReSimErrorMode ERROR_MODE_;
};

/**
 * @brief Do not support conversions unless an implementation is provided.
 *
 * LibReSim uses specialization to select the appropriate implementation of a
 * conversion. If specialization fails to find an implementation, i.e. the
 * requested conversion is unsupported, this default implementation is
 * selected. Inheriting std::false_type, however, will cause compilation to fail
 * since LibReSim verifies that an implementation exists by checking that it is
 * true_type.
 *
 * WARNING! This class should never be used manually!
 */
template <typename FromType, typename ToType> struct LibReSimImpl : public std::false_type
{
	LibReSimImpl() = delete;
};

/**
 * @brief Convert FromType to ToType using the Algo Pipeline
 *
 * LibReSim selects the appropriate implementation to convert between the
 * requested types. Implementations inherit true_type, while the default
 * (unimplemented) is false_type. SFINAE provides a compilation error if a
 * specialization is not found.
 */
template <typename FromType, typename ToType, ReSimErrorMode error_mode_ = PRINT,
          // Provide a compile error if a specialization isn't found
          typename = typename std::enable_if<LibReSimImpl<FromType, ToType>::value>::type>
struct LibReSim : LibReSimImpl<FromType, ToType>
{
	static_assert(std::is_base_of<LibReSimInterface<FromType, ToType>, LibReSimImpl<FromType, ToType>>::value,
	              "LibReSim Bug! LibReSimImpls should implement the interface.");

	// Use the selected LibReSimImpl's constructor
	using LibReSimImpl<FromType, ToType>::LibReSimImpl;
};

} // namespace ReSim

#endif // LIBRESIM__H
