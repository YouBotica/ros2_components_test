// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_SWC_INTERFACE_ADAPTER_H
#define LUM_COMMON_SWC_SWC_INTERFACE_ADAPTER_H

#include <memory>

#include <lum_common_swc/i_software_component.h>

namespace lum {
namespace common {
namespace swc {

/// @brief Template interface to allow dependency injection of types where both ISoftwarecomponent
/// and another interface are used like ICommunication.
template <typename T>
class ISwcInterfaceAdapter
{
public:
  using Type = T;

  ISwcInterfaceAdapter() = default;
  virtual ~ISwcInterfaceAdapter() = default;
  ISwcInterfaceAdapter(const ISwcInterfaceAdapter&) = delete;
  ISwcInterfaceAdapter& operator=(const ISwcInterfaceAdapter&) = delete;
  ISwcInterfaceAdapter(ISwcInterfaceAdapter&&) = delete;
  ISwcInterfaceAdapter& operator=(ISwcInterfaceAdapter&&) = delete;

  /// @brief Gets a reference to the ISoftwareComponent interface.
  /// @return The ISoftwareComponent interface.
  virtual ISoftwareComponent& getSwc() = 0;

  /// @brief Gets a reference to the template interface type.
  /// @return The template interface type.
  virtual T& getType() = 0;
};

/// @brief Implementation of the ISwcInterfaceAdapter Interface.
template <typename T, typename U>
class SwcInterfaceAdapter : public ISwcInterfaceAdapter<T>
{
public:
  explicit SwcInterfaceAdapter(U& elem) : elem_{elem}
  {
    static_assert(std::is_base_of<ISoftwareComponent, U>::value,
                  "Has to derive from ISoftwareComponent");
    static_assert(std::is_base_of<T, U>::value, "Has to derive from specified interface type");
  }

  ~SwcInterfaceAdapter() override = default;
  SwcInterfaceAdapter(const SwcInterfaceAdapter<T, U>& rhs) : elem_{rhs.elem_} {}
  SwcInterfaceAdapter& operator=(const SwcInterfaceAdapter<T, U>& rhs)
  {
    elem_ = rhs.elem_;
    return *this;
  }
  SwcInterfaceAdapter(SwcInterfaceAdapter<T, U>&& rhs) noexcept : elem_{rhs.elem_} {}
  SwcInterfaceAdapter& operator=(SwcInterfaceAdapter<T, U>&& rhs) noexcept
  {
    elem_ = rhs.elem_;
    return *this;
  }

  /// @copydoc ISwcInterfaceAdapter::getSwc
  common::swc::ISoftwareComponent& getSwc() override { return elem_; };

  /// @copydoc ISwcInterfaceAdapter::getType
  T& getType() override { return elem_; };

private:
  U& elem_;
};

/// @brief Utility to construct SwcInterfaceAdapters.
template <typename T, typename U>
SwcInterfaceAdapter<T, U> makeSwcAdapter(U& elem)
{
  return SwcInterfaceAdapter<T, U>{elem};
}

/// @brief Utility to construct SwcInterfaceAdapters.
template <typename T, typename U>
std::unique_ptr<SwcInterfaceAdapter<T, U>> makeUniqueSwcAdapter(U& elem)
{
  return std::make_unique<SwcInterfaceAdapter<T, U>>(elem);
}

} // namespace swc
} // namespace common
} // namespace lum

#endif
