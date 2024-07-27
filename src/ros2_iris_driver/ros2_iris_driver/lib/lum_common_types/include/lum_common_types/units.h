// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_UNITS_H
#define LUM_COMMON_TYPES_UNITS_H

#include <cmath>
#include <type_traits>

namespace lum {
namespace common {
namespace types {
namespace units {

/// @namespace lum::common::types::units Data types for SI unit values

/// @brief Distance in meters.
/// @details A zero-overhead class for ensuring unit safety for meter distance values. With it, one
/// cannot forget to convert to the desired distance units before use.
/// @tparam T The floating-point type for the value
/// @tparam Enable Only floating point types are accepted. All others rejected at compile-time.
template <typename T,
          typename Enable = typename std::enable_if<std::is_floating_point<T>::value>::type>
class DistanceInMeters
{
public:
  /// @brief The exposed floating point type
  using Type = T;

  /// @brief Create distance.
  /// @param[in] distance_in_meters Value in meters.
  constexpr explicit DistanceInMeters(const T distance_in_meters = T(0))
      : distance_in_meters_{distance_in_meters}
  {
  }

  /// @brief Get distance.
  /// @return Value in meters.
  constexpr T inMeters() const { return distance_in_meters_; }

private:
  T distance_in_meters_{};
};

// Forward declaration for mutual use
template <typename T, typename Enable>
class AngleInDegrees;

/// @brief Angle in radians.
/// @details A zero-overhead class for ensuring unit safety for radian angle values. With it, one
/// cannot forget to convert to the desired angle units before use.
/// @tparam T The floating-point type for the value
/// @tparam Enable Only floating point types are accepted. All others rejected at compile-time.
template <typename T,
          typename Enable = typename std::enable_if<std::is_floating_point<T>::value>::type>
class AngleInRadians
{
public:
  /// @brief The exposed floating point type
  using Type = T;

  /// @brief Create angle.
  /// @param[in] angle_in_radians Value in radians.
  constexpr explicit AngleInRadians(const T angle_in_radians = T(0))
      : angle_in_radians_{angle_in_radians}
  {
  }

  /// @brief Create angle.
  /// @param[in] angle_in_degrees Value in degrees.
  constexpr explicit AngleInRadians(const AngleInDegrees<T, Enable> angle_in_degrees);

  /// @brief Get angle.
  /// @return Value in degrees.
  constexpr T inDegrees() const { return angle_in_radians_ * static_cast<T>(180.0 / M_PI); }

  /// @brief Get angle.
  /// @return Value in radians.
  constexpr T inRadians() const { return angle_in_radians_; }

private:
  T angle_in_radians_{};
};

/// @brief Angle in degrees.
/// @details A zero-overhead class for ensuring unit safety for degree angle angles. With it, one
/// cannot forget to convert to the desired angle units before use.
/// @tparam T The floating-point type for the value
/// @tparam Enable Only floating point types are accepted. All others rejected at compile-time.
template <typename T,
          typename Enable = typename std::enable_if<std::is_floating_point<T>::value>::type>
class AngleInDegrees
{
public:
  /// @brief The exposed floating point type
  using Type = T;

  /// @brief Create angle.
  /// @param[in] angle_in_degrees Value in degrees.
  constexpr explicit AngleInDegrees(const T angle_in_degrees = T(0))
      : angle_in_degrees_{angle_in_degrees}
  {
  }

  /// @brief Create angle.
  /// @param[in] angle_in_radians Value in radians.
  constexpr explicit AngleInDegrees(const AngleInRadians<T, Enable> angle_in_radians)
      : angle_in_degrees_{angle_in_radians.inDegrees()}
  {
  }

  /// @brief Get angle.
  /// @return Value in degrees.
  constexpr T inDegrees() const { return angle_in_degrees_; }

  /// @brief Get angle.
  /// @return Value in radians.
  constexpr T inRadians() const { return angle_in_degrees_ * static_cast<T>(M_PI / 180.0); }

private:
  T angle_in_degrees_{};
};

// Define the conversion constructor, now that both classes are defined
template <typename T, typename Enable>
constexpr AngleInRadians<T, Enable>::AngleInRadians(
  const AngleInDegrees<T, Enable> angle_in_degrees)
    : angle_in_radians_{angle_in_degrees.inRadians()}
{
}

// Forward declaration for mutual use
template <typename T, typename Enable>
class TemperatureInKelvin;
template <typename T, typename Enable>
class TemperatureInFahrenheit;

/// @brief Temperature in Celsius.
/// @details A zero-overhead class for ensuring unit safety for Celsius temperature values. With it,
/// one cannot forget to convert to the desired temperature units before use.
/// @tparam T The floating-point type for the value
/// @tparam Enable Only floating point types are accepted. All others rejected at compile-time.
template <typename T,
          typename Enable = typename std::enable_if<std::is_floating_point<T>::value>::type>
class TemperatureInCelsius
{
public:
  /// @brief The exposed floating point type
  using Type = T;

  /// @brief Create temperature.
  /// @param[in] temp_in_celsius Value in Celsius.
  constexpr explicit TemperatureInCelsius(const T temp_in_celsius = T(0))
      : temp_in_celsius_{temp_in_celsius}
  {
  }

  /// @brief Create temperature.
  /// @param[in] temp_in_kelvin Value in Kelvin.
  constexpr explicit TemperatureInCelsius(const TemperatureInKelvin<T, Enable> temp_in_kelvin);

  /// @brief Create temperature.
  /// @param[in] temp_in_fahrenheit Value in Fahrenheit.
  constexpr explicit TemperatureInCelsius(
    const TemperatureInFahrenheit<T, Enable> temp_in_fahrenheit);

  /// @brief Get temperature.
  /// @return Value in Kelvin.
  constexpr T inKelvin() const { return temp_in_celsius_ + static_cast<T>(273.15); }

  /// @brief Get temperature.
  /// @return Value in Fahrenheit.
  constexpr T inFahrenheit() const
  {
    return (temp_in_celsius_ * static_cast<T>(9.0 / 5.0)) + static_cast<T>(32);
  }

  /// @brief Get temperature.
  /// @return Value in Celsius.
  constexpr T inCelsius() const { return temp_in_celsius_; }

private:
  T temp_in_celsius_{};
};

/// @brief Temperature in Kelvin.
/// @details A zero-overhead class for ensuring unit safety for kelvin temperature values. With it,
/// one cannot forget to convert to the desired temperature units before use.
/// @tparam T The floating-point type for the value
/// @tparam Enable Only floating point types are accepted. All others rejected at compile-time.
template <typename T,
          typename Enable = typename std::enable_if<std::is_floating_point<T>::value>::type>
class TemperatureInKelvin
{
public:
  /// @brief The exposed floating point type
  using Type = T;

  /// @brief Create temperature.
  /// @param[in] temp_in_kelvin Value in Kelvin.
  constexpr explicit TemperatureInKelvin(const T temp_in_kelvin = T(0))
      : temp_in_kelvin_{temp_in_kelvin}
  {
  }

  /// @brief Create temperature.
  /// @param[in] temp_in_celsius Value in Celsius.
  constexpr explicit TemperatureInKelvin(const TemperatureInCelsius<T, Enable> temp_in_celsius)
      : temp_in_kelvin_{temp_in_celsius.inKelvin()}
  {
  }

  /// @brief Create temperature.
  /// @param[in] temp_in_fahrenheit Value in Fahrenheit.
  constexpr explicit TemperatureInKelvin(
    const TemperatureInFahrenheit<T, Enable> temp_in_fahrenheit);

  /// @brief Get temperature.
  /// @return Value in Celsius.
  constexpr T inCelsius() const { return temp_in_kelvin_ - static_cast<T>(273.15); }

  /// @brief Get temperature.
  /// @return Value in Fahrenheit.
  constexpr T inFahrenheit() const
  {
    return (temp_in_kelvin_ - static_cast<T>(273.15)) * static_cast<T>(9.0 / 5.0) +
           static_cast<T>(32);
  }

  /// @brief Get temperature.
  /// @return Value in Kelvin.
  constexpr T inKelvin() const { return temp_in_kelvin_; }

private:
  T temp_in_kelvin_{};
};

/// @brief Temperature in Fahrenheit.
/// @details A zero-overhead class for ensuring unit safety for Fahrenheit temperature values. With
/// it, one cannot forget to convert to the desired temperature units before use.
/// @tparam T The floating-point type for the value
/// @tparam Enable Only floating point types are accepted. All others rejected at compile-time.
template <typename T,
          typename Enable = typename std::enable_if<std::is_floating_point<T>::value>::type>
class TemperatureInFahrenheit
{
public:
  /// @brief The exposed floating point type
  using Type = T;

  /// @brief Create temperature.
  /// @param[in] temp_in_fahrenheit Value in Fahrenheit.
  constexpr explicit TemperatureInFahrenheit(const T temp_in_fahrenheit = T(0))
      : temp_in_fahrenheit_{temp_in_fahrenheit}
  {
  }

  /// @brief Create temperature.
  /// @param[in] temp_in_celsius Value in Celsius.
  constexpr explicit TemperatureInFahrenheit(const TemperatureInCelsius<T, Enable> temp_in_celsius)
      : temp_in_fahrenheit_{temp_in_celsius.inFahrenheit()}
  {
  }

  /// @brief Create temperature.
  /// @param[in] temp_in_kelvin Value in Kelvin.
  constexpr explicit TemperatureInFahrenheit(const TemperatureInKelvin<T, Enable> temp_in_kelvin)
      : temp_in_fahrenheit_{temp_in_kelvin.inFahrenheit()}
  {
  }

  /// @brief Get temperature.
  /// @return Value in Celsius.
  constexpr T inCelsius() const
  {
    return (temp_in_fahrenheit_ - static_cast<T>(32)) * static_cast<T>(5.0 / 9.0);
  }

  /// @brief Get temperature.
  /// @return Value in Kelvin.
  constexpr T inKelvin() const
  {
    return (temp_in_fahrenheit_ - static_cast<T>(32)) * static_cast<T>(5.0 / 9.0) +
           static_cast<T>(273.15);
  }

  /// @brief Get temperature.
  /// @return Value in Fahrenheit.
  constexpr T inFahrenheit() const { return temp_in_fahrenheit_; }

private:
  T temp_in_fahrenheit_{};
};

// Define the conversion constructors
template <typename T, typename Enable>
constexpr TemperatureInCelsius<T, Enable>::TemperatureInCelsius(
  const TemperatureInFahrenheit<T, Enable> temp_in_fahrenheit)
    : temp_in_celsius_{temp_in_fahrenheit.inCelsius()}
{
}
template <typename T, typename Enable>
constexpr TemperatureInCelsius<T, Enable>::TemperatureInCelsius(
  const TemperatureInKelvin<T, Enable> temp_in_kelvin)
    : temp_in_celsius_{temp_in_kelvin.inCelsius()}
{
}
template <typename T, typename Enable>
constexpr TemperatureInKelvin<T, Enable>::TemperatureInKelvin(
  const TemperatureInFahrenheit<T, Enable> temp_in_fahrenheit)
    : temp_in_kelvin_{temp_in_fahrenheit.inKelvin()}
{
}

/// @brief Frequency in Hertz.
/// @details A zero-overhead class for ensuring unit safety for frequency values. With
/// it, one cannot forget to convert to the desired frequency units before use.
/// @tparam T The floating-point type for the value
/// @tparam Enable Only floating point types are accepted. All others rejected at compile-time.
template <typename T,
          typename Enable = typename std::enable_if<std::is_floating_point<T>::value>::type>
class FrequencyInHertz
{
public:
  /// @brief The exposed floating point type
  using Type = T;

  /// @brief Create frequency.
  /// @param[in] frequency_in_hertz Value in Hertz.
  constexpr explicit FrequencyInHertz(const T frequency_in_hertz = T(0))
      : frequency_in_hertz_{frequency_in_hertz}
  {
  }

  /// @brief Get frequency.
  /// @return Value in Hertz.
  constexpr T inHertz() const { return frequency_in_hertz_; }

  /// @brief Get frequency.
  /// @return Value as period of time.
  constexpr T asPeriod() const { return T(1) / frequency_in_hertz_; }

private:
  T frequency_in_hertz_{};
};

} // namespace units
} // namespace types
} // namespace common
} // namespace lum

#endif
