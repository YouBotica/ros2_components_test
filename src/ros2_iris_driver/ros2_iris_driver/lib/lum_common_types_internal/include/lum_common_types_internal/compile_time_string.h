// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_INTERNAL_COMPILE_TIME_STRING_H
#define LUM_COMMON_TYPES_INTERNAL_COMPILE_TIME_STRING_H

#include <ostream>

// AUTOSAR Rule A16-0-1 (required, implementation, automated)
// The pre-processor shall only be used for unconditional and conditional file inclusion
// and include guards, and using the following directives: (1) #ifndef, (2) #ifdef,
// (3) #if, (4) #if defined, (5) #elif, (6) #else, (7) #define, (8) #endif, (9) #include.

// Exception justification: The functionality of retrieving the current function signature
// as string is not available in C++14 and needs to be compiler-specific. Using these macros
// does not change the preprocessor state or influence the preprocessor instructions mentioned
// in the AUTOSAR rule.
#if defined(_MSC_VER)
#define LUM_THIS_FUNCTION_NAME __FUNCSIG__
#else
#define LUM_THIS_FUNCTION_NAME __PRETTY_FUNCTION__
#endif

namespace lum {
namespace common {
namespace types {
namespace utils {

namespace detail {
template <std::size_t N>
constexpr std::size_t compileTimeLength(const char (&)[N]) noexcept
{
  return N - 1;
}
} // namespace detail

/// @brief String which holds a compile-time constant char literal with compile-time operations on
/// it.
class CompileTimeConstantString
{
public:
  using const_iterator = const char*;

  /// @brief Construct from quoted char literal
  /// @tparam N String length (can be auto-deducted)
  /// @param[in] a String content
  template <std::size_t N>
  constexpr explicit CompileTimeConstantString(const char (&a)[N]) noexcept
      : data_{static_cast<const char*>(a)}, size_{N - 1U}
  {
  }

  /// @brief Construct from quoted char literal
  /// @param[in] n String length
  /// @param[in] p String content
  constexpr CompileTimeConstantString(const char* p, std::size_t n) noexcept : data_{p}, size_{n} {}

  /// @brief Get content
  /// @return Pointer to start of string
  constexpr const char* data() const noexcept { return data_; }
  /// @brief Get length
  /// @return String length in bytes
  constexpr std::size_t size() const noexcept { return size_; }

  /// @brief Get content iterator
  /// @return Iterator pointing to first string char
  constexpr const_iterator begin() const noexcept { return data_; }
  /// @brief Get content iterator
  /// @return Iterator pointing after the last string char
  constexpr const_iterator end() const noexcept { return data_ + size_; }

private:
  const char* const data_;   ///< pointer to char literal
  const std::size_t size_{}; ///< length of char literal in bytes
};

inline std::ostream& operator<<(std::ostream& os, const CompileTimeConstantString& s)
{
  return os.write(s.data(), s.size());
}

/// @brief Get the template type as compile-time constant string by extracting it from the mangled
/// function name.
/// @note This function relies on compiler version-dependent values and may break in the future, but
/// there are tests that will catch this.
template <typename T>
constexpr CompileTimeConstantString getTypeName()
{
  CompileTimeConstantString p{LUM_THIS_FUNCTION_NAME};
#if defined(__clang__)
  constexpr std::size_t LEFT_TRIM =
    detail::compileTimeLength("lum::common::types::utils::CompileTimeConstantString "
                              "lum::common::types::utils::getTypeName() [T = ");
  constexpr std::size_t RIGHT_TRIM = detail::compileTimeLength("]");
#elif defined(__GNUC__)
  constexpr std::size_t LEFT_TRIM =
    detail::compileTimeLength("constexpr lum::common::types::utils::CompileTimeConstantString "
                              "lum::common::types::utils::getTypeName() [with T = ");
  constexpr std::size_t RIGHT_TRIM = detail::compileTimeLength("]");
#elif defined(_MSC_VER)
  constexpr std::size_t LEFT_TRIM =
    detail::compileTimeLength("class lum::common::types::utils::CompileTimeConstantString __cdecl "
                              "lum::common::types::utils::getTypeName<");
  constexpr std::size_t RIGHT_TRIM = detail::compileTimeLength(">(void)");
#endif
  return {p.data() + LEFT_TRIM, p.size() - LEFT_TRIM - RIGHT_TRIM};
}

} // namespace utils
} // namespace types
} // namespace common
} // namespace lum

#endif
