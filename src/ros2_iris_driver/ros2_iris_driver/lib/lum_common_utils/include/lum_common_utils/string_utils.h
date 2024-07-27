// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_UTILS_STRING_UTILS_H
#define LUM_COMMON_UTILS_STRING_UTILS_H

#include <algorithm>
#include <array>
#include <cctype>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

#include <lum_common_types/utils.h>

namespace lum {
namespace common {
namespace utils {

/// @brief Helper function to extract string between 2 strings
/// @param input_string [in] input string
/// @param first_delimiter [in] first delimiter to begin filtering
/// @param second_delimiter [in] seconds delimiter where filtering of string should end
inline std::string filterStringBetweenTwoSubstrings(const std::string& input_string,
                                                    const std::string& first_delimiter,
                                                    const std::string& second_delimiter)
{
  const auto first = input_string.find(first_delimiter);
  const auto last = input_string.find(second_delimiter);
  std::string filtered_string;
  if (first > input_string.size() || last > input_string.size())
  {
    return filtered_string;
  }

  filtered_string =
    input_string.substr(first + first_delimiter.size(), last - first - first_delimiter.size());
  return filtered_string;
}

/// @brief Helper function to remove delimiters and return vector of strings
/// @param str [in] input string
/// @param delimiter [in] delimiter
/// @return vector of strings separated by delimiter
// NOLINTNEXTLINE(misc-definitions-in-headers)
inline std::vector<std::string> filterStringWithDelimeter(const std::string& str,
                                                          const std::string& delimiter)
{
  std::vector<std::string> strings;
  if (str.empty())
  {
    return strings;
  }
  std::string::size_type pos = 0;
  std::string::size_type prev = 0;
  while ((pos = str.find(delimiter, prev)) != std::string::npos)
  {
    strings.push_back(str.substr(prev, pos - prev));
    prev = pos + 1;
  }

  // To get the last substring (or only, if delimiter is not found)
  strings.push_back(str.substr(prev));

  return strings;
}

/// @brief String replacement all instances of one substring to another
/// @param [out] str output string
/// @param [in] source source substring being replaced
/// @param [in] target target substring that is replacing
/// @return number of replacements being made
// NOLINTNEXTLINE(misc-definitions-in-headers)
inline std::uint32_t
strReplaceAll(std::string& str, const std::string& source, const std::string& target)
{
  if (source.empty())
  {
    return 0;
  }

  // deterine how much do we advance after each replacement
  const size_t increment = (target.find(source) != std::string::npos)
                             ? target.length() // if target contains source we will advance potition
                                               // to target length to avoid recursion
                             : 0;

  std::uint32_t replacements{0}; // number of replacements being made
  size_t pos{0};                 // current position
  while (true)
  {
    pos = str.find(source, pos);
    if (pos == std::string::npos)
    {
      break; // we reached the end
    }
    str.replace(pos, source.length(), target);
    pos += increment;
    ++replacements;
  }

  return replacements;
}

/// @brief Trim spaces from from beginning of the string (in place)
/// @param [in, out] str string to be trimmed
inline void strLeftTrim(std::string& str)
{
  str.erase(str.begin(), std::find_if(str.begin(), str.end(), [](unsigned char ch) {
              return std::isspace(ch) == 0;
            }));
}

/// @brief Trim spaces from from end of the string (in place)
/// @param [in, out] str string to be trimmed
inline void strRightTrim(std::string& str)
{
  str.erase(
    std::find_if(str.rbegin(), str.rend(), [](unsigned char ch) { return std::isspace(ch) == 0; })
      .base(),
    str.end());
}

/// @brief Trim spaces from from both ends of the string (in place)
/// @param [in, out] str string to be trimmed
inline void strTrim(std::string& str)
{
  strLeftTrim(str);
  strRightTrim(str);
}

/// @brief Transforms string to lowercase (in place)
/// @param [in] input string to be made all lowercase
inline void toLower(std::string& input)
{
  std::transform(input.begin(), input.end(), input.begin(), [](auto c) {
    return static_cast<decltype(c)>(std::tolower(c));
  });
}

/// @brief Checks the given input string if ends with ending_candidate
/// @param [in] input string to be compared, e.g. a full file path with an extension
/// @param [in] ending_candidate string to be compared, e.g. a file extension
/// @return returns true if the end of the string exactly matches the ending_candidate, otherwise
/// false
inline bool endsWith(const std::string& input, const std::string& ending_candidate)
{
  if (input.length() < ending_candidate.length())
  {
    return false;
  }
  return 0 == input.compare(input.length() - ending_candidate.length(),
                            ending_candidate.length(),
                            ending_candidate);
}

/// @brief Helper function to join a vector of strings, optionally with a delimiter string
/// @param inputs [in] vector of input strings
/// @param delimiter [in] string to be put in between the different inputs
std::string join(const std::vector<std::string>& inputs, const std::string& delimiter = {""});

/// @brief Converts an array of integers representing a semantic version to a dot-delimited string
/// @param semver The semantic version number to convert
/// @return The string version of the given semver
inline std::string toSemVerString(const std::array<std::uint8_t, 3U>& semver)
{
  using types::utils::toPrintableInt;
  std::ostringstream stream;
  stream << toPrintableInt(semver[0]) << "." << toPrintableInt(semver[1]) << "."
         << toPrintableInt(semver[2]);
  return stream.str();
}

} // namespace utils
} // namespace common
} // namespace lum

#endif // LUM_COMMON_UTILS_STRING_UTILS_H
