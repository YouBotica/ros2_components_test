// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_EXCEPTIONS_INIT_H
#define LUM_COMMON_EXCEPTIONS_INIT_H

#include <array>
#include <exception>
#include <string>

namespace lum {
namespace common {
namespace exceptions {

/// @namespace lum::common::exceptions Luminar exceptions

/// @brief Luminar exception class
// PRQA S:D0002 2659 1 # deviations/D0002/deviation.md
class GenericException : public std::exception
{
public:
  /// @brief Max length of diagnostic message (without trailing zero)
  static constexpr std::size_t MAX_MESSAGE_LENGTH{511U};

  /// @brief Get exception diagnostics string
  /// @return Pointer to null-terminated diagnostics string
  const char* what() const noexcept final;

  /// @brief Exchange content of two exceptions
  /// @note The static works here for our case and is compliant with AUTOSAR in contrast to the
  /// friend declaration used with traditional ADL techniques.
  /// @param [inout] lhs The first exception
  /// @param [inout] rhs The second exception
  static void swap(GenericException& lhs, GenericException& rhs) noexcept
  {
    using std::swap; // enable ADL
    swap(lhs.message_, rhs.message_);
  }

  ~GenericException() noexcept override = default;

protected:
  // Constructors are protected as only the derived exception classes should be instantiated
  explicit GenericException(const char* const message) noexcept;

  GenericException() noexcept;
  GenericException(GenericException&& rhs) noexcept;
  GenericException& operator=(GenericException&& rhs) noexcept;
  GenericException(const GenericException& rhs) noexcept;
  GenericException& operator=(const GenericException& rhs) noexcept;

  void append(const char* const message_to_append) noexcept;

private:
  std::array<char, MAX_MESSAGE_LENGTH + 1U> message_{};
};

/// @brief Error reading an environment variable
class EnvironmentVariableException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  explicit EnvironmentVariableException(const char* const message) noexcept;

  /// @copydoc GenericException::GenericException
  explicit EnvironmentVariableException(const std::string& message) noexcept
      : EnvironmentVariableException{message.c_str()}
  {
  }
};

/// @brief Error thrown from a communication software component
class CommunicationException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  explicit CommunicationException(const char* const message) noexcept;

  /// @copydoc GenericException::GenericException
  explicit CommunicationException(const std::string& message) noexcept
      : CommunicationException{message.c_str()}
  {
  }
};

/// @brief Error thrown on string parsing errors
class StringParsingException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  explicit StringParsingException(const char* const message) noexcept;

  /// @copydoc GenericException::GenericException
  explicit StringParsingException(const std::string& message) noexcept
      : StringParsingException{message.c_str()}
  {
  }
};

/// @brief Error thrown on numeric errors
class NumericException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  explicit NumericException(const char* const message) noexcept;

  /// @copydoc GenericException::GenericException
  explicit NumericException(const std::string& message) noexcept : NumericException{message.c_str()}
  {
  }
};

/// @brief Error thrown on invalid function parameter
class InvalidParameterException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  explicit InvalidParameterException(const char* const message) noexcept;

  /// @copydoc GenericException::GenericException
  explicit InvalidParameterException(const std::string& message) noexcept
      : InvalidParameterException{message.c_str()}
  {
  }
};

/// @brief Error file does not exist
class FileNotFoundException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  explicit FileNotFoundException(const char* const message) noexcept;

  /// @copydoc GenericException::GenericException
  explicit FileNotFoundException(const std::string& message) noexcept
      : FileNotFoundException{message.c_str()}
  {
  }
};

/// @brief Error opening and reading input file
class InputFileException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  explicit InputFileException(const char* const message) noexcept;

  /// @copydoc GenericException::GenericException
  explicit InputFileException(const std::string& message) noexcept
      : InputFileException{message.c_str()}
  {
  }
};

/// @brief Error providing a Null dependency to class constructor where not allowed
class NullDependencyException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  explicit NullDependencyException(const char* const message) noexcept;

  /// @copydoc GenericException::GenericException
  explicit NullDependencyException(const std::string& message) noexcept
      : NullDependencyException{message.c_str()}
  {
  }
};

/// @brief Error index on point cloud out of bounds
class PointCloudIndexOutOfBoundsException final : public GenericException
{
public:
  /// @brief Exception constructor
  /// @param [in] value The erroneous index value
  /// @param [in] low The lower permissible bound
  /// @param [in] high The upper permissible bound
  PointCloudIndexOutOfBoundsException(std::size_t value,
                                      std::size_t low,
                                      std::size_t high) noexcept;

  std::size_t getValue() const noexcept { return value_; }
  std::size_t getLowBound() const noexcept { return low_; }
  std::size_t getHighBound() const noexcept { return high_; }

private:
  std::size_t value_;
  std::size_t low_;
  std::size_t high_;
};

/// @brief Error invalid constructor parameter
class StructuredPointCloudInvalidConstructionException final : public GenericException
{
public:
  /// @brief Exception constructor
  /// @param [in] message Diagnostics message
  /// @param [in] value The erroneous value
  StructuredPointCloudInvalidConstructionException(const char* const message,
                                                   std::size_t value) noexcept;

  /// @brief Delegating constructor
  /// @param [in] message Diagnostics message
  /// @param [in] value The erroneous value
  StructuredPointCloudInvalidConstructionException(const std::string& message,
                                                   const std::size_t value) noexcept
      : StructuredPointCloudInvalidConstructionException{message.c_str(), value}
  {
  }

  std::size_t getValue() const noexcept { return value_; }

private:
  std::size_t value_;
};

/// @brief Error index on structured point cloud out of bounds
class StructuredPointCloudIndexOutOfBoundsException final : public GenericException
{
public:
  /// @brief Exception constructor
  /// @param [in] value The erroneous index value
  /// @param [in] low The lower permissible bound
  /// @param [in] high The upper permissible bound
  StructuredPointCloudIndexOutOfBoundsException(std::size_t value,
                                                std::size_t low,
                                                std::size_t high) noexcept;

  std::size_t getValue() const noexcept { return value_; }
  std::size_t getLowBound() const noexcept { return low_; }
  std::size_t getHighBound() const noexcept { return high_; }

private:
  std::size_t value_;
  std::size_t low_;
  std::size_t high_;
};

/// @brief Error if convex hull size is less than 3 vertices
class ConvexHullInsufficientException final : public GenericException
{
public:
  /// @brief Exception constructor
  ConvexHullInsufficientException() noexcept;
};

/// @brief Error if PCAP playback speed is zero
class InvalidPlaybackSpeedException final : public GenericException
{
public:
  /// @brief Exception constructor
  /// @param [in] value The erroneous index value
  explicit InvalidPlaybackSpeedException(float value) noexcept;

  float getValue() const noexcept { return value_; }

private:
  float value_;
};

/// @brief Error if scenario parsing fails
class ScenarioParsingException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  explicit ScenarioParsingException(const char* const message) noexcept;

  /// @copydoc GenericException::GenericException
  explicit ScenarioParsingException(const std::string& message) noexcept
      : ScenarioParsingException{message.c_str()}
  {
  }
};

/// @brief Error while performing inference on a model
class TensorRTException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  explicit TensorRTException(const char* const message) noexcept;

  /// @copydoc GenericException::GenericException
  explicit TensorRTException(const std::string& message) noexcept
      : TensorRTException{message.c_str()}
  {
  }
};

/// @brief Error in GPU Tensor code
class GPUTensorException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  GPUTensorException() noexcept;
};

/// @brief class for handling exceptions for dimension mismatch in optimizer
class DimensionMisMatchException final : public GenericException
{
public:
  /// @brief Constructor
  /// @param [in] state_dim dimensions of the input state to the optimizer
  /// @param [in] dim dimension of a query state in the optimizer
  DimensionMisMatchException(std::size_t state_dim, std::size_t dim) noexcept;

  std::size_t getStateDim() const noexcept { return state_dim_; }
  std::size_t getDim() const noexcept { return dim_; }

private:
  std::size_t state_dim_;
  std::size_t dim_;
};

/// @brief Error while performing inference on a model
class InferenceException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  InferenceException() noexcept;
};

/// @brief Error trying to compute odometry in precomputed mode
class PrecomputedOdometryException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  PrecomputedOdometryException() noexcept;
};

/// @brief Error when emptying memory cached on GPU by libtorch
class EmptyCacheException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  EmptyCacheException() noexcept;
};

/// @brief Error while reading a binary model file
class ReadBinaryException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  explicit ReadBinaryException(const char* const message) noexcept;

  /// @copydoc GenericException::GenericException
  explicit ReadBinaryException(const std::string& message) noexcept
      : ReadBinaryException{message.c_str()}
  {
  }
};

/// @brief Error when binary model file is not found
class ModelFileNotFoundException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  explicit ModelFileNotFoundException(const char* const message) noexcept;

  /// @copydoc GenericException::GenericException
  explicit ModelFileNotFoundException(const std::string& message) noexcept
      : ModelFileNotFoundException{message.c_str()}
  {
  }
};

/// @brief Exception thrown when two clouds with the same size are expected but we receive different
/// sizes
class CloudSizeMismatchException final : public GenericException
{
public:
  /// @brief Constructor
  /// @param [in] cloud1_size size of the first cloud
  /// @param [in] cloud2_size size of the second cloud
  CloudSizeMismatchException(std::size_t cloud1_size, std::size_t cloud2_size) noexcept;

  std::size_t getCloud1Size() const noexcept { return cloud1_size_; }
  std::size_t getCloud2Size() const noexcept { return cloud2_size_; }

private:
  std::size_t cloud1_size_;
  std::size_t cloud2_size_;
};

/// @brief Error configuring heap memory allocator
class AllocatorException final : public GenericException
{
public:
  /// @brief Constructor
  /// @param [in] block_size Requested block size
  /// @param [in] chunk_size Requested chunk size
  AllocatorException(std::size_t block_size, std::size_t chunk_size) noexcept;

  std::size_t getRequestedBlockSize() const noexcept { return block_size_; }
  std::size_t getRequestedChunkSize() const noexcept { return chunk_size_; }

private:
  std::size_t block_size_;
  std::size_t chunk_size_;
};

/// @brief Error configuring runtime allocator
class RuntimeAllocatorException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  explicit RuntimeAllocatorException(const char* const message) noexcept;

  /// @copydoc GenericException::GenericException
  explicit RuntimeAllocatorException(const std::string& message) noexcept
      : RuntimeAllocatorException{message.c_str()}
  {
  }
};

/// @brief Error allocating heap memory
class RuntimeAllocatorOutOfMemoryException final : public GenericException
{
public:
  /// @brief Constructor
  /// @param [in] requested_amount Number of bytes requested from heap
  explicit RuntimeAllocatorOutOfMemoryException(std::size_t requested_amount) noexcept;

  std::size_t getRequestedBytes() const noexcept { return requested_bytes_; }

private:
  std::size_t requested_bytes_;
};

/// @brief Error allocating heap memory
class HeapMonitorOutOfMemoryException final : public GenericException
{
public:
  /// @brief Constructor
  /// @param [in] requested_amount Number of bytes requested from heap
  explicit HeapMonitorOutOfMemoryException(std::size_t requested_amount) noexcept;

  std::size_t getRequestedBytes() const noexcept { return requested_bytes_; }

private:
  std::size_t requested_bytes_;
};

/// @brief Error of heap monitor when allocations are disabled
class HeapMonitorAllocationsDisabledException final : public GenericException
{
public:
  /// @brief Constructor
  /// @param [in] requested_amount Number of bytes requested from heap
  explicit HeapMonitorAllocationsDisabledException(std::size_t requested_amount) noexcept;

  std::size_t getRequestedBytes() const noexcept { return requested_bytes_; }

private:
  std::size_t requested_bytes_;
};

/// @brief Error of DTC manager when an illegal diagnostic code is encountered
class DtcManagerIllegalDiagnosticCodeException final : public GenericException
{
public:
  /// @copydoc GenericException::GenericException
  explicit DtcManagerIllegalDiagnosticCodeException(const char* const message) noexcept;

  /// @copydoc GenericException::GenericException
  explicit DtcManagerIllegalDiagnosticCodeException(const std::string& message) noexcept
      : DtcManagerIllegalDiagnosticCodeException{message.c_str()}
  {
  }
};

} // namespace exceptions
} // namespace common
} // namespace lum

#endif // LUM_COMMON_EXCEPTIONS_INIT_H
