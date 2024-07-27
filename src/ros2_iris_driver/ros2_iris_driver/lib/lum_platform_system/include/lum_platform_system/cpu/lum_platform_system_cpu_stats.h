// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_SYSTEM_CPU_LUM_PLATFORM_SYSTEM_CPU_STATS_H
#define LUM_PLATFORM_SYSTEM_CPU_LUM_PLATFORM_SYSTEM_CPU_STATS_H

#include <string>
#include <vector>

namespace lum {
namespace platform {
namespace system {
namespace cpu_stats {

///@brief This class helps calculate system cpu's utilization, ram, temperature in real time
class CpuStatsManager
{

  std::vector<std::size_t>
    previous_idle_time_; ///< this stores the previously recorded cpu idle time per core
  std::vector<std::size_t> previous_total_time_; ///< this is a list of cpu total time per core
  std::vector<std::int64_t>
    cpu_load_; ///< The list of cpu load per core that is calculated every heartbeat
  std::int8_t temperature_sensor_id_{-1}; ///< Temperature Sensor index that is associated with the
  ///< cpu. Every system has a different index
  std::vector<std::string> memory_usage_cmd_; ///< Command for memory usage
  bool is_intialized_{false};

  ///@brief Function that updates the CPU utilization from the file when called
  void updateCpuTimes();

  ///@brief Function takes in a list of strings (cpu times) extracted from a text file /proc/stats
  ///@param[in] A list of cpu times corresponding to single core
  ///@param[in] Index of that core
  void calculateCpuUtilization(const std::vector<std::size_t>& cpu_times, const std::size_t& index);

  ///@brief Function to convert from string to integer
  ///@param[in] Input string representing a number
  ///@param[in] default value incase of wrong input
  ///@return number string converted to integer type
  std::int64_t getIntFromString(const std::string& str, const std::int64_t default_value = -1);

  ///@brief Function to initialize the Cpu structs (reading num of cores, initialize the structs
  /// with sizes)
  void initCpuInfo();

public:
  ///@brief Constructor
  CpuStatsManager();

  ///@brief Function that returns the list of utilization per core in percentage
  ///@return A list of utilization per core in percentage. Invalid operations will have -1 as output
  std::vector<std::int64_t>& getCpuUtilizationList();

  ///@brief Function to get Cpu Ram Usage
  ///@return Memory usage in uin64t_t in kilobytes. Returns -1 if operation fails
  std::int64_t getCpuMemoryUsage();

  ///@brief Function to get Cpu temperature
  ///@return Temperature in uint64_t in degree C. Returns -1 if operation fails.
  std::int64_t getCpuTemperature();
};
} // namespace cpu_stats
} // namespace system
} // namespace platform
} // namespace lum

#endif
