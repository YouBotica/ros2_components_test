#ifndef TIME_HELPERS_HPP
#define TIME_HELPERS_HPP

static const uint8_t SYS_CLK_PERIOD_NS{5};

template <typename timeT, typename = typename std::enable_if<std::is_arithmetic<timeT>::value, timeT>::type>
static inline timeT sys_time_to_ns(timeT sys_time)
{
	return static_cast<timeT>(sys_time * SYS_CLK_PERIOD_NS);
}

template <typename timeT, typename = typename std::enable_if<std::is_arithmetic<timeT>::value, timeT>::type>
static inline timeT ns_to_sys_time(timeT ns)
{
	return static_cast<timeT>(ns / SYS_CLK_PERIOD_NS);
}

#endif // TIME_HELPERS_HPP
