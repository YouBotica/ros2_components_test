// Generated by gencpp from file lum_ros_lidar_sensor_msgs/UDPPacket2.msg
// This is generated in the PDK repo, please copy any updates from there!
// DO NOT EDIT!

#ifndef LUM_ROS_LIDAR_SENSOR_MSGS_MESSAGE_UDPPACKET2_H
#define LUM_ROS_LIDAR_SENSOR_MSGS_MESSAGE_UDPPACKET2_H

#include <map>
#include <string>
#include <vector>

#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>
#include <ros/serialization.h>
#include <ros/types.h>

namespace lum_ros_lidar_sensor_msgs
{
template <class ContainerAllocator> struct UDPPacket2_
{
	typedef UDPPacket2_<ContainerAllocator> Type;

	UDPPacket2_() : bytes(), size(0), sourcePort(0), destPort(0), checksum(0) {}
	UDPPacket2_(const ContainerAllocator &_alloc) : bytes(_alloc), size(0), sourcePort(0), destPort(0), checksum(0)
	{
		(void) _alloc;
	}

	typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other> _bytes_type;
	_bytes_type                                                                                bytes;

	typedef uint16_t _size_type;
	_size_type       size;

	typedef uint16_t _sourcePort_type;
	_sourcePort_type sourcePort;

	typedef uint16_t _destPort_type;
	_destPort_type   destPort;

	typedef uint16_t _checksum_type;
	_checksum_type   checksum;

	typedef boost::shared_ptr<::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator>>       Ptr;
	typedef boost::shared_ptr<::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator> const> ConstPtr;

}; // struct UDPPacket2_

typedef ::lum_ros_lidar_sensor_msgs::UDPPacket2_<std::allocator<void>> UDPPacket2;

typedef boost::shared_ptr<::lum_ros_lidar_sensor_msgs::UDPPacket2>       UDPPacket2Ptr;
typedef boost::shared_ptr<::lum_ros_lidar_sensor_msgs::UDPPacket2 const> UDPPacket2ConstPtr;

// constants requiring out of line definition

template <typename ContainerAllocator>
std::ostream &operator<<(std::ostream &s, const ::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator> &v)
{
	ros::message_operations::Printer<::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator>>::stream(s, "", v);
	return s;
}

template <typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator1> &lhs,
                const ::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator2> &rhs)
{
	return lhs.bytes == rhs.bytes && lhs.size == rhs.size && lhs.sourcePort == rhs.sourcePort && lhs.destPort == rhs.destPort &&
	       lhs.checksum == rhs.checksum;
}

template <typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator1> &lhs,
                const ::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator2> &rhs)
{
	return !(lhs == rhs);
}

} // namespace lum_ros_lidar_sensor_msgs

namespace ros
{
namespace message_traits
{

template <class ContainerAllocator> struct IsFixedSize<::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator>> : FalseType
{
};

template <class ContainerAllocator>
struct IsFixedSize<::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator> const> : FalseType
{
};

template <class ContainerAllocator> struct IsMessage<::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator>> : TrueType
{
};

template <class ContainerAllocator>
struct IsMessage<::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator> const> : TrueType
{
};

template <class ContainerAllocator> struct HasHeader<::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator>> : FalseType
{
};

template <class ContainerAllocator>
struct HasHeader<::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator> const> : FalseType
{
};

template <class ContainerAllocator> struct MD5Sum<::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator>>
{
	static const char *value() { return "bbf96b0e38b89d2b2208f0d8c6704cd8"; }

	static const char *   value(const ::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator> &) { return value(); }
	static const uint64_t static_value1 = 0xbbf96b0e38b89d2bULL;
	static const uint64_t static_value2 = 0x2208f0d8c6704cd8ULL;
};

template <class ContainerAllocator> struct DataType<::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator>>
{
	static const char *value() { return "lum_ros_lidar_sensor_msgs/UDPPacket2"; }

	static const char *value(const ::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator> &) { return value(); }
};

template <class ContainerAllocator> struct Definition<::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator>>
{
	static const char *value()
	{
		return "uint8[] bytes\n"
		       "uint16 size\n"
		       "uint16 sourcePort\n"
		       "uint16 destPort\n"
		       "uint16 checksum\n";
	}

	static const char *value(const ::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator> &) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template <class ContainerAllocator> struct Serializer<::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator>>
{
	template <typename Stream, typename T> inline static void allInOne(Stream &stream, T m)
	{
		stream.next(m.bytes);
		stream.next(m.size);
		stream.next(m.sourcePort);
		stream.next(m.destPort);
		stream.next(m.checksum);
	}

	ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct UDPPacket2_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template <class ContainerAllocator> struct Printer<::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator>>
{
	template <typename Stream>
	static void stream(Stream &s, const std::string &indent, const ::lum_ros_lidar_sensor_msgs::UDPPacket2_<ContainerAllocator> &v)
	{
		s << indent << "bytes[]" << std::endl;
		for (size_t i = 0; i < v.bytes.size(); ++i)
		{
			s << indent << "  bytes[" << i << "]: ";
			Printer<uint8_t>::stream(s, indent + "  ", v.bytes[i]);
		}
		s << indent << "size: ";
		Printer<uint16_t>::stream(s, indent + "  ", v.size);
		s << indent << "sourcePort: ";
		Printer<uint16_t>::stream(s, indent + "  ", v.sourcePort);
		s << indent << "destPort: ";
		Printer<uint16_t>::stream(s, indent + "  ", v.destPort);
		s << indent << "checksum: ";
		Printer<uint16_t>::stream(s, indent + "  ", v.checksum);
	}
};

} // namespace message_operations
} // namespace ros

#endif // LUM_ROS_LIDAR_SENSOR_MSGS_MESSAGE_UDPPACKET2_H