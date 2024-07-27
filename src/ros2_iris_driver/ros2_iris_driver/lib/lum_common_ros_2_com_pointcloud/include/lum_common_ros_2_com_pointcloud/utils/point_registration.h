// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_UTILS_POINT_REGISTRATION_H
#define LUM_COMMON_SWC_UTILS_POINT_REGISTRATION_H

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace lum {
namespace common {
namespace swc {
namespace point_registration {

template <typename PointType>
void registerPointFields();

///@brief this class provide access to static storage of (ros)pcl point fields
template <typename PointType>
class PointFields
{
public:
  static void registerField(const std::string& field_name,
                            const std::uint32_t field_offset,
                            const std::uint8_t data_type,
                            const std::uint32_t field_count = 1U)
  {
    sensor_msgs::msg::PointField field;
    field.name = field_name;
    field.offset = field_offset;
    field.datatype = data_type;
    field.count = field_count;
    fields().emplace_back(field);
  }

  static sensor_msgs::msg::PointCloud2::_fields_type getFields()
  {
    if (fields().empty())
    {
      registerPointFields<PointType>();
    }
    return fields();
  }

private:
  static sensor_msgs::msg::PointCloud2::_fields_type& fields()
  {
    static sensor_msgs::msg::PointCloud2::_fields_type fields{};
    return fields;
  }
};

///@brief this macro registers point fields
#ifndef LUM_COM_REGISTER_ROS_2_POINT_STRUCT
#define LUM_COM_REGISTER_ROS_2_POINT_STRUCT(struct_type)                                            \
  template <>                                                                                      \
  inline void lum::common::swc::point_registration::registerPointFields<struct_type>()
#endif

#ifndef LUM_COM_DECLARE_ROS_2_POINT_FIELD
#define LUM_COM_DECLARE_ROS_2_POINT_FIELD(struct_type, struct_member_name, field_name, field_type)  \
  lum::common::swc::point_registration::PointFields<struct_type>::registerField(                   \
    #field_name, offsetof(struct_type, struct_member_name), sensor_msgs::msg::PointField::field_type);
#endif

#ifndef LUM_COM_DECLARE_ROS_2_POINT_ARRAY_FIELD
#define LUM_COM_DECLARE_ROS_2_POINT_ARRAY_FIELD(                                                    \
  struct_type, struct_member_name, field_name, field_type, array_size)                             \
  lum::common::swc::point_registration::PointFields<struct_type>::registerField(                   \
    #field_name,                                                                                   \
    offsetof(struct_type, struct_member_name),                                                     \
    sensor_msgs::msg::PointField::field_type,                                                           \
    array_size);
#endif

} // namespace point_registration
} // namespace swc
} // namespace common
} // namespace lum

#endif // LUM_COMMON_SWC_UTILS_POINT_REGISTRATION_H
