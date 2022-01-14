/**
 * @file /src/kobuki_bumper2pc.cpp
 *
 * @brief Bumper to pointcloud nodelet class implementation.
 *
 * @author Jorge Santos, Yujin Robot
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "kobuki_bumper2pc/bumper_to_pc.hpp"
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>

namespace kobuki_bumper2pc
{

BumperToPC::BumperToPC(const rclcpp::NodeOptions &options)
    : Node("bumper_to_pc", options),
      P_INF_X(+100*sin(0.34906585)),
      P_INF_Y(+100*cos(0.34906585)),
      N_INF_Y(-100*cos(0.34906585)),
      ZERO(0), prev_bumper(0), prev_cliff(0)
{
  init();
}

BumperToPC::~BumperToPC()
{
  if(pc_pub_) pc_pub_.reset();
  if(sensor_sub_) sensor_sub_.reset();
}

void BumperToPC::init()
{
  this->declare_parameter("pointcloud_radius", 0.25);
  this->declare_parameter("pointcloud_height", 0.04);
  this->declare_parameter("side_point_angle", 0.34906585);
  this->declare_parameter("base_link_name", "base_link");

  pc_radius_ = this->get_parameter("pointcloud_radius").as_double();
  pc_height_ = this->get_parameter("pointcloud_height").as_double();
  auto angle = this->get_parameter("side_point_angle").as_double();
  auto base_link_frame = this->get_parameter("base_link_name").as_string();

  // Lateral points x/y coordinates; we need to store float values to memcopy later
  p_side_x_ = + pc_radius_*sin(angle); // angle degrees from vertical
  p_side_y_ = + pc_radius_*cos(angle); // angle degrees from vertical
  n_side_y_ = - pc_radius_*cos(angle); // angle degrees from vertical

  // Prepare constant parts of the pointcloud message to be  published
  pointcloud_.header.frame_id = base_link_frame;
  pointcloud_.width  = 3;
  pointcloud_.height = 1;
  pointcloud_.fields.resize(3);

  // Set x/y/z as the only fields
  pointcloud_.fields[0].name = "x";
  pointcloud_.fields[1].name = "y";
  pointcloud_.fields[2].name = "z";

  int offset = 0;
  // All offsets are *4, as all field data types are float32
  for (size_t d = 0; d < pointcloud_.fields.size(); ++d, offset += 4)
  {
    pointcloud_.fields[d].count    = 1;
    pointcloud_.fields[d].offset   = offset;
    pointcloud_.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  pointcloud_.point_step = offset;
  pointcloud_.row_step   = pointcloud_.point_step * pointcloud_.width;

  pointcloud_.data.resize(3 * pointcloud_.point_step);
  pointcloud_.is_bigendian = false;
  pointcloud_.is_dense     = true;

  // Bumper/cliff "points" fix coordinates (the others depend on sensor activation/deactivation)

  // y: always 0 for central bumper
  memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[1].offset], &ZERO, sizeof(float));

  // z: constant elevation from base frame
  memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
  memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));
  memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[2].offset], &pc_height_, sizeof(float));

  pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 1);
  sensor_sub_ = this->create_subscription<kobuki_ros_interfaces::msg::SensorState>("cor_sensors", 1,
                std::bind(&BumperToPC::coreSensorCB, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Bumper/cliff pointcloud configured at distance %f and height %f from base frame", pc_radius_, pc_height_);
}

void BumperToPC::coreSensorCB(const kobuki_ros_interfaces::msg::SensorState::ConstSharedPtr msg)
{
  if(pc_pub_->get_subscription_count() == 0) {
    return;
  }

  // We publish just one "no events" pc (with all three points far away) and stop spamming when bumper/cliff conditions disappear
  if (! msg->bumper && ! msg->cliff && ! prev_bumper && ! prev_cliff)
    return;

  prev_bumper = msg->bumper;
  prev_cliff  = msg->cliff;

  // We replicate the sensors order of bumper/cliff event messages: LEFT = 0, CENTER = 1 and RIGHT = 2
  // For any of {left/center/right} with no bumper/cliff event, we publish a faraway point that won't get used
  if ((msg->bumper & kobuki_ros_interfaces::msg::SensorState::BUMPER_LEFT) ||
      (msg->cliff  & kobuki_ros_interfaces::msg::SensorState::CLIFF_LEFT))
  {
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &p_side_y_, sizeof(float));
  }
  else
  {
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &P_INF_Y, sizeof(float));
  }

  if ((msg->bumper & kobuki_ros_interfaces::msg::SensorState::BUMPER_CENTRE) ||
      (msg->cliff  & kobuki_ros_interfaces::msg::SensorState::CLIFF_CENTRE))
  {
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &pc_radius_, sizeof(float));
  }
  else
  {
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
  }

  if ((msg->bumper & kobuki_ros_interfaces::msg::SensorState::BUMPER_RIGHT) ||
      (msg->cliff  & kobuki_ros_interfaces::msg::SensorState::CLIFF_RIGHT))
  {
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &n_side_y_, sizeof(float));
  }
  else
  {
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &N_INF_Y, sizeof(float));
  }

  pointcloud_.header.stamp = msg->header.stamp;
  pc_pub_->publish(pointcloud_);
}

} // namespace kobuki_bumper2pc

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kobuki_bumper2pc::BumperToPC)
