#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/utils.h>

class OrbMapTf : public rclcpp::Node
{
public:
  OrbMapTf()
  : Node("orb_map_tf")
  {
    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "orb/odom", 10,
      std::bind(&OrbMapTf::handle_odometry, this, std::placeholders::_1));
  }

private:
  void handle_odometry(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
    geometry_msgs::msg::TransformStamped msg_wb;
    
    msg_wb.transform.translation.x = msg->pose.pose.position.x;
    msg_wb.transform.translation.y = msg->pose.pose.position.y;
    msg_wb.transform.translation.z = msg->pose.pose.position.z;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    msg_wb.transform.rotation.x = msg->pose.pose.orientation.x;
    msg_wb.transform.rotation.y = msg->pose.pose.orientation.y;
    msg_wb.transform.rotation.z = msg->pose.pose.orientation.z;
    msg_wb.transform.rotation.w = msg->pose.pose.orientation.w;

    tf2::Transform twb;
    tf2::fromMsg(msg_wb.transform, twb);
    tf2::Transform two = transform_to_target(twb, msg->child_frame_id, "odom");

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = "map";
    t.child_frame_id = "odom";

    tf2::convert(two, t.transform);
    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  tf2::Transform transform_to_target(tf2::Transform tf_in,
                                   std::string frame_in, std::string frame_target)
  {
    tf2::Transform tf_orig2target;
    tf2::Transform tf_map2target;
    try
    {
        // Get the transform from camera to target
        geometry_msgs::msg::TransformStamped tf_msg = tf_buffer_->lookupTransform(frame_in, frame_target, tf2::TimePointZero);
        // Convert to tf2
        tf2::fromMsg(tf_msg.transform, tf_orig2target);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_INFO(this->get_logger(), "%s", ex.what());
        tf_orig2target.setIdentity();
    }
    // Transform from map to target
    tf_map2target = tf_in * tf_orig2target;
    return tf_map2target;
  }


  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrbMapTf>());
  rclcpp::shutdown();
  return 0;
}