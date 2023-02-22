#include <opencv2/core/core.hpp>
#include "System.h"
#include "Converter.h"
#include "Eigen/Core"
#include "sophus/se3.hpp"

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/time.hpp"
#include <cart_interfaces/srv/scale_factor.hpp>

#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/utils.h>

#include <map>

using std::placeholders::_1;
using std::placeholders::_2;

class Mono : public rclcpp::Node
{
  public:
    Mono();
    ~Mono();

  private:
    void timer_callback();
    void load_params();
    void srv_callback(const std::shared_ptr<cart_interfaces::srv::ScaleFactor::Request> request,
          std::shared_ptr<cart_interfaces::srv::ScaleFactor::Response> response);
    void set_offset(std::string from_id, long unsigned int map_id);
    void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
    void publish_pose(Sophus::SE3f twc, rclcpp::Time msg_time) const;
    void sendTransform(tf2::Transform trf, rclcpp::Time msg_time) const;
    tf2::Transform SE3f_to_tfTransform(Sophus::SE3f T_SE3f) const;
    void publish_tf_transform(Sophus::SE3f T_SE3f, rclcpp::Time msg_time);
    tf2::Transform TransformToTarget (tf2::Transform tf_in,
                                      std::string frame_in, std::string frame_target);

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_cb_handle_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tcw_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    rclcpp::Service<cart_interfaces::srv::ScaleFactor>::SharedPtr service;

    ORB_SLAM3::System * orb_slam;
    long unsigned int current_map_id;
    int tracking_state;
    double scale_factor_param;

    std::map<long unsigned int, tf2::Transform> tf_offsets;
    tf2::Transform current_transform;

    std::string map_frame_id_param;
    std::string camera_frame_id_param;
    std::string target_frame_id_param;
    std::string voc_file_name_param;
    std::string settings_file_name_param;
    bool use_viewer_param;
};
