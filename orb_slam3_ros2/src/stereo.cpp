#include "orb_slam3_ros2/stereo.hpp"
#include <memory>
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

Stereo::Stereo() : rclcpp::Node("orb_slam_Stereo"),
                   current_map_id(0), pre_map_id(0), has_transform(false)
{
  load_params();

  tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  if (!publish_raw_param) {
    if (publish_tf_param)
      tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    track_result_publisher = create_publisher<orb_slam_msgs::msg::TrackResult>("stereo/track", 10);
    odom_publisher = create_publisher<nav_msgs::msg::Odometry>("stereo/odom", 10);
  } else {
    twc_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("stereo/twc", 10);
  }

  orb_slam = new ORB_SLAM3::System(
      voc_file_name_param,
      settings_file_name_param,
      ORB_SLAM3::System::STEREO,
      use_viewer_param);

  if (!publish_raw_param && publish_tf_param) {
    tf2::Transform tf0;
    tf0.setIdentity();
    tf_offsets[current_map_id] = tf0;
    // set_offset(tf2::Transform(tf2::Quaternion::getIdentity()), target_frame_id_param, current_map_id);
    current_transform = tf_offsets[current_map_id];
  }

  rclcpp::QoS qos(5);
  auto rmw_qos_profile = qos.get_rmw_qos_profile();
  subscriber_image_left.subscribe(this, image_left_topic_param, rmw_qos_profile);
  subscriber_image_right.subscribe(this, image_right_topic_param, rmw_qos_profile);

  image_sync = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(subscriber_image_left, subscriber_image_right, 5);
  image_sync->registerCallback(std::bind(&Stereo::topic_callback, this, std::placeholders::_1, std::placeholders::_2));

  if (publish_tf_param) {
    timer_ = this->create_wall_timer(
      50ms, std::bind(&Stereo::timer_callback, this));  
  }
  // service = create_service<orb_slam_msgs::srv::ScaleFactor>("set_scale_factor",
  //       std::bind(&Stereo::srv_callback, this, _1, _2));
}

Stereo::~Stereo()
{
  // Stop all threads
  orb_slam->Shutdown();

  // Save camera trajectory
  // orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  delete orb_slam;
}

void Stereo::load_params()
{
  declare_parameter("map_frame_id", rclcpp::ParameterValue(std::string("map")));
  declare_parameter("target_frame_id", rclcpp::ParameterValue(std::string("odom")));
  declare_parameter("base_frame_id", rclcpp::ParameterValue(std::string("base_footprint")));
  declare_parameter("camera_frame_id", rclcpp::ParameterValue(std::string("camera_link")));
  declare_parameter("voc_file", rclcpp::ParameterValue(std::string("file_not_set")));
  declare_parameter("settings_file", rclcpp::ParameterValue(std::string("file_not_set")));
  declare_parameter("use_viewer", rclcpp::ParameterValue(true));
  declare_parameter("publish_raw", rclcpp::ParameterValue(false));
  declare_parameter("publish_tf", rclcpp::ParameterValue(false));
  // declare_parameter("scale_factor", rclcpp::ParameterValue(1.0));
  declare_parameter("image_left_topic", rclcpp::ParameterValue(std::string("/camera/image_left_raw")));
  declare_parameter("image_right_topic", rclcpp::ParameterValue(std::string("/camera/image_right_raw")));

  get_parameter("map_frame_id", map_frame_id_param);
  get_parameter("target_frame_id", target_frame_id_param);
  get_parameter("camera_frame_id", camera_frame_id_param);
  get_parameter("base_frame_id", base_frame_id_param);
  get_parameter("voc_file", voc_file_name_param);
  get_parameter("settings_file", settings_file_name_param);
  get_parameter("image_left_topic", image_left_topic_param);
  get_parameter("image_right_topic", image_right_topic_param);
  get_parameter("use_viewer", use_viewer_param);
  get_parameter("publish_raw", publish_raw_param);
  get_parameter("publish_tf", publish_tf_param);
  // scale_factor_param = get_parameter("scale_factor").as_double();

  RCLCPP_INFO(get_logger(), "VOC: %s", voc_file_name_param.c_str());
  RCLCPP_INFO(get_logger(), "Settings: %s", settings_file_name_param.c_str());
  if (publish_raw_param)
    RCLCPP_INFO(get_logger(), "Publish raw Twc");
  if (publish_tf_param)
    RCLCPP_INFO(get_logger(), "Publish TF");
  else
    RCLCPP_INFO(get_logger(), "Publish stereo/odom");
}

// void Stereo::srv_callback(const std::shared_ptr<orb_slam_msgs::srv::ScaleFactor::Request> request,
//       std::shared_ptr<orb_slam_msgs::srv::ScaleFactor::Response> response)
// {
//   response->responce = true;
//   scale_factor_param = request->scale_factor;
//   RCLCPP_INFO(get_logger(), "set scale factor: [%f]", request->scale_factor);
// }


void Stereo::topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr &image_left_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &image_right_msg)
{
  cv_bridge::CvImageConstPtr cv_left_in_ptr;
  cv_bridge::CvImageConstPtr cv_right_in_ptr;
  try
  {
    // RCLCPP_INFO(get_logger(), "image callback cv_bridge");
    cv_left_in_ptr = cv_bridge::toCvCopy(image_left_msg, sensor_msgs::image_encodings::MONO8);
    cv_right_in_ptr = cv_bridge::toCvCopy(image_right_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  rclcpp::Time msg_time = image_left_msg->header.stamp;
  Sophus::SE3f tcw = orb_slam->TrackStereo(cv_left_in_ptr->image, cv_right_in_ptr->image, msg_time.seconds());
  tracking_state = orb_slam->GetTrackingState();
  if (tracking_state == ORB_SLAM3::Tracking::OK) {
    current_map_id = orb_slam->GetCurrentMapId();
    Sophus::SE3f twc = tcw.inverse();
    if (publish_raw_param) {
      publish_twc(twc, msg_time);
    } else if (publish_tf_param) {
      update_tf_transform(twc, msg_time);
      publish_pose(twc, msg_time);
    } else {
      publish_odom(twc, msg_time);
    }
  }
}

void Stereo::timer_callback() 
{
  if (!publish_raw_param && has_transform)
    sendTransform(current_transform, this->get_clock()->now());
}

void Stereo::publish_pose(Sophus::SE3f twc, rclcpp::Time msg_time) const
{
  orb_slam_msgs::msg::TrackResult msg;
  msg.header.frame_id = map_frame_id_param;
  msg.header.stamp = msg_time;

  msg.pose.position.x = twc.translation().z();
  msg.pose.position.y = -twc.translation().x();
  msg.pose.position.z = -twc.translation().y();

  msg.pose.orientation.w = twc.unit_quaternion().coeffs().w();
  msg.pose.orientation.x = twc.unit_quaternion().coeffs().z();
  msg.pose.orientation.y = -twc.unit_quaternion().coeffs().x();
  msg.pose.orientation.z = -twc.unit_quaternion().coeffs().y();

  msg.map_id = current_map_id;

  track_result_publisher->publish(msg);
}

void Stereo::publish_twc(Sophus::SE3f twc, rclcpp::Time msg_time) const
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = map_frame_id_param;
  msg.header.stamp = msg_time;

  msg.pose.position.x = twc.translation().x();
  msg.pose.position.y = twc.translation().y();
  msg.pose.position.z = twc.translation().z();

  msg.pose.orientation.w = twc.unit_quaternion().coeffs().w();
  msg.pose.orientation.x = twc.unit_quaternion().coeffs().x();
  msg.pose.orientation.y = twc.unit_quaternion().coeffs().y();
  msg.pose.orientation.z = twc.unit_quaternion().coeffs().z();

  twc_publisher->publish(msg);
}

void Stereo::update_tf_transform(Sophus::SE3f T_SE3f, rclcpp::Time msg_time)
{
  tf2::Transform tf_wc = SE3f_to_tfTransform(T_SE3f);
  // tf2::Transform tf_map2target;

  current_transform = TransformToTarget(tf_wc, camera_frame_id_param, target_frame_id_param);
  has_transform = true;
  // sendTransform(current_transform, msg_time);
  // sendTransform(current_transform, this->get_clock()->now());
}

void Stereo::publish_odom(Sophus::SE3f twc, rclcpp::Time msg_time) const
{
  Eigen::Quaternion<float> const q_se3f = twc.unit_quaternion();
  Eigen::Vector3f t_vec = twc.translation();

  tf2::Quaternion q_tf;
  q_tf.setX(q_se3f.z());
  q_tf.setY(-q_se3f.x());
  q_tf.setZ(-q_se3f.y());
  q_tf.setW(q_se3f.w());

  tf2::Vector3 t_tf(
      t_vec(2),
      -t_vec(0),
      -t_vec(1));


  nav_msgs::msg::Odometry odom;

  odom.header.stamp = msg_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = base_frame_id_param;

  odom.pose.pose.position.x = t_vec(2);
  odom.pose.pose.position.y = -t_vec(0);
  odom.pose.pose.position.z = -t_vec(1);
  odom.pose.pose.orientation.w = q_se3f.w();
  odom.pose.pose.orientation.x = q_se3f.z();
  odom.pose.pose.orientation.y = -q_se3f.x();
  odom.pose.pose.orientation.z = -q_se3f.y();

  odom_publisher->publish(odom);
}

void Stereo::sendTransform(tf2::Transform tf_map2target, rclcpp::Time msg_time) const
{
  std_msgs::msg::Header header;
  header.stamp = msg_time;
  header.frame_id = map_frame_id_param;

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header = header;
  tf_msg.child_frame_id = target_frame_id_param;

  tf2::convert(tf_map2target, tf_msg.transform);

  tf_broadcaster->sendTransform(tf_msg);
}

void Stereo::set_offset(tf2::Transform tf_in, std::string map_frame_id, long unsigned int map_id)
{
  tf2::Transform tf0;
  try
  {
    // Get the transform from target to camera
    geometry_msgs::msg::TransformStamped tf_msg = tf_buffer->lookupTransform(camera_frame_id_param, map_frame_id, tf2::TimePointZero);
    tf2::fromMsg(tf_msg.transform, tf0);
    RCLCPP_INFO(this->get_logger(), "set offset: map id: %ld", map_id);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_INFO(this->get_logger(), "set offset error %s", ex.what());
    tf0.setIdentity();
  }
  tf_offsets[map_id] = (tf_in * tf0).inverse();
}

tf2::Transform Stereo::TransformToTarget(tf2::Transform tf_in,
                                       std::string frame_in, std::string frame_target)
{
  tf2::Transform tf_orig2target;
  tf2::Transform tf_map2target;
  try
  {
    // auto itr = tf_offsets.find(current_map_id);
    // if (itr == tf_offsets.end()) {
    if (current_map_id != pre_map_id) {
      set_offset(tf_in, map_frame_id_param, current_map_id);
      pre_map_id = current_map_id;
    }
    // Get the transform from camera to target
    geometry_msgs::msg::TransformStamped tf_msg = tf_buffer->lookupTransform(frame_in, frame_target, tf2::TimePointZero);
    // Convert to tf2
    tf2::fromMsg(tf_msg.transform, tf_orig2target);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_INFO(this->get_logger(), "%s", ex.what());
    tf_orig2target.setIdentity();
  }
  // Transform from map to target
  tf2::Transform offset = tf_offsets.at(current_map_id);
  tf_map2target = offset * tf_in * tf_orig2target;
  return tf_map2target;
}

tf2::Transform Stereo::SE3f_to_tfTransform(Sophus::SE3f T_SE3f) const
{
  Eigen::Quaternion<float> const q_se3f = T_SE3f.unit_quaternion();
  Eigen::Vector3f t_vec = T_SE3f.translation();

  tf2::Quaternion q_tf;
  q_tf.setX(q_se3f.z());
  q_tf.setY(-q_se3f.x());
  q_tf.setZ(-q_se3f.y());
  q_tf.setW(q_se3f.w());

  tf2::Vector3 t_tf(
      t_vec(2),
      -t_vec(0),
      -t_vec(1));

  // tf2::Vector3 t_tf(
  //     t_vec(2) * scale_factor_param,
  //     -t_vec(0) * scale_factor_param,
  //     -t_vec(1) * scale_factor_param);

  return tf2::Transform(q_tf, t_tf);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Stereo>());
  rclcpp::shutdown();
  return 0;
}
