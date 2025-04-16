#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class LivoxRepubNode : public rclcpp::Node {
public:
  LivoxRepubNode();

private:
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_out_;
  int point_cloud_type_;
  int combine_cloud_size_;
  int num_skip_cloud_;
  std::vector<livox_ros_driver2::msg::CustomMsg::SharedPtr> livox_data_;
  std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> pointcloud_data_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> pcl_out_;
  std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg_;
  std::mutex mutex_;
  bool pcl_msg_init_;
  int i_counter;
  float publish_rate_;
  rclcpp::CallbackGroup::CallbackGroup::SharedPtr livox_cb_group_;
  rclcpp::CallbackGroup::CallbackGroup::SharedPtr pub_cb_group_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  void pubTimerCallback();
  void livox_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};