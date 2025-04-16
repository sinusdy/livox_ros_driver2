#include "livox_repub_node.hpp"

using livox_ros_driver2::msg::CustomMsg;
using std::placeholders::_1;

LivoxRepubNode::LivoxRepubNode() : Node("livox_repub_node"), i_counter(0), pcl_msg_init_(false) {
  RCLCPP_INFO(this->get_logger(), "Starting Livox Republisher Node");

  this->declare_parameter("point_cloud_type", 1);
  this->get_parameter("point_cloud_type", point_cloud_type_);

  this->declare_parameter("combine_cloud_size", 1);
  this->get_parameter("combine_cloud_size", combine_cloud_size_);

  this->declare_parameter("num_skip_cloud", 2);
  this->get_parameter("num_skip_cloud", num_skip_cloud_);

  this->declare_parameter("publish_rate", 2.0);
  this->get_parameter("publish_rate", publish_rate_);

  livox_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  pub_pcl_out_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox_pcl0", 1);

  if (point_cloud_type_ == 1) {
    sub_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar", rclcpp::SensorDataQoS().keep_last(1), std::bind(&LivoxRepubNode::livox_callback, this, _1));
  } else if (point_cloud_type_ == 0) {
    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", rclcpp::SensorDataQoS().keep_last(1), std::bind(&LivoxRepubNode::pcl_callback, this, _1));
  }

  //TODO: ADD TIMER BCOS THE RAW POINT CLOUD CALLBACK IS TOO SLOW IN THE FIRST PLACE
  pub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  double pub_interval_time = 1 / publish_rate_;
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Publish interval:" << pub_interval_time);
  auto pub_interval = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(pub_interval_time));
  pub_timer_ = this->create_wall_timer(
                pub_interval, std::bind(&LivoxRepubNode::pubTimerCallback, this),
                pub_cb_group_);
}

void LivoxRepubNode::pubTimerCallback() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (point_cloud_type_ == 1 ) {
    if (pcl_msg_init_ && livox_data_.size() < combine_cloud_size_) {
      RCLCPP_INFO(this->get_logger(), "Publishing previously combined cloud");
      pub_pcl_out_->publish(*cloud_msg_);
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(), "Inside not previously combined cloud");
      pcl_out_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
      RCLCPP_INFO_STREAM(this->get_logger(), "Livox data size : " << livox_data_.size());
      if (livox_data_.size() >= combine_cloud_size_) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Inside combining cloud logic");
        for (auto livox_msg : livox_data_) {
          auto time_end = livox_msg->points.back().offset_time;

          for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
            pcl::PointXYZI pt;
            pt.x = livox_msg->points[i].x;
            pt.y = livox_msg->points[i].y;
            pt.z = livox_msg->points[i].z;
            pt.intensity = livox_msg->points[i].reflectivity * 0.1f;

            pcl_out_->push_back(pt);
          }
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "PCL points size: " << pcl_out_->size());
        auto timebase_ns = livox_data_.front()->timebase;
        rclcpp::Time timestamp = rclcpp::Time(timebase_ns);
        pcl_msg_init_ = true;
        cloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*pcl_out_, *cloud_msg_);
        cloud_msg_->header.stamp = timestamp;
        cloud_msg_->header.frame_id = "livox_frame";  // Update to your frame_id
        // RCLCPP_INFO_STREAM(this->get_logger(), "Cloud msg point size: " << cloud_msg_->points.size());
        pub_pcl_out_->publish(*cloud_msg_);
        livox_data_.clear();
      }
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "In PointCloud2 callback");
    if (pcl_msg_init_ && pointcloud_data_.size() < combine_cloud_size_) {
      RCLCPP_INFO(this->get_logger(), "Publishing previously combined cloud");
      pub_pcl_out_->publish(*cloud_msg_);
    } else {
      RCLCPP_INFO(this->get_logger(), "In combining code");
      pcl_out_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
      for (auto data : pointcloud_data_) {
        pcl::PointCloud<pcl::PointXYZI> pcl_data;
        pcl::fromROSMsg(*data, pcl_data);

        RCLCPP_INFO_STREAM(this->get_logger(), "PCL data length: " << pcl_data.points.size());
        // *pcl_out_ += pcl_data;
        for (auto point : pcl_data) {
          pcl_out_->push_back(point);
        }
        RCLCPP_INFO(this->get_logger(), "Copied point cloud...");
      }
      rclcpp::Time timestamp = pointcloud_data_.front()->header.stamp;
      pcl_msg_init_ = true;
      cloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
      RCLCPP_INFO(this->get_logger(), "Publishing combined point cloud...");
      pcl::toROSMsg(*pcl_out_, *cloud_msg_);
      cloud_msg_->header.stamp = timestamp;
      cloud_msg_->header.frame_id = "livox_frame";  // Update to your frame_id

      pub_pcl_out_->publish(*cloud_msg_);
      pointcloud_data_.clear();
    }
  }
}

void LivoxRepubNode::livox_callback(const CustomMsg::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  livox_data_.push_back(msg);
}

void LivoxRepubNode::pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  pointcloud_data_.push_back(msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<LivoxRepubNode>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
