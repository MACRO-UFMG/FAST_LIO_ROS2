#include <memory>
#include <string>

#include "imu_frame_adapter/transform.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace imu_frame_adapter
{

class ImuFrameAdapterNode : public rclcpp::Node
{
public:
  ImuFrameAdapterNode()
  : Node("imu_frame_adapter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/camera/camera/imu");
    output_topic_ = declare_parameter<std::string>("output_topic", "/camera/camera/imu_body");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "camera_imu_body_frame");
    const int queue_depth = declare_parameter<int>("queue_depth", 200);

    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    qos.keep_last(static_cast<size_t>(queue_depth));

    pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, qos);
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_, qos,
      std::bind(&ImuFrameAdapterNode::callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "IMU frame adapter: input=%s output=%s frame_id=%s "
      "convention: x_body=z_opt, y_body=-x_opt, z_body=-y_opt (R_BO fixed)",
      input_topic_.c_str(), output_topic_.c_str(), output_frame_id_.c_str());
  }

private:
  void callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    sensor_msgs::msg::Imu out;
    out.header.stamp = msg->header.stamp;  // preserve exactly; never use now()
    out.header.frame_id = output_frame_id_;

    rotate_vector(
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
      out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z);
    rotate_vector(
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
      out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z);

    if (!warned_nonfinite_ &&
      (!is_finite3(
        out.angular_velocity.x, out.angular_velocity.y,
        out.angular_velocity.z) ||
      !is_finite3(
        out.linear_acceleration.x, out.linear_acceleration.y,
        out.linear_acceleration.z)))
    {
      RCLCPP_WARN(get_logger(), "Non-finite IMU sample detected; publishing transformed values");
      warned_nonfinite_ = true;
    }

    std::array<double, 9> in_cov{};
    std::array<double, 9> out_cov{};
    for (size_t i = 0; i < 9; ++i) {
      in_cov[i] = msg->angular_velocity_covariance[i];
    }
    rotate_covariance(in_cov, out_cov);
    for (size_t i = 0; i < 9; ++i) {
      out.angular_velocity_covariance[i] = out_cov[i];
      in_cov[i] = msg->linear_acceleration_covariance[i];
    }
    rotate_covariance(in_cov, out_cov);
    for (size_t i = 0; i < 9; ++i) {
      out.linear_acceleration_covariance[i] = out_cov[i];
    }

    // RealSense orientation unavailable: preserve convention (cov[0] == -1).
    out.orientation = msg->orientation;
    out.orientation_covariance = msg->orientation_covariance;

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  bool warned_nonfinite_{false};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

}  // namespace imu_frame_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_frame_adapter::ImuFrameAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
