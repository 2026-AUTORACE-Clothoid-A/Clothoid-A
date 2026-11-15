// Image hub: one C++ subscriber for the 30 Hz raw camera image, small topics for the Python nodes.
//
// Loaded into the camera driver's component container (launch/hub.launch.py container:=...), it gets
// the raw image intra-process, without the DDS copy. Each output is computed and published only
// while something subscribes to it. Receive cost per format: docs/PIPELINE_LOAD_PLAN.md 0.2 / 4.2.
#include <memory>
#include <string>

#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "clothoid_image_hub/hub_ops.hpp"

namespace clothoid_image_hub
{

using sensor_msgs::msg::Image;

class ImageHub : public rclcpp::Node
{
public:
  explicit ImageHub(const rclcpp::NodeOptions & options)
  : Node("image_hub", options),
    throttle_10hz_(declare_parameter("rate_10hz", 10.0)),
    throttle_small_(declare_parameter("small_rate_hz", 0.0)),
    throttle_bev_(declare_parameter("bev_rate_hz", 0.0))
  {
    const auto in = declare_parameter("input_topic", std::string("/camera/color/image_raw"));
    small_w_ = declare_parameter("small_width", 320);
    small_h_ = declare_parameter("small_height", 240);
    // same names as clothoid_lane config/camera_<car>.yaml ("/**" parameters)
    warp_.x_h = declare_parameter("warp_x_h", 60);
    warp_.x_l = declare_parameter("warp_x_l", 327);
    warp_.y_h = declare_parameter("warp_y_h", 91);
    warp_.y_l = declare_parameter("warp_y_l", 2);

    // reliable keep-last 1: Python sensor-data (best effort) and rviz (reliable) subscribers both match
    const auto out_qos = rclcpp::QoS(1);
    if (declare_parameter("enable_10hz", false)) {   // bringup use_image_10hz does the same
      pub_10hz_ = create_publisher<Image>(
        declare_parameter("topic_10hz", std::string("/camera/color/image_10hz")), out_qos);
    }
    if (declare_parameter("enable_small", true)) {
      pub_small_ = create_publisher<Image>(
        declare_parameter("topic_small", std::string("/camera/color/image_small")), out_qos);
    }
    if (declare_parameter("enable_bev", false)) {
      pub_bev_ = create_publisher<Image>(
        declare_parameter("topic_bev", std::string("/camera/color/image_bev")), out_qos);
    }
    sub_ = create_subscription<Image>(
      in, rclcpp::SensorDataQoS().keep_last(1),
      [this](Image::ConstSharedPtr msg) {on_image(msg);});
    RCLCPP_INFO(
      get_logger(), "image hub on %s: 10hz %s, small %dx%d %s, bev %s, intra-process %s", in.c_str(),
      pub_10hz_ ? "on" : "off", small_w_, small_h_, pub_small_ ? "on" : "off",
      pub_bev_ ? "on" : "off", options.use_intra_process_comms() ? "yes" : "no");
  }

private:
  static bool wanted(const rclcpp::Publisher<Image>::SharedPtr & pub)
  {
    return pub && (pub->get_subscription_count() + pub->get_intra_process_subscription_count()) > 0;
  }

  static double seconds(const Image & msg)
  {
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
  }

  Image::UniquePtr to_msg(const std_msgs::msg::Header & header, const cv::Mat & bgr)
  {
    auto out = std::make_unique<Image>();
    out->header = header;
    out->height = bgr.rows;
    out->width = bgr.cols;
    out->encoding = "bgr8";
    out->step = static_cast<uint32_t>(bgr.cols * 3);
    out->data.assign(bgr.data, bgr.data + bgr.total() * 3);
    return out;
  }

  void on_image(const Image::ConstSharedPtr & msg)
  {
    const double t = seconds(*msg);
    if (wanted(pub_10hz_) && throttle_10hz_.take(t)) {
      pub_10hz_->publish(*msg);   // unchanged raw, like topic_tools throttle
    }
    const bool small = wanted(pub_small_) && throttle_small_.take(t);
    const bool bev = wanted(pub_bev_) && throttle_bev_.take(t);
    if (!small && !bev) {
      return;
    }
    const bool rgb = msg->encoding == "rgb8";
    if (!rgb && msg->encoding != "bgr8") {
      RCLCPP_WARN_ONCE(get_logger(), "unsupported encoding %s (rgb8 / bgr8 only)", msg->encoding.c_str());
      return;
    }
    // wraps the message buffer, no copy
    const cv::Mat frame(msg->height, msg->width, CV_8UC3,
      const_cast<uint8_t *>(msg->data.data()), msg->step);
    if (small) {
      pub_small_->publish(to_msg(msg->header, small_bgr(frame, rgb, small_w_, small_h_)));
    }
    if (bev) {
      if (bev_matrix_.empty() || warp_.width != frame.cols || warp_.height != frame.rows) {
        warp_.width = frame.cols;
        warp_.height = frame.rows;
        bev_matrix_ = warp_matrix(warp_);
      }
      cv::Mat warped;
      cv::warpPerspective(frame, warped, bev_matrix_, cv::Size(warp_.warp_w, warp_.warp_h),
        cv::INTER_LINEAR);
      if (rgb) {
        cv::cvtColor(warped, warped, cv::COLOR_RGB2BGR);
      }
      pub_bev_->publish(to_msg(msg->header, warped));
    }
  }

  Throttle throttle_10hz_, throttle_small_, throttle_bev_;
  int small_w_, small_h_;
  WarpParams warp_;
  cv::Mat bev_matrix_;
  rclcpp::Publisher<Image>::SharedPtr pub_10hz_, pub_small_, pub_bev_;
  rclcpp::Subscription<Image>::SharedPtr sub_;
};

}  // namespace clothoid_image_hub

RCLCPP_COMPONENTS_REGISTER_NODE(clothoid_image_hub::ImageHub)
