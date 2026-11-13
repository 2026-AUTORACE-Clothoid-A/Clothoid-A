// ROS-free pieces of the image hub, unit tested in test/test_hub_ops.cpp.
#pragma once

#include <opencv2/core.hpp>

namespace clothoid_image_hub
{

// Passes one frame per `period` seconds of message time. A frame counts as due once
// 0.8 period has passed, so a 28.6 Hz camera still gives every third frame for 10 Hz.
class Throttle
{
public:
  explicit Throttle(double rate_hz);
  bool take(double stamp_sec);

private:
  double min_gap_;
  double last_ = -1.0;
};

// rgb8/bgr8 640x480 -> bgr8 w x h (INTER_AREA). `is_rgb` swaps the channels after the resize,
// which is cheaper than before.
cv::Mat small_bgr(const cv::Mat & frame, bool is_rgb, int width, int height);

// Same matrix as clothoid_lane.lane_detector.make_warp_matrix (camera_limo*.yaml warp_*).
struct WarpParams
{
  int width = 640, height = 480;      // camera image
  int warp_w = 320, warp_h = 240;     // bird's-eye view
  int x_h = 60, x_l = 327, y_h = 91, y_l = 2;
};
cv::Mat warp_matrix(const WarpParams & p);

}  // namespace clothoid_image_hub
