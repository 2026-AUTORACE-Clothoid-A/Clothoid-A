#include "clothoid_image_hub/hub_ops.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace clothoid_image_hub
{

Throttle::Throttle(double rate_hz)
: min_gap_(rate_hz > 0.0 ? 0.8 / rate_hz : 0.0) {}

bool Throttle::take(double stamp_sec)
{
  // a stamp going back (bag loop, clock reset) restarts the throttle
  if (last_ < 0.0 || stamp_sec < last_ || stamp_sec - last_ >= min_gap_) {
    last_ = stamp_sec;
    return true;
  }
  return false;
}

cv::Mat small_bgr(const cv::Mat & frame, bool is_rgb, int width, int height)
{
  cv::Mat small;
  cv::resize(frame, small, cv::Size(width, height), 0, 0, cv::INTER_AREA);
  if (is_rgb) {
    cv::cvtColor(small, small, cv::COLOR_RGB2BGR);
  }
  return small;
}

cv::Mat warp_matrix(const WarpParams & p)
{
  const float w = static_cast<float>(p.width), h = static_cast<float>(p.height);
  const float half = static_cast<float>(p.height / 2);
  std::vector<cv::Point2f> src = {
    {static_cast<float>(p.x_h), half + p.y_h},        // top left
    {static_cast<float>(-p.x_l), h - p.y_l},          // bottom left
    {w - p.x_h, half + p.y_h},                        // top right
    {w + p.x_l, h - p.y_l},                           // bottom right
  };
  std::vector<cv::Point2f> dst = {
    {0.f, 0.f}, {0.f, static_cast<float>(p.warp_h)},
    {static_cast<float>(p.warp_w), 0.f},
    {static_cast<float>(p.warp_w), static_cast<float>(p.warp_h)},
  };
  cv::Mat m = cv::getPerspectiveTransform(src, dst);
  if (!cv::checkRange(m) || std::abs(cv::determinant(m)) < 1e-12) {
    throw std::invalid_argument("degenerate warp parameters (corners collapse)");
  }
  return m;
}

}  // namespace clothoid_image_hub
