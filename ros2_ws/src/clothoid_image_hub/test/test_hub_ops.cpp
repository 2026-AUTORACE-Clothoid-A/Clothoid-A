#include <gtest/gtest.h>

#include <cmath>

#include <opencv2/imgproc.hpp>

#include "clothoid_image_hub/hub_ops.hpp"

using clothoid_image_hub::Throttle;
using clothoid_image_hub::WarpParams;

static int passed(double camera_hz, double out_hz, double seconds)
{
  Throttle th(out_hz);
  int n = 0;
  const int frames = static_cast<int>(seconds * camera_hz);
  for (int i = 0; i < frames; ++i) {
    n += th.take(i / camera_hz) ? 1 : 0;   // frame index, no accumulated rounding
  }
  return n;
}

TEST(Throttle, TenOutOfThirty)
{
  EXPECT_EQ(passed(30.0, 10.0, 3.0), 30);
  EXPECT_EQ(passed(28.6, 10.0, 3.0), 29);   // limo2 camera: every third frame still
}

TEST(Throttle, ZeroPassesAll)
{
  EXPECT_EQ(passed(30.0, 0.0, 1.0), 30);
}

TEST(Throttle, StampBackRestarts)
{
  Throttle th(10.0);
  EXPECT_TRUE(th.take(5.0));
  EXPECT_FALSE(th.take(5.03));
  EXPECT_TRUE(th.take(1.0));
}

TEST(SmallBgr, SizeAndChannelSwap)
{
  cv::Mat rgb(480, 640, CV_8UC3, cv::Scalar(10, 20, 30));
  cv::Mat out = clothoid_image_hub::small_bgr(rgb, true, 320, 240);
  ASSERT_EQ(out.cols, 320);
  ASSERT_EQ(out.rows, 240);
  const auto px = out.at<cv::Vec3b>(100, 100);
  EXPECT_EQ(px[0], 30);
  EXPECT_EQ(px[2], 10);
}

TEST(Warp, MatchesLaneDetector)
{
  // clothoid_lane make_warp_matrix(LaneParams(x_h=60, x_l=327, y_h=91, y_l=2)), numpy float64
  const double expected[3][3] = {
    {-0.26169097766948157, -0.68894155345639041, 243.74111285423413},
    {0.0, -1.7276945158383119, 571.86688474248126},
    {0.0, -0.0043058847091024402, 1.0}};
  cv::Mat m = clothoid_image_hub::warp_matrix(WarpParams{});
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      EXPECT_NEAR(m.at<double>(r, c), expected[r][c], 1e-9 * (1.0 + std::abs(expected[r][c])));
    }
  }
}

TEST(Warp, DegenerateThrows)
{
  WarpParams p;
  p.x_h = 320;   // both top corners at x 320
  EXPECT_THROW(clothoid_image_hub::warp_matrix(p), std::invalid_argument);
}
