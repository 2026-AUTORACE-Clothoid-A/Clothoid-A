#include "obstacle_detector/scan_line_extractor.h"
#include "obstacle_detector/utilities/figure_fitting.h"
#include "obstacle_detector/utilities/math_utilities.h"

using obstacle_detector::Point;
using obstacle_detector::PointSet;
using obstacle_detector::PointIterator;
using obstacle_detector::Segment;

namespace scan_line_extractor
{

ScanLineExtractor::ScanLineExtractor(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private)
{
  p_active_ = true;  // 기본값: 활성화
  updateParams();

  if (p_active_)
  {
    scan_sub_ = nh_.subscribe("scan", 10, &ScanLineExtractor::scanCallback, this);

    // RViz용 Marker
    if (p_publish_markers_)
      marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("scan_line_segments_markers", 10);

    // 선분을 Obstacles 형식으로 발행
    obstacles_pub_ = nh_.advertise<obstacle_detector::Obstacles>("scan_line_obstacles", 10);
  }

  ROS_INFO_STREAM("[ScanLineExtractor] Initialized. Subscribing /scan, publishing scan_line_obstacles"
                  << (p_publish_markers_ ? " + scan_line_segments_markers" : ""));
}

void ScanLineExtractor::updateParams()
{
  nh_private_.param<bool>("active", p_active_, true);
  nh_private_.param<bool>("use_split_and_merge", p_use_split_and_merge_, true);

  nh_private_.param<int>("min_group_points", p_min_group_points_, 5);

  // 터널 환경: 거리 0.25~0.35m 근처, 벽이 촘촘히 찍히는 걸 기준으로 살짝 타이트하게
  nh_private_.param<double>("max_group_distance",    p_max_group_distance_,    0.03);   // 인접 포인트 3cm 이내면 같은 그룹
  nh_private_.param<double>("distance_proportion",   p_distance_proportion_,   0.004);  // 각도 커질수록 허용거리 증가
  nh_private_.param<double>("max_split_distance",    p_max_split_distance_,    0.02);   // 선에서 2cm 이상 벗어나면 split 후보
  nh_private_.param<double>("max_merge_separation",  p_max_merge_separation_,  0.05);
  nh_private_.param<double>("max_merge_spread",      p_max_merge_spread_,      0.03);
  nh_private_.param<double>("min_segment_length", p_min_segment_length_, 0.1);

  // ROI: 필요하면 전방/주변 특정 영역만 보이게 필터링
  nh_private_.param<bool>("use_roi",    p_use_roi_,    true);
  nh_private_.param<double>("roi_x_min", p_roi_x_min_, -1.0);
  nh_private_.param<double>("roi_x_max", p_roi_x_max_,  1.0);
  nh_private_.param<double>("roi_y_min", p_roi_y_min_, -0.5);
  nh_private_.param<double>("roi_y_max", p_roi_y_max_,  0.5);

  nh_private_.param<std::string>("frame_id",        p_frame_id_,        std::string("")); // 빈 문자열이면 scan frame 사용
  nh_private_.param<double>("marker_lifetime",      p_marker_lifetime_, 0.1);
  nh_private_.param<bool>("publish_markers",        p_publish_markers_, true);

  ROS_INFO_STREAM("[ScanLineExtractor] params:"
                  << " min_group_points=" << p_min_group_points_
                  << " max_group_distance=" << p_max_group_distance_
                  << " distance_proportion=" << p_distance_proportion_
                  << " max_split_distance=" << p_max_split_distance_
                  << " max_merge_separation=" << p_max_merge_separation_
                  << " max_merge_spread=" << p_max_merge_spread_
                  << " use_roi=" << (p_use_roi_ ? "true" : "false"));
}

void ScanLineExtractor::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  if (!p_active_) return;

  base_frame_id_ = scan_msg->header.frame_id;
  stamp_         = scan_msg->header.stamp;

  input_points_.clear();

  double phi = scan_msg->angle_min;

  // === inf / NaN / 범위 밖은 모두 스킵 ===
  for (const float r : scan_msg->ranges)
  {
    if (std::isfinite(r) &&
        r >= scan_msg->range_min &&
        r <= scan_msg->range_max)
    {
      input_points_.push_back(Point::fromPoolarCoords(r, phi));
    }

    phi += scan_msg->angle_increment;
  }

  if (input_points_.size() < static_cast<size_t>(p_min_group_points_))
  {
    segments_.clear();
    publishOutputs();
    return;
  }

  processPoints();
}

void ScanLineExtractor::processPoints()
{
  segments_.clear();

  groupPoints();    // 포인트 묶으면서 Segment 검출
  mergeSegments();  // 인접/공선 세그먼트 병합
  publishOutputs(); // Obstacles + MarkerArray 발행

  input_points_.clear();
}

void ScanLineExtractor::groupPoints()
{
  static double sin_dp = std::sin(2.0 * p_distance_proportion_);

  PointSet point_set;
  point_set.begin       = input_points_.begin();
  point_set.end         = input_points_.begin();
  point_set.num_points  = 1;
  point_set.is_visible  = true;

  // 리스트의 두 번째 원소부터 순회
  if (input_points_.size() <= 1)
  {
    detectSegments(point_set);
    return;
  }

  PointIterator start_it = input_points_.begin();
  PointIterator point    = start_it;
  ++point; // second element

  for (; point != input_points_.end(); ++point)
  {
    double range    = (*point).length();
    double distance = (*point - *point_set.end).length();

    if (distance < p_max_group_distance_ + range * p_distance_proportion_)
    {
      // 같은 그룹에 계속 추가
      point_set.end = point;
      point_set.num_points++;
    }
    else
    {
      double prev_range = (*point_set.end).length();

      // Heron 공식으로 빔 사이 각도 추정
      double p = (range + prev_range + distance) / 2.0;
      double S = std::sqrt(std::max(0.0, p * (p - range) * (p - prev_range) * (p - distance)));
      double sin_d = 2.0 * S / (range * prev_range + 1e-9); // 0 나누기 방지

      if (std::abs(sin_d) < sin_dp && range < prev_range)
        point_set.is_visible = false;

      detectSegments(point_set);

      // 새 그룹 시작
      point_set.begin      = point;
      point_set.end        = point;
      point_set.num_points = 1;
      point_set.is_visible = (std::abs(sin_d) > sin_dp || range < prev_range);
    }
  }

  // 마지막 그룹도 세그먼트 검출
  detectSegments(point_set);
}

void ScanLineExtractor::detectSegments(const PointSet& point_set)
{
  if (point_set.num_points < p_min_group_points_)
    return;

  // Iterative End-Point Fit 형태로 초기 세그먼트
  Segment segment(*point_set.begin, *point_set.end);

  if (p_use_split_and_merge_)
    segment = obstacle_detector::fitSegment(point_set);

  PointIterator set_divider;
  double max_distance = 0.0;
  double distance     = 0.0;

  int split_index = 0; // 1부터 세는 인덱스
  int point_index = 0;

  // Split & Merge – ObstacleExtractor와 동일
  for (PointIterator point = point_set.begin; point != point_set.end; ++point)
  {
    ++point_index;

    distance = segment.distanceTo(*point);

    if (distance >= max_distance)
    {
      double r = (*point).length();

      if (distance > p_max_split_distance_ + r * p_distance_proportion_)
      {
        max_distance = distance;
        set_divider  = point;
        split_index  = point_index;
      }
    }
  }

  // 충분히 긴 세트만 분할
  if (max_distance > 0.0 &&
      split_index > p_min_group_points_ &&
      split_index < point_set.num_points - p_min_group_points_)
  {
    set_divider = input_points_.insert(set_divider, *set_divider);  // 나누는 점을 복제

    PointSet subset1, subset2;
    subset1.begin       = point_set.begin;
    subset1.end         = set_divider;
    subset1.num_points  = split_index;
    subset1.is_visible  = point_set.is_visible;

    subset2.begin       = ++set_divider;
    subset2.end         = point_set.end;
    subset2.num_points  = point_set.num_points - split_index;
    subset2.is_visible  = point_set.is_visible;

    detectSegments(subset1);
    detectSegments(subset2);
  }
  else
  {
    if (!p_use_split_and_merge_)
      segment = obstacle_detector::fitSegment(point_set);

    segments_.push_back(segment);
  }
}

void ScanLineExtractor::mergeSegments()
{
  for (auto i = segments_.begin(); i != segments_.end(); ++i)
  {
    for (auto j = i; j != segments_.end(); ++j)
    {
      Segment merged_segment;

      if (compareSegments(*i, *j, merged_segment))
      {
        auto temp_itr = segments_.insert(i, merged_segment);
        segments_.erase(i);
        segments_.erase(j);
        i = --temp_itr; // 새 세그먼트에 대해서 다시 검사
        break;
      }
    }
  }
}

bool ScanLineExtractor::compareSegments(const Segment& s1,
                                       const Segment& s2,
                                       Segment& merged_segment)
{
  if (&s1 == &s2)
    return false;

  // 세그먼트는 CCW 기준이어야 함
  if (s1.first_point.cross(s2.first_point) < 0.0)
    return compareSegments(s2, s1, merged_segment);

  if (checkSegmentsProximity(s1, s2))
  {
    std::vector<PointSet> point_sets;
    point_sets.insert(point_sets.end(), s1.point_sets.begin(), s1.point_sets.end());
    point_sets.insert(point_sets.end(), s2.point_sets.begin(), s2.point_sets.end());

    Segment segment = obstacle_detector::fitSegment(point_sets);

    if (checkSegmentsCollinearity(segment, s1, s2))
    {
      merged_segment = segment;
      return true;
    }
  }

  return false;
}

bool ScanLineExtractor::checkSegmentsProximity(const Segment& s1, const Segment& s2)
{
  return (s1.trueDistanceTo(s2.first_point) < p_max_merge_separation_ ||
          s1.trueDistanceTo(s2.last_point)  < p_max_merge_separation_ ||
          s2.trueDistanceTo(s1.first_point) < p_max_merge_separation_ ||
          s2.trueDistanceTo(s1.last_point)  < p_max_merge_separation_);
}

bool ScanLineExtractor::checkSegmentsCollinearity(const Segment& segment,
                                                  const Segment& s1,
                                                  const Segment& s2)
{
  return (segment.distanceTo(s1.first_point) < p_max_merge_spread_ &&
          segment.distanceTo(s1.last_point)  < p_max_merge_spread_ &&
          segment.distanceTo(s2.first_point) < p_max_merge_spread_ &&
          segment.distanceTo(s2.last_point)  < p_max_merge_spread_);
}

void ScanLineExtractor::publishOutputs()
{
  // ===============================
  // 1) Obstacles 메시지 (segments만)
  // ===============================
  obstacle_detector::Obstacles obstacles_msg;
  obstacles_msg.header.stamp = stamp_;

  // frame_id: param이 비어 있으면 스캔 frame 그대로 사용
  if (!p_frame_id_.empty())
    obstacles_msg.header.frame_id = p_frame_id_;
  else
    obstacles_msg.header.frame_id = base_frame_id_;

  for (const Segment& s : segments_)
  {
    const Point& p1 = s.first_point;
    const Point& p2 = s.last_point;

    if (s.length() < p_min_segment_length_)
      continue;

    // ROI 필터링 (둘 다 ROI 밖이면 스킵)
    if (p_use_roi_)
    {
      bool p1_in =
        (p1.x >= p_roi_x_min_ && p1.x <= p_roi_x_max_ &&
         p1.y >= p_roi_y_min_ && p1.y <= p_roi_y_max_);
      bool p2_in =
        (p2.x >= p_roi_x_min_ && p2.x <= p_roi_x_max_ &&
         p2.y >= p_roi_y_min_ && p2.y <= p_roi_y_max_);

      if (!p1_in && !p2_in)
        continue;
    }

    obstacle_detector::SegmentObstacle seg;
    seg.first_point.x = p1.x;
    seg.first_point.y = p1.y;
    seg.first_point.z = 0.0;

    seg.last_point.x  = p2.x;
    seg.last_point.y  = p2.y;
    seg.last_point.z  = 0.0;

    obstacles_msg.segments.push_back(seg);
  }

  // circles는 이 노드에서는 사용하지 않으니 비워둠
  // obstacles_msg.circles = {}

  obstacles_pub_.publish(obstacles_msg);

  // ===============================
  // 2) RViz MarkerArray (옵션)
  // ===============================
  if (!p_publish_markers_ || !marker_pub_)
    return;

  visualization_msgs::MarkerArray arr;

  // 기존 Marker 지우기
  visualization_msgs::Marker clear;
  clear.header.frame_id = obstacles_msg.header.frame_id;
  clear.header.stamp    = stamp_;
  clear.ns              = "scan_line_segments";
  clear.id              = 0;
  clear.action          = visualization_msgs::Marker::DELETEALL;
  arr.markers.push_back(clear);

  int id = 1;

  for (const Segment& s : segments_)
  {
    const Point& p1 = s.first_point;
    const Point& p2 = s.last_point;

    // 길이 필터
    if (s.length() < p_min_segment_length_)
      continue;

    if (p_use_roi_)
    {
      bool p1_in =
        (p1.x >= p_roi_x_min_ && p1.x <= p_roi_x_max_ &&
         p1.y >= p_roi_y_min_ && p1.y <= p_roi_y_max_);
      bool p2_in =
        (p2.x >= p_roi_x_min_ && p2.x <= p_roi_x_max_ &&
         p2.y >= p_roi_y_min_ && p2.y <= p_roi_y_max_);

      if (!p1_in && !p2_in)
        continue;
    }

    visualization_msgs::Marker m;
    m.header.frame_id = obstacles_msg.header.frame_id;
    m.header.stamp    = stamp_;
    m.ns              = "scan_line_segments";
    m.id              = id++;
    m.type            = visualization_msgs::Marker::LINE_LIST;
    m.action          = visualization_msgs::Marker::ADD;
    m.lifetime        = ros::Duration(p_marker_lifetime_);

    m.scale.x = 0.02;  // 선 두께

    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 1.0;
    m.color.a = 1.0;

    geometry_msgs::Point gp1, gp2;
    gp1.x = p1.x; gp1.y = p1.y; gp1.z = 0.0;
    gp2.x = p2.x; gp2.y = p2.y; gp2.z = 0.0;

    m.points.push_back(gp1);
    m.points.push_back(gp2);

    arr.markers.push_back(m);
  }

  marker_pub_.publish(arr);
}

} // namespace scan_line_extractor

// ===================== main =====================

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_line_extractor_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  scan_line_extractor::ScanLineExtractor node(nh, nh_private);

  ros::spin();
  return 0;
}
