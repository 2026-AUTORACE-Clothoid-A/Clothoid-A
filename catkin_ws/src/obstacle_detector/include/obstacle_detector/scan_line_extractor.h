#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

#include <list>
#include <string>

#include "obstacle_detector/utilities/point.h"
#include "obstacle_detector/utilities/segment.h"
#include "obstacle_detector/utilities/point_set.h"
#include "obstacle_detector/Obstacles.h"          // Obstacles, SegmentObstacle

namespace scan_line_extractor
{

class ScanLineExtractor
{
public:
  ScanLineExtractor(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~ScanLineExtractor() = default;

private:
  // 콜백
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

  // 파이프라인
  void updateParams();
  void processPoints();
  void groupPoints();
  void detectSegments(const obstacle_detector::PointSet& point_set);
  void mergeSegments();
  void publishOutputs();   // Obstacles + MarkerArray 둘 다 발행

  // 유틸 (ObstacleExtractor와 동일)
  bool compareSegments(const obstacle_detector::Segment& s1,
                       const obstacle_detector::Segment& s2,
                       obstacle_detector::Segment& merged_segment);
  bool checkSegmentsProximity(const obstacle_detector::Segment& s1,
                              const obstacle_detector::Segment& s2);
  bool checkSegmentsCollinearity(const obstacle_detector::Segment& segment,
                                 const obstacle_detector::Segment& s1,
                                 const obstacle_detector::Segment& s2);

  // 노드 핸들
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS I/O
  ros::Subscriber scan_sub_;
  ros::Publisher  marker_pub_;
  ros::Publisher  obstacles_pub_;   // ★ 새로 추가: Obstacles 메시지 발행

  // 데이터
  std::list<obstacle_detector::Point>   input_points_;
  std::list<obstacle_detector::Segment> segments_;

  // 공통 정보
  ros::Time    stamp_;
  std::string  base_frame_id_;

  // 파라미터 (ObstacleExtractor 스타일)
  bool   p_active_;
  bool   p_use_split_and_merge_;

  int    p_min_group_points_;
  double p_max_group_distance_;
  double p_distance_proportion_;
  double p_max_split_distance_;
  double p_max_merge_separation_;
  double p_max_merge_spread_;

  // 길이 필터
  double p_min_segment_length_;  // [m] 이 값보다 짧은 선은 무시

  // ROI (선분 publish 시 필터링용 – 옵션)
  bool   p_use_roi_;
  double p_roi_x_min_;
  double p_roi_x_max_;
  double p_roi_y_min_;
  double p_roi_y_max_;

  // Marker 설정
  std::string p_frame_id_;        // (옵션) 강제 frame id, 안 쓰면 scan frame 그대로 사용
  double      p_marker_lifetime_;
  bool        p_publish_markers_; // MarkerArray 출력 on/off
};

} // namespace scan_line_extractor
