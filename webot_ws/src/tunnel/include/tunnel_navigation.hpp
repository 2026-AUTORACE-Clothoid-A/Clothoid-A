#pragma once

#include <ros/ros.h>
#include <laserline/LineSegmentArray.h>

#include <tf2_ros/transform_listener.h>
#include <Eigen/Core>

#include <vector>
#include <utility>
#include <string>
#include <limits>
#include <cmath>

class TunnelNavigationNode
{
public:
    TunnelNavigationNode();
    void spin();

private:
    // 콜백: line_segments 수신
    void lineCallback(const laserline::LineSegmentArray::ConstPtr& msg);

    // 원점(0,0)에서 p0-p1을 지나는 직선까지의 수직 거리 (무한 직선 기준)
    double perpendicularDistanceToLine(const Eigen::Vector2d& p0,
                                       const Eigen::Vector2d& p1) const;

    // 원점(0,0)에서 선분 [p0, p1] 까지의 최소 유클리디안 거리
    double euclideanDistanceToSegment(const Eigen::Vector2d& p0,
                                      const Eigen::Vector2d& p1) const;

    // (1,0) 과 라인 방향 각도가 angle_thresh_rad 이내인지
    bool isAlignedWithHeading(const std::pair<Eigen::Vector2d, Eigen::Vector2d>& seg,
                              double angle_thresh_rad = 10.0 * M_PI / 180.0) const;

    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber line_sub_;
    ros::Publisher  cmd_pub_;
    ros::Publisher  tunnel_mission_pub_;   // 터널 미션 완료 여부 (bool) 발행

    // TF2
    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 파라미터
    std::string target_frame_;    // 보통 "base_link"
    double stanley_k_gain_;
    double max_steer_rad_;
    double command_speed_;

    // 라인 저장 (base_link 좌표계, x 오름차순 정렬된 세그먼트)
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> segments_base_;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> left_segments_;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> right_segments_;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> unknown_segments_;

    // 터널 미션 상태
    bool mission_started_;    // 시작 판단이 한 번이라도 참이었던 적 있는지
    bool mission_finished_;   // 종료 판단이 한 번이라도 참이었던 적 있는지
    bool has_progress_;       // 진행 판단이 한 번이라도 참이었던 적 있는지

    // 디버깅용 마지막 조향각
    double last_steering_angle_;
};
