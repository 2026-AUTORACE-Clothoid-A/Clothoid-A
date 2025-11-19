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

    // 원점(0,0)에서 라인(무한 직선)에 대한 수직 거리
    double perpendicularDistanceToLine(const Eigen::Vector2d& p0,
                                        const Eigen::Vector2d& p1) const;

    // 차량 진행 방향 (1,0) 과 라인 방향 각도가 angle_thresh_rad 이내인지
    bool isAlignedWithHeading(const std::pair<Eigen::Vector2d, Eigen::Vector2d>& seg,
                                double angle_thresh_rad = 10.0 * M_PI / 180.0) const;

    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber line_sub_;
    ros::Publisher  cmd_pub_;     // 터널 환경일 때 Ackermann 조향 명령 발행

    // TF2
    tf2_ros::Buffer           tf_buffer_;
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

    // 터널 환경 여부
    bool is_tunnel_env_;

    // 마지막 조향각 (디버깅용으로 쓰고 싶을 때)
    double last_steering_angle_;
};
