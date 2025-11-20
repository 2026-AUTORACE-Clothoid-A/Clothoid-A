#ifndef RUBBER_CONE_NODE_HPP
#define RUBBER_CONE_NODE_HPP

#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

class RubberConeNode
{
public:
    RubberConeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    // ===== ROS =====
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber obstacles_sub_;
    ros::Publisher  control_pub_;
    ros::Publisher  left_boundary_path_pub_;
    ros::Publisher  right_boundary_path_pub_;
    ros::Publisher  centerline_path_pub_;

    // ===== 상태 =====
    enum class MissionState
    {
        IDLE,
        RUNNING
    };

    MissionState mission_state_;
    bool  is_obstacles_;
    bool  is_look_forward_point_;

    // raw_obstacles → circles 중심점 (base_link 기준)
    std::vector<Eigen::Vector2d> points_;

    // ===== 파라미터 =====
    double speed_;                         // 주행 속도
    double vehicle_length_;                // 차량 휠베이스 근사
    double lfd_;                           // look-ahead distance
    double angle_from_left_first_cone_;    // 첫 콘 탐색 각도 [deg] (옆 방향 기준)
    double dist_from_left_first_cone_;     // 왼쪽 첫 콘까지 허용 거리
    double dist_from_right_first_cone_;    // 오른쪽 첫 콘까지 허용 거리
    double dist_between_cones_;            // 라인 상 인접 콘 간 거리
    double max_turn_angle_deg_;            // 세그먼트 사이 최대 허용 방향 변화 [deg]
    double weight_dist_;                   // 비용함수에서 거리 가중치
    double weight_angle_;                  // 비용함수에서 각도 가중치

    // 스플라인 센터라인 샘플링 간격 [m]
    double centerline_ds_;

    // 미션 시작/진행 판정 영역 (base_link 기준)
    // start 영역
    double start_x_min_;
    double start_x_max_;
    double start_y_min_;
    double start_y_max_;
    int    start_min_count_;

    // ing 영역
    double ing_x_min_;
    double ing_x_max_;
    double ing_y_min_;
    double ing_y_max_;
    int    ing_min_count_;

    // 제어 메시지
    ackermann_msgs::AckermannDriveStamped target_control_;

    // ===== 콜백 =====
    void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& msg);

    // ===== 1) 입력 처리 =====
    void updatePointsFromObstacles(const obstacle_detector::Obstacles& msg);

    // ===== 2) 미션 상태 판정 =====
    bool checkMissionStartCondition() const;
    bool checkMissionContinueCondition() const;

    // ===== 3) 경계 추출 =====
    std::vector<Eigen::Vector2d> extractSidePoints(bool left_side) const;
    bool selectFirstCone(const std::vector<Eigen::Vector2d>& side_points,
                         bool left_side,
                         Eigen::Vector2d& out_first_cone) const;

    // 각도 유틸
    double normalizeAngle(double a) const;
    double angleDiff(double a, double b) const;

    // 한 측 라인을 첫 콘에서부터 앞으로 이어서 생성
    void buildLineFromFirst(const std::vector<Eigen::Vector2d>& all_points,
                            const Eigen::Vector2d& first_point,
                            std::vector<Eigen::Vector2d>& line) const;

    // 전체 경계 (left_line, right_line) 구성
    void buildBoundaries(std::vector<Eigen::Vector2d>& left_line,
                         std::vector<Eigen::Vector2d>& right_line) const;

    // ===== 4) 센터라인 / 스티어링 =====
    // left/right_line → raw center line + spline 보간
    void buildCenterLine(const std::vector<Eigen::Vector2d>& left_line,
                         const std::vector<Eigen::Vector2d>& right_line,
                         std::vector<Eigen::Vector2d>& center_line) const;

    // raw_center_line 을 arc-length 파라미터로 3차 스플라인 보간하여 재샘플
    void smoothCenterLineWithSpline(const std::vector<Eigen::Vector2d>& raw_center,
                                    std::vector<Eigen::Vector2d>& smooth_center) const;

    // center_line → steering angle
    double computeSteeringFromCenterLine(std::vector<Eigen::Vector2d>& center_line);

    void publishControl(double steering);

    void fillPathMsg(const std::vector<Eigen::Vector2d>& pts,
                     const std::string& frame_id,
                     nav_msgs::Path& msg) const;

    // center_line / 경계 → nav_msgs::Path 퍼블리시
    void publishCenterlinePath(const std::vector<Eigen::Vector2d>& center_line);
    void publishBoundaryPaths(const std::vector<Eigen::Vector2d>& left_line,
                              const std::vector<Eigen::Vector2d>& right_line);
};

#endif // RUBBER_CONE_NODE_HPP
