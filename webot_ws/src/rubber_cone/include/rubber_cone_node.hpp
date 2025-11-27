#ifndef RUBBER_CONE_NODE_HPP
#define RUBBER_CONE_NODE_HPP

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>

class RubberConeNode
{
public:
    RubberConeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    // ===== ROS =====
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber left_boundary_sub_;
    ros::Subscriber right_boundary_sub_;
    ros::Subscriber mission_state_sub_;

    ros::Publisher  control_pub_;
    ros::Publisher  centerline_path_pub_;

    // ===== 상태 =====
    bool mission_active_;
    bool prev_mission_active_;
    bool has_left_boundary_;
    bool has_right_boundary_;
    bool is_look_forward_point_;

    double last_steering_;
    bool   has_last_steering_;

    double max_steer_rad_;   // 최대 조향각 (rad)
    double reverse_speed_;   // 후진 속도 (m/s)

    // ===== 파라미터 =====
    double speed_;          // 주행 속도
    double vehicle_length_; // 차량 휠베이스 근사
    double lfd_;            // look-ahead distance
    double centerline_ds_;  // 센터라인 샘플링 간격
    int    lsq_window_size_; // 로컬 3차 최소제곱 윈도우 크기(홀수)

    ros::Time last_left_boundary_stamp_;
    ros::Time last_right_boundary_stamp_;
    double    boundary_timeout_sec_;   // 경계가 이 시간 이상 안 들어오면 무효로 간주

    // 최신 경계 점들 (base_link 기준)
    std::vector<Eigen::Vector2d> left_line_;
    std::vector<Eigen::Vector2d> right_line_;

    // 제어 메시지
    ackermann_msgs::AckermannDriveStamped target_control_;

    // ===== 콜백 =====
    void leftBoundaryCallback(const nav_msgs::Path::ConstPtr& msg);
    void rightBoundaryCallback(const nav_msgs::Path::ConstPtr& msg);
    void missionStateCallback(const std_msgs::Bool::ConstPtr& msg);

    // ===== 메인 처리 =====
    void updateAndControl();

    // ===== 센터라인 생성 & 스무딩 =====
    void buildCenterLine(const std::vector<Eigen::Vector2d>& left_line,
                         const std::vector<Eigen::Vector2d>& right_line,
                         std::vector<Eigen::Vector2d>& center_line) const;

    // 기존 tk::spline 기반 arc-length 스플라인 보간
    void smoothCenterLineWithSpline(const std::vector<Eigen::Vector2d>& raw_center,
                                    std::vector<Eigen::Vector2d>& smooth_center) const;

    // 로컬 3차 최소제곱(Savitzky–Golay 스타일) 스무딩
    void smoothCenterLineLeastSquaresCubic(const std::vector<Eigen::Vector2d>& raw_center,
                                           std::vector<Eigen::Vector2d>& smoothed_center) const;

    // ===== 스티어링 계산 & 퍼블리시 =====
    double computeSteeringFromCenterLine(const std::vector<Eigen::Vector2d>& center_line);
    void publishControl(double steering);
    void publishReverseControl(double steering);

    // ===== Path 유틸 =====
    void fillPathMsg(const std::vector<Eigen::Vector2d>& pts,
                     const std::string& frame_id,
                     nav_msgs::Path& msg) const;
    void publishCenterlinePath(const std::vector<Eigen::Vector2d>& center_line);
};

#endif // RUBBER_CONE_NODE_HPP
