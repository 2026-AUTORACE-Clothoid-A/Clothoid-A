#include "tunnel_navigation.hpp"

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

TunnelNavigationNode::TunnelNavigationNode()
:   nh_(),
    private_nh_("~"),
    tf_buffer_(),
    tf_listener_(tf_buffer_),
    is_tunnel_env_(false),
    last_steering_angle_(0.0)
{
    // 파라미터 로드
    private_nh_.param<std::string>("target_frame", target_frame_, std::string("base_link"));

    private_nh_.param("stanley_k_gain", stanley_k_gain_, 1.0);
    double max_steer_deg;
    private_nh_.param("max_steer_deg", max_steer_deg, 21.0);
    max_steer_rad_ = max_steer_deg * M_PI / 180.0;

    private_nh_.param("tunnel_speed", command_speed_, 0.4);  // 터널에서 달릴 속도 [m/s]

    // 구독/발행
    line_sub_ = nh_.subscribe("scan_line_obstacles", 1,
                            &TunnelNavigationNode::lineCallback, this);
    cmd_pub_  = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("tunnel_cmd", 1);

    ROS_INFO_STREAM("TunnelNavigationNode initialized. target_frame=" << target_frame_
                    << ", stanley_k_gain=" << stanley_k_gain_
                    << ", max_steer_deg=" << max_steer_deg
                    << ", tunnel_speed=" << command_speed_);
}

void TunnelNavigationNode::spin()
{
    ros::spin();
}

// 원점(0,0)에서 p0-p1을 지나는 직선까지의 수직 거리 (무한 직선 기준)
double TunnelNavigationNode::perpendicularDistanceToLine(const Eigen::Vector2d& p0,
                                                         const Eigen::Vector2d& p1) const
{
    Eigen::Vector2d v = p1 - p0;
    double denom = v.norm();
    if (denom < 1e-6)
        return std::numeric_limits<double>::max();

    // 2D에서 |p0 x p1| / |p1 - p0|
    double num = std::abs(p0.x() * p1.y() - p0.y() * p1.x());
    return num / denom;
}

// 원점(0,0)에서 선분 [p0, p1] 까지의 최소 유클리디안 거리
double TunnelNavigationNode::euclideanDistanceToSegment(const Eigen::Vector2d& p0,
                                                        const Eigen::Vector2d& p1) const
{
    Eigen::Vector2d v = p1 - p0;
    double len2 = v.squaredNorm();
    if (len2 < 1e-12)
        return p0.norm();  // 거의 점인 경우

    // 선분 [p0, p1] 위에서 원점에 가장 가까운 점의 파라미터 t (0~1)
    double t = -p0.dot(v) / len2;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    Eigen::Vector2d proj = p0 + t * v;
    return proj.norm();
}

// (1,0) 과 라인 방향 각도가 angle_thresh_rad 이내인지
bool TunnelNavigationNode::isAlignedWithHeading(
    const std::pair<Eigen::Vector2d, Eigen::Vector2d>& seg,
    double angle_thresh_rad) const
{
    Eigen::Vector2d d = seg.second - seg.first;
    double norm = d.norm();
    if (norm < 1e-3)
        return false;

    d /= norm;
    const Eigen::Vector2d heading(1.0, 0.0);

    double cosang = d.dot(heading);
    if (cosang >  1.0) cosang =  1.0;
    if (cosang < -1.0) cosang = -1.0;

    double angle = std::acos(cosang);  // [0, pi]
    return (angle <= angle_thresh_rad);
}

void TunnelNavigationNode::lineCallback(const obstacle_detector::Obstacles::ConstPtr& msg)
{
    // 이전 데이터 초기화
    segments_base_.clear();
    left_segments_.clear();
    right_segments_.clear();
    unknown_segments_.clear();
    is_tunnel_env_ = false;

    const std::string& src_frame = msg->header.frame_id;
    if (src_frame.empty())
    {
        ROS_WARN_THROTTLE(1.0, "LineSegmentArray header.frame_id is empty.");
        return;
    }

    // TF: src_frame (보통 "laser") -> target_frame (보통 "base_link")
    geometry_msgs::TransformStamped tf;
    try
    {
        tf = tf_buffer_.lookupTransform(
                target_frame_,          // to
                src_frame,              // from
                msg->header.stamp,      // same timestamp as lines
                ros::Duration(0.1));
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "TF lookup failed in TunnelNavigationNode: %s", ex.what());
        return;
    }

    // 1) 좌표 변환 + x 정렬 + 좌/우/unknown 분류
    for (const auto& seg : msg->segments)
    {
        geometry_msgs::PointStamped p_start, p_end;
        p_start.header = msg->header;
        p_end.header   = msg->header;
        p_start.point  = seg.first_point;
        p_end.point    = seg.last_point;

        geometry_msgs::PointStamped p_start_bl, p_end_bl;
        tf2::doTransform(p_start, p_start_bl, tf);
        tf2::doTransform(p_end,   p_end_bl,   tf);

        Eigen::Vector2d start_bl(p_start_bl.point.x, p_start_bl.point.y);
        Eigen::Vector2d end_bl  (p_end_bl.point.x,   p_end_bl.point.y);

        // x가 더 작은 쪽을 시작점으로 정렬
        Eigen::Vector2d p0, p1;
        if (start_bl.x() <= end_bl.x())
        {
            p0 = start_bl;
            p1 = end_bl;
        }
        else
        {
            p0 = end_bl;
            p1 = start_bl;
        }

        segments_base_.emplace_back(p0, p1);

        const double y0 = p0.y();
        const double y1 = p1.y();

        if (y0 > 0.0 && y1 > 0.0)
        {
            // 왼쪽 라인
            left_segments_.emplace_back(p0, p1);
        }
        else if (y0 < 0.0 && y1 < 0.0)
        {
            // 오른쪽 라인
            right_segments_.emplace_back(p0, p1);
        }
        else
        {
            // 위 조건 둘 다 아니면 unknown
            unknown_segments_.emplace_back(p0, p1);
        }
    }

    // 2) 터널 환경 판단 + 양쪽에서 "가장 가까운" 라인 선택
    bool left_ok  = false;
    bool right_ok = false;

    std::pair<Eigen::Vector2d, Eigen::Vector2d> best_left_seg;
    std::pair<Eigen::Vector2d, Eigen::Vector2d> best_right_seg;
    double best_left_dist  = std::numeric_limits<double>::max();
    double best_right_dist = std::numeric_limits<double>::max();

    // 왼쪽 라인들: 진행 방향과 10도 이내 + 원점과의 유클리디안 거리가 최소인 선분
    for (const auto& seg : left_segments_)
    {
        if (!isAlignedWithHeading(seg))
            continue;

        double d = euclideanDistanceToSegment(seg.first, seg.second);
        if (d < best_left_dist)
        {
            best_left_dist = d;
            best_left_seg  = seg;
            left_ok = true;
        }
    }

    // 오른쪽 라인들: 진행 방향과 10도 이내 + 원점과의 유클리디안 거리가 최소인 선분
    for (const auto& seg : right_segments_)
    {
        if (!isAlignedWithHeading(seg))
            continue;

        double d = euclideanDistanceToSegment(seg.first, seg.second);
        if (d < best_right_dist)
        {
            best_right_dist = d;
            best_right_seg  = seg;
            right_ok = true;
        }
    }

    is_tunnel_env_ = (left_ok && right_ok);

    ROS_DEBUG_STREAM("TunnelNavigationNode: total="   << segments_base_.size()
                    << " left="               << left_segments_.size()
                    << " right="              << right_segments_.size()
                    << " unknown="            << unknown_segments_.size()
                    << " tunnel_env="         << (is_tunnel_env_ ? "true" : "false"));

    if (!is_tunnel_env_)
    {
        // 터널이 아니면 조향 명령 발행 X
        ROS_INFO_STREAM("TunnelNavigationNode: not in tunnel environment");
        return;
    }

    // 3) 스탠리 형태로 조향각 계산

    // 👉 이제 여기서는 선택된 선분에 대해 "수직 거리"를 다시 계산해서 사용
    double d_left  = perpendicularDistanceToLine(best_left_seg.first,  best_left_seg.second);
    double d_right = perpendicularDistanceToLine(best_right_seg.first, best_right_seg.second);

    // 두 벽 사이 중앙에서 얼마나 벗어났는지 근사
    // d_left == d_right 이면 중앙 → cross_track_error = 0
    double cross_track_error = 0.5 * (d_left - d_right);
    // 필요 시 부호 바꾸고 싶으면 0.5 * (d_right - d_left) 로 변경

    // 벽 방향 벡터
    Eigen::Vector2d dir_left  = best_left_seg.second  - best_left_seg.first;
    Eigen::Vector2d dir_right = best_right_seg.second - best_right_seg.first;

    if (dir_left.norm() < 1e-3 || dir_right.norm() < 1e-3)
    {
        ROS_WARN_THROTTLE(1.0, "TunnelNavigationNode: line direction too small, skip steering computation.");
        return;
    }

    dir_left.normalize();
    dir_right.normalize();

    // 두 벽 방향의 평균을 터널 축 방향으로 사용
    Eigen::Vector2d center_dir = dir_left + dir_right;
    const Eigen::Vector2d heading(1.0, 0.0);

    if (center_dir.norm() < 1e-3)
    {
        // 거의 반대 방향이면 그냥 진행 방향 사용
        center_dir = heading;
    }
    else
    {
        center_dir.normalize();
    }

    // 차량 yaw를 0이라고 보면, center_dir의 yaw가 heading error
    double heading_error = std::atan2(center_dir.y(), center_dir.x());
    while (heading_error >  M_PI) heading_error -= 2.0 * M_PI;
    while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

    // 스탠리 cross-track steering (속도는 일단 command_speed_ 사용)
    double speed = command_speed_;
    double cross_track_steering = std::atan2(stanley_k_gain_ * cross_track_error, speed + 0.1);

    double steering_angle = heading_error + cross_track_steering;
    ROS_INFO_STREAM("Tunnel heading_error: " << heading_error * 180.0 / M_PI   << " [deg]");
    ROS_INFO_STREAM("Tunnel cross_track_steering: " << cross_track_steering * 180.0 / M_PI   << " [deg]");

    // 조향각 제한
    if (steering_angle >  max_steer_rad_) steering_angle =  max_steer_rad_;
    if (steering_angle < -max_steer_rad_) steering_angle = -max_steer_rad_;

    last_steering_angle_ = steering_angle;

    // 최종 조향각 터미널 출력 (deg)
    double steering_deg = steering_angle * 180.0 / M_PI;
    ROS_INFO_STREAM("Tunnel steering angle: " << steering_deg   << " [deg]");

    // 4) Ackermann 조향 명령 발행
    ackermann_msgs::AckermannDriveStamped cmd;
    cmd.header.stamp    = msg->header.stamp;
    cmd.header.frame_id = target_frame_;
    cmd.drive.speed          = command_speed_;
    cmd.drive.steering_angle = steering_angle;

    cmd_pub_.publish(cmd);
}

// main 함수
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tunnel_navigation_node");
    TunnelNavigationNode node;
    node.spin();
    return 0;
}
