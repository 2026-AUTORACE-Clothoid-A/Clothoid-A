#include "rubber_cone_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

// 스플라인 헤더 (사용자가 제공한 tk::spline)
#include "spline.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ===== 생성자 =====
RubberConeNode::RubberConeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , mission_state_(MissionState::IDLE)
    , is_obstacles_(false)
    , is_look_forward_point_(false)
{
    // ----- 파라미터 로드 -----
    pnh_.param("speed",                       speed_,                       0.4);   // m/s
    pnh_.param("vehicle_length",              vehicle_length_,              0.25);  // m
    pnh_.param("lfd",                         lfd_,                         0.6);   // m
    pnh_.param("angle_from_left_first_cone",  angle_from_left_first_cone_,  45.0);   // m
    pnh_.param("dist_from_left_first_cone",   dist_from_left_first_cone_,   1.5);   // m
    pnh_.param("dist_from_right_first_cone",  dist_from_right_first_cone_,  1.5);   // m
    pnh_.param("dist_between_cones",          dist_between_cones_,          0.4);   // m
    pnh_.param("max_turn_angle_deg",           max_turn_angle_deg_,          45.0);  // 세그먼트 사이 최대 회전 각
    pnh_.param("weight_dist",                  weight_dist_,                 1.0);   // 거리 가중치
    pnh_.param("weight_angle",                 weight_angle_,                0.5);   // 각도 가중치

    // 스플라인 샘플링 간격 (센터라인 상에서 몇 m마다 샘플링할지)
    pnh_.param("centerline_ds",               centerline_ds_,               0.05);  // 5cm 간격

    // start: 0 < x < 1.3, -0.6 < y < 0.6, 개수 >= 6
    pnh_.param("start_x_min",     start_x_min_,     0.0);
    pnh_.param("start_x_max",     start_x_max_,     1.3);
    pnh_.param("start_y_min",     start_y_min_,    -0.6);
    pnh_.param("start_y_max",     start_y_max_,     0.6);
    pnh_.param("start_min_count", start_min_count_, 6);

    // ing: 0 < x < 1.0, -1.0 < y < 1.0, 개수 >= 4
    pnh_.param("ing_x_min",       ing_x_min_,       0.0);
    pnh_.param("ing_x_max",       ing_x_max_,       2.0);
    pnh_.param("ing_y_min",       ing_y_min_,      -2.0);
    pnh_.param("ing_y_max",       ing_y_max_,       2.0);
    pnh_.param("ing_min_count",   ing_min_count_,   4);

    // ----- ROS -----
    obstacles_sub_ = nh_.subscribe(
        "/raw_obstacles", 1,
        &RubberConeNode::obstaclesCallback, this);

    control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(
        "high_level/ackermann_cmd_mux/input/nav_1", 1);

    centerline_path_pub_ = nh_.advertise<nav_msgs::Path>(
        "rubber_cone_centerline", 1, false);

    left_boundary_path_pub_  = nh_.advertise<nav_msgs::Path>("left_boundary_path", 1, false);
    right_boundary_path_pub_ = nh_.advertise<nav_msgs::Path>("right_boundary_path", 1, false);

    // 제어 메시지 초기화
    target_control_.drive.steering_angle = 0.0;
    target_control_.drive.speed          = speed_;
}

// ===== 콜백: /raw_obstacles =====
void RubberConeNode::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& msg)
{
    is_obstacles_ = true;
    updatePointsFromObstacles(*msg);

    switch (mission_state_)
    {
    case MissionState::IDLE:
        if (checkMissionStartCondition())
        {
            mission_state_ = MissionState::RUNNING;
            ROS_INFO("[RubberConeNode] Mission START");
        }
        break;

    case MissionState::RUNNING:
    {
        if (!checkMissionContinueCondition())
        {
            mission_state_ = MissionState::IDLE;
            ROS_INFO("[RubberConeNode] Mission END (no longer in cone zone)");
            publishControl(0.0);
            break;
        }

        // 1) 경계 만들기
        std::vector<Eigen::Vector2d> left_line, right_line;
        buildBoundaries(left_line, right_line);

        publishBoundaryPaths(left_line, right_line);

        // 2) 센터라인 계산 (스플라인 보간 포함)
        std::vector<Eigen::Vector2d> center_line;
        buildCenterLine(left_line, right_line, center_line);

        publishCenterlinePath(center_line);

        // 3) 스티어링 계산 & 발행
        double steering = computeSteeringFromCenterLine(center_line);
        publishControl(steering);

        break;
    }
    default:
        break;
    }
}

// ===== 1) 입력 처리 =====
void RubberConeNode::updatePointsFromObstacles(const obstacle_detector::Obstacles& msg)
{
    points_.clear();
    points_.reserve(msg.circles.size());

    for (const auto& c : msg.circles)
    {
        // base_link 기준 x: 전방(+), y: 좌측(+)
        points_.emplace_back(c.center.x, c.center.y);
    }
}

// ===== 2) 미션 상태 판정 =====
bool RubberConeNode::checkMissionStartCondition() const
{
    int count = 0;
    for (const auto& p : points_)
    {
        double x = p.x();
        double y = p.y();
        if (x > start_x_min_ && x < start_x_max_ &&
            y > start_y_min_ && y < start_y_max_)
        {
            ++count;
            if (count >= start_min_count_)
                return true;
        }
    }
    return false;
}

bool RubberConeNode::checkMissionContinueCondition() const
{
    int count = 0;
    for (const auto& p : points_)
    {
        double x = p.x();
        double y = p.y();
        if (x > ing_x_min_ && x < ing_x_max_ &&
            y > ing_y_min_ && y < ing_y_max_)
        {
            ++count;
            if (count >= ing_min_count_)
                return true;
        }
    }
    return false;
}

// ===== 3) 경계 추출 =====

// 왼쪽(y>0) / 오른쪽(y<0) 포인트 분리
std::vector<Eigen::Vector2d> RubberConeNode::extractSidePoints(bool left_side) const
{
    std::vector<Eigen::Vector2d> side_points;
    side_points.reserve(points_.size());

    if (left_side)
    {
        for (const auto& p : points_)
        {
            if (p.y() > 0.0)  // base_link에서 좌측
                side_points.push_back(p);
        }
    }
    else
    {
        for (const auto& p : points_)
        {
            if (p.y() < 0.0)  // base_link에서 우측
                side_points.push_back(p);
        }
    }

    return side_points;
}

// 각 측에서 첫 번째 콘 선택 (angle 필터 + 가장 가까운 점)
bool RubberConeNode::selectFirstCone(const std::vector<Eigen::Vector2d>& side_points,
                                     bool left_side,
                                     Eigen::Vector2d& out_first_cone) const
{
    if (side_points.empty())
        return false;

    // 1단계: 각도 필터링 - "옆"에 있는 콘만 후보로
    std::vector<Eigen::Vector2d> filtered;
    filtered.reserve(side_points.size());

    // 옆 방향을 기준으로 한 각도 범위
    for (const auto& p : side_points)
    {
        double ang_deg = std::atan2(p.y(), p.x()) * 180.0 / M_PI;

        if (left_side)
        {
            // 왼쪽 옆 방향 근처만 사용
            if (ang_deg > angle_from_left_first_cone_)
                filtered.push_back(p);
        }
        else
        {
            // 오른쪽 옆 방향 근처만 사용
            if (ang_deg < -angle_from_left_first_cone_)
                filtered.push_back(p);
        }
    }

    const auto& candidates = filtered.empty() ? side_points : filtered;

    // 2단계: 거리 기준 가장 가까운 점 선택
    double best_dist = std::numeric_limits<double>::infinity();
    Eigen::Vector2d best_point;
    bool found = false;

    for (const auto& p : candidates)
    {
        double d = p.norm();
        if (d < best_dist)
        {
            best_dist = d;
            best_point = p;
            found = true;
        }
    }

    if (!found)
        return false;

    // 3단계: 시작 콘까지의 최대 거리 조건
    double max_first_dist = left_side ? dist_from_left_first_cone_ : dist_from_right_first_cone_;
    if (best_dist > max_first_dist)
        return false;

    out_first_cone = best_point;
    return true;
}

double RubberConeNode::normalizeAngle(double a) const
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

double RubberConeNode::angleDiff(double a, double b) const
{
    // a - b 를 [-pi, pi] 범위로
    return normalizeAngle(a - b);
}


// 한 측 라인을 첫 콘에서부터 앞으로 이어서 생성
void RubberConeNode::buildLineFromFirst(const std::vector<Eigen::Vector2d>& all_points,
                                        const Eigen::Vector2d& first_point,
                                        std::vector<Eigen::Vector2d>& line) const
{
    line.clear();
    if (all_points.empty())
        return;

    // 시작점 추가
    line.push_back(first_point);
    Eigen::Vector2d current_point = first_point;

    const double max_step_dist = dist_between_cones_;        // 기본 거리 한계
    const double max_turn_angle_rad = max_turn_angle_deg_ * M_PI / 180.0;

    while (true)
    {
        bool found = false;
        double best_cost = std::numeric_limits<double>::infinity();
        Eigen::Vector2d best_point;

        for (const auto& p : all_points)
        {
            // 0) 이미 라인에 포함된 콘이면 제외
            bool already_used = false;
            for (const auto& lp : line)
            {
                if ((p - lp).norm() < 1e-4)
                {
                    already_used = true;
                    break;
                }
            }
            if (already_used)
                continue;

            // 1) "앞쪽" 콘만 사용 (base_link: x가 커질수록 전방)
            if (p.x() <= current_point.x())
                continue;

            // 2) 거리 제약
            Eigen::Vector2d diff_vec = p - current_point;
            double dist = diff_vec.norm();
            if (dist > max_step_dist)
                continue;

            // 3) 각도 제약: 라인에 두 개 이상 포인트가 있을 때만 적용
            double angle_penalty = 0.0;
            if (line.size() >= 2)
            {
                const Eigen::Vector2d& second_last = line[line.size() - 2];
                const Eigen::Vector2d& last        = line[line.size() - 1];

                Eigen::Vector2d v_prev = last - second_last; // 이전 세그먼트
                Eigen::Vector2d v_curr = p    - last;        // 후보 세그먼트

                if (v_prev.norm() < 1e-6 || v_curr.norm() < 1e-6)
                    continue; // 너무 가까운 점들 → 노이즈로 취급

                double ang_prev = std::atan2(v_prev.y(), v_prev.x());
                double ang_curr = std::atan2(v_curr.y(), v_curr.x());
                double dtheta   = std::fabs(angleDiff(ang_curr, ang_prev));

                // 각도 제약: 너무 급격히 꺾이면 제외
                if (dtheta > max_turn_angle_rad)
                    continue;

                angle_penalty = dtheta;
            }
            else
            {
                // 첫 번째 세그먼트인 경우: 차량 진행 방향(x+)과의 각도 차이를 penalty로 줄 수도 있음
                Eigen::Vector2d v_curr = p - current_point;
                if (v_curr.norm() < 1e-6)
                    continue;
                double ang_curr = std::atan2(v_curr.y(), v_curr.x());
                double ang_car  = 0.0; // base_link: x+ 방향
                angle_penalty = std::fabs(angleDiff(ang_curr, ang_car));
                if (angle_penalty > max_turn_angle_rad)
                    continue;
            }

            // 4) 비용 함수: 거리 + 각도
            double cost = weight_dist_ * dist + weight_angle_ * angle_penalty;

            if (cost < best_cost)
            {
                best_cost  = cost;
                best_point = p;
                found      = true;
            }
        }

        if (!found)
            break;

        line.push_back(best_point);
        current_point = best_point;
    }
}

// 전체 경계 (left_line, right_line) 구성
void RubberConeNode::buildBoundaries(std::vector<Eigen::Vector2d>& left_line,
                                     std::vector<Eigen::Vector2d>& right_line) const
{
    left_line.clear();
    right_line.clear();

    // 왼쪽
    auto left_points = extractSidePoints(true);
    if (!left_points.empty())
    {
        Eigen::Vector2d first_left;
        if (selectFirstCone(left_points, true, first_left))
        {
            buildLineFromFirst(points_, first_left, left_line);
        }
    }

    // 오른쪽
    auto right_points = extractSidePoints(false);
    if (!right_points.empty())
    {
        Eigen::Vector2d first_right;
        if (selectFirstCone(right_points, false, first_right))
        {
            buildLineFromFirst(points_, first_right, right_line);
        }
    }
}

// ===== 4) 센터라인 / 스티어링 =====

// raw center line 을 spline 으로 부드럽게 보간
void RubberConeNode::smoothCenterLineWithSpline(const std::vector<Eigen::Vector2d>& raw_center,
                                                std::vector<Eigen::Vector2d>& smooth_center) const
{
    smooth_center.clear();

    const std::size_t N = raw_center.size();
    if (N < 3)
    {
        // 포인트가 너무 적으면 그냥 복사해서 사용
        smooth_center = raw_center;
        return;
    }

    // 1) arc-length 파라미터 s 계산
    std::vector<double> s(N, 0.0);
    for (std::size_t i = 1; i < N; ++i)
    {
        double ds = (raw_center[i] - raw_center[i - 1]).norm();
        s[i] = s[i - 1] + ds;
    }

    double s_total = s.back();
    if (s_total < 1e-6)
    {
        smooth_center = raw_center;
        return;
    }

    // 2) x(s), y(s) 데이터 준비
    std::vector<double> xs(N), ys(N);
    for (std::size_t i = 0; i < N; ++i)
    {
        xs[i] = raw_center[i].x();
        ys[i] = raw_center[i].y();
    }

    // 3) tk::spline 으로 각각 보간
    tk::spline sx, sy;
    sx.set_points(s, xs, tk::spline::cspline);
    sy.set_points(s, ys, tk::spline::cspline);

    // 4) s를 일정 간격(centerline_ds_)으로 나눠서 샘플링
    double ds_sample = centerline_ds_;
    if (ds_sample <= 1e-4)
        ds_sample = s_total / static_cast<double>(N - 1); // fallback

    int M = static_cast<int>(std::ceil(s_total / ds_sample)) + 1;
    if (M < 2) M = 2;

    smooth_center.reserve(M);
    for (int i = 0; i < M; ++i)
    {
        double si = ds_sample * static_cast<double>(i);
        if (si > s_total)
            si = s_total;

        double x = sx(si);
        double y = sy(si);
        smooth_center.emplace_back(x, y);
    }
}

// left/right_line → center_line (스플라인 보간 포함)
void RubberConeNode::buildCenterLine(const std::vector<Eigen::Vector2d>& left_line,
                                     const std::vector<Eigen::Vector2d>& right_line,
                                     std::vector<Eigen::Vector2d>& center_line) const
{
    center_line.clear();
    std::vector<Eigen::Vector2d> raw_center;

    // 1) 원래 Python 코드와 동일한 방식으로 raw_center 구성
    if (!left_line.empty() && !right_line.empty())
    {
        const std::size_t L = left_line.size();
        const std::size_t R = right_line.size();

        if (L > R)
        {
            for (std::size_t i = 0; i < R; ++i)
                raw_center.push_back(0.5 * (left_line[i] + right_line[i]));

            Eigen::Vector2d diff = raw_center[R - 1] - left_line[R - 1];
            for (std::size_t i = R; i < L; ++i)
                raw_center.push_back(left_line[i] + diff);
        }
        else if (L < R)
        {
            for (std::size_t i = 0; i < L; ++i)
                raw_center.push_back(0.5 * (left_line[i] + right_line[i]));

            Eigen::Vector2d diff = raw_center[L - 1] - right_line[L - 1];
            for (std::size_t i = L; i < R; ++i)
                raw_center.push_back(right_line[i] + diff);
        }
        else
        {
            for (std::size_t i = 0; i < L; ++i)
                raw_center.push_back(0.5 * (left_line[i] + right_line[i]));
        }
    }
    else if (!left_line.empty() && right_line.empty())
    {
        Eigen::Vector2d diff = -left_line.front();
        for (const auto& lp : left_line)
            raw_center.push_back(lp + diff);
    }
    else if (left_line.empty() && !right_line.empty())
    {
        Eigen::Vector2d diff = -right_line.front();
        for (const auto& rp : right_line)
            raw_center.push_back(rp + diff);
    }

    // 2) 스플라인 보간으로 부드러운 center_line 생성
    smoothCenterLineWithSpline(raw_center, center_line);
}

// center_line → steering angle
double RubberConeNode::computeSteeringFromCenterLine(std::vector<Eigen::Vector2d>& center_line)
{
    if (center_line.empty())
    {
        is_look_forward_point_ = false;
        return 0.0;
    }

    // x 오름차순 정렬 (앞쪽부터)
    std::sort(center_line.begin(), center_line.end(),
              [](const Eigen::Vector2d& a, const Eigen::Vector2d& b)
              {
                  return a.x() < b.x();
              });

    Eigen::Vector2d first = center_line.front();
    double theta = 0.0;
    is_look_forward_point_ = false;

    for (const auto& p : center_line)
    {
        double dis = p.norm();
        if (dis >= lfd_)
        {
            is_look_forward_point_ = true;
            Eigen::Vector2d v = p; // + first;
            theta = std::atan2(v.y(), v.x());
            break;
        }
    }

    if (!is_look_forward_point_)
    {
        const Eigen::Vector2d& last = center_line.back();
        Eigen::Vector2d v = last; // + first;
        theta = std::atan2(v.y(), v.x() + vehicle_length_);
    }

    double steering = -std::atan2(2.0 * vehicle_length_ * std::sin(theta), lfd_);
    return steering;
}

void RubberConeNode::publishControl(double steering)
{
    target_control_.header.stamp = ros::Time::now();
    target_control_.drive.speed  = speed_;
    target_control_.drive.steering_angle = steering;

    control_pub_.publish(target_control_);

    ROS_INFO_STREAM_THROTTLE(1.0,
        "[RubberConeNode] mission running, steering = " << steering * 180.0 / M_PI << "deg.");
}

void RubberConeNode::fillPathMsg(const std::vector<Eigen::Vector2d>& pts,
                                 const std::string& frame_id,
                                 nav_msgs::Path& msg) const
{
    msg.header.stamp    = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.poses.clear();
    msg.poses.reserve(pts.size());

    for (const auto& p : pts)
    {
        geometry_msgs::PoseStamped ps;
        ps.header = msg.header;
        ps.pose.position.x = p.x();
        ps.pose.position.y = p.y();
        ps.pose.position.z = 0.0;

        ps.pose.orientation.x = 0.0;
        ps.pose.orientation.y = 0.0;
        ps.pose.orientation.z = 0.0;
        ps.pose.orientation.w = 1.0;

        msg.poses.push_back(ps);
    }
}

void RubberConeNode::publishCenterlinePath(const std::vector<Eigen::Vector2d>& center_line)
{
    if (center_line.empty())
        return;

    nav_msgs::Path path_msg;
    fillPathMsg(center_line, "base_link", path_msg);
    centerline_path_pub_.publish(path_msg);
}

void RubberConeNode::publishBoundaryPaths(const std::vector<Eigen::Vector2d>& left_line,
                                          const std::vector<Eigen::Vector2d>& right_line)
{
    // left
    if (!left_line.empty())
    {
        nav_msgs::Path left_msg;
        fillPathMsg(left_line, "base_link", left_msg);
        left_boundary_path_pub_.publish(left_msg);
    }

    // right
    if (!right_line.empty())
    {
        nav_msgs::Path right_msg;
        fillPathMsg(right_line, "base_link", right_msg);
        right_boundary_path_pub_.publish(right_msg);
    }
}

// ===== main =====
int main(int argc, char** argv)
{
    ros::init(argc, argv, "rubber_cone_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    RubberConeNode node(nh, pnh);

    ros::spin();
    return 0;
}
