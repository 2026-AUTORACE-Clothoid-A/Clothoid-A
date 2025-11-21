#include "cone_sorter_node.hpp"

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace cone_sorter
{

// ===== 유틸: 벡터 사이 각도 (0 ~ pi) =====
static double angleBetween(const Eigen::Vector2d& a, const Eigen::Vector2d& b)
{
    double na = a.norm();
    double nb = b.norm();
    if (na < 1e-9 || nb < 1e-9)
        return 0.0;

    double c = a.dot(b) / (na * nb);
    c = std::max(-1.0, std::min(1.0, c));
    return std::acos(c);
}

static double deg2rad(double d) { return d * M_PI / 180.0; }

// ===== 생성자 =====
ConeSorterNode::ConeSorterNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , mission_state_(MissionState::IDLE)
    , is_obstacles_(false)
{
    // ---------- 파라미터 로드 (스케일카 환경 기본값) ----------
    pnh_.param("max_dist_to_first",        max_dist_to_first_,        1.0);   // m
    pnh_.param("max_dist_between_cones",   max_dist_between_cones_,   0.4);   // m
    pnh_.param("max_n_neighbors",          max_n_neighbors_,          10);    // 개
    pnh_.param("max_trace_length",         max_trace_length_,         20);    // 콘 개수

    pnh_.param("threshold_directional_angle_deg", threshold_directional_angle_deg_, 30.0);  // deg
    pnh_.param("threshold_absolute_angle_deg",    threshold_absolute_angle_deg_,   45.0);  // deg

    pnh_.param("ellipse_major_radius_factor", ellipse_major_radius_factor_, 2.0);  // * max_dist_between_cones_
    pnh_.param("ellipse_minor_radius_factor", ellipse_minor_radius_factor_, 1.0);  // * max_dist_between_cones_
    pnh_.param("len_override_factor",         len_override_factor_,        2.0);   // * max_dist_between_cones_

    pnh_.param("car_size",                    car_size_,                  0.5);    // m (대략 차량 길이)

    // 각도 비용에서 "꺾였다"고 보는 기준
    pnh_.param("angle_cost_threshold_deg",      angle_cost_threshold_deg_,      30.0);

    // 콘 간격이 이 값보다 크면 residual distance cost에 반영
    pnh_.param("residual_dist_threshold",       residual_dist_threshold_,       max_dist_between_cones_);

    // wrong_direction_cost 에서 사용할 각도 임계값
    pnh_.param("wrong_dir_threshold_deg",       wrong_dir_threshold_deg_,       30.0);

    // 주변 콘 검색 파라미터 (cones_on_either_cost)
    pnh_.param("cones_side_search_dist",        cones_side_search_dist_,        0.8);
    pnh_.param("cones_side_search_angle_deg",   cones_side_search_angle_deg_,   120.0);

    // ---- 항목별 가중치 ----
    pnh_.param("w_angle_cost",          w_angle_cost_,          1000.0);
    pnh_.param("w_residual_dist_cost",  w_residual_dist_cost_,   200.0);
    pnh_.param("w_num_cones_cost",      w_num_cones_cost_,      5000.0);
    pnh_.param("w_initial_dir_cost",    w_initial_dir_cost_,    1000.0);
    pnh_.param("w_change_dir_cost",     w_change_dir_cost_,     2000.0);
    pnh_.param("w_cones_side_cost",     w_cones_side_cost_,     1000.0);
    pnh_.param("w_wrong_dir_cost",      w_wrong_dir_cost_,      1000.0);

    // --- 러버콘 스타일 첫 점 선택용 파라미터 ---
    pnh_.param("angle_from_left_first_cone", angle_from_left_first_cone_deg_, 45.0);  // deg
    pnh_.param("dist_from_left_first_cone",  dist_from_left_first_cone_,      1.0);   // m
    pnh_.param("dist_from_right_first_cone", dist_from_right_first_cone_,     1.0);   // m

    // --- 미션 판정 영역 파라미터 ---
    // start: 0 < x < 1.3, -0.6 < y < 0.6, 개수 >= 6 (RubberCone 기본값과 동일)
    pnh_.param("start_x_min",     start_x_min_,     0.0);
    pnh_.param("start_x_max",     start_x_max_,     1.3);
    pnh_.param("start_y_min",     start_y_min_,    -0.6);
    pnh_.param("start_y_max",     start_y_max_,     0.6);
    pnh_.param("start_min_count", start_min_count_, 6);

    // ing: 0 < x < 2.0, -2.0 < y < 2.0, 개수 >= 4
    pnh_.param("ing_x_min",       ing_x_min_,       0.0);
    pnh_.param("ing_x_max",       ing_x_max_,       2.0);
    pnh_.param("ing_y_min",       ing_y_min_,      -2.0);
    pnh_.param("ing_y_max",       ing_y_max_,       2.0);
    pnh_.param("ing_min_count",   ing_min_count_,   4);

    // base_link 기준: 차량은 원점, x+ 정면
    car_pos_ = Eigen::Vector2d(0.0, 0.0);
    car_dir_ = Eigen::Vector2d(1.0, 0.0);

    obstacles_sub_ = nh_.subscribe(
        "/raw_obstacles", 1,
        &ConeSorterNode::obstaclesCallback, this);

    // 경계 퍼블리셔 (nav_msgs/Path)
    left_boundary_pub_  = nh_.advertise<nav_msgs::Path>("left_boundary_path", 1, true);
    right_boundary_pub_ = nh_.advertise<nav_msgs::Path>("right_boundary_path", 1, true);

    // 미션 상태 퍼블리셔 (true: 미션 구간, false: 미션 아님)
    mission_state_pub_  = nh_.advertise<std_msgs::Bool>("cone_mission_active", 1, true);
}

// ===== raw_obstacles 콜백 =====
void ConeSorterNode::obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& msg)
{
    is_obstacles_ = true;
    cones_.clear();
    cones_.reserve(msg->circles.size());

    for (const auto& c : msg->circles)
    {
        // base_link 기준 원형 장애물 중심
        cones_.emplace_back(c.center.x, c.center.y);
    }

    // ----- 미션 상태 업데이트 -----
    switch (mission_state_)
    {
    case MissionState::IDLE:
        if (checkMissionStartCondition())
        {
            mission_state_ = MissionState::RUNNING;
            ROS_INFO("[ConeSorterNode] Mission START");
        }
        break;

    case MissionState::RUNNING:
        if (!checkMissionContinueCondition())
        {
            mission_state_ = MissionState::IDLE;
            ROS_INFO("[ConeSorterNode] Mission END (no longer in cone zone)");
            break;
        }
        // 미션 진행 중일 때만 경계 생성 파이프라인 수행
        processOnce();
        break;
    default:
        break;
    }

    // ----- 미션 상태 토픽 발행 (true: RUNNING, false: IDLE) -----
    std_msgs::Bool m;
    m.data = (mission_state_ == MissionState::RUNNING);
    mission_state_pub_.publish(m);
}

// ===== 미션 상태 판정 =====
bool ConeSorterNode::checkMissionStartCondition() const
{
    int count = 0;
    for (const auto& p : cones_)
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

bool ConeSorterNode::checkMissionContinueCondition() const
{
    int count = 0;
    for (const auto& p : cones_)
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

// ===== Stage1: 기존 mask/select (폴백용으로 유지) =====
void ConeSorterNode::maskConeCanBeFirst(
    Side side,
    std::vector<double>& distances,
    std::vector<bool>& mask_valid) const
{
    const std::size_t N = cones_.size();
    distances.assign(N, 0.0);
    mask_valid.assign(N, false);

    if (N == 0)
        return;

    double car_theta = std::atan2(car_dir_.y(), car_dir_.x());
    double cos_t = std::cos(-car_theta);
    double sin_t = std::sin(-car_theta);

    // 타원 파라미터
    double a = max_dist_to_first_;      // major radius
    double b = max_dist_to_first_ * 2.0;      // minor radius

    int expected_sign = (side == Side::LEFT) ? +1 : -1;

    for (std::size_t i = 0; i < N; ++i)
    {
        Eigen::Vector2d p = cones_[i];

        // 차량에서 콘까지 벡터
        Eigen::Vector2d d = p - car_pos_;

        // 차량 진행 방향 기준 회전 (car_dir -> x축)
        double xr = d.x() * cos_t + d.y() * sin_t;
        double yr = -d.x() * sin_t + d.y() * cos_t;

        double angle_rel = std::atan2(yr, xr); // 차량 기준 각
        double dist = d.norm();

        distances[i] = dist;

        // 타원 내부 체크
        Eigen::Vector2d m = car_dir_.normalized();          // major 방향
        Eigen::Vector2d n(-m.y(), m.x());                   // minor 방향
        double x = d.dot(m);
        double y = d.dot(n);
        double val = (x * x) / (a * a) + (y * y) / (b * b);
        bool in_ellipse = (val <= 1.0);

        // 좌/우 side & 각도 범위 조건
        int sign_angle = (angle_rel > 0.0) ? +1 : (angle_rel < 0.0 ? -1 : 0);
        bool mask_valid_side = (sign_angle == expected_sign);

        bool mask_valid_angle_max = std::fabs(angle_rel) < (M_PI - M_PI / 5.0);  // < 144도
        bool mask_valid_angle_min = std::fabs(angle_rel) > (M_PI / 10.0);       // > 18도

        mask_valid[i] = in_ellipse && mask_valid_side && mask_valid_angle_max && mask_valid_angle_min;
    }
}

int ConeSorterNode::selectStartingCone(
    Side /*side*/,
    const std::vector<int>* indices_to_skip,
    const std::vector<double>& distances,
    const std::vector<bool>& mask_valid) const
{
    const std::size_t N = cones_.size();
    if (N == 0)
        return -1;

    std::vector<bool> valid = mask_valid;  // 로컬 복사

    if (indices_to_skip)
    {
        for (int idx : *indices_to_skip)
        {
            if (idx >= 0 && static_cast<std::size_t>(idx) < N)
                valid[idx] = false;
        }
    }

    int best_idx = -1;
    double best_dist = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < N; ++i)
    {
        if (!valid[i])
            continue;

        double d = distances[i];
        if (d < best_dist)
        {
            best_dist = d;
            best_idx = static_cast<int>(i);
        }
    }

    if (best_idx < 0)
        return -1;

    if (best_dist > max_dist_to_first_)
        return -1;

    return best_idx;
}

std::vector<int> ConeSorterNode::selectFirstKStartingCones(Side side) const
{
    std::vector<int> result;

    const std::size_t N = cones_.size();
    if (N < 1)
        return result;

    // 1) mask + 거리 계산
    std::vector<double> distances;
    std::vector<bool> mask_valid;
    maskConeCanBeFirst(side, distances, mask_valid);

    // 2) 첫 번째 시작 콘
    int index_1 = selectStartingCone(side, nullptr, distances, mask_valid);
    if (index_1 < 0)
        return result;

    // 3) 차량 앞쪽 콘들은 두 번째 시작 콘 후보에서 제외
    std::vector<int> indices_to_skip;
    indices_to_skip.reserve(N);

    for (std::size_t i = 0; i < N; ++i)
    {
        Eigen::Vector2d d = cones_[i] - car_pos_;
        double ang = angleBetween(d, car_dir_);  // 0~pi

        if (ang < M_PI / 2.0)  // 차량 앞쪽(±90도 이내)
            indices_to_skip.push_back(static_cast<int>(i));
    }

    if (std::find(indices_to_skip.begin(), indices_to_skip.end(), index_1) == indices_to_skip.end())
        indices_to_skip.push_back(index_1);

    // 4) 두 번째 시작 콘 (차량 뒤쪽 쪽)
    int index_2 = selectStartingCone(side, &indices_to_skip, distances, mask_valid);
    if (index_2 < 0)
    {
        // 두 번째 콘이 없으면 한 개만 사용
        result.push_back(index_1);
        return result;
    }

    // 5) 두 콘 방향을 차량 진행 방향과 맞게 정렬
    Eigen::Vector2d dir1 = cones_[index_1] - cones_[index_2];
    Eigen::Vector2d dir2 = cones_[index_2] - cones_[index_1];

    double angle1 = angleBetween(dir1, car_dir_);
    double angle2 = angleBetween(dir2, car_dir_);

    if (angle1 > angle2)
        std::swap(index_1, index_2);

    // 6) 두 콘 사이 거리 조건
    double dist_between = (cones_[index_1] - cones_[index_2]).norm();
    if (dist_between > max_dist_between_cones_ * 1.1 || dist_between < 0.1)
    {
        result.push_back(index_1);
        return result;
    }

    // 7) 두 개 모두 사용
    result.push_back(index_2);
    result.push_back(index_1);
    return result;
}

// ===== 러버콘 로직 기반: 측별 인덱스 추출 =====
std::vector<int> ConeSorterNode::extractSideIndices(Side side) const
{
    std::vector<int> idx;
    idx.reserve(cones_.size());

    if (side == Side::LEFT)
    {
        for (std::size_t i = 0; i < cones_.size(); ++i)
        {
            if (cones_[i].y() > 0.0)
                idx.push_back(static_cast<int>(i));
        }
    }
    else
    {
        for (std::size_t i = 0; i < cones_.size(); ++i)
        {
            if (cones_[i].y() < 0.0)
                idx.push_back(static_cast<int>(i));
        }
    }
    return idx;
}

// ===== 러버콘 selectFirstCone() 기반 시작 콘 인덱스 선택 =====
bool ConeSorterNode::selectFirstConeIndex(Side side, int& out_idx) const
{
    auto side_indices = extractSideIndices(side);
    if (side_indices.empty())
        return false;

    const double ang_th = angle_from_left_first_cone_deg_;
    std::vector<int> filtered;
    filtered.reserve(side_indices.size());

    // 1단계: 각도 필터
    for (int idx : side_indices)
    {
        const auto& p = cones_[idx];
        double ang_deg = std::atan2(p.y(), p.x()) * 180.0 / M_PI;

        if (side == Side::LEFT)
        {
            // 왼쪽: ang > angle_from_left_first_cone
            if (ang_deg > ang_th)
                filtered.push_back(idx);
        }
        else
        {
            // 오른쪽: ang < -angle_from_left_first_cone
            if (ang_deg < -ang_th)
                filtered.push_back(idx);
        }
    }

    const std::vector<int>& candidates = filtered;

    // 2단계: 거리 최소 후보 선택
    double best_dist = std::numeric_limits<double>::infinity();
    int best_idx = -1;
    for (int idx : candidates)
    {
        double d = cones_[idx].norm();
        if (d < best_dist)
        {
            best_dist = d;
            best_idx = idx;
        }
    }

    if (best_idx < 0)
        return false;

    // 3단계: 시작 콘까지의 최대 거리 조건
    double max_first_dist = (side == Side::LEFT)
                          ? dist_from_left_first_cone_
                          : dist_from_right_first_cone_;

    if (best_dist > max_first_dist)
    {
        // 조건 실패 시: 기존 Stage1 로직으로 폴백
        // std::vector<int> legacy = selectFirstKStartingCones(side);
        // if (!legacy.empty())
        // {
        //     out_idx = legacy.front();
        //     return true;
        // }
        return false;
    }

    out_idx = best_idx;
    return true;
}

// ===== Stage2: 거리 행렬 (제곱 거리) =====
Eigen::MatrixXd ConeSorterNode::computeDistanceMatrix(const std::vector<Eigen::Vector2d>& points) const
{
    const int N = static_cast<int>(points.size());
    Eigen::MatrixXd dist_sq = Eigen::MatrixXd::Zero(N, N);

    const double INF = std::numeric_limits<double>::infinity();

    for (int i = 0; i < N; ++i)
    {
        dist_sq(i, i) = INF;
        for (int j = i + 1; j < N; ++j)
        {
            Eigen::Vector2d d = points[i] - points[j];
            double val = d.squaredNorm();
            dist_sq(i, j) = val;
            dist_sq(j, i) = val;
        }
    }
    return dist_sq;
}

// ===== Stage2: 각 점에서 k개의 최근접 이웃 찾기 =====
std::vector<std::vector<int>> ConeSorterNode::findKClosestInPointCloud(
    const Eigen::MatrixXd& dist_sq,
    int k) const
{
    const int N = static_cast<int>(dist_sq.rows());
    std::vector<std::vector<int>> neighbors(N);

    if (N == 0)
        return neighbors;

    k = std::max(1, std::min(k, N - 1));

    std::vector<int> idxs(N);
    for (int i = 0; i < N; ++i)
        idxs[i] = i;

    for (int i = 0; i < N; ++i)
    {
        // 자기 자신을 제외한 인덱스 집합
        std::vector<int> order = idxs;
        std::sort(order.begin(), order.end(),
                  [&](int a, int b)
                  {
                      return dist_sq(i, a) < dist_sq(i, b);
                  });

        neighbors[i].clear();
        neighbors[i].reserve(k);
        int picked = 0;
        for (int id : order)
        {
            if (id == i)
                continue;
            neighbors[i].push_back(id);
            if (++picked >= k)
                break;
        }
    }
    return neighbors;
}

// ===== Stage2: BFS =====
std::vector<int> ConeSorterNode::breadthFirstOrder(
    const Eigen::MatrixXi& adjacency,
    int start_idx) const
{
    const int N = static_cast<int>(adjacency.rows());
    std::vector<int> order;
    if (N == 0 || start_idx < 0 || start_idx >= N)
        return order;

    std::vector<uint8_t> visited(N, 0);
    std::vector<int> queue(N, -1);
    int q_head = 0;
    int q_tail = 0;

    queue[0] = start_idx;
    visited[start_idx] = 1;

    while (q_head <= q_tail)
    {
        int node = queue[q_head++];
        order.push_back(node);

        for (int j = 0; j < N; ++j)
        {
            if (adjacency(node, j) && !visited[j])
            {
                visited[j] = 1;
                queue[++q_tail] = j;
            }
        }
    }

    return order;
}

// ===== Stage2: 인접 행렬 생성 + reachable node 계산 =====
void ConeSorterNode::createAdjacencyMatrix(
    const std::vector<Eigen::Vector2d>& points,
    int n_neighbors,
    int start_idx,
    double max_dist,
    Eigen::MatrixXi& adjacency_out,
    std::vector<int>& reachable_out) const
{
    const int N = static_cast<int>(points.size());
    adjacency_out = Eigen::MatrixXi();
    reachable_out.clear();

    if (N == 0 || start_idx < 0 || start_idx >= N)
        return;

    // 1) 거리 행렬
    Eigen::MatrixXd dist_sq = computeDistanceMatrix(points);

    // 2) 각 점에서 k개의 최근접 이웃 인덱스
    auto k_closest = findKClosestInPointCloud(dist_sq, n_neighbors);

    // 3) 인접 행렬 초기화
    Eigen::MatrixXi adjacency = Eigen::MatrixXi::Zero(N, N);

    for (int i = 0; i < N; ++i)
    {
        for (int j : k_closest[i])
        {
            if (j >= 0 && j < N)
                adjacency(i, j) = 1;
        }
    }

    // 4) 최대 거리 제한
    double max_d2 = max_dist * max_dist;
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            if (dist_sq(i, j) > max_d2)
                adjacency(i, j) = 0;
        }
    }

    // 5) 무향 그래프 변환
    for (int i = 0; i < N; ++i)
    {
        for (int j = i + 1; j < N; ++j)
        {
            int v = (adjacency(i, j) && adjacency(j, i)) ? 1 : 0;
            adjacency(i, j) = v;
            adjacency(j, i) = v;
        }
    }

    // 6) BFS로 start_idx에서 도달 가능한 노드들 찾기
    reachable_out = breadthFirstOrder(adjacency, start_idx);

    adjacency_out = adjacency;
}

// ========== 3단계: DFS/스택으로 end configurations 탐색 ==========

double ConeSorterNode::normalizeAngle(double a) const
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

double ConeSorterNode::angleDifference(double a, double b) const
{
    return normalizeAngle(a - b);
}

bool ConeSorterNode::isInsideLocalEllipse(
    const Eigen::Vector2d& second_last,
    const Eigen::Vector2d& last,
    const Eigen::Vector2d& candidate) const
{
    Eigen::Vector2d major = last - second_last;
    double norm_major = major.norm();
    if (norm_major < 1e-6)
        return true;

    double a = ellipse_major_radius_factor_ * max_dist_between_cones_;
    double b = ellipse_minor_radius_factor_ * max_dist_between_cones_;
    if (a < 1e-6 || b < 1e-6)
        return true;

    Eigen::Vector2d e1 = major / norm_major;
    Eigen::Vector2d e2(-e1.y(), e1.x());

    Eigen::Vector2d d = candidate - last;
    double u = d.dot(e1);
    double v = d.dot(e2);

    double val = (u * u) / (a * a) + (v * v) / (b * b);
    return val <= 1.0;
}

bool ConeSorterNode::secondConeOnExpectedSide(
    const Eigen::Vector2d& candidate,
    Side side) const
{
    Eigen::Vector2d v = candidate - car_pos_;

    double angle_car = std::atan2(car_dir_.y(), car_dir_.x());
    double angle_to_candidate = std::atan2(v.y(), v.x());
    double diff = angleDifference(angle_to_candidate, angle_car);

    int expected_sign = (side == Side::LEFT) ? +1 : -1;

    int s = 0;
    if (diff > 0.0) s = +1;
    else if (diff < 0.0) s = -1;

    // 기대한 방향 또는 약간의 오차 허용 (5deg)
    if (s == expected_sign)
        return true;

    double tol = 5.0 * M_PI / 180.0;
    if (std::fabs(diff) < tol)
        return true;

    return false;
}

bool ConeSorterNode::segmentsIntersect(
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2,
    const Eigen::Vector2d& q1,
    const Eigen::Vector2d& q2,
    double eps) const
{
    auto cross = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b)
    {
        return a.x() * b.y() - a.y() * b.x();
    };

    auto onSegment = [&](const Eigen::Vector2d& a,
                         const Eigen::Vector2d& b,
                         const Eigen::Vector2d& p)
    {
        return (p.x() >= std::min(a.x(), b.x()) - eps &&
                p.x() <= std::max(a.x(), b.x()) + eps &&
                p.y() >= std::min(a.y(), b.y()) - eps &&
                p.y() <= std::max(a.y(), b.y()) + eps);
    };

    Eigen::Vector2d r = p2 - p1;
    Eigen::Vector2d s = q2 - q1;

    double rxs = cross(r, s);
    double qpxr = cross(q1 - p1, r);

    if (std::fabs(rxs) < eps && std::fabs(qpxr) < eps)
    {
        // colinear
        if (onSegment(p1, p2, q1) || onSegment(p1, p2, q2) ||
            onSegment(q1, q2, p1) || onSegment(q1, q2, p2))
        {
            return true;
        }
        return false;
    }

    if (std::fabs(rxs) < eps && std::fabs(qpxr) >= eps)
        return false; // 평행, 교차 없음

    double t = cross(q1 - p1, s) / rxs;
    double u = cross(q1 - p1, r) / rxs;

    if (t >= -eps && t <= 1.0 + eps &&
        u >= -eps && u <= 1.0 + eps)
    {
        return true;
    }

    return false;
}

bool ConeSorterNode::segmentIntersectsCar(
    const Eigen::Vector2d& p0,
    const Eigen::Vector2d& p1) const
{
    // 차량을 선분으로 근사 (뒤쪽으로 car_size_/2, 앞쪽으로 car_size_)
    Eigen::Vector2d dir_norm = car_dir_.normalized();
    Eigen::Vector2d car_start = car_pos_ - dir_norm * (car_size_ * 0.5);
    Eigen::Vector2d car_end   = car_pos_ + dir_norm * (car_size_);

    return segmentsIntersect(p0, p1, car_start, car_end);
}

bool ConeSorterNode::canAddNeighbor(
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<int>& current_attempt,
    int depth,
    int candidate_idx,
    Side side,
    double threshold_dir_angle_rad,
    double threshold_abs_angle_rad) const
{
    // 0) 이미 경로에 포함된 노드인지 체크
    for (int i = 0; i <= depth; ++i)
    {
        if (current_attempt[i] == candidate_idx)
            return false;
    }

    const Eigen::Vector2d& candidate = points[candidate_idx];

    // 1) 두 번째 콘이면, 차량 기준 좌/우 확인
    if (depth == 0)
    {
        if (!secondConeOnExpectedSide(candidate, side))
            return false;
    }

    // 2) 타원 안인지
    if (depth >= 1)
    {
        const Eigen::Vector2d& second_last = points[current_attempt[depth - 1]];
        const Eigen::Vector2d& last        = points[current_attempt[depth]];
        if (!isInsideLocalEllipse(second_last, last, candidate))
            return false;
    }

    // 3) 각도 제약
    if (depth >= 1)
    {
        const Eigen::Vector2d& second_last = points[current_attempt[depth - 1]];
        const Eigen::Vector2d& last        = points[current_attempt[depth]];

        Eigen::Vector2d v1 = last - second_last;
        Eigen::Vector2d v2 = candidate - last;

        if (v1.norm() < 1e-6 || v2.norm() < 1e-6)
            return false;

        double angle1 = std::atan2(v1.y(), v1.x());
        double angle2 = std::atan2(v2.y(),  v2.x());

        double diff = angleDifference(angle2, angle1);
        double len_last_to_candidate = v2.norm();

        if (std::fabs(diff) > threshold_abs_angle_rad)
            return false;

        double override_dist = len_override_factor_ * max_dist_between_cones_;

        if (side == Side::LEFT)
        {
            if (!(diff < threshold_dir_angle_rad || len_last_to_candidate < override_dist))
                return false;
        }
        else
        {
            if (!(diff > -threshold_dir_angle_rad || len_last_to_candidate < override_dist))
                return false;
        }

        // 방향 전환(sign) 급변 체크
        if (depth >= 2)
        {
            const Eigen::Vector2d& third_last = points[current_attempt[depth - 2]];
            Eigen::Vector2d v0 = second_last - third_last;

            if (v0.norm() > 1e-6)
            {
                double angle0 = std::atan2(v0.y(), v0.x());
                double diff2  = angleDifference(angle1, angle0);

                int s1 = (diff  > 0.0) - (diff  < 0.0);
                int s2 = (diff2 > 0.0) - (diff2 < 0.0);

                if (s1 != 0 && s2 != 0 && s1 != s2 &&
                    std::fabs(diff - diff2) > 1.3) // ~75deg
                {
                    return false;
                }
            }
        }
    }

    // 5) 차량과 선분 교차 여부
    if (depth >= 0)
    {
        const Eigen::Vector2d& last = points[current_attempt[depth]];
        if (segmentIntersectsCar(last, candidate))
            return false;
    }

    return true;
}

ConeSorterNode::TraceSearchResult ConeSorterNode::findAllEndConfigurations(
    const std::vector<Eigen::Vector2d>& points,
    const Eigen::MatrixXi& adjacency,
    int start_idx,
    int reachable_count,
    Side side) const
{
    TraceSearchResult result;

    const int N = static_cast<int>(points.size());
    if (N < 3) return result;

    int target_length = std::min(reachable_count, max_trace_length_);
    if (target_length < 3)
        return result;

    std::vector<int> current_attempt(target_length, -1);
    std::vector<std::pair<int, int>> stack;
    stack.reserve(target_length * 4);

    // 시작 노드 push
    stack.emplace_back(start_idx, 0);

    double th_dir  = threshold_directional_angle_deg_ * M_PI / 180.0;
    double th_abs  = threshold_absolute_angle_deg_  * M_PI / 180.0;

    while (!stack.empty())
    {
        auto [node, depth] = stack.back();
        stack.pop_back();

        // 현재 경로에 노드 기록
        current_attempt[depth] = node;
        for (int i = depth + 1; i < target_length; ++i)
            current_attempt[i] = -1;

        // 이웃 후보 찾기
        std::vector<int> neighbors;
        neighbors.reserve(N);
        for (int j = 0; j < N; ++j)
        {
            if (adjacency(node, j) != 0)
                neighbors.push_back(j);
        }

        bool has_valid_neighbor = false;
        if (depth < target_length - 1)
        {
            for (int nbr : neighbors)
            {
                if (canAddNeighbor(points, current_attempt, depth, nbr,
                                   side, th_dir, th_abs))
                {
                    has_valid_neighbor = true;
                    stack.emplace_back(nbr, depth + 1);
                }
            }
        }

        // 더 이상 확장할 수 없으면 leaf → end configuration 저장
        if (!has_valid_neighbor)
        {
            int used = 0;
            while (used < target_length && current_attempt[used] != -1)
                ++used;

            if (used >= 3)
            {
                std::vector<int> cfg(current_attempt.begin(),
                                     current_attempt.begin() + used);
                result.end_configs.push_back(std::move(cfg));
            }
        }
    }

    // 중복 제거
    std::sort(result.end_configs.begin(), result.end_configs.end());
    result.end_configs.erase(
        std::unique(result.end_configs.begin(), result.end_configs.end()),
        result.end_configs.end());

    return result;
}

// ===== Stage4: 비용 함수들 =====

// ===== angle_cost: 가운데 콘 기준 앞/뒤 벡터의 각도 =====
double ConeSorterNode::angleCostTerm(
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<int>& trace) const
{
    const int n = static_cast<int>(trace.size());
    if (n < 3) return 0.0;

    const double th = deg2rad(angle_cost_threshold_deg_);

    double sum_cost = 0.0;
    double count = 0.0;
    int n_under_th = 0;

    for (int i = 1; i < n - 1; ++i)
    {
        const Eigen::Vector2d& prev = points[trace[i - 1]];
        const Eigen::Vector2d& mid  = points[trace[i]];
        const Eigen::Vector2d& next = points[trace[i + 1]];

        Eigen::Vector2d v_next = next - mid;
        Eigen::Vector2d v_prev = prev - mid;

        if (v_next.norm() < 1e-6 || v_prev.norm() < 1e-6)
            continue;

        double ang = angleBetween(v_next, v_prev);  // 0~pi

        double c = (M_PI - ang) / M_PI; // ang 작을수록(더 꺾일수록) 비용↑
        sum_cost += c;
        count += 1.0;

        if (ang < th) n_under_th++;
    }

    if (count == 0.0) return 0.0;

    double avg = sum_cost / count;
    double factor = static_cast<double>(n_under_th + 1);

    return avg * factor;
}

// ===== residual_distance_cost: 너무 떨어진 구간 벌점 =====
double ConeSorterNode::residualDistanceCostTerm(
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<int>& trace) const
{
    const int n = static_cast<int>(trace.size());
    if (n < 2) return 0.0;

    double th = residual_dist_threshold_;
    double sum = 0.0;
    double count = 0.0;

    for (int i = 0; i < n - 1; ++i)
    {
        double d = (points[trace[i + 1]] - points[trace[i]]).norm();
        double r = std::max(0.0, d - th);
        sum += r * r;   // 초과분 제곱으로 크게 벌점
        count += 1.0;
    }

    if (count == 0.0) return 0.0;
    return sum / count;
}

// ===== num_cones_cost: 콘이 많을수록(긴 경계) 비용↓ =====
double ConeSorterNode::numberOfConesCostTerm(
    const std::vector<int>& trace) const
{
    const int n = static_cast<int>(trace.size());
    if (n == 0) return 0.0;
    return 1.0 / static_cast<double>(n);
}

// ===== initial_dir_cost: 첫 구간의 방향이 차량 진행 방향과 얼마나 어긋나는지 =====
double ConeSorterNode::initialDirectionCostTerm(
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<int>& trace) const
{
    if (trace.size() < 2) return 0.0;

    Eigen::Vector2d v = points[trace[1]] - points[trace[0]];
    if (v.norm() < 1e-6) return 0.0;

    return angleBetween(v, car_dir_);  // rad, 0에 가까울수록 좋음
}

// ===== change_dir_cost: 곡률 부호가 자주 바뀌는지(좌↔우가 빈번) =====
double ConeSorterNode::changeOfDirectionCostTerm(
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<int>& trace) const
{
    const int n = static_cast<int>(trace.size());
    if (n < 4) return 0.0; // 최소 3개 세그먼트 필요

    std::vector<double> seg_angles;
    seg_angles.reserve(n - 1);

    for (int i = 0; i < n - 1; ++i)
    {
        Eigen::Vector2d v = points[trace[i + 1]] - points[trace[i]];
        if (v.norm() < 1e-6) continue;
        seg_angles.push_back(std::atan2(v.y(), v.x()));
    }

    const int m = static_cast<int>(seg_angles.size());
    if (m < 3) return 0.0;

    std::vector<double> diff(m - 1);
    for (int i = 1; i < m; ++i)
        diff[i - 1] = angleDifference(seg_angles[i], seg_angles[i - 1]);

    double cost = 0.0;
    for (int i = 0; i < static_cast<int>(diff.size()) - 1; ++i)
    {
        double d0 = diff[i];
        double d1 = diff[i + 1];
        int s0 = (d0 > 0.0) - (d0 < 0.0);
        int s1 = (d1 > 0.0) - (d1 < 0.0);

        if (s0 != 0 && s1 != 0 && s0 != s1)
            cost += std::fabs(d0 - d1);
    }

    return cost;
}

// ===== cones_side_cost: 기대하는 쪽에 콘이 많이 보이면 좋고, 반대편이 많으면 나쁨 =====
double ConeSorterNode::conesOnEitherSideCostTerm(
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<int>& trace,
    Side side) const
{
    const int N_all = static_cast<int>(points.size());
    const int n = static_cast<int>(trace.size());
    if (N_all == 0 || n == 0) return 0.0;

    const double search_dist  = cones_side_search_dist_;
    const double search_angle = deg2rad(cones_side_search_angle_deg_);

    int good = 0;
    int bad  = 0;

    for (int t_i = 0; t_i < n; ++t_i)
    {
        int idx = trace[t_i];
        const Eigen::Vector2d& p = points[idx];

        // 로컬 진행 방향(탠젠트) 계산
        Eigen::Vector2d forward(1.0, 0.0);
        if (n >= 2)
        {
            if (t_i == 0)
                forward = points[trace[1]] - points[trace[0]];
            else if (t_i == n - 1)
                forward = points[trace[n - 1]] - points[trace[n - 2]];
            else
                forward = points[trace[t_i + 1]] - points[trace[t_i - 1]];
        }
        if (forward.norm() < 1e-6) continue;
        forward.normalize();

        for (int j = 0; j < N_all; ++j)
        {
            if (j == idx) continue;

            Eigen::Vector2d v = points[j] - p;
            double d = v.norm();
            if (d < 1e-6 || d > search_dist) continue;

            double ang = angleBetween(forward, v);
            if (ang > 0.5 * search_angle) continue;

            // cross 로 좌/우 판별
            double cross = forward.x() * v.y() - forward.y() * v.x();
            bool is_left  = (cross > 0.0);
            bool is_right = (cross < 0.0);

            if (side == Side::LEFT)
            {
                if (is_left)  good++;
                if (is_right) bad++;
            }
            else
            {
                if (is_right) good++;
                if (is_left)  bad++;
            }
        }
    }

    double diff = static_cast<double>(good - bad);
    if (diff <= 0.0) return 1e6;  // 기대 방향보다 반대편이 많으면 큰 벌점
    return 1.0 / diff;           // diff 클수록 비용↓
}

// ===== wrong_dir_cost: "원하지 않는 방향"으로 크게 꺾이는 부분 벌점 =====
double ConeSorterNode::wrongDirectionCostTerm(
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<int>& trace,
    Side side) const
{
    const int n = static_cast<int>(trace.size());
    if (n < 3) return 0.0;

    std::vector<double> seg_angles;
    seg_angles.reserve(n - 1);

    for (int i = 0; i < n - 1; ++i)
    {
        Eigen::Vector2d v = points[trace[i + 1]] - points[trace[i]];
        if (v.norm() < 1e-6) continue;
        seg_angles.push_back(std::atan2(v.y(), v.x()));
    }

    const int m = static_cast<int>(seg_angles.size());
    if (m < 2) return 0.0;

    std::vector<double> diff(m - 1);
    for (int i = 1; i < m; ++i)
        diff[i - 1] = angleDifference(seg_angles[i], seg_angles[i - 1]);

    const int unwanted_sign = (side == Side::LEFT) ? +1 : -1;
    const double th = deg2rad(wrong_dir_threshold_deg_);

    double cost = 0.0;
    for (double d : diff)
    {
        int s = (d > 0.0) - (d < 0.0);
        if (s == unwanted_sign && std::fabs(d) > th)
            cost += std::fabs(d);
    }

    return cost;
}

ConeSorterNode::TraceCostTerms ConeSorterNode::computeTraceCostTerms(
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<int>& trace,
    Side side) const
{
    TraceCostTerms c;
    c.angle_cost         = angleCostTerm(points, trace);
    c.residual_dist_cost = residualDistanceCostTerm(points, trace);
    c.num_cones_cost     = numberOfConesCostTerm(trace);
    c.initial_dir_cost   = initialDirectionCostTerm(points, trace);
    c.change_dir_cost    = changeOfDirectionCostTerm(points, trace);
    c.cones_side_cost    = conesOnEitherSideCostTerm(points, trace, side);
    c.wrong_dir_cost     = wrongDirectionCostTerm(points, trace, side);
    return c;
}

double ConeSorterNode::computeWeightedTraceCost(const TraceCostTerms& t) const
{
    double sum_w =
        w_angle_cost_ +
        w_residual_dist_cost_ +
        w_num_cones_cost_ +
        w_initial_dir_cost_ +
        w_change_dir_cost_ +
        w_cones_side_cost_ +
        w_wrong_dir_cost_;

    if (sum_w <= 1e-9) sum_w = 1.0;

    double total =
        w_angle_cost_          * t.angle_cost +
        w_residual_dist_cost_  * t.residual_dist_cost +
        w_num_cones_cost_      * t.num_cones_cost +
        w_initial_dir_cost_    * t.initial_dir_cost +
        w_change_dir_cost_     * t.change_dir_cost +
        w_cones_side_cost_     * t.cones_side_cost +
        w_wrong_dir_cost_      * t.wrong_dir_cost;

    return total / sum_w;
}

// ===== 경계 퍼블리시 (nav_msgs/Path) =====
void ConeSorterNode::publishBoundary(
    const std::vector<Eigen::Vector2d>& pts,
    const ros::Publisher& pub,
    const std::string& frame_id) const
{
    if (!pub || pts.empty())
        return;

    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = frame_id;

    path_msg.poses.reserve(pts.size());
    for (const auto& p : pts)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = p.x();
        pose.pose.position.y = p.y();
        pose.pose.position.z = 0.0;

        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;

        path_msg.poses.push_back(pose);
    }

    pub.publish(path_msg);
}

// ========== 전체 파이프라인 호출 ==========

void ConeSorterNode::processOnce()
{
    const std::size_t N = cones_.size();
    if (N < 3)
    {
        ROS_WARN_THROTTLE(1.0, "[ConeSorterNode] Not enough cones (%zu) for sorting.", N);
        return;
    }

    // --- Stage1: 러버콘 스타일 시작 콘 선정 (필요 시 기존 로직으로 폴백) ---
    int start_idx_left  = -1;
    int start_idx_right = -1;

    if (!selectFirstConeIndex(Side::LEFT, start_idx_left))
    {
        ROS_WARN_THROTTLE(1.0, "[ConeSorterNode] No valid LEFT starting cone (Rubber logic + fallback).");
    }
    if (!selectFirstConeIndex(Side::RIGHT, start_idx_right))
    {
        ROS_WARN_THROTTLE(1.0, "[ConeSorterNode] No valid RIGHT starting cone (Rubber logic + fallback).");
    }

    if (start_idx_left < 0 && start_idx_right < 0)
    {
        ROS_WARN_THROTTLE(1.0, "[ConeSorterNode] No valid starting cones for LEFT/RIGHT.");
        return;
    }

    ROS_INFO_THROTTLE(0.5,
        "[Stage1] left_start_idx=%d, right_start_idx=%d",
        start_idx_left, start_idx_right);

    // --- Stage2: 인접 행렬 + reachable (한 쪽 인덱스로 adjacency 구성 후,
    //           각 측에서 BFS 별도 수행) ---
    Eigen::MatrixXi adjacency;
    std::vector<int> reachable_dummy;

    int any_start = (start_idx_left >= 0) ? start_idx_left : start_idx_right;
    createAdjacencyMatrix(
        cones_,
        max_n_neighbors_,
        any_start,
        max_dist_between_cones_,
        adjacency,
        reachable_dummy);

    if (adjacency.rows() == 0)
    {
        ROS_WARN_THROTTLE(1.0, "[ConeSorterNode] Empty adjacency matrix.");
        return;
    }

    std::vector<int> reachable_left;
    std::vector<int> reachable_right;

    if (start_idx_left >= 0)
        reachable_left = breadthFirstOrder(adjacency, start_idx_left);
    if (start_idx_right >= 0)
        reachable_right = breadthFirstOrder(adjacency, start_idx_right);

    ROS_INFO_THROTTLE(0.5,
        "[Stage2] adjacency_size=%dx%d, reachable_left=%zu, reachable_right=%zu",
        adjacency.rows(), adjacency.cols(),
        reachable_left.size(), reachable_right.size());

    // --- Stage3: end configurations 탐색 ---
    TraceSearchResult traces_left;
    TraceSearchResult traces_right;

    if (start_idx_left >= 0 && !reachable_left.empty())
    {
        traces_left = findAllEndConfigurations(
            cones_, adjacency, start_idx_left,
            static_cast<int>(reachable_left.size()),
            Side::LEFT);
    }

    if (start_idx_right >= 0 && !reachable_right.empty())
    {
        traces_right = findAllEndConfigurations(
            cones_, adjacency, start_idx_right,
            static_cast<int>(reachable_right.size()),
            Side::RIGHT);
    }

    ROS_INFO_THROTTLE(0.5,
        "[Stage3] left_candidates=%zu, right_candidates=%zu",
        traces_left.end_configs.size(), traces_right.end_configs.size());

    // --- Stage4: 각 후보에 대해 score 계산 후 최종 왼/오 경계 선택 ---
    std::vector<Eigen::Vector2d> left_boundary_pts;
    std::vector<Eigen::Vector2d> right_boundary_pts;

    if (!traces_left.end_configs.empty())
    {
        double best_score = std::numeric_limits<double>::infinity();
        std::vector<int> best_cfg;

        for (const auto& cfg : traces_left.end_configs)
        {
            TraceCostTerms terms = computeTraceCostTerms(cones_, cfg, Side::LEFT);
            double cost          = computeWeightedTraceCost(terms);
            if (cost  < best_score)
            {
                best_score = cost;
                best_cfg   = cfg;
            }
        }

        left_boundary_pts.reserve(best_cfg.size());
        for (int idx : best_cfg)
            left_boundary_pts.push_back(cones_.at(idx));
    }

    if (!traces_right.end_configs.empty())
    {
        double best_score = std::numeric_limits<double>::infinity();
        std::vector<int> best_cfg;

        for (const auto& cfg : traces_right.end_configs)
        {
            TraceCostTerms terms = computeTraceCostTerms(cones_, cfg, Side::RIGHT);
            double cost          = computeWeightedTraceCost(terms);
            if (cost < best_score)
            {
                best_score = cost;
                best_cfg   = cfg;
            }
        }

        right_boundary_pts.reserve(best_cfg.size());
        for (int idx : best_cfg)
            right_boundary_pts.push_back(cones_.at(idx));
    }

    if (!left_boundary_pts.empty())
        publishBoundary(left_boundary_pts, left_boundary_pub_, "base_link");
    if (!right_boundary_pts.empty())
        publishBoundary(right_boundary_pts, right_boundary_pub_, "base_link");

    ROS_INFO_THROTTLE(1.0,
        "[ConeSorterNode] chosen left size=%zu, right size=%zu",
        left_boundary_pts.size(), right_boundary_pts.size());
}

} // namespace cone_sorter

// ===== main =====
int main(int argc, char** argv)
{
    ros::init(argc, argv, "cone_sorter_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    cone_sorter::ConeSorterNode node(nh, pnh);

    ros::spin();
    return 0;
}
