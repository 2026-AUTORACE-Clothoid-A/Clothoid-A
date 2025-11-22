#include "cone_matching_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ===== 유틸 함수들 =====

// 2D 벡터의 방향각 (파이썬 angle_from_2d_vector)
static double angleFromVector(const Eigen::Vector2d& v)
{
    return std::atan2(v.y(), v.x());
}

// 2D 벡터 회전 (파이썬 rotate)
static Eigen::Vector2d rotateVec(const Eigen::Vector2d& v, double angle)
{
    double c = std::cos(angle);
    double s = std::sin(angle);
    return Eigen::Vector2d(
        c * v.x() - s * v.y(),
        s * v.x() + c * v.y());
}

// 두 벡터 사이 각 (0 ~ pi) (파이썬 vec_angle_between)
static double vecAngleBetween(const Eigen::Vector2d& a, const Eigen::Vector2d& b)
{
    double na = a.norm();
    double nb = b.norm();
    if (na < 1e-9 || nb < 1e-9)
        return 0.0;

    double c = a.dot(b) / (na * nb);
    c = std::max(-1.0, std::min(1.0, c));
    return std::acos(c);
}

// ===== 생성자 =====
ConeMatchingNode::ConeMatchingNode(ros::NodeHandle& nh)
    : nh_(nh)
    , left_received_(false)
    , right_received_(false)
{
    // 파라미터 로드 (필요하면 launch에서 override)
    nh_.param("min_track_width",              min_track_width_,              0.7);   // m
    nh_.param("max_search_range",             max_search_range_,             3.0);   // m
    nh_.param("max_search_angle_deg",         max_search_angle_deg_,         120.0); // deg
    nh_.param("matches_should_be_monotonic",  matches_should_be_monotonic_,  true);

    // 차량은 base_link 원점 기준 (x 전방, y 좌측)
    car_pos_ = Eigen::Vector2d(0.0, 0.0);
    car_dir_ = Eigen::Vector2d(1.0, 0.0);

    // 초기 state 파라미터 세팅
    state_.min_track_width = min_track_width_;
    state_.max_search_range = max_search_range_;
    state_.max_search_angle_rad = max_search_angle_deg_ * M_PI / 180.0;
    state_.matches_should_be_monotonic = matches_should_be_monotonic_;

    // 구독: 콘 정렬 노드에서 퍼블리시하는 경계 Path
    left_boundary_sub_ = nh_.subscribe(
        "/cone_sorter_node/left_boundary_path",
        1,
        &ConeMatchingNode::leftBoundaryCallback,
        this);

    right_boundary_sub_ = nh_.subscribe(
        "/cone_sorter_node/right_boundary_path",
        1,
        &ConeMatchingNode::rightBoundaryCallback,
        this);

    // 퍼블리셔: 가상 콘 포함 최종 경계 + 매칭 라인
    left_virtual_path_pub_  = nh_.advertise<nav_msgs::Path>("left_boundary_with_virtual",  1, true);
    right_virtual_path_pub_ = nh_.advertise<nav_msgs::Path>("right_boundary_with_virtual", 1, true);
    match_lines_pub_        = nh_.advertise<visualization_msgs::Marker>("matched_cones_marker", 1, true);

    ROS_INFO("[ConeMatchingNode] Initialized. Waiting for left/right boundary paths...");
}

// ===== 콜백 =====
void ConeMatchingNode::leftBoundaryCallback(const nav_msgs::Path::ConstPtr& msg)
{
    last_left_path_ = *msg;
    left_received_ = true;
    processOnce();
}

void ConeMatchingNode::rightBoundaryCallback(const nav_msgs::Path::ConstPtr& msg)
{
    last_right_path_ = *msg;
    right_received_ = true;
    processOnce();
}

// ===== Stage 1: set_new_input (파이썬 ConeMatching.set_new_input) =====
void ConeMatchingNode::setNewInputFromBoundaries()
{
    // 왼/오 경계 Path -> Eigen 좌표 배열로 변환
    input_.sorted_left.clear();
    input_.sorted_right.clear();

    input_.sorted_left.reserve(last_left_path_.poses.size());
    for (const auto& pose_stamped : last_left_path_.poses)
    {
        const auto& p = pose_stamped.pose.position;
        input_.sorted_left.emplace_back(p.x, p.y);
    }

    input_.sorted_right.reserve(last_right_path_.poses.size());
    for (const auto& pose_stamped : last_right_path_.poses)
    {
        const auto& p = pose_stamped.pose.position;
        input_.sorted_right.emplace_back(p.x, p.y);
    }

    // slam_position, slam_direction 역할 (지금은 base_link 원점)
    input_.slam_position  = car_pos_;
    input_.slam_direction = car_dir_;
}

// ===== Stage 1: transition_input_to_state (파이썬 ConeMatching.transition_input_to_state) =====
void ConeMatchingNode::transitionInputToState()
{
    // 파라미터 최신값 반영
    state_.min_track_width = min_track_width_;
    state_.max_search_range = max_search_range_;
    state_.max_search_angle_rad = max_search_angle_deg_ * M_PI / 180.0;
    state_.matches_should_be_monotonic = matches_should_be_monotonic_;

    // 입력을 상태로 복사
    state_.position_global  = input_.slam_position;
    state_.direction_global = input_.slam_direction;

    state_.sorted_left  = input_.sorted_left;
    state_.sorted_right = input_.sorted_right;
}

// ===== Stage 1 디버그 출력 =====
void ConeMatchingNode::printStage1DebugInfo() const
{
    std::ostringstream oss;
    oss << "[ConeMatchingNode][Stage1] "
        << "left_cones="  << state_.sorted_left.size()
        << ", right_cones=" << state_.sorted_right.size()
        << ", min_track_width=" << state_.min_track_width
        << ", max_search_range=" << state_.max_search_range
        << ", max_search_angle_deg=" << (state_.max_search_angle_rad * 180.0 / M_PI)
        << ", monotonic=" << (state_.matches_should_be_monotonic ? "true" : "false");

    ROS_INFO_STREAM_THROTTLE(0.5, oss.str());
}

// ===== Stage 2: calculate_match_search_direction (파이썬과 1:1) =====
std::vector<Eigen::Vector2d> ConeMatchingNode::calculateMatchSearchDirection(
    const std::vector<Eigen::Vector2d>& cones,
    ConeSide side) const
{
    std::vector<Eigen::Vector2d> out;
    const std::size_t N = cones.size();
    if (N <= 1)
        return out; // 파이썬은 assert, 여기선 안전하게 빈 벡터 리턴

    out.resize(N, Eigen::Vector2d::Zero());

    auto computeForIdxs = [&](int idx0, int idx1) -> Eigen::Vector2d
    {
        Eigen::Vector2d track_dir = cones[idx1] - cones[idx0];
        if (track_dir.norm() < 1e-9)
            track_dir = Eigen::Vector2d(1.0, 0.0); // fallback

        double rot_angle = (side == ConeSide::RIGHT) ? (M_PI / 2.0) : (-M_PI / 2.0);
        Eigen::Vector2d search_dir = rotateVec(track_dir, rot_angle);
        double n = search_dir.norm();
        if (n < 1e-9) n = 1.0;
        return search_dir / n;
    };

    out[0]     = computeForIdxs(0, static_cast<int>(std::min<std::size_t>(1, N - 1)));
    out[N - 1] = computeForIdxs(static_cast<int>(N) - 2, static_cast<int>(N) - 1);

    for (std::size_t i = 1; i + 1 < N; ++i)
    {
        out[i] = computeForIdxs(static_cast<int>(i) - 1, static_cast<int>(i) + 1);
    }

    return out;
}

// ===== Stage 2: find_boolean_mask_of_all_potential_matches (파이썬과 1:1) =====
std::vector<std::vector<bool>> ConeMatchingNode::findBooleanMaskOfAllPotentialMatches(
    const std::vector<Eigen::Vector2d>& start_points,
    const std::vector<Eigen::Vector2d>& directions,
    const std::vector<Eigen::Vector2d>& other_side_cones,
    const std::vector<Eigen::Vector2d>& other_side_directions,
    double major_radius,
    double minor_radius,
    double max_search_angle_rad) const
{
    const std::size_t M = start_points.size();
    const std::size_t N = other_side_cones.size();

    std::vector<std::vector<bool>> mask(M, std::vector<bool>(N, false));
    if (M == 0 || N == 0)
        return mask;

    const double a2 = major_radius * major_radius;
    const double b2 = minor_radius * minor_radius;

    // (M,) 각 방향 벡터의 angle
    std::vector<double> angles(M, 0.0);
    for (std::size_t i = 0; i < M; ++i)
        angles[i] = angleFromVector(directions[i]);

    // (M, N) 방향 차이: vec_angle_between(directions[i], other_side_directions[j])
    std::vector<std::vector<double>> angle_diff(M, std::vector<double>(N, 0.0));
    for (std::size_t i = 0; i < M; ++i)
    {
        for (std::size_t j = 0; j < N; ++j)
        {
            angle_diff[i][j] = vecAngleBetween(directions[i], other_side_directions[j]);
        }
    }

    // (M, N, 2): start_points -> other_side_cones 벡터
    std::vector<std::vector<Eigen::Vector2d>> vec_start_to_other(
        M, std::vector<Eigen::Vector2d>(N, Eigen::Vector2d::Zero()));

    for (std::size_t i = 0; i < M; ++i)
    {
        for (std::size_t j = 0; j < N; ++j)
        {
            vec_start_to_other[i][j] = other_side_cones[j] - start_points[i];
        }
    }

    // 첫 번째 루프: 타원 + 각도 조건
    for (std::size_t i = 0; i < M; ++i)
    {
        double angle = angles[i];
        double cos_ang = std::cos(-angle);
        double sin_ang = std::sin(-angle);

        for (std::size_t j = 0; j < N; ++j)
        {
            Eigen::Vector2d v = vec_start_to_other[i][j];

            // 회전 (rotated_start_point_to_other_side)
            Eigen::Vector2d r(
                cos_ang * v.x() - sin_ang * v.y(),
                sin_ang * v.x() + cos_ang * v.y());

            // 타원 내부 체크 s < 1
            double s_val = (r.x() * r.x()) / a2 + (r.y() * r.y()) / b2;
            bool inside_ellipse = (s_val < 1.0);

            bool keep = inside_ellipse;
            if (keep)
            {
                // angle_of_rotated_start_point_to_other_side / 2 > max_search_angle 이면 제거
                double ang_rot = angleFromVector(r);
                bool angle_over_threshold = std::fabs(ang_rot / 2.0) > max_search_angle_rad;

                // angle_diff_start_direction_other_direction < pi/2 이면 제거
                bool direction_diff_over_threshold = angle_diff[i][j] < (M_PI / 2.0);

                if (angle_over_threshold || direction_diff_over_threshold)
                    keep = false;
            }

            mask[i][j] = keep;
        }
    }

    // 두 번째 루프: 각 start_point에 대해 "가까운 후보 최대 2개"만 남기기
    for (std::size_t i = 0; i < M; ++i)
    {
        std::vector<double> dist(N, 0.0);
        for (std::size_t j = 0; j < N; ++j)
        {
            if (!mask[i][j])
            {
                dist[j] = std::numeric_limits<double>::infinity();
            }
            else
            {
                dist[j] = vec_start_to_other[i][j].norm();
            }
        }

        std::vector<int> order(N);
        for (std::size_t j = 0; j < N; ++j) order[j] = static_cast<int>(j);

        std::sort(order.begin(), order.end(),
                  [&](int a, int b)
                  {
                      return dist[a] < dist[b];
                  });

        std::vector<int> chosen;
        for (std::size_t k = 0; k < order.size() && chosen.size() < 2; ++k)
        {
            int idx = order[k];
            if (std::isfinite(dist[idx]))
                chosen.push_back(idx);
        }

        std::vector<bool> new_row(N, false);
        for (int idx : chosen)
            new_row[idx] = true;

        mask[i] = new_row;
    }

    return mask;
}

// ===== Stage 2: select_best_match_candidate (파이썬과 1:1) =====
std::vector<int> ConeMatchingNode::selectBestMatchCandidate(
    const std::vector<Eigen::Vector2d>& matchable_cones,
    const std::vector<Eigen::Vector2d>& /*match_directions*/,
    const std::vector<std::vector<bool>>& match_boolean_mask,
    const std::vector<Eigen::Vector2d>& other_side_cones,
    bool matches_should_be_monotonic) const
{
    const std::size_t M = matchable_cones.size();
    std::vector<int> matches(M, -1);

    if (other_side_cones.empty() || M == 0)
        return matches;

    const std::size_t N = other_side_cones.size();

    // 거리 행렬 (제곱 거리)
    std::vector<std::vector<double>> dist_sq(M, std::vector<double>(N, 0.0));
    for (std::size_t i = 0; i < M; ++i)
    {
        for (std::size_t j = 0; j < N; ++j)
        {
            Eigen::Vector2d d = matchable_cones[i] - other_side_cones[j];
            dist_sq[i][j] = d.squaredNorm();
        }
    }

    // 각 행에서 argmin
    for (std::size_t i = 0; i < M; ++i)
    {
        double best_val = std::numeric_limits<double>::infinity();
        int best_idx = -1;
        for (std::size_t j = 0; j < N; ++j)
        {
            if (dist_sq[i][j] < best_val)
            {
                best_val = dist_sq[i][j];
                best_idx = static_cast<int>(j);
            }
        }
        matches[i] = best_idx;
    }

    // matches_should_be_monotonic 적용 (파이썬 동일)
    if (matches_should_be_monotonic && !matches.empty())
    {
        int current_max_value = matches[0];
        for (std::size_t i = 1; i < matches.size(); ++i)
        {
            if (matches[i] < 0)
                continue;

            current_max_value = std::max(current_max_value, matches[i]);
            if (matches[i] != current_max_value)
            {
                matches[i] = -1;
            }
            else
            {
                matches[i] = current_max_value;
            }
        }
    }

    // match_boolean_mask.any(axis=1) == False 인 행은 -1로 설정
    for (std::size_t i = 0; i < M; ++i)
    {
        bool any_true = false;
        for (bool b : match_boolean_mask[i])
        {
            if (b)
            {
                any_true = true;
                break;
            }
        }
        if (!any_true)
        {
            matches[i] = -1;
        }
    }

    return matches;
}

// ===== Stage 2: calculate_matches_for_side (파이썬과 1:1) =====
void ConeMatchingNode::calculateMatchesForSide(
    const std::vector<Eigen::Vector2d>& cones,
    ConeSide cone_side,
    const std::vector<Eigen::Vector2d>& other_side_cones,
    double major_radius,
    double minor_radius,
    double max_search_angle_rad,
    bool matches_should_be_monotonic,
    std::vector<int>& out_matches,
    std::vector<Eigen::Vector2d>& out_search_dirs) const
{
    out_matches.clear();
    out_search_dirs.clear();

    const std::size_t M = cones.size();
    out_matches.assign(M, -1);

    if (M == 0)
        return;

    if (M > 1)
    {
        // search_directions (자기 쪽)
        std::vector<Eigen::Vector2d> search_dirs = calculateMatchSearchDirection(cones, cone_side);

        // 상대쪽 search_directions
        std::vector<Eigen::Vector2d> other_search_dirs;
        if (other_side_cones.size() > 1)
        {
            ConeSide other_side = (cone_side == ConeSide::LEFT) ? ConeSide::RIGHT : ConeSide::LEFT;
            other_search_dirs = calculateMatchSearchDirection(other_side_cones, other_side);
        }

        // 후보 마스크
        auto mask = findBooleanMaskOfAllPotentialMatches(
            cones,               // start_points
            search_dirs,         // directions
            other_side_cones,
            other_search_dirs,
            major_radius,
            minor_radius,
            max_search_angle_rad);

        // 최종 매칭 인덱스
        out_matches = selectBestMatchCandidate(
            cones,
            search_dirs,
            mask,
            other_side_cones,
            matches_should_be_monotonic);

        out_search_dirs = std::move(search_dirs);
    }
    else
    {
        // 콘이 1개인 경우: 파이썬과 같이 matches = -1, search_dirs 빈 벡터
        out_matches.assign(M, -1);
        out_search_dirs.clear();
    }
}

// ===== Stage 2 전체 실행 (LEFT/RIGHT 양쪽) =====
void ConeMatchingNode::runStage2Matching()
{
    matches_left_to_right_.clear();
    matches_right_to_left_.clear();
    search_dirs_left_.clear();
    search_dirs_right_.clear();

    double major_radius = state_.max_search_range * 1.5;
    double minor_radius = state_.min_track_width;
    double max_angle    = state_.max_search_angle_rad;

    // LEFT 기준 → RIGHT 매칭
    calculateMatchesForSide(
        state_.sorted_left,
        ConeSide::LEFT,
        state_.sorted_right,
        major_radius,
        minor_radius,
        max_angle,
        state_.matches_should_be_monotonic,
        matches_left_to_right_,
        search_dirs_left_);

    // RIGHT 기준 → LEFT 매칭
    calculateMatchesForSide(
        state_.sorted_right,
        ConeSide::RIGHT,
        state_.sorted_left,
        major_radius,
        minor_radius,
        max_angle,
        state_.matches_should_be_monotonic,
        matches_right_to_left_,
        search_dirs_right_);

    std::size_t cnt_left = 0;
    for (int idx : matches_left_to_right_)
        if (idx >= 0) ++cnt_left;

    std::size_t cnt_right = 0;
    for (int idx : matches_right_to_left_)
        if (idx >= 0) ++cnt_right;

    ROS_INFO_STREAM_THROTTLE(
        0.5,
        "[ConeMatchingNode][Stage2] left->right matched=" << cnt_left
        << " / " << matches_left_to_right_.size()
        << ", right->left matched=" << cnt_right
        << " / " << matches_right_to_left_.size());
}

// ===== Stage 3: calculate_positions_of_virtual_cones (파이썬과 1:1) =====
std::vector<Eigen::Vector2d> ConeMatchingNode::calculatePositionsOfVirtualCones(
    const std::vector<Eigen::Vector2d>& cones,
    const std::vector<int>& indices_of_unmatched_cones,
    const std::vector<Eigen::Vector2d>& search_directions,
    double min_track_width) const
{
    std::vector<Eigen::Vector2d> virtual_cones;
    for (int idx : indices_of_unmatched_cones)
    {
        if (idx < 0) continue;
        std::size_t i = static_cast<std::size_t>(idx);
        if (i >= cones.size() || i >= search_directions.size()) continue;

        Eigen::Vector2d v = cones[i] + search_directions[i] * min_track_width;
        virtual_cones.push_back(v);
    }
    return virtual_cones;
}

// ===== Stage 3: trace_angles_between (파이썬과 동일 개념) =====
std::vector<double> ConeMatchingNode::traceAnglesBetween(
    const std::vector<Eigen::Vector2d>& cones) const
{
    std::vector<double> angles;
    const std::size_t N = cones.size();
    if (N < 3) return angles;

    angles.resize(N - 2, 0.0);
    for (std::size_t i = 1; i + 1 < N; ++i)
    {
        Eigen::Vector2d v_prev = cones[i]   - cones[i - 1];
        Eigen::Vector2d v_next = cones[i+1] - cones[i];

        if (v_prev.norm() < 1e-9 || v_next.norm() < 1e-9)
        {
            angles[i - 1] = 0.0;
        }
        else
        {
            angles[i - 1] = vecAngleBetween(v_prev, v_next);
        }
    }
    return angles;
}

// ===== Stage 3: calculate_insert_index_for_one_cone (파이썬과 1:1) =====
int ConeMatchingNode::calculateInsertIndexForOneCone(
    const Eigen::Vector2d& car_position,
    const std::vector<Eigen::Vector2d>& final_cones,
    const Eigen::Vector2d& virtual_cone) const
{
    if (final_cones.empty())
        return 0;

    double dist_virtual = (virtual_cone - car_position).norm();
    double dist_existing = (final_cones[0] - car_position).norm();

    if (dist_virtual < dist_existing)
        return 0;
    else
        return 1;
}

// ===== Stage 3: calculate_insert_index_of_new_cone (파이썬과 1:1) =====
int ConeMatchingNode::calculateInsertIndexOfNewCone(
    int closest_index,
    int second_closest_index,
    bool cone_is_between_closest_two) const
{
    if (cone_is_between_closest_two)
    {
        return std::min(closest_index, second_closest_index) + 1;
    }
    else
    {
        if (closest_index < second_closest_index)
            return closest_index;
        else if (closest_index > second_closest_index)
            return closest_index + 1;
        else
            throw std::runtime_error("calculateInsertIndexOfNewCone: unreachable code");
    }
}

// ===== Stage 3: insert_virtual_cones_to_existing (파이썬과 1:1) =====
std::vector<Eigen::Vector2d> ConeMatchingNode::insertVirtualConesToExisting(
    const std::vector<Eigen::Vector2d>& other_side_cones,
    const std::vector<Eigen::Vector2d>& other_side_virtual_cones,
    const Eigen::Vector2d& car_position) const
{
    std::vector<Eigen::Vector2d> existing_cones;
    std::vector<Eigen::Vector2d> cones_to_insert;

    if (other_side_cones.size() > other_side_virtual_cones.size())
    {
        existing_cones = other_side_cones;
        cones_to_insert = other_side_virtual_cones;
    }
    else
    {
        existing_cones = other_side_virtual_cones;
        cones_to_insert = other_side_cones;
    }

    if (cones_to_insert.empty())
        return existing_cones;

    // 삽입 순서: 각 virtual cone이 existing과의 최소 거리 기준으로 정렬
    const std::size_t M = cones_to_insert.size();
    const std::size_t N = existing_cones.size();

    std::vector<double> min_distsq(M, 0.0);
    for (std::size_t i = 0; i < M; ++i)
    {
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t j = 0; j < N; ++j)
        {
            Eigen::Vector2d d = cones_to_insert[i] - existing_cones[j];
            double dsq = d.squaredNorm();
            if (dsq < best) best = dsq;
        }
        min_distsq[i] = best;
    }

    std::vector<int> order_to_insert(M);
    for (std::size_t i = 0; i < M; ++i) order_to_insert[i] = static_cast<int>(i);
    std::sort(order_to_insert.begin(), order_to_insert.end(),
              [&](int a, int b)
              {
                  return min_distsq[a] < min_distsq[b];
              });

    for (int idx_ins : order_to_insert)
    {
        Eigen::Vector2d cone_to_insert = cones_to_insert[idx_ins];

        std::size_t curN = existing_cones.size();
        if (curN == 0)
        {
            existing_cones.push_back(cone_to_insert);
            continue;
        }

        // existing과의 거리
        std::vector<double> dist(curN, 0.0);
        for (std::size_t j = 0; j < curN; ++j)
        {
            dist[j] = (existing_cones[j] - cone_to_insert).norm();
        }

        std::vector<int> idx_sorted(curN);
        for (std::size_t j = 0; j < curN; ++j) idx_sorted[j] = static_cast<int>(j);
        std::sort(idx_sorted.begin(), idx_sorted.end(),
                  [&](int a, int b)
                  {
                      return dist[a] < dist[b];
                  });

        int index_to_insert = -1;
        if (idx_sorted.size() == 1)
        {
            index_to_insert = calculateInsertIndexForOneCone(car_position, existing_cones, cone_to_insert);
        }
        else
        {
            int closest_index = idx_sorted[0];
            int second_index  = idx_sorted[1];

            if (std::abs(closest_index - second_index) != 1)
            {
                // 인접한 인덱스가 아니면 스킵
                continue;
            }

            Eigen::Vector2d virtual_to_closest = existing_cones[closest_index] - cone_to_insert;
            Eigen::Vector2d virtual_to_second  = existing_cones[second_index]  - cone_to_insert;

            double angle_between_virtuals =
                vecAngleBetween(virtual_to_closest, virtual_to_second);

            bool cone_is_between_closest_two = (angle_between_virtuals > (M_PI / 2.0));

            index_to_insert = calculateInsertIndexOfNewCone(
                closest_index,
                second_index,
                cone_is_between_closest_two);
        }

        if (index_to_insert < 0)
            continue;

        if (index_to_insert > static_cast<int>(existing_cones.size()))
            index_to_insert = static_cast<int>(existing_cones.size());

        existing_cones.insert(existing_cones.begin() + index_to_insert, cone_to_insert);
    }

    // trace_angles_between으로 너무 예각(<85deg) 구간 제거
    std::vector<double> angles = traceAnglesBetween(existing_cones); // size = N-2
    std::size_t N2 = existing_cones.size();
    if (!angles.empty() && N2 == angles.size() + 2)
    {
        std::vector<bool> mask_low(N2, false);
        const double thr = 85.0 * M_PI / 180.0;

        for (std::size_t k = 0; k < angles.size(); ++k)
        {
            if (angles[k] < thr)
                mask_low[k + 1] = true; // 내부 포인트에 대응
        }

        bool any_low = false;
        for (bool b : mask_low) if (b) { any_low = true; break; }

        if (any_low)
        {
            std::vector<Eigen::Vector2d> filtered;
            filtered.reserve(N2);
            for (std::size_t i = 0; i < N2; ++i)
            {
                if (!mask_low[i])
                    filtered.push_back(existing_cones[i]);
            }
            existing_cones.swap(filtered);
        }
    }

    return existing_cones;
}

// ===== Stage 3: combine_and_sort_virtual_with_real (파이썬과 1:1) =====
void ConeMatchingNode::combineAndSortVirtualWithReal(
    const std::vector<Eigen::Vector2d>& other_side_cones,
    const std::vector<Eigen::Vector2d>& other_side_virtual_cones,
    const Eigen::Vector2d& car_pos,
    const Eigen::Vector2d& /*car_dir*/,
    std::vector<Eigen::Vector2d>& sorted_combined_cones,
    std::vector<bool>& mask_is_virtual) const
{
    if (other_side_cones.empty())
    {
        sorted_combined_cones = other_side_virtual_cones;
        mask_is_virtual.assign(other_side_virtual_cones.size(), true);
        return;
    }

    if (other_side_virtual_cones.empty())
    {
        sorted_combined_cones = other_side_cones;
        mask_is_virtual.assign(other_side_cones.size(), false);
        return;
    }

    // virtual + real을 하나의 trace로 삽입/정렬
    sorted_combined_cones = insertVirtualConesToExisting(
        other_side_cones, other_side_virtual_cones, car_pos);

    // eps보다 멀리 떨어진 점들은 "virtual"로 표기
    const double eps = 1e-2;
    const double eps2 = eps * eps;

    mask_is_virtual.assign(sorted_combined_cones.size(), false);

    for (std::size_t i = 0; i < sorted_combined_cones.size(); ++i)
    {
        const Eigen::Vector2d& p = sorted_combined_cones[i];
        double min_d2 = std::numeric_limits<double>::infinity();
        for (std::size_t j = 0; j < other_side_cones.size(); ++j)
        {
            Eigen::Vector2d d = p - other_side_cones[j];
            double dsq = d.squaredNorm();
            if (dsq < min_d2) min_d2 = dsq;
        }
        if (min_d2 > eps2)
            mask_is_virtual[i] = true;
    }
}

// ===== Stage 3: calculate_cones_for_other_side (파이썬과 1:1) =====
void ConeMatchingNode::calculateConesForOtherSide(
    const std::vector<Eigen::Vector2d>& cones,
    ConeSide cone_side,
    double major_radius,
    double minor_radius,
    double max_search_angle_rad,
    const std::vector<Eigen::Vector2d>& other_side_cones,
    double min_track_width,
    const Eigen::Vector2d& car_pos,
    const Eigen::Vector2d& car_dir,
    bool matches_should_be_monotonic,
    std::vector<Eigen::Vector2d>& combined_and_sorted_cones,
    std::vector<bool>& mask_is_virtual) const
{
    // 1) 우선 매칭 계산 (calculate_matches_for_side)
    std::vector<int> matches_for_each_selectable_cone;
    std::vector<Eigen::Vector2d> search_directions;

    calculateMatchesForSide(
        cones,
        cone_side,
        other_side_cones,
        major_radius,
        minor_radius,
        max_search_angle_rad,
        matches_should_be_monotonic,
        matches_for_each_selectable_cone,
        search_directions);

    const std::size_t M = cones.size();

    // 2) 매칭이 없는 콘 인덱스 추출
    std::vector<int> indices_no_match;
    indices_no_match.reserve(M);
    for (std::size_t i = 0; i < matches_for_each_selectable_cone.size(); ++i)
    {
        if (matches_for_each_selectable_cone[i] == -1)
            indices_no_match.push_back(static_cast<int>(i));
    }

    // 3) 가상 콘 위치 계산
    std::vector<Eigen::Vector2d> positions_of_virtual_cones =
        calculatePositionsOfVirtualCones(
            cones,
            indices_no_match,
            search_directions,
            min_track_width);

    // 4) other_side에 virtual cone 삽입 및 정렬
    combineAndSortVirtualWithReal(
        other_side_cones,
        positions_of_virtual_cones,
        car_pos,
        car_dir,
        combined_and_sorted_cones,
        mask_is_virtual);

    // 5) 만약 최종 trace의 길이가 2 미만이면, 원본으로 롤백
    if (combined_and_sorted_cones.size() < 2)
    {
        combined_and_sorted_cones = other_side_cones;
        mask_is_virtual.assign(other_side_cones.size(), false);
    }
}

// ===== Stage 4: match_both_sides_with_virtual_cones (파이썬과 1:1) =====
void ConeMatchingNode::matchBothSidesWithVirtualCones(
    const std::vector<Eigen::Vector2d>& left_cones_with_virtual,
    const std::vector<Eigen::Vector2d>& right_cones_with_virtual,
    double major_radius,
    double minor_radius,
    double max_search_angle_rad,
    bool matches_should_be_monotonic,
    std::vector<int>& left_to_right_matches,
    std::vector<int>& right_to_left_matches) const
{
    std::vector<Eigen::Vector2d> dummy_dirs;

    // 왼쪽 기준 → 오른쪽 매칭
    calculateMatchesForSide(
        left_cones_with_virtual,
        ConeSide::LEFT,
        right_cones_with_virtual,
        major_radius,
        minor_radius,
        max_search_angle_rad,
        matches_should_be_monotonic,
        left_to_right_matches,
        dummy_dirs);

    // 오른쪽 기준 → 왼쪽 매칭
    calculateMatchesForSide(
        right_cones_with_virtual,
        ConeSide::RIGHT,
        left_cones_with_virtual,
        major_radius,
        minor_radius,
        max_search_angle_rad,
        matches_should_be_monotonic,
        right_to_left_matches,
        dummy_dirs);
}

// ===== Stage 3+4: calculate_virtual_cones_for_both_sides (파이썬과 1:1) =====
void ConeMatchingNode::calculateVirtualConesForBothSides()
{
    left_cones_with_virtual_.clear();
    right_cones_with_virtual_.clear();
    left_mask_is_virtual_.clear();
    right_mask_is_virtual_.clear();
    final_left_to_right_matches_.clear();
    final_right_to_left_matches_.clear();

    // 입력 복사 (파이썬은 인자로 들어오므로 값 복사와 동일)
    std::vector<Eigen::Vector2d> left  = state_.sorted_left;
    std::vector<Eigen::Vector2d> right = state_.sorted_right;

    // 코너 수가 너무 불균형하면 한쪽 폐기 (파이썬 discard_one_side 로직)
    std::size_t lenL = left.size();
    std::size_t lenR = right.size();

    if (lenL < 2 && lenR < 2)
    {
        // 둘 다 거의 없는 경우
        return;
    }

    std::size_t min_len = std::min(lenL, lenR);
    std::size_t max_len = std::max(lenL, lenR);

    bool discard_one_side = (min_len == 0) || ((min_len > 0) && (max_len / static_cast<double>(min_len) > 2.0));
    if (discard_one_side)
    {
        if (lenL < lenR)
            left.clear();
        else
            right.clear();
    }

    double major_radius = state_.max_search_range * 1.5;
    double minor_radius = state_.min_track_width;
    double max_angle    = state_.max_search_angle_rad;

    // 1) LEFT → RIGHT : 가상 콘을 RIGHT 쪽에 생성
    if (left.size() >= 2)
    {
        calculateConesForOtherSide(
            left,
            ConeSide::LEFT,
            major_radius,
            minor_radius,
            max_angle,
            right,
            state_.min_track_width,
            state_.position_global,
            state_.direction_global,
            state_.matches_should_be_monotonic,
            right_cones_with_virtual_,
            right_mask_is_virtual_);
    }
    else
    {
        right_cones_with_virtual_ = right;
        right_mask_is_virtual_.assign(right.size(), false);
    }

    // 2) RIGHT → LEFT : 가상 콘을 LEFT 쪽에 생성
    if (right.size() >= 2)
    {
        calculateConesForOtherSide(
            right,
            ConeSide::RIGHT,
            major_radius,
            minor_radius,
            max_angle,
            left,
            state_.min_track_width,
            state_.position_global,
            state_.direction_global,
            state_.matches_should_be_monotonic,
            left_cones_with_virtual_,
            left_mask_is_virtual_);
    }
    else
    {
        left_cones_with_virtual_ = left;
        left_mask_is_virtual_.assign(left.size(), false);
    }

    // 3) 가상 콘 포함 상태에서 최종 매칭
    matchBothSidesWithVirtualCones(
        left_cones_with_virtual_,
        right_cones_with_virtual_,
        major_radius,
        minor_radius,
        max_angle,
        state_.matches_should_be_monotonic,
        final_left_to_right_matches_,
        final_right_to_left_matches_);

    ROS_INFO_STREAM_THROTTLE(
        0.5,
        "[ConeMatchingNode][Stage3+4] left_with_virtual="
        << left_cones_with_virtual_.size()
        << " (virtual mask size=" << left_mask_is_virtual_.size()
        << "), right_with_virtual=" << right_cones_with_virtual_.size()
        << " (virtual mask size=" << right_mask_is_virtual_.size()
        << "), final_matches_L2R=" << final_left_to_right_matches_.size()
        << ", final_matches_R2L=" << final_right_to_left_matches_.size());
}

// ===== 최종 결과 RViz 퍼블리시 =====
void ConeMatchingNode::publishResults() const
{
    if (left_cones_with_virtual_.empty() && right_cones_with_virtual_.empty())
        return;

    std::string frame_id = "base_link";
    if (!last_left_path_.header.frame_id.empty())
        frame_id = last_left_path_.header.frame_id;
    else if (!last_right_path_.header.frame_id.empty())
        frame_id = last_right_path_.header.frame_id;

    ros::Time stamp = ros::Time::now();

    // 1) 왼쪽 가상 콘 포함 경계 Path
    if (!left_cones_with_virtual_.empty())
    {
        nav_msgs::Path left_path_msg;
        left_path_msg.header.stamp = stamp;
        left_path_msg.header.frame_id = frame_id;

        left_path_msg.poses.reserve(left_cones_with_virtual_.size());
        for (const auto& p : left_cones_with_virtual_)
        {
            geometry_msgs::PoseStamped ps;
            ps.header = left_path_msg.header;
            ps.pose.position.x = p.x();
            ps.pose.position.y = p.y();
            ps.pose.position.z = 0.0;
            ps.pose.orientation.w = 1.0; // 단위 quaternion
            left_path_msg.poses.push_back(ps);
        }
        left_virtual_path_pub_.publish(left_path_msg);
    }

    // 2) 오른쪽 가상 콘 포함 경계 Path
    if (!right_cones_with_virtual_.empty())
    {
        nav_msgs::Path right_path_msg;
        right_path_msg.header.stamp = stamp;
        right_path_msg.header.frame_id = frame_id;

        right_path_msg.poses.reserve(right_cones_with_virtual_.size());
        for (const auto& p : right_cones_with_virtual_)
        {
            geometry_msgs::PoseStamped ps;
            ps.header = right_path_msg.header;
            ps.pose.position.x = p.x();
            ps.pose.position.y = p.y();
            ps.pose.position.z = 0.0;
            ps.pose.orientation.w = 1.0;
            right_path_msg.poses.push_back(ps);
        }
        right_virtual_path_pub_.publish(right_path_msg);
    }

    // 3) 매칭된 좌/우 콘 쌍을 LineList로 시각화
    if (!final_left_to_right_matches_.empty() && !left_cones_with_virtual_.empty() && !right_cones_with_virtual_.empty())
    {
        visualization_msgs::Marker line_list;
        line_list.header.stamp = stamp;
        line_list.header.frame_id = frame_id;
        line_list.ns = "cone_matches";
        line_list.id = 0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.scale.x = 0.02; // 선 두께 (스케일카면 0.01~0.03 정도)
        line_list.color.r = 0.0f;
        line_list.color.g = 1.0f;
        line_list.color.b = 0.0f;
        line_list.color.a = 1.0f;
        line_list.pose.orientation.w = 1.0;

        for (std::size_t i = 0; i < final_left_to_right_matches_.size(); ++i)
        {
            int j = final_left_to_right_matches_[i];
            if (j < 0) continue;
            if (i >= left_cones_with_virtual_.size()) continue;
            if (static_cast<std::size_t>(j) >= right_cones_with_virtual_.size()) continue;

            geometry_msgs::Point pL, pR;
            pL.x = left_cones_with_virtual_[i].x();
            pL.y = left_cones_with_virtual_[i].y();
            pL.z = 0.0;

            pR.x = right_cones_with_virtual_[j].x();
            pR.y = right_cones_with_virtual_[j].y();
            pR.z = 0.0;

            line_list.points.push_back(pL);
            line_list.points.push_back(pR);
        }

        match_lines_pub_.publish(line_list);
    }
}

// ===== 전체 파이프라인 (Stage1~4) =====
void ConeMatchingNode::runMatchingPipeline()
{
    // Stage 1: 입력 세팅 + 상태로 전이
    setNewInputFromBoundaries();
    transitionInputToState();
    printStage1DebugInfo();

    // Stage 2: 원본 경계 기준 매칭 (디버그용 or 참고용)
    runStage2Matching();

    // Stage 3+4: 가상 콘 생성 + 최종 매칭
    calculateVirtualConesForBothSides();

    // RViz로 퍼블리시
    publishResults();
}

// ===== 두 경계가 모두 들어왔을 때 호출되는 메인 처리 함수 =====
void ConeMatchingNode::processOnce()
{
    if (!left_received_ || !right_received_)
    {
        // 둘 중 하나라도 아직 안 들어왔으면 기다림
        return;
    }

    if (last_left_path_.poses.empty() && last_right_path_.poses.empty())
    {
        ROS_WARN_THROTTLE(1.0,
            "[ConeMatchingNode] Both left/right boundary paths are empty. Skipping.");
        return;
    }

    runMatchingPipeline();
}

// ===== main =====
int main(int argc, char** argv)
{
    ros::init(argc, argv, "cone_matching_node");
    ros::NodeHandle nh("~");

    ConeMatchingNode node(nh);

    ros::spin();
    return 0;
}
