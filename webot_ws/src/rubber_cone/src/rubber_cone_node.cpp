#include "rubber_cone_node.hpp"

// 스플라인 헤더
#include "spline.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ====== Delaunay + MST 헬퍼들 (로컬 anonymous namespace) ======
namespace {

struct DTPoint
{
    double x;
    double y;
};

struct DTTri
{
    int a, b, c;
};

// robust orient2d + ccw
inline double orient2d_filt(const DTPoint& a, const DTPoint& b, const DTPoint& c)
{
    double det = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    double absa = std::fabs(b.x - a.x) + std::fabs(b.y - a.y);
    double absb = std::fabs(c.x - a.x) + std::fabs(c.y - a.y);
    double err  = (absa * absb) * std::numeric_limits<double>::epsilon() * 4.0;
    if (std::fabs(det) > err)
        return det;

    long double adx = (long double)b.x - (long double)a.x;
    long double ady = (long double)b.y - (long double)a.y;
    long double bdx = (long double)c.x - (long double)a.x;
    long double bdy = (long double)c.y - (long double)a.y;
    long double detl = adx * bdy - ady * bdx;
    return (double)detl;
}

inline bool ccw(const DTPoint& a, const DTPoint& b, const DTPoint& c)
{
    return orient2d_filt(a, b, c) > 0.0;
}

// incircle_filt
inline double incircle_filt(const DTPoint& a, const DTPoint& b, const DTPoint& c, const DTPoint& d)
{
    double adx = a.x - d.x, ady = a.y - d.y;
    double bdx = b.x - d.x, bdy = b.y - d.y;
    double cdx = c.x - d.x, cdy = c.y - d.y;
    double ad = adx * adx + ady * ady;
    double bd = bdx * bdx + bdy * bdy;
    double cd = cdx * cdx + cdy * cdy;

    double det =
        adx * (bdy * cd - bd * cdy)
      - ady * (bdx * cd - bd * cdx)
      + ad  * (bdx * cdy - bdy * cdx);

    double mags = (std::fabs(adx) + std::fabs(ady)) *
                  (std::fabs(bdx) + std::fabs(bdy)) *
                  (std::fabs(cdx) + std::fabs(cdy));
    double err = mags * std::numeric_limits<double>::epsilon() * 16.0;
    if (std::fabs(det) > err)
        return det;

    long double AX = a.x, AY = a.y;
    long double BX = b.x, BY = b.y;
    long double CX = c.x, CY = c.y;
    long double DX = d.x, DY = d.y;

    long double adxl = AX - DX, adyl = AY - DY;
    long double bdxl = BX - DX, bdyl = BY - DY;
    long double cdxl = CX - DX, cdyl = CY - DY;

    long double adl = adxl * adxl + adyl * adyl;
    long double bdl = bdxl * bdxl + bdyl * bdyl;
    long double cdl = cdxl * cdxl + cdyl * cdyl;

    long double detl =
        adxl * (bdyl * cdl - bdl * cdyl)
      - adyl * (bdxl * cdl - bdl * cdxl)
      + adl  * (bdxl * cdyl - bdyl * cdxl);

    return (double)detl;
}

// Bowyer–Watson Delaunay
std::vector<DTTri> bowyerWatson(const std::vector<DTPoint>& pts)
{
    std::vector<DTPoint> P = pts;
    const int n0 = (int)P.size();
    if (n0 < 3)
        return {}; // 삼각분할 불가

    DTPoint lo{+1e300, +1e300}, hi{-1e300, -1e300};
    for (const auto& p : P)
    {
        lo.x = std::min(lo.x, p.x);
        lo.y = std::min(lo.y, p.y);
        hi.x = std::max(hi.x, p.x);
        hi.y = std::max(hi.y, p.y);
    }

    DTPoint c{ (lo.x + hi.x) * 0.5, (lo.y + hi.y) * 0.5 };
    double d = std::max(hi.x - lo.x, hi.y - lo.y) * 1000.0 + 1.0;

    // super-triangle
    P.push_back({c.x - 2.0 * d, c.y - d});
    P.push_back({c.x + 2.0 * d, c.y - d});
    P.push_back({c.x,           c.y + 2.0 * d});
    int s1 = n0;
    int s2 = n0 + 1;
    int s3 = n0 + 2;

    std::vector<DTTri> T;
    T.push_back({s1, s2, s3});

    for (int ip = 0; ip < n0; ++ip)
    {
        const DTPoint& p = P[ip];

        std::vector<int> bad;
        bad.reserve(T.size());
        for (int t = 0; t < (int)T.size(); ++t)
        {
            auto& tr = T[t];
            if (!ccw(P[tr.a], P[tr.b], P[tr.c]))
                std::swap(tr.b, tr.c);

            if (incircle_filt(P[tr.a], P[tr.b], P[tr.c], p) > 0.0)
                bad.push_back(t);
        }

        struct Edge { int u, v; };
        std::vector<Edge> poly;

        auto addE = [&](int u, int v)
        {
            for (auto it = poly.begin(); it != poly.end(); ++it)
            {
                if (it->u == v && it->v == u)
                {
                    poly.erase(it);
                    return;
                }
            }
            poly.push_back({u, v});
        };

        std::vector<char> del(T.size(), 0);
        for (int id : bad)
        {
            del[id] = 1;
            auto tr = T[id];
            addE(tr.a, tr.b);
            addE(tr.b, tr.c);
            addE(tr.c, tr.a);
        }

        std::vector<DTTri> keep;
        keep.reserve(T.size());
        for (int i = 0; i < (int)T.size(); ++i)
        {
            if (!del[i])
                keep.push_back(T[i]);
        }
        T.swap(keep);

        for (const auto& e : poly)
        {
            DTTri nt{e.u, e.v, ip};
            if (!ccw(P[nt.a], P[nt.b], P[nt.c]))
                std::swap(nt.b, nt.c);
            T.push_back(nt);
        }
    }

    std::vector<DTTri> out;
    out.reserve(T.size());
    for (auto tr : T)
    {
        if (tr.a >= n0 || tr.b >= n0 || tr.c >= n0)
            continue;
        if (!ccw(P[tr.a], P[tr.b], P[tr.c]))
            std::swap(tr.b, tr.c);
        out.push_back(tr);
    }

    return out;
}

struct EdgeKey
{
    int u, v;
    bool operator<(const EdgeKey& o) const
    {
        if (u != o.u) return u < o.u;
        return v < o.v;
    }
};

// ===== MST 기반 센터라인 정렬 =====
struct DSU
{
    std::vector<int> p;
    std::vector<int> r;

    explicit DSU(int n) : p(n), r(n, 0)
    {
        for (int i = 0; i < n; ++i) p[i] = i;
    }

    int find(int x)
    {
        if (p[x] == x) return x;
        return p[x] = find(p[x]);
    }

    bool unite(int a, int b)
    {
        a = find(a);
        b = find(b);
        if (a == b) return false;
        if (r[a] < r[b]) std::swap(a, b);
        p[b] = a;
        if (r[a] == r[b]) ++r[a];
        return true;
    }
};

void dfsFarthest(int u,
                 int parent,
                 double acc,
                 const std::vector<std::vector<int>>& adj,
                 const std::vector<Eigen::Vector2d>& pts,
                 int& best_node,
                 double& best_dist)
{
    if (acc > best_dist)
    {
        best_dist = acc;
        best_node = u;
    }
    for (int v : adj[u])
    {
        if (v == parent) continue;
        double w = (pts[v] - pts[u]).norm();
        dfsFarthest(v, u, acc + w, adj, pts, best_node, best_dist);
    }
}

void dfsDiameter(int u,
                 int parent_u,
                 double acc,
                 const std::vector<std::vector<int>>& adj,
                 const std::vector<Eigen::Vector2d>& pts,
                 std::vector<int>& parent,
                 int& best_node,
                 double& best_dist)
{
    parent[u] = parent_u;
    if (acc > best_dist)
    {
        best_dist = acc;
        best_node = u;
    }
    for (int v : adj[u])
    {
        if (v == parent_u) continue;
        double w = (pts[v] - pts[u]).norm();
        dfsDiameter(v, u, acc + w, adj, pts, parent, best_node, best_dist);
    }
}

// raw_center → MST → 최장 경로 추출 → ordered_center
bool orderCenterByMST(const std::vector<Eigen::Vector2d>& raw_center,
                      std::vector<Eigen::Vector2d>& ordered_center)
{
    const int N = (int)raw_center.size();
    if (N < 2)
    {
        ordered_center = raw_center;
        return false;
    }

    struct Edge
    {
        int u, v;
        double w;
    };

    std::vector<Edge> edges;
    edges.reserve(N * (N - 1) / 2);
    for (int i = 0; i < N; ++i)
    {
        for (int j = i + 1; j < N; ++j)
        {
            double w = (raw_center[i] - raw_center[j]).norm();
            edges.push_back({i, j, w});
        }
    }

    std::sort(edges.begin(), edges.end(),
              [](const Edge& a, const Edge& b) { return a.w < b.w; });

    DSU dsu(N);
    std::vector<std::vector<int>> adj(N);
    int used_edges = 0;
    for (const auto& e : edges)
    {
        if (dsu.unite(e.u, e.v))
        {
            adj[e.u].push_back(e.v);
            adj[e.v].push_back(e.u);
            ++used_edges;
            if (used_edges == N - 1) break;
        }
    }

    if (used_edges == 0)
    {
        ordered_center = raw_center;
        return false;
    }

    // 1) 임의의 점(0번)에서 가장 먼 점 f1 찾기
    int f1 = 0;
    double best_dist = -1.0;
    dfsFarthest(0, -1, 0.0, adj, raw_center, f1, best_dist);

    // 2) f1에서 가장 먼 점 f2 찾으면서 parent 기록 → 직경 경로
    int f2 = f1;
    best_dist = -1.0;
    std::vector<int> parent(N, -1);
    dfsDiameter(f1, -1, 0.0, adj, raw_center, parent, f2, best_dist);

    // 3) f2에서 parent를 타고 f1까지 올라가며 경로 재구성
    std::vector<int> path_idx;
    for (int cur = f2; cur != -1; cur = parent[cur])
        path_idx.push_back(cur);
    std::reverse(path_idx.begin(), path_idx.end()); // f1 → f2

    if (path_idx.empty())
    {
        ordered_center = raw_center;
        return false;
    }

    // 4) 차량(0,0)에 더 가까운 끝점을 시작으로 방향 맞추기
    double d_start = raw_center[path_idx.front()].squaredNorm();
    double d_end   = raw_center[path_idx.back()].squaredNorm();
    if (d_start > d_end)
        std::reverse(path_idx.begin(), path_idx.end());

    ordered_center.clear();
    ordered_center.reserve(path_idx.size());
    for (int idx : path_idx)
        ordered_center.push_back(raw_center[idx]);

    return true;
}

} // anonymous namespace

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
    pnh_.param("angle_from_left_first_cone",  angle_from_left_first_cone_, 45.0);  // deg
    pnh_.param("dist_from_left_first_cone",   dist_from_left_first_cone_,   1.5);   // m
    pnh_.param("dist_from_right_first_cone",  dist_from_right_first_cone_,  1.5);   // m
    pnh_.param("dist_between_cones",          dist_between_cones_,          0.4);   // m
    pnh_.param("max_turn_angle_deg",          max_turn_angle_deg_,         45.0);   // 세그먼트 사이 최대 회전 각
    pnh_.param("weight_dist",                 weight_dist_,                 1.0);   // 거리 가중치
    pnh_.param("weight_angle",                weight_angle_,                0.5);   // 각도 가중치

    // 스플라인 샘플링 간격 (센터라인 상에서 몇 m마다 샘플링할지)
    pnh_.param("centerline_ds",               centerline_ds_,               0.05);  // 5cm 간격

    // start: 0 < x < 1.3, -0.6 < y < 0.6, 개수 >= 6
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

    // ----- ROS -----
    obstacles_sub_ = nh_.subscribe(
        "/raw_obstacles", 1,
        &RubberConeNode::obstaclesCallback, this);

    control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(
        "high_level/ackermann_cmd_mux/input/nav_1", 1);

    centerline_path_pub_ = nh_.advertise<nav_msgs::Path>(
        "rubber_cone_centerline", 1, false);

    left_boundary_path_pub_  = nh_.advertise<nav_msgs::Path>(
        "left_boundary_path", 1, false);
    right_boundary_path_pub_ = nh_.advertise<nav_msgs::Path>(
        "right_boundary_path", 1, false);

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

        // 2) 센터라인 계산 (Delaunay + MST + 스플라인 보간)
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

    for (const auto& p : side_points)
    {
        double ang_deg = std::atan2(p.y(), p.x()) * 180.0 / M_PI;

        if (left_side)
        {
            // 왼쪽 옆 방향(약 90도 근처) 이상만 사용
            if (ang_deg > angle_from_left_first_cone_)
                filtered.push_back(p);
        }
        else
        {
            // 오른쪽 옆 방향(-90도 근처) 이하만 사용
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

    const double max_step_dist     = dist_between_cones_;             // 기본 거리 한계
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

            // 3) 각도 제약
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
                // 첫 번째 세그먼트인 경우: 차량 진행 방향(x+)과의 각도 차이를 penalty로
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
            // 왼쪽 포인트만 사용해서 왼쪽 경계 생성
            buildLineFromFirst(left_points, first_left, left_line);
        }
    }

    // 오른쪽
    auto right_points = extractSidePoints(false);
    if (!right_points.empty())
    {
        Eigen::Vector2d first_right;
        if (selectFirstCone(right_points, false, first_right))
        {
            // 오른쪽 포인트만 사용해서 오른쪽 경계 생성
            buildLineFromFirst(right_points, first_right, right_line);
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

// left/right_line → center_line (Delaunay + label-different midpoints + MST + spline)
void RubberConeNode::buildCenterLine(const std::vector<Eigen::Vector2d>& left_line,
                                     const std::vector<Eigen::Vector2d>& right_line,
                                     std::vector<Eigen::Vector2d>& center_line) const
{
    center_line.clear();

    // 0) 양쪽 경계 모두 없는 경우
    if (left_line.empty() && right_line.empty())
        return;

    // 1) 델로네용 점 + 라벨 구성
    std::vector<DTPoint> P;
    std::vector<int>     label;  // 0 = left, 1 = right

    bool has_left  = !left_line.empty();
    bool has_right = !right_line.empty();

    if (has_left)
    {
        for (const auto& p : left_line)
        {
            P.push_back(DTPoint{p.x(), p.y()});
            label.push_back(0);
        }
    }
    if (has_right)
    {
        for (const auto& p : right_line)
        {
            P.push_back(DTPoint{p.x(), p.y()});
            label.push_back(1);
        }
    }

    // 한쪽 경계만 있을 때: 첫 점을 y축 대칭해서 가짜 반대편 추가
    if (has_left && !has_right)
    {
        const Eigen::Vector2d& p0 = left_line.front();
        P.push_back(DTPoint{p0.x(), -p0.y()});
        label.push_back(1);
    }
    else if (!has_left && has_right)
    {
        const Eigen::Vector2d& p0 = right_line.front();
        P.push_back(DTPoint{p0.x(), -p0.y()});
        label.push_back(0);
    }

    const int N = (int)P.size();
    if (N == 0)
        return;

    // 점이 너무 적으면 삼각분할 대신 간단 처리
    std::vector<Eigen::Vector2d> raw_center;

    // 점이 너무 적으면(삼각분할 의미 없음) → 경계 기반 센터라인 사용
    if (N < 3)
    {
        std::vector<Eigen::Vector2d> raw_center;

        if (!left_line.empty() && !right_line.empty())
        {
            // 양쪽 경계가 모두 있을 때:
            // 각 측 라인이 x 증가 방향으로 정렬되어 있으니
            // 인덱스 맞춰서 단순 평균으로 센터라인 생성
            int m = std::min(left_line.size(), right_line.size());
            raw_center.reserve(m);
            for (int i = 0; i < m; ++i)
            {
                raw_center.emplace_back(
                    0.5 * (left_line[i].x() + right_line[i].x()),
                    0.5 * (left_line[i].y() + right_line[i].y())
                );
            }
        }
        else if (!left_line.empty() || !right_line.empty())
        {
            // 한쪽 경계만 있을 때:
            // 그냥 그 라인을 그대로 따라가도록 센터라인을 그 라인으로 둠
            // const auto& src = !left_line.empty() ? left_line : right_line;
            // raw_center = src;
            return;
        }
        else
        {
            // 여기 오면 사실 P.size() < 3과 모순이지만 안전하게 리턴
            return;
        }

        if (!raw_center.empty())
            smoothCenterLineWithSpline(raw_center, center_line);

        return;
    }

    // 2) 델로네 삼각분할
    auto tris = bowyerWatson(P);
    if (tris.empty())
    {
        return;
    }

    // 3) 라벨이 서로 다른 엣지의 중점 모으기
    std::set<EdgeKey> mixed_edges;

    for (const auto& tr : tris)
    {
        int vs[3] = {tr.a, tr.b, tr.c};
        for (int i = 0; i < 3; ++i)
        {
            int u = vs[i];
            int v = vs[(i + 1) % 3];

            if (u == v) continue;
            if (label[u] == label[v]) continue;

            EdgeKey ek;
            ek.u = std::min(u, v);
            ek.v = std::max(u, v);
            mixed_edges.insert(ek);
        }
    }

    raw_center.reserve(mixed_edges.size());
    for (const auto& e : mixed_edges)
    {
        const auto& pa = P[e.u];
        const auto& pb = P[e.v];
        raw_center.emplace_back(
            0.5 * (pa.x + pb.x),
            0.5 * (pa.y + pb.y));
    }

    if (raw_center.empty())
        return;

    // 4) MST 기반 최장 경로 순서로 재정렬 후 스플라인 스무딩
    std::vector<Eigen::Vector2d> ordered_center;
    bool ok = orderCenterByMST(raw_center, ordered_center);
    if (!ok)
        ordered_center = raw_center;

    smoothCenterLineWithSpline(ordered_center, center_line);
}

// center_line → steering angle
double RubberConeNode::computeSteeringFromCenterLine(std::vector<Eigen::Vector2d>& center_line)
{
    if (center_line.empty())
    {
        is_look_forward_point_ = false;
        return 0.0;
    }

    double theta = 0.0;
    is_look_forward_point_ = false;

    // center_line 은 MST + 스플라인으로 "앞에서 뒤로" 정렬되어 있다고 가정
    for (const auto& p : center_line)
    {
        double dis = p.norm();
        if (dis >= lfd_)
        {
            is_look_forward_point_ = true;
            theta = std::atan2(p.y(), p.x());
            break;
        }
    }

    if (!is_look_forward_point_)
    {
        const Eigen::Vector2d& last = center_line.back();
        theta = std::atan2(last.y(), last.x() + vehicle_length_);
    }

    double steering = std::atan2(2.0 * vehicle_length_ * std::sin(theta), lfd_);
    return steering;
}

void RubberConeNode::publishControl(double steering)
{
    target_control_.header.stamp = ros::Time::now();
    target_control_.drive.speed  = speed_;
    target_control_.drive.steering_angle = steering;

    control_pub_.publish(target_control_);

    ROS_INFO_STREAM_THROTTLE(
        1.0,
        "[RubberConeNode] mission running, steering = "
        << steering * 180.0 / M_PI << " deg.");
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
