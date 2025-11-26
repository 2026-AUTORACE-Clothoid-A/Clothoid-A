#include "rubber_cone_node.hpp"

// 스플라인 헤더 (사용하던 tk::spline 그대로)
#include "spline.h"

#include <algorithm>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ====== Delaunay + MST 헬퍼들 (anonymous namespace) ======
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
inline double incircle_filt(
    const DTPoint& a, const DTPoint& b,
    const DTPoint& c, const DTPoint& d)
{
    double adx = a.x - d.x, ady = a.y - d.y;
    double bdx = b.x - d.x, bdy = b.y - d.y;
    double cdx = c.x - d.x, cdy = c.y - d.y;
    double ad  = adx * adx + ady * ady;
    double bd  = bdx * bdx + bdy * bdy;
    double cd  = cdx * cdx + cdy * cdy;

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
        return {};

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

// MST 기반 centerline ordering
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

    // 1) 임의의 점(0)에서 가장 먼 점 f1
    int f1 = 0;
    double best_dist = -1.0;
    dfsFarthest(0, -1, 0.0, adj, raw_center, f1, best_dist);

    // 2) f1에서 가장 먼 점 f2 (직경)
    int f2 = f1;
    best_dist = -1.0;
    std::vector<int> parent(N, -1);
    dfsDiameter(f1, -1, 0.0, adj, raw_center, parent, f2, best_dist);

    // 3) f2 → f1 parent 따라가며 path 구성
    std::vector<int> path_idx;
    for (int cur = f2; cur != -1; cur = parent[cur])
        path_idx.push_back(cur);
    std::reverse(path_idx.begin(), path_idx.end());

    if (path_idx.empty())
    {
        ordered_center = raw_center;
        return false;
    }

    // 4) 차량(0,0)에 더 가까운 끝을 시작점으로 설정
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

// ===== RubberConeNode 구현 =====

RubberConeNode::RubberConeNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , mission_active_(false)
    , prev_mission_active_(false)
    , has_left_boundary_(false)
    , has_right_boundary_(false)
    , is_look_forward_point_(false)
    , last_steering_(0.0)
    , has_last_steering_(false)
    , last_left_boundary_stamp_(0.0)
    , last_right_boundary_stamp_(0.0)
{
    // ----- 파라미터 로드 -----
    pnh_.param("speed",          speed_,          0.4);  // m/s
    pnh_.param("vehicle_length", vehicle_length_, 0.25); // m
    pnh_.param("lfd",            lfd_,            0.6);  // m
    pnh_.param("centerline_ds",  centerline_ds_,  0.05); // 5 cm

    // 로컬 최소제곱 윈도우 크기 (홀수, 5 이상 권장)
    pnh_.param("lsq_window_size", lsq_window_size_, 5);
    if (lsq_window_size_ < 5)
        lsq_window_size_ = 5;
    if (lsq_window_size_ % 2 == 0)
        lsq_window_size_ += 1;

    pnh_.param("boundary_timeout", boundary_timeout_sec_, 0.3);

    // ----- ROS 구독/퍼블리시 -----
    left_boundary_sub_ = nh_.subscribe(
        "left_boundary_path", 1,
        &RubberConeNode::leftBoundaryCallback, this);

    right_boundary_sub_ = nh_.subscribe(
        "right_boundary_path", 1,
        &RubberConeNode::rightBoundaryCallback, this);

    mission_state_sub_ = nh_.subscribe(
        "cone_mission_active", 1,
        &RubberConeNode::missionStateCallback, this);

    control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(
        "high_level/ackermann_cmd_mux/input/nav_1", 1);

    centerline_path_pub_ = nh_.advertise<nav_msgs::Path>(
        "rubber_cone_centerline", 1, false);

    // 제어 초기값
    target_control_.drive.steering_angle = 0.0;
    target_control_.drive.speed          = speed_;
}

// ===== 콜백 =====

void RubberConeNode::leftBoundaryCallback(const nav_msgs::Path::ConstPtr& msg)
{
    left_line_.clear();
    left_line_.reserve(msg->poses.size());
    for (const auto& ps : msg->poses)
    {
        left_line_.emplace_back(ps.pose.position.x,
                                ps.pose.position.y);
    }
    has_left_boundary_ = !left_line_.empty();

    last_left_boundary_stamp_ = msg->header.stamp;
    if (last_left_boundary_stamp_.isZero())
        last_left_boundary_stamp_ = ros::Time::now();

    if (mission_active_)
        updateAndControl();
}

void RubberConeNode::rightBoundaryCallback(const nav_msgs::Path::ConstPtr& msg)
{
    right_line_.clear();
    right_line_.reserve(msg->poses.size());
    for (const auto& ps : msg->poses)
    {
        right_line_.emplace_back(ps.pose.position.x,
                                 ps.pose.position.y);
    }
    has_right_boundary_ = !right_line_.empty();

    last_right_boundary_stamp_ = msg->header.stamp;
    if (last_right_boundary_stamp_.isZero())
        last_right_boundary_stamp_ = ros::Time::now();

    if (mission_active_)
        updateAndControl();
}


void RubberConeNode::missionStateCallback(const std_msgs::Bool::ConstPtr& msg)
{
    mission_active_ = msg->data;

    if (mission_active_ && !prev_mission_active_)
    {
        ROS_INFO("[RubberConeNode] Mission START (cone_mission_active = true)");
        if (has_left_boundary_ || has_right_boundary_)
            updateAndControl();
    }
    else if (!mission_active_ && prev_mission_active_)
    {
        ROS_INFO("[RubberConeNode] Mission END (cone_mission_active = false)");
        is_look_forward_point_ = false;

        // ★ 옵션: 미션 끝나면 경계 상태도 리셋 ★
        has_left_boundary_  = false;
        has_right_boundary_ = false;
        left_line_.clear();
        right_line_.clear();
        last_left_boundary_stamp_  = ros::Time(0);
        last_right_boundary_stamp_ = ros::Time(0);
    }

    prev_mission_active_ = mission_active_;
}

// ===== 메인 처리 =====

void RubberConeNode::updateAndControl()
{
    if (!mission_active_)
        return;

    // 현재 시간 기준으로 경계의 신선도 체크
    ros::Time now = ros::Time::now();

    bool left_fresh  = has_left_boundary_  &&
                       !last_left_boundary_stamp_.isZero() &&
                       (now - last_left_boundary_stamp_).toSec() <= boundary_timeout_sec_;

    bool right_fresh = has_right_boundary_ &&
                       !last_right_boundary_stamp_.isZero() &&
                       (now - last_right_boundary_stamp_).toSec() <= boundary_timeout_sec_;

    // 디버그용 로그
    if (!left_fresh && has_left_boundary_)
    {
        ROS_WARN_THROTTLE(1.0,
            "[RubberConeNode] left boundary stale (age=%.3f s > %.3f s)",
            (now - last_left_boundary_stamp_).toSec(), boundary_timeout_sec_);
    }
    if (!right_fresh && has_right_boundary_)
    {
        ROS_WARN_THROTTLE(1.0,
            "[RubberConeNode] right boundary stale (age=%.3f s > %.3f s)",
            (now - last_right_boundary_stamp_).toSec(), boundary_timeout_sec_);
    }

    // 둘 다 오래됐으면 제어 안 함
    if (!left_fresh && !right_fresh)
    {
        ROS_WARN_THROTTLE(1.0,
            "[RubberConeNode] All boundaries stale. Skip control.");
        return;
    }

    // 신선한 경계만 넘겨서 센터라인 생성
    std::vector<Eigen::Vector2d> left_for_center;
    std::vector<Eigen::Vector2d> right_for_center;

    if (left_fresh)
        left_for_center = left_line_;
    if (right_fresh)
        right_for_center = right_line_;

    std::vector<Eigen::Vector2d> center_line;
    buildCenterLine(left_for_center, right_for_center, center_line);

    if (center_line.empty())
    {
        ROS_WARN_THROTTLE(1.0, "[RubberConeNode] Centerline is empty.");
        return;
    }

    publishCenterlinePath(center_line);

    double steering = computeSteeringFromCenterLine(center_line);
    publishControl(steering);
}

// ===== 센터라인 생성 =====

// Delaunay + label-different edge midpoints + MST + (로컬 LSQ + 스플라인)
void RubberConeNode::buildCenterLine(const std::vector<Eigen::Vector2d>& left_line,
                                     const std::vector<Eigen::Vector2d>& right_line,
                                     std::vector<Eigen::Vector2d>& center_line) const
{
    center_line.clear();

    bool has_left  = !left_line.empty();
    bool has_right = !right_line.empty();

    if (!has_left && !has_right)
        return;

    // 1) Delaunay용 점 + 라벨 구성
    std::vector<DTPoint> P;
    std::vector<int>     label; // 0 = left, 1 = right

    if (has_left)
    {
        for (const auto& p : left_line)
        {
            P.push_back({p.x(), p.y()});
            label.push_back(0);
        }
    }
    if (has_right)
    {
        for (const auto& p : right_line)
        {
            P.push_back({p.x(), p.y()});
            label.push_back(1);
        }
    }

    // 한쪽 경계만 있을 때: 첫 점 y-대칭으로 가짜 반대편 추가 (안전장치)
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

    // 점이 너무 적으면: 간단히 양쪽 평균 or 한쪽 경계 사용
    if (N < 3)
    {
        std::vector<Eigen::Vector2d> raw_center;

        if (has_left && has_right)
        {
            int m = std::min(left_line.size(), right_line.size());
            raw_center.reserve(m);
            for (int i = 0; i < m; ++i)
            {
                raw_center.emplace_back(
                    0.5 * (left_line[i].x() + right_line[i].x()),
                    0.5 * (left_line[i].y() + right_line[i].y()));
            }
        }
        else if (has_left || has_right)
        {
            const auto& src = has_left ? left_line : right_line;
            raw_center = src;
        }

        if (!raw_center.empty())
        {
            // 로컬 LSQ + 스플라인 적용
            std::vector<Eigen::Vector2d> lsq_center;
            smoothCenterLineLeastSquaresCubic(raw_center, lsq_center);
            smoothCenterLineWithSpline(lsq_center, center_line);
        }
        return;
    }

    // 2) Delaunay 삼각분할
    auto tris = bowyerWatson(P);
    if (tris.empty())
    {
        return;
    }

    // 3) 라벨이 서로 다른 엣지의 중점 모으기
    struct EdgeKey
    {
        int u, v;
        bool operator<(const EdgeKey& o) const
        {
            if (u != o.u) return u < o.u;
            return v < o.v;
        }
    };

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

    std::vector<Eigen::Vector2d> raw_center;
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

    // 4) MST 기반 최장 경로 순서 정렬
    std::vector<Eigen::Vector2d> ordered_center;
    bool ok = orderCenterByMST(raw_center, ordered_center);
    if (!ok)
        ordered_center = raw_center;

    // 5) 로컬 3차 최소제곱 스무딩
    std::vector<Eigen::Vector2d> lsq_center;
    smoothCenterLineLeastSquaresCubic(ordered_center, lsq_center);

    // 6) tk::spline으로 arc-length 보간 + 일정 간격 샘플링
    smoothCenterLineWithSpline(lsq_center, center_line);
}

// ===== 로컬 3차 최소제곱 스무딩 =====

void RubberConeNode::smoothCenterLineLeastSquaresCubic(
    const std::vector<Eigen::Vector2d>& raw_center,
    std::vector<Eigen::Vector2d>& smoothed_center) const
{
    smoothed_center.clear();
    const int N = static_cast<int>(raw_center.size());
    if (N == 0)
        return;

    if (N <= 3)
    {
        smoothed_center = raw_center;
        return;
    }

    // 1) arc-length 파라미터 s 계산
    std::vector<double> s(N, 0.0);
    for (int i = 1; i < N; ++i)
    {
        double ds = (raw_center[i] - raw_center[i - 1]).norm();
        s[i] = s[i - 1] + ds;
    }

    smoothed_center.resize(N);

    int window_size = lsq_window_size_;
    int half = window_size / 2;

    for (int i = 0; i < N; ++i)
    {
        int j0 = std::max(0, i - half);
        int j1 = std::min(N - 1, i + half);
        int m  = j1 - j0 + 1;

        if (m < 4)
        {
            smoothed_center[i] = raw_center[i];
            continue;
        }

        Eigen::MatrixXd A(m, 4);
        Eigen::VectorXd bx(m), by(m);

        double s0 = s[i]; // 중심 기준 (t = s - s0)

        int row = 0;
        for (int j = j0; j <= j1; ++j, ++row)
        {
            double t  = s[j] - s0;
            double t2 = t * t;
            double t3 = t2 * t;

            A(row, 0) = 1.0;
            A(row, 1) = t;
            A(row, 2) = t2;
            A(row, 3) = t3;

            bx(row) = raw_center[j].x();
            by(row) = raw_center[j].y();
        }

        Eigen::Vector4d cx = A.colPivHouseholderQr().solve(bx);
        Eigen::Vector4d cy = A.colPivHouseholderQr().solve(by);

        double x0 = cx[0]; // t=0 에서의 값
        double y0 = cy[0];

        smoothed_center[i] = Eigen::Vector2d(x0, y0);
    }
}

// ===== 스플라인 보간 =====

void RubberConeNode::smoothCenterLineWithSpline(
    const std::vector<Eigen::Vector2d>& raw_center,
    std::vector<Eigen::Vector2d>& smooth_center) const
{
    smooth_center.clear();
    const std::size_t N = raw_center.size();

    if (N < 3)
    {
        smooth_center = raw_center;
        return;
    }

    // 1) arc-length s
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

    // 2) x(s), y(s)
    std::vector<double> xs(N), ys(N);
    for (std::size_t i = 0; i < N; ++i)
    {
        xs[i] = raw_center[i].x();
        ys[i] = raw_center[i].y();
    }

    tk::spline sx, sy;
    sx.set_points(s, xs, tk::spline::cspline);
    sy.set_points(s, ys, tk::spline::cspline);

    // 3) 일정 간격 샘플링
    double ds_sample = centerline_ds_;
    if (ds_sample <= 1e-4)
        ds_sample = s_total / static_cast<double>(N - 1);

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

double RubberConeNode::computeSteeringFromCenterLine(
    const std::vector<Eigen::Vector2d>& center_line)
{
    const int N = static_cast<int>(center_line.size());
    if (N == 0)
    {
        is_look_forward_point_ = false;

        // 센터라인도 없으면 이전 조향각 유지 (없으면 0.0)
        if (has_last_steering_)
            return last_steering_;
        return 0.0;
    }

    // 1) base_link(0,0)에서 가장 가까운 점 인덱스 찾기
    int    closest_idx = 0;
    double best_d2     = center_line[0].squaredNorm();

    for (int i = 1; i < N; ++i)
    {
        double d2 = center_line[i].squaredNorm();
        if (d2 < best_d2)
        {
            best_d2     = d2;
            closest_idx = i;
        }
    }

    Eigen::Vector2d target = center_line[closest_idx];
    is_look_forward_point_ = false;

    // 2) 진행 방향 결정: center_line 상에서 +1 / -1 중,
    //    차량 진행 방향 (+x)과 더 잘 맞는 쪽으로
    Eigen::Vector2d car_dir(1.0, 0.0);   // base_link 기준 전방
    int dir = 1;

    double score_plus  = -1e9;
    double score_minus = -1e9;

    if (closest_idx + 1 < N)
    {
        Eigen::Vector2d v = center_line[closest_idx + 1] - center_line[closest_idx];
        if (v.norm() > 1e-6)
            score_plus = v.normalized().dot(car_dir);
    }
    if (closest_idx - 1 >= 0)
    {
        Eigen::Vector2d v = center_line[closest_idx - 1] - center_line[closest_idx];
        if (v.norm() > 1e-6)
            score_minus = v.normalized().dot(car_dir);
    }

    if (score_plus >= score_minus)
        dir = +1;
    else
        dir = -1;

    // 3) closest_idx에서 선택한 방향(dir)으로 LfD만큼 arc-length 누적
    double s_acc  = 0.0;
    int    cur_idx = closest_idx;

    while (true)
    {
        int next_idx = cur_idx + dir;
        if (next_idx < 0 || next_idx >= N)
            break;

        double ds = (center_line[next_idx] - center_line[cur_idx]).norm();
        s_acc += ds;
        cur_idx = next_idx;

        if (s_acc >= lfd_)
        {
            target = center_line[cur_idx];
            is_look_forward_point_ = true;
            break;
        }
    }

    // 4) LfD 지점을 못 찾았으면 이전 조향각 유지
    if (!is_look_forward_point_)
    {
        if (has_last_steering_)
        {
            // LfD 못 채울 정도로 센터라인이 짧으면
            // 새로운 조향각 계산 대신 이전 값을 그대로 사용
            return last_steering_;
        }

        // 첫 프레임 등: 이전 조향각도 없으면
        // 최소한 base_link에 가장 가까운 점을 향해서 조향
        target = center_line[closest_idx];
    }

    // 5) Pure Pursuit 형식 조향각 계산 (변경 없음)
    double theta = std::atan2(target.y(), target.x() + vehicle_length_);
    double steering = std::atan2(2.0 * vehicle_length_ * std::sin(theta), lfd_);

    return steering;
}


void RubberConeNode::publishControl(double steering)
{
    target_control_.header.stamp = ros::Time::now();
    target_control_.drive.speed  = speed_;
    target_control_.drive.steering_angle = steering;

    control_pub_.publish(target_control_);

    // 마지막 조향각 갱신
    last_steering_ = steering;
    has_last_steering_ = true;

    ROS_INFO_STREAM_THROTTLE(
        1.0,
        "[RubberConeNode] mission active, steering = "
        << steering * 180.0 / M_PI << " deg.");
}

// ===== Path 유틸 =====

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

void RubberConeNode::publishCenterlinePath(
    const std::vector<Eigen::Vector2d>& center_line)
{
    if (center_line.empty())
        return;

    nav_msgs::Path path_msg;
    fillPathMsg(center_line, "base_link", path_msg);
    centerline_path_pub_.publish(path_msg);
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
