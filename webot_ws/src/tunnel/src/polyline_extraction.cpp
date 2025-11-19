// polyline_extraction_node.cpp
//
// Probabilistic polyline extraction from a single 2D LaserScan
// (Schaefer et al., IROS 2018 - Stage 1: Polyline Extraction).
//
// - Subscribes: sensor_msgs/LaserScan ("scan")
// - Publishes:  visualization_msgs::Marker ("polylines")
//   -> LINE_LIST of all extracted polylines
//
// Dependencies: Eigen (for 2D vectors)

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <Eigen/Core>
#include <vector>
#include <cmath>
#include <limits>
#include <string>

// ------------------------------- Data types -------------------------------

struct LSScan {
    Eigen::Vector2d origin;           // sensor origin in world (여기선 (0,0) 가정)
    std::vector<double> angles;       // 각 레이의 각도
    std::vector<double> ranges;       // 측정 반경
    std::vector<bool>   valid;        // 유효 측정 (finite & within range)
    double range_max;                 // 센서 최대 범위
};

struct Polyline {
    std::vector<Eigen::Vector2d> pts; // 정점들 (world frame)
};

// ------------------------------- Utilities --------------------------------

inline double cross2d(const Eigen::Vector2d& a, const Eigen::Vector2d& b)
{
    return a.x() * b.y() - a.y() * b.x();
}

Eigen::Vector2d endpointFromScan(const LSScan& ls, int k)
{
    double a = ls.angles[k];
    double r = ls.ranges[k];
    return ls.origin + Eigen::Vector2d(r * std::cos(a), r * std::sin(a));
}

LSScan buildLSScan(const sensor_msgs::LaserScan& msg)
{
    LSScan ls;
    ls.origin = Eigen::Vector2d(0.0, 0.0);  // 센서 프레임을 world로 사용한다고 가정
    const size_t K = msg.ranges.size();
    ls.angles.resize(K);
    ls.ranges.resize(K);
    ls.valid.resize(K);
    ls.range_max = msg.range_max;

    double angle = msg.angle_min;
    for (size_t i = 0; i < K; ++i, angle += msg.angle_increment) {
        ls.angles[i] = angle;
        double r = msg.ranges[i];
        bool v = std::isfinite(r) && r >= msg.range_min && r <= msg.range_max;
        ls.ranges[i] = r;
        ls.valid[i]  = v;
    }
    return ls;
}

// ray: o + t*d (t >= 0)
// segment: p0 + u*(p1-p0), u ∈ [0,1]
// return t (distance along ray) or INF if no intersection
double intersectRaySegment(const Eigen::Vector2d& o,
                           const Eigen::Vector2d& d,
                           const Eigen::Vector2d& p0,
                           const Eigen::Vector2d& p1)
{
    Eigen::Vector2d s = p1 - p0;
    double denom = cross2d(d, s);
    if (std::fabs(denom) < 1e-9) {
        return std::numeric_limits<double>::infinity();
    }
    Eigen::Vector2d po = p0 - o;
    double t = cross2d(po, s) / denom;
    double u = cross2d(po, d) / denom;

    if (t >= 0.0 && u >= 0.0 && u <= 1.0) {
        return t;
    }
    return std::numeric_limits<double>::infinity();
}

// xpmLocal: LSScan + "국소 polyline(점들)"에 대해
// 각 레이가 이 polyline과 처음 교차하는 거리 r_pred[k]와
// 실제로 교차하는지 여부 hit_mask[k]를 계산
void xpmLocal(const LSScan& ls,
              const std::vector<Eigen::Vector2d>& pts,
              std::vector<double>& r_pred,
              std::vector<uint8_t>& hit_mask)
{
    const int K = static_cast<int>(ls.angles.size());
    const double INF = std::numeric_limits<double>::infinity();

    r_pred.assign(K, INF);
    hit_mask.assign(K, 0);

    for (int k = 0; k < K; ++k) {
        if (!ls.valid[k]) continue;

        Eigen::Vector2d o = ls.origin;
        Eigen::Vector2d d(std::cos(ls.angles[k]), std::sin(ls.angles[k]));

        double best_t = INF;
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            double t = intersectRaySegment(o, d, pts[i], pts[i + 1]);
            if (std::isfinite(t) && t > 0.0 && t < best_t) {
                best_t = t;
            }
        }

        if (std::isfinite(best_t)) {
            r_pred[k] = best_t;
            hit_mask[k] = 1;
        }
    }
}

// --------------------------- Initial polyline L ---------------------------
// Algorithm 1, lines 2–8 에 대응.
// 인접 스캔 끝점들 중 조건을 만족하는 것만 연결해서 polyline들을 만든다.
// 추가 조건: 끝점 b, c 가 모두
//   -1.0 <= x < 1.0,  -0.5 <= y <= 0.5 영역 안에 있을 때만 사용.

std::vector<Polyline> buildInitialPolylines(const LSScan& ls,
                                            double lmax,
                                            double rmax)
{
    const int K = static_cast<int>(ls.ranges.size());
    std::vector<Polyline> lines;
    if (K < 2) return lines;

    Polyline current;
    bool has_curr = false;

    auto endpoint = [&](int k) {
        return endpointFromScan(ls, k);
    };

    auto in_roi = [](const Eigen::Vector2d& p) {
        return (p.x() >= -1.0 && p.x() < 1.0 &&
                p.y() >= -0.5 && p.y() <= 0.5);
    };

    for (int k = 0; k < K - 1; ++k) {
        bool ok1 = ls.valid[k]     && ls.ranges[k]     <= rmax;
        bool ok2 = ls.valid[k + 1] && ls.ranges[k + 1] <= rmax;

        bool connected = false;
        if (ok1 && ok2) {
            Eigen::Vector2d b = endpoint(k);
            Eigen::Vector2d c = endpoint(k + 1);

            // 추가된 ROI 조건: b, c 둘 다 ROI 안에 있어야 함
            if (in_roi(b) && in_roi(c)) {
                double dist = (c - b).norm();
                if (dist <= lmax) {
                    // 조건 만족 → 세그먼트 추가
                    if (!has_curr) {
                        current.pts.clear();
                        current.pts.push_back(b);
                        has_curr = true;
                    }
                    current.pts.push_back(c);
                    connected = true;
                }
            }
        }

        // 조건을 만족하지 못하면 현재 polyline 종료
        if (!connected) {
            if (has_curr && current.pts.size() >= 2) {
                lines.push_back(current);
            }
            has_curr = false;
            current.pts.clear();
        }
    }

    if (has_curr && current.pts.size() >= 2) {
        lines.push_back(current);
    }

    // 여기서는 끝점-처음점 연결(완전 polygon)까지는 구현하지 않았다.
    // 필요하면 마지막/처음 endpoint도 lmax + ROI 조건으로 연결해서 polygon 만들면 됨.

    return lines;
}

// -------------------- Vertex removal error e_j (논문식) ------------------
//
// 내부 정점(v_{j-1}, v_j, v_{j+1})에 대해:
//   L_local = [v_{j-1}, v_j, v_{j+1}]  (두 세그먼트)
//   L_rm    = [v_{j-1}, v_{j+1}]      (한 세그먼트)
//   X(L, v_j) = L_local 과 실제 교차하는 레이 집합
//   e_j = Σ_{k∈X(L,v_j)} [ (d^2(z_k, L\{v_j}) - d^2(z_k, L)) / Σ ]
//
// 끝 정점(폴리라인 시작/끝)에 대해서는 extrlin.m 과 동일한
// per-line 상수 dr 을 이용한 휴리스틱을 사용.

double removalErrorInternalVertex(const LSScan& ls,
                                  const Polyline& pl,
                                  int j,
                                  double sigma2)
{
    const int N = static_cast<int>(pl.pts.size());
    if (j <= 0 || j >= N - 1) {
        return std::numeric_limits<double>::infinity();
    }

    // L_local = [v_{j-1}, v_j, v_{j+1}]
    std::vector<Eigen::Vector2d> local_pts = {
        pl.pts[j - 1],
        pl.pts[j],
        pl.pts[j + 1]
    };

    // L_rm = [v_{j-1}, v_{j+1}]
    std::vector<Eigen::Vector2d> rm_pts = {
        pl.pts[j - 1],
        pl.pts[j + 1]
    };

    std::vector<double> r_local, r_rm;
    std::vector<uint8_t> hit_local, hit_rm;

    xpmLocal(ls, local_pts, r_local, hit_local);
    xpmLocal(ls, rm_pts,   r_rm,    hit_rm);

    const int K = static_cast<int>(ls.ranges.size());
    double e = 0.0;

    for (int k = 0; k < K; ++k) {
        // k ∈ X(L, v_j): 원래 두 세그먼트와 교차하는 레이만 사용
        if (!hit_local[k]) continue;

        double z   = ls.ranges[k];
        double rL  = r_local[k];
        double rRm = r_rm[k];

        if (!std::isfinite(rL)) continue; // 안전장치

        double dL   = z - rL;
        double dRm  = z - rRm; // rRm가 INF면 dRm^2→INF → e_j가 매우 커져서 제거 안 됨

        double drsq    = (dL  * dL)  / sigma2;
        double drsq_rm = (dRm * dRm) / sigma2;

        e += (drsq_rm - drsq);
    }

    if (!std::isfinite(e)) {
        return std::numeric_limits<double>::infinity();
    }
    return e;
}

// 끝 정점 (시작/끝)용 휴리스틱 (extrlin.m 의 polyline 부분과 같은 로직)
double removalErrorEndpointVertex(const LSScan& ls,
                                  const Polyline& pl,
                                  int j,
                                  double dr)
{
    const int N = static_cast<int>(pl.pts.size());
    if (j != 0 && j != N - 1) {
        return std::numeric_limits<double>::infinity();
    }

    const double drsq_const = dr * dr;

    // 세그먼트 선택: [0,1] 또는 [N-2, N-1]
    std::vector<Eigen::Vector2d> seg_pts;
    if (j == 0) {
        if (N < 2) return std::numeric_limits<double>::infinity();
        seg_pts = { pl.pts[0], pl.pts[1] };
    } else {
        if (N < 2) return std::numeric_limits<double>::infinity();
        seg_pts = { pl.pts[N - 2], pl.pts[N - 1] };
    }

    std::vector<double> r_seg;
    std::vector<uint8_t> hit_seg;

    xpmLocal(ls, seg_pts, r_seg, hit_seg);

    const int K = static_cast<int>(ls.ranges.size());
    double e = 0.0;

    // iret = isfinite(r) 에 해당: hit_seg == 1 인 레이만 사용 → X(L, v_j)
    for (int k = 0; k < K; ++k) {
        if (!hit_seg[k]) continue;

        double z = ls.ranges[k];
        double r = r_seg[k];
        if (!std::isfinite(r)) continue;

        double d = z - r;
        double drsq = d * d;

        // sum(drsqrm - drsq)
        e += (drsq_const - drsq);
    }

    // extrlin.m: e(j) = sum(drsqrm-drsq) - drsqrm*(po.count>2);
    if (N > 2) {
        e -= drsq_const;
    }

    if (!std::isfinite(e)) {
        return std::numeric_limits<double>::infinity();
    }
    return e;
}

double vertexRemovalError(const LSScan& ls,
                          const Polyline& pl,
                          int j,
                          double dr,
                          double sigma2)
{
    const int N = static_cast<int>(pl.pts.size());
    if (N < 2) {
        return std::numeric_limits<double>::infinity();
    }

    if (j == 0 || j == N - 1) {
        // 폴리라인 시작/끝 정점
        return removalErrorEndpointVertex(ls, pl, j, dr);
    } else {
        // 내부 정점
        return removalErrorInternalVertex(ls, pl, j, sigma2);
    }
}

// -------------------------- Greedy simplification -------------------------
//
// Algorithm 1, lines 11–21:
//  - 전체 L 에 대해 정점 제거 오차 e_j 계산
//  - J(L) > J_max 인 동안, e_j 최소인 정점부터 제거
//  - 매번 제거 후, 해당 폴리라인의 이웃 정점 두 개만 e_j 업데이트

size_t totalVertices(const std::vector<Polyline>& lines)
{
    size_t sum = 0;
    for (const auto& pl : lines) {
        sum += pl.pts.size();
    }
    return sum;
}

void greedySimplify(std::vector<Polyline>& lines,
                    const LSScan& ls,
                    int Jmax,
                    double dr,
                    double sigma2)
{
    if (Jmax <= 0) return;
    if (lines.empty()) return;

    // 초기 e_j 계산
    std::vector<std::vector<double>> err(lines.size());
    for (size_t i = 0; i < lines.size(); ++i) {
        int N = static_cast<int>(lines[i].pts.size());
        err[i].resize(N);
        for (int j = 0; j < N; ++j) {
            err[i][j] = vertexRemovalError(ls, lines[i], j, dr, sigma2);
        }
    }

    auto Jcurrent = [&]() -> int {
        return static_cast<int>(totalVertices(lines));
    };

    while (Jcurrent() > Jmax) {
        double best_e = std::numeric_limits<double>::infinity();
        int best_i = -1;
        int best_j = -1;

        // 전 정점 중 최소 e_j 찾기
        for (size_t i = 0; i < lines.size(); ++i) {
            int N = static_cast<int>(lines[i].pts.size());
            if (N == 0) continue;
            for (int j = 0; j < N; ++j) {
                double e = err[i][j];
                if (!std::isfinite(e)) continue;
                if (e < best_e) {
                    best_e = e;
                    best_i = static_cast<int>(i);
                    best_j = j;
                }
            }
        }

        if (best_i < 0) {
            // 더 이상 제거할 정점이 없음
            break;
        }

        Polyline& pl = lines[best_i];
        std::vector<double>& evec = err[best_i];
        int Nold = static_cast<int>(pl.pts.size());
        if (Nold == 0) {
            continue;
        }

        // 선택된 정점 제거
        pl.pts.erase(pl.pts.begin() + best_j);
        evec.erase(evec.begin() + best_j);
        int Nnew = static_cast<int>(pl.pts.size());

        if (Nnew <= 1) {
            // 폴리라인이 점으로 붕괴 → 전체 삭제 (extrlin에서 count<=2 제거와 대응)
            lines.erase(lines.begin() + best_i);
            err.erase(err.begin() + best_i);

            if (lines.empty()) break;
        } else {
            // 이웃 정점 두 개만 e_j 업데이트
            int left  = best_j - 1;
            int right = best_j;

            if (left >= 0 && left < Nnew) {
                evec[left] = vertexRemovalError(ls, pl, left, dr, sigma2);
            }
            if (right >= 0 && right < Nnew) {
                evec[right] = vertexRemovalError(ls, pl, right, dr, sigma2);
            }
        }
    }
}

// ----------------------------- ROS Node class -----------------------------

class PolylineExtractionNode {
public:
    PolylineExtractionNode()
        : nh_(), pnh_("~")
    {
        pnh_.param<std::string>("marker_topic", marker_topic_, std::string("polylines"));
        pnh_.param<std::string>("marker_frame", marker_frame_id_, std::string("")); // 빈 문자열이면 scan frame 사용
        pnh_.param<int>("max_vertices", max_vertices_, 50);
        pnh_.param<double>("lmax", lmax_, 0.1);          // 인접 끝점 최대 거리
        pnh_.param<double>("rmax", rmax_param_, 0.0);    // 0이면 scan.range_max 사용
        pnh_.param<double>("dr", dr_param_, 0.05);       // 끝 정점 제거 페널티
        pnh_.param<double>("sigma2", sigma2_param_, 1.0);// 센서 분산 (공통 상수)

        scan_sub_ = nh_.subscribe("scan", 1,
                                  &PolylineExtractionNode::scanCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
            marker_topic_, 1, true);

        ROS_INFO("PolylineExtractionNode initialized:"
                 " max_vertices=%d, lmax=%.3f, dr=%.3f, sigma2=%.3f",
                 max_vertices_, lmax_, dr_param_, sigma2_param_);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber scan_sub_;
    ros::Publisher marker_pub_;

    std::string marker_topic_;
    std::string marker_frame_id_;
    int    max_vertices_;
    double lmax_;
    double rmax_param_;
    double dr_param_;
    double sigma2_param_;

    void scanCallback(const sensor_msgs::LaserScanConstPtr& msg)
    {
        LSScan ls = buildLSScan(*msg);
        double rmax_use = (rmax_param_ > 0.0) ? rmax_param_ : ls.range_max;

        // 1) Algorithm 1 (1–10): 초기 폴리라인 생성
        auto lines = buildInitialPolylines(ls, lmax_, rmax_use);

        if (lines.empty()) {
            publishEmpty(msg->header);
            return;
        }

        // 2) Algorithm 1 (11–21): 탐욕적 정점 제거 (J(L) <= max_vertices 까지)
        if (max_vertices_ > 0) {
            greedySimplify(lines, ls, max_vertices_, dr_param_, sigma2_param_);
        }

        // 3) RViz Marker 로 publish
        publishLinesAsMarker(lines, msg->header);
    }

    void publishEmpty(const std_msgs::Header& header)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        if (!marker_frame_id_.empty()) {
            marker.header.frame_id = marker_frame_id_;
        }
        marker.ns = "polylines";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub_.publish(marker);
    }

    void publishLinesAsMarker(const std::vector<Polyline>& lines,
                              const std_msgs::Header& header)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        if (!marker_frame_id_.empty()) {
            marker.header.frame_id = marker_frame_id_;
        }
        marker.ns = "polylines";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.03; // line width
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker.pose.orientation.w = 1.0;

        for (const auto& pl : lines) {
            if (pl.pts.size() < 2) continue;
            for (size_t i = 0; i + 1 < pl.pts.size(); ++i) {
                geometry_msgs::Point p0, p1;
                p0.x = pl.pts[i].x();
                p0.y = pl.pts[i].y();
                p0.z = 0.0;
                p1.x = pl.pts[i + 1].x();
                p1.y = pl.pts[i + 1].y();
                p1.z = 0.0;
                marker.points.push_back(p0);
                marker.points.push_back(p1);
            }
        }

        marker_pub_.publish(marker);
    }
};

// ---------------------------------- main ----------------------------------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "polyline_extraction_node");
    PolylineExtractionNode node;
    ros::spin();
    return 0;
}
