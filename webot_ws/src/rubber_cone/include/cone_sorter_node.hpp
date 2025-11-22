#ifndef CONE_SORTER_NODE_HPP
#define CONE_SORTER_NODE_HPP

#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>
#include <utility>
#include <string>

namespace cone_sorter
{

class ConeSorterNode
{
public:
    explicit ConeSorterNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber obstacles_sub_;

    // 결과 경계를 다른 노드에서 쓸 수 있도록 퍼블리시
    ros::Publisher left_boundary_pub_;
    ros::Publisher right_boundary_pub_;

    // 미션 상태 퍼블리셔 (true: 미션 진행 중, false: 미션 아님)
    ros::Publisher mission_state_pub_;

    // ====== 미션 상태 ======
    enum class MissionState
    {
        IDLE,
        RUNNING
    };

    MissionState mission_state_;
    bool is_obstacles_;

    // ====== 파라미터들 (전부 rosparam으로 튜닝 가능하게) ======
    double max_dist_to_first_;         // 시작 콘으로 인정할 최대 거리 [m]
    double max_dist_between_cones_;    // 인접 콘 간 정상 거리 기준 [m]
    int    max_n_neighbors_;           // 인접 행렬에서 허용할 최대 이웃 수
    int    max_trace_length_;          // 한 트레이스(경계)에서 최대 콘 개수
    double threshold_directional_angle_deg_;   // 방향성 각도 임계값 [deg]
    double threshold_absolute_angle_deg_;      // 절대 각도 임계값 [deg]
    double ellipse_major_radius_factor_;       // 타원 장축 = factor * max_dist_between_cones_
    double ellipse_minor_radius_factor_;       // 타원 단축 = factor * max_dist_between_cones_
    double len_override_factor_;               // len_last_to_candidate < factor * max_dist_between_cones_
    double car_size_;                          // 차량 길이(충돌 체크용) [m]

    // --- 러버콘 스타일 첫 점 선택용 파라미터 ---
    double angle_from_left_first_cone_deg_; // 왼쪽 첫 콘 각도 기준 [deg]
    double dist_from_left_first_cone_;      // 왼쪽 첫 콘까지 허용 거리 [m]
    double dist_from_right_first_cone_;     // 오른쪽 첫 콘까지 허용 거리 [m]

    // --- 미션 판정 영역 파라미터 (base_link 기준) ---
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

    // ===== 비용 항목 구조체 =====
    struct TraceCostTerms
    {
        double angle_cost         = 0.0; // calc_angle_cost_for_configuration
        double residual_dist_cost = 0.0; // calc_distance_cost
        double num_cones_cost     = 0.0; // 1 / N
        double initial_dir_cost   = 0.0; // calc_initial_direction_cost
        double change_dir_cost    = 0.0; // calc_change_of_direction_cost
        double cones_side_cost    = 0.0; // calc_cones_on_either_cost
        double wrong_dir_cost     = 0.0; // calc_wrong_direction_cost
    };

    // Stage4: 비용 함수 가중치 (나중에 필요하면 조절)
    double w_angle_cost_;
    double w_residual_dist_cost_;
    double w_num_cones_cost_;
    double w_initial_dir_cost_;
    double w_change_dir_cost_;
    double w_cones_side_cost_;
    double w_wrong_dir_cost_;

    // ---- 각 항목에서 사용할 추가 파라미터 ----
    double angle_cost_threshold_deg_;        // 각도 < 이 값이면 "꺾인 구간"으로 간주 (기본 40deg)
    double residual_dist_threshold_;         // 정상적인 콘 간격 기준 (m)
    double wrong_dir_threshold_deg_;         // wrong_direction_cost 에서 사용할 각도 임계값
    double cones_side_search_dist_;          // 주변 콘 검색 거리 (m)
    double cones_side_search_angle_deg_;     // 주변 콘 검색 각도 (deg)

    // base_link 기준 차량 자세 (지금은 고정: 원점, x+ 정면)
    Eigen::Vector2d car_pos_;
    Eigen::Vector2d car_dir_;

    // 최신 콘 위치들 (base_link 좌표계, circles center)
    std::vector<Eigen::Vector2d> cones_;

    // 왼/오른쪽 경계를 나눌 때 쓸 enum
    enum class Side
    {
        LEFT = +1,
        RIGHT = -1
    };

    // ====== ROS 콜백 및 전체 파이프라인 실행 ======
    void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr& msg);
    void processOnce();

    // ====== 미션 상태 판정 ======
    bool checkMissionStartCondition() const;
    bool checkMissionContinueCondition() const;

    // ====== 1단계: 시작 콘 선택 ======
    // (기존 함수들: 폴백용으로 남겨둠)
    void maskConeCanBeFirst(
        Side side,
        std::vector<double>& distances,
        std::vector<bool>& mask_valid) const;

    int selectStartingCone(
        Side side,
        const std::vector<int>* indices_to_skip,
        const std::vector<double>& distances,
        const std::vector<bool>& mask_valid) const;

    std::vector<int> selectFirstKStartingCones(Side side) const;

    // --- 러버콘 로직 기반 새 시작 콘 선택 ---
    std::vector<int> extractSideIndices(Side side) const;
    bool selectFirstConeIndex(Side side, int& out_idx) const;

    // ====== 2단계: 거리 행렬 / 인접 행렬 / BFS ======
    Eigen::MatrixXd computeDistanceMatrix(
        const std::vector<Eigen::Vector2d>& points) const;

    std::vector<std::vector<int>> findKClosestInPointCloud(
        const Eigen::MatrixXd& dist_sq,
        int k) const;

    void createAdjacencyMatrix(
        const std::vector<Eigen::Vector2d>& points,
        int n_neighbors,
        int start_idx,
        double max_dist,
        Eigen::MatrixXi& adjacency_out,
        std::vector<int>& reachable_out) const;

    std::vector<int> breadthFirstOrder(
        const Eigen::MatrixXi& adjacency,
        int start_idx) const;

    // ====== 3단계: DFS 기반 end configurations(경계 후보) 탐색 ======
    struct TraceSearchResult
    {
        // 각 벡터가 하나의 경계 후보 (콘 인덱스 시퀀스)
        std::vector<std::vector<int>> end_configs;
    };

    TraceSearchResult findAllEndConfigurations(
        const std::vector<Eigen::Vector2d>& points,
        const Eigen::MatrixXi& adjacency,
        int start_idx,
        int reachable_count,
        Side side) const;

    bool canAddNeighbor(
        const std::vector<Eigen::Vector2d>& points,
        const std::vector<int>& current_attempt,
        int depth,
        int candidate_idx,
        Side side,
        double threshold_dir_angle_rad,
        double threshold_abs_angle_rad) const;

    bool isInsideLocalEllipse(
        const Eigen::Vector2d& second_last,
        const Eigen::Vector2d& last,
        const Eigen::Vector2d& candidate) const;

    bool secondConeOnExpectedSide(
        const Eigen::Vector2d& candidate,
        Side side) const;

    bool segmentsIntersect(
        const Eigen::Vector2d& p1,
        const Eigen::Vector2d& p2,
        const Eigen::Vector2d& q1,
        const Eigen::Vector2d& q2,
        double eps = 1e-6) const;

    bool segmentIntersectsCar(
        const Eigen::Vector2d& p0,
        const Eigen::Vector2d& p1) const;

    // 유틸: 각도 정규화 / 차이
    double normalizeAngle(double a) const;
    double angleDifference(double a, double b) const;

    // ====== 4단계: 비용 함수 + 최종 경계 선택 & 퍼블리시 ======

    // ---- 세분화된 비용 계산 함수 ----
    TraceCostTerms computeTraceCostTerms(
        const std::vector<Eigen::Vector2d>& points,
        const std::vector<int>& trace,
        Side side) const;

    double computeWeightedTraceCost(const TraceCostTerms& t) const;

    // 세부 항목별 내부 계산 함수들
    double angleCostTerm(
        const std::vector<Eigen::Vector2d>& points,
        const std::vector<int>& trace) const;

    double residualDistanceCostTerm(
        const std::vector<Eigen::Vector2d>& points,
        const std::vector<int>& trace) const;

    double numberOfConesCostTerm(
        const std::vector<int>& trace) const;

    double initialDirectionCostTerm(
        const std::vector<Eigen::Vector2d>& points,
        const std::vector<int>& trace) const;

    double changeOfDirectionCostTerm(
        const std::vector<Eigen::Vector2d>& points,
        const std::vector<int>& trace) const;

    double conesOnEitherSideCostTerm(
        const std::vector<Eigen::Vector2d>& points,
        const std::vector<int>& trace,
        Side side) const;

    double wrongDirectionCostTerm(
        const std::vector<Eigen::Vector2d>& points,
        const std::vector<int>& trace,
        Side side) const;

    void publishBoundary(
        const std::vector<Eigen::Vector2d>& pts,
        const ros::Publisher& pub,
        const std::string& frame_id) const;
};

} // namespace cone_sorter

#endif // CONE_SORTER_NODE_HPP
