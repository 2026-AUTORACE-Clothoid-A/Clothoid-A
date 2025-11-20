#ifndef CONE_MATCHING_NODE_HPP
#define CONE_MATCHING_NODE_HPP

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <vector>

// 좌/우 콘 타입 (파이썬 ConeTypes 대응)
enum class ConeSide
{
    LEFT  = 0,
    RIGHT = 1
};

class ConeMatchingNode
{
public:
    explicit ConeMatchingNode(ros::NodeHandle& nh);

private:
    ros::NodeHandle nh_;

    // ===== 구독 =====
    ros::Subscriber left_boundary_sub_;
    ros::Subscriber right_boundary_sub_;

    // 최근에 받은 경계 Path
    nav_msgs::Path last_left_path_;
    nav_msgs::Path last_right_path_;
    bool left_received_;
    bool right_received_;

    // ===== RViz 퍼블리셔 =====
    // 가상 콘 포함 최종 경계
    ros::Publisher left_virtual_path_pub_;
    ros::Publisher right_virtual_path_pub_;
    // 매칭된 콘 쌍을 LineList로 시각화
    ros::Publisher match_lines_pub_;

    // ===== 파라미터 (스케일카 환경 기준, rosparam으로 튜닝) =====
    double min_track_width_;              // 최소 트랙 폭 [m]
    double max_search_range_;             // 매칭 검색 최대 거리 [m]
    double max_search_angle_deg_;         // 전방 시야각 [deg]
    bool   matches_should_be_monotonic_;  // 매칭 인덱스 단조 증가 제약 여부

    // 차량 상태 (현재는 base_link 원점 고정)
    Eigen::Vector2d car_pos_;  // (0,0)
    Eigen::Vector2d car_dir_;  // (1,0) 전방

    // ===== 파이썬 ConeMatchingInput 대응 =====
    struct ConeMatchingInput
    {
        std::vector<Eigen::Vector2d> sorted_left;   // 왼쪽 경계 (정렬된 콘)
        std::vector<Eigen::Vector2d> sorted_right;  // 오른쪽 경계 (정렬된 콘)
        Eigen::Vector2d slam_position;              // 차량 위치
        Eigen::Vector2d slam_direction;             // 차량 방향
    } input_;

    // ===== 파이썬 ConeMatchingState 대응 =====
    struct ConeMatchingState
    {
        double min_track_width = 0.0;
        double max_search_range = 0.0;
        double max_search_angle_rad = 0.0;
        bool   matches_should_be_monotonic = true;

        std::vector<Eigen::Vector2d> sorted_left;
        std::vector<Eigen::Vector2d> sorted_right;
        Eigen::Vector2d position_global;
        Eigen::Vector2d direction_global;
    } state_;

    // ===== Stage 2 결과 (원본 경계 기준) =====
    std::vector<int> matches_left_to_right_;     // 왼쪽 콘 기준 → 오른쪽 콘 인덱스
    std::vector<int> matches_right_to_left_;     // 오른쪽 콘 기준 → 왼쪽 콘 인덱스
    std::vector<Eigen::Vector2d> search_dirs_left_;
    std::vector<Eigen::Vector2d> search_dirs_right_;

    // ===== Stage 3·4 결과 (가상 콘 포함 최종 경계 & 매칭) =====
    std::vector<Eigen::Vector2d> left_cones_with_virtual_;
    std::vector<Eigen::Vector2d> right_cones_with_virtual_;
    std::vector<bool> left_mask_is_virtual_;
    std::vector<bool> right_mask_is_virtual_;
    std::vector<int> final_left_to_right_matches_;
    std::vector<int> final_right_to_left_matches_;

    // ===== 콜백 =====
    void leftBoundaryCallback(const nav_msgs::Path::ConstPtr& msg);
    void rightBoundaryCallback(const nav_msgs::Path::ConstPtr& msg);

    // 두 경계 모두 들어왔을 때 한 번 처리
    void processOnce();

    // ===== Stage 1: 입력 → Input 구조체 세팅 =====
    void setNewInputFromBoundaries();

    // ===== Stage 1: Input → State 전이 =====
    void transitionInputToState();

    // Stage1 디버그 출력
    void printStage1DebugInfo() const;

    // ===== Stage 2: 한쪽 경계 기준 매칭 후보 & 인덱스 계산 =====

    // 파이썬 calculate_match_search_direction
    std::vector<Eigen::Vector2d> calculateMatchSearchDirection(
        const std::vector<Eigen::Vector2d>& cones,
        ConeSide side) const;

    // 파이썬 find_boolean_mask_of_all_potential_matches
    std::vector<std::vector<bool>> findBooleanMaskOfAllPotentialMatches(
        const std::vector<Eigen::Vector2d>& start_points,
        const std::vector<Eigen::Vector2d>& directions,
        const std::vector<Eigen::Vector2d>& other_side_cones,
        const std::vector<Eigen::Vector2d>& other_side_directions,
        double major_radius,
        double minor_radius,
        double max_search_angle_rad) const;

    // 파이썬 select_best_match_candidate
    std::vector<int> selectBestMatchCandidate(
        const std::vector<Eigen::Vector2d>& matchable_cones,
        const std::vector<Eigen::Vector2d>& match_directions,
        const std::vector<std::vector<bool>>& match_boolean_mask,
        const std::vector<Eigen::Vector2d>& other_side_cones,
        bool matches_should_be_monotonic) const;

    // 파이썬 calculate_matches_for_side
    void calculateMatchesForSide(
        const std::vector<Eigen::Vector2d>& cones,
        ConeSide cone_side,
        const std::vector<Eigen::Vector2d>& other_side_cones,
        double major_radius,
        double minor_radius,
        double max_search_angle_rad,
        bool matches_should_be_monotonic,
        std::vector<int>& out_matches,
        std::vector<Eigen::Vector2d>& out_search_dirs) const;

    // Stage 2 전체 실행 (left/right 양쪽)
    void runStage2Matching();

    // ===== Stage 3: 가상 콘 위치 계산 & 삽입 =====

    // 파이썬 calculate_positions_of_virtual_cones
    std::vector<Eigen::Vector2d> calculatePositionsOfVirtualCones(
        const std::vector<Eigen::Vector2d>& cones,
        const std::vector<int>& indices_of_unmatched_cones,
        const std::vector<Eigen::Vector2d>& search_directions,
        double min_track_width) const;

    // 파이썬 trace_angles_between에 대응 (각 내부 포인트의 코너 각도)
    std::vector<double> traceAnglesBetween(
        const std::vector<Eigen::Vector2d>& cones) const;

    // 파이썬 insert_virtual_cones_to_existing
    std::vector<Eigen::Vector2d> insertVirtualConesToExisting(
        const std::vector<Eigen::Vector2d>& other_side_cones,
        const std::vector<Eigen::Vector2d>& other_side_virtual_cones,
        const Eigen::Vector2d& car_position) const;

    // 파이썬 calculate_insert_index_for_one_cone
    int calculateInsertIndexForOneCone(
        const Eigen::Vector2d& car_position,
        const std::vector<Eigen::Vector2d>& final_cones,
        const Eigen::Vector2d& virtual_cone) const;

    // 파이썬 calculate_insert_index_of_new_cone
    int calculateInsertIndexOfNewCone(
        int closest_index,
        int second_closest_index,
        bool cone_is_between_closest_two) const;

    // 파이썬 combine_and_sort_virtual_with_real
    void combineAndSortVirtualWithReal(
        const std::vector<Eigen::Vector2d>& other_side_cones,
        const std::vector<Eigen::Vector2d>& other_side_virtual_cones,
        const Eigen::Vector2d& car_pos,
        const Eigen::Vector2d& car_dir,
        std::vector<Eigen::Vector2d>& sorted_combined_cones,
        std::vector<bool>& mask_is_virtual) const;

    // 파이썬 calculate_cones_for_other_side
    void calculateConesForOtherSide(
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
        std::vector<bool>& mask_is_virtual) const;

    // ===== Stage 4: 양쪽 가상 콘 포함 후 최종 매칭 =====

    // 파이썬 match_both_sides_with_virtual_cones
    void matchBothSidesWithVirtualCones(
        const std::vector<Eigen::Vector2d>& left_cones_with_virtual,
        const std::vector<Eigen::Vector2d>& right_cones_with_virtual,
        double major_radius,
        double minor_radius,
        double max_search_angle_rad,
        bool matches_should_be_monotonic,
        std::vector<int>& left_to_right_matches,
        std::vector<int>& right_to_left_matches) const;

    // 파이썬 calculate_virtual_cones_for_both_sides
    void calculateVirtualConesForBothSides();

    // 최종 결과 RViz 퍼블리시
    void publishResults() const;

    // 전체 파이프라인 (Stage1~4)
    void runMatchingPipeline();
};

#endif // CONE_MATCHING_NODE_HPP
