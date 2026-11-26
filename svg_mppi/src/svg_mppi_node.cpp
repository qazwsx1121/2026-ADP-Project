// svg_mppi_node.cpp
//
// ROS 2 (Humble) wrapper that runs the OFFICIAL SVG-MPPI solver
// (kohonda/proj-svg_mppi, ICRA 2024) inside the ForzaETH race stack.
//
// This one node replaces the three ROS 1 nodes of the official repo:
//   reference_sdf_generator  -> build_reference_sdf()   (ported 1:1)
//   local_costmap_generator  -> build_local_costmap()   (ported 1:1, PCL-free)
//   mppi_controller_ros      -> control_loop()          (ported 1:1)
// The solver itself (mpc_base / forward_mppi / svg_mppi ...) is compiled
// unmodified from the official repository (see CMakeLists.txt).
//
// [추가 2026-11-19] (공식 solver 는 그대로, 이 연결 노드에서만)
//   1) guide.*  : 장애물 순서를 고려한 가이드 기준 경로 (기본 꺼짐, guide.enable)
//      라이다 점 중 정적 지도(/map) 벽이 아닌 것 = 장애물 -> 레이스라인 Frenet 좌표로 투영
//      -> 앞쪽 lookahead 구간 전체를 DP 로 풀어 "어느 쪽으로 지나갈지" 순서를 한 번에 정함
//      -> 그만큼 옆으로 옮긴 레이스라인으로 reference SDF 를 만듦 (= SVG 의 비용 지형을 그 방향으로 유도)
//   2) 주행 중 파라미터 변경 (ros2 param set /svg_mppi ...)
//      - solver 파라미터(common.*, svg_mppi.*, forward_mppi.* ...): 새 solver 를 백그라운드에서 만들어
//        제어 주기 사이에 교체 (solver 는 항상 1개)
//      - 노드 파라미터(reference_sdf.*, reference_speed, constant_speed_mode, guide.*): 즉시 반영
//
// I/O (defaults match the race stack, same as TLN):
//   in : /scan (LaserScan), /car_state/odom (Odometry, pf or gt by pose_source),
//        /global_waypoints_scaled (f110_msgs/WpntArray, speed_scaling applied), /map (static map, guide only)
//   out: /drive (AckermannDriveStamped)   only while drive_publish_enabled=true
//   viz: /mppi/best_path, /mppi/candidate_paths, /mppi/local_costmap, /mppi/reference_path,
//        /mppi/calculation_time [ms], /mppi/collision_rate, /mppi/cost, /mppi/speed,
//        /mppi/guide_path, /mppi/guide_obstacles, /mppi/guide_offset

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <grid_map_core/grid_map_core.hpp>

#include "mppi_controller/common.hpp"
#include "mppi_controller/forward_mppi.hpp"
#include "mppi_controller/mpc_template.hpp"
#include "mppi_controller/reverse_mppi.hpp"
#include "mppi_controller/stein_variational_guided_mppi.hpp"
#include "mppi_controller/stein_variational_mpc.hpp"

using namespace std::chrono_literals;

namespace {
// layer names used inside the official solver (mpc_base.hpp)
const std::string kCollisionLayer = "collision_layer";
const std::string kDistanceLayer = "distance_field";
const std::string kAngleLayer = "angle_field";
const std::string kSpeedLayer = "speed_field";

double yaw_from_quat(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double steady_sec() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Pose2D {
  double x = 0.0, y = 0.0, yaw = 0.0, vel = 0.0;
  double s = 0.0, psi = 0.0, dl = 1.0, dr = 1.0;  // raceline only: arc length, tangent, track bounds
  double kappa = 0.0;                              // raceline only: curvature [1/m] (from the global planner)
};

// static map distance field (meters to the nearest wall / unknown cell)
struct StaticMap {
  double ox = 0.0, oy = 0.0, res = 0.05;
  int w = 0, h = 0;
  std::vector<float> dist;
  float at(double x, double y) const {
    const int c = static_cast<int>(std::floor((x - ox) / res));
    const int r = static_cast<int>(std::floor((y - oy) / res));
    if (c < 0 || r < 0 || c >= w || r >= h) return 0.0f;
    return dist[static_cast<size_t>(r) * w + c];
  }
};

// live-tunable guide settings (copied once per sdf cycle)
struct GuideCfg {
  bool enable = false;
  double lookahead = 6.0;      // [m] obstacle sequence window ahead of the car
  double ds = 0.25;            // [m] DP station spacing
  double dd = 0.05;            // [m] DP lateral resolution
  double wall_clear = 0.30;    // [m] reference must stay this far from static walls
  double obs_clear_d = 0.45;   // [m] lateral center distance raceline-offset <-> obstacle point (blocked below)
  double obs_clear_s = 0.45;   // [m] longitudinal extent of the blocked zone around an obstacle point
  double obs_depth = 0.30;     // [m] obstacle assumed to extend this far behind its visible face
  double soft_margin = 0.25;   // [m] extra lateral margin with a soft cost
  double w_ref = 1.0;          // stay near raceline
  double w_smooth = 3.0;       // lateral slope
  double w_prev = 2.0;         // stick to previous plan (hysteresis: no side flip-flop)
  double w_obs = 20.0;         // soft margin weight
  double w_side = 30.0;        // passing an obstacle on the other side than the previous plan
  double max_slope = 0.8;      // [m/m] max lateral change per meter
  double static_margin = 0.15; // [m] scan point is an obstacle if this far from any static wall
  double memory = 1.0;         // [s] keep obstacle cells this long after last seen
  double blend = 1.5;          // [m] return to raceline after the window
  double speed_scale = 1.0;    // speed factor where the reference is shifted (|offset| > 0.1 m)
  double max_range = 6.5;      // [m] ignore scan points beyond this
  // [11/22] generic rule (no track coordinates): pause the guide where the raceline itself is very tight.
  // In a hairpin, shifting the reference sideways sends the car wide into walls, so the plain raceline is safer there.
  double max_curvature = 0.0;  // [1/m] pause guide if max |kappa| of the raceline near the car exceeds this (0 = never)
  double curv_ahead = 2.0;     // [m] raceline window ahead of the car for that check
  double curv_back = 0.5;      // [m] ... and behind
  // [11/26] ...but not while avoiding: if obstacle points are within this distance ahead of the car (scan-based),
  // the guide keeps running through the tight part. Pausing there snapped the reference back onto the obstacle
  // it was passing (hairpin-entrance crashes, seed 42/5). 0 = always pause (11/22 behaviour).
  double curv_keep_obs = 2.5;  // [m]
};

struct ObsPt {
  double s, d, x, y;
};
}  // namespace

class SVGMPPINode : public rclcpp::Node {
 public:
  SVGMPPINode() : Node("svg_mppi") {
    // ---------------- topics / switches ----------------
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/car_state/odom");
    waypoints_topic_ = declare_parameter<std::string>("waypoints_topic", "/global_waypoints_scaled");
    drive_topic_ = declare_parameter<std::string>("drive_topic", "/drive");
    map_frame_ = declare_parameter<std::string>("map_frame_id", "map");
    robot_frame_param_ = declare_parameter<std::string>("robot_frame_id", "");  // "" = odom child_frame_id (e.g. car_state/base_link)
    drive_publish_enabled_ = declare_parameter<bool>("drive_publish_enabled", false);
    laser_offset_x_ = declare_parameter<double>("laser_offset_x", 0.275);  // used only if TF base_link<-laser fails
    visualize_candidates_ = declare_parameter<bool>("visualize_candidates", false);
    publish_costmap_every_ = declare_parameter<int>("publish_costmap_every", 4);
    static_map_topic_ = declare_parameter<std::string>("static_map_topic", "/map");
    debug_dump_dir_ = declare_parameter<std::string>("debug.dump_dir", "");  // [진단] 비우면 꺼짐
    debug_flip_thresh_ = declare_parameter<double>("debug.flip_threshold", 0.12);
    debug_num_candidates_ = declare_parameter<int>("debug.num_candidates", 400);

    // ---------------- mppi_controller.yaml (official names) ----------------
    control_sampling_time_ = declare_parameter<double>("control_sampling_time", 0.025);
    constant_speed_mode_ = declare_parameter<bool>("constant_speed_mode", false);
    reference_speed_ = declare_parameter<double>("reference_speed", 1.5);
    steer_1st_delay_param_ = declare_parameter<double>("steer_1st_delay", 0.1);
    mpc_mode_ = declare_parameter<std::string>("mpc_mode", "svg_mppi");

    mppi::Params& p = params_;
    p.common.is_localize_less_mode = false;
    p.common.steer_1st_delay = steer_1st_delay_param_;
    p.common.reference_speed = reference_speed_;
    p.common.q_speed = 0.0;
    p.common.q_terminal_speed = 0.0;
#define SP(T, NAME, DEF, FIELD) sp<T>(NAME, DEF, [](mppi::Params& q, T v) { q.FIELD = v; })
    SP(int, "common.thread_num", 12, common.thread_num);
    SP(int, "common.prediction_step_size", 15, common.prediction_step_size);
    SP(double, "common.prediction_interval", 0.05, common.prediction_interval);
    SP(double, "common.steer_delay", 0.025, common.steer_delay);
    SP(double, "common.max_steer_angle", 0.45, common.max_steer_angle);
    SP(double, "common.min_steer_angle", -0.45, common.min_steer_angle);
    SP(double, "common.max_accel", 5.0, common.max_accel);
    SP(double, "common.min_accel", -3.0, common.min_accel);
    SP(std::string, "common.speed_prediction_mode", std::string("reference"), common.speed_prediction_mode);
    SP(double, "common.lr", 0.135, common.lr);
    SP(double, "common.lf", 0.189, common.lf);
    SP(double, "common.q_dist", 1.0, common.q_dist);
    SP(double, "common.q_angle", 0.01, common.q_angle);
    SP(double, "common.collision_weight", 1.0, common.collision_weight);
    SP(double, "common.q_terminal_dist", 1.0, common.q_terminal_dist);
    SP(double, "common.q_terminal_angle", 0.01, common.q_terminal_angle);

    SP(int, "forward_mppi.sample_batch_num", 10000, forward_mppi.sample_batch_num);
    SP(double, "forward_mppi.lambda", 3.0, forward_mppi.lambda);
    SP(double, "forward_mppi.alpha", 0.1, forward_mppi.alpha);
    SP(double, "forward_mppi.non_biased_sampling_rate", 0.1, forward_mppi.non_biased_sampling_rate);
    SP(double, "forward_mppi.steer_cov", 0.1, forward_mppi.steer_cov);
    SP(int, "forward_mppi.num_itr_for_grad_estimation", 0, forward_mppi.num_itr_for_grad_estimation);
    SP(double, "forward_mppi.step_size_for_grad_estimation", 0.001, forward_mppi.step_size_for_grad_estimation);
    SP(int, "forward_mppi.sample_num_for_grad_estimation", 5, forward_mppi.sample_num_for_grad_estimation);
    SP(double, "forward_mppi.steer_cov_for_grad_estimation", 0.001, forward_mppi.steer_cov_for_grad_estimation);

    SP(int, "reverse_mppi.sample_batch_num", 200, reverse_mppi.sample_batch_num);
    SP(double, "reverse_mppi.negative_ratio", 1.0, reverse_mppi.negative_ratio);
    SP(bool, "reverse_mppi.is_sample_rejection", true, reverse_mppi.is_sample_rejection);
    SP(double, "reverse_mppi.sample_inflation_ratio", 2.0, reverse_mppi.sample_inflation_ratio);
    SP(int, "reverse_mppi.iteration_num", 50, reverse_mppi.iteration_num);
    SP(double, "reverse_mppi.step_size", 0.05, reverse_mppi.step_size);
    SP(double, "reverse_mppi.warm_start_ratio", 0.5, reverse_mppi.warm_start_ratio);
    SP(double, "reverse_mppi.lambda", 3.0, reverse_mppi.lambda);
    SP(double, "reverse_mppi.alpha", 0.1, reverse_mppi.alpha);
    SP(double, "reverse_mppi.non_biased_sampling_rate", 0.1, reverse_mppi.non_biased_sampling_rate);
    SP(double, "reverse_mppi.steer_cov", 0.1, reverse_mppi.steer_cov);

    SP(int, "stein_variational_mpc.sample_batch_num", 500, stein_variational_mpc.sample_batch_num);
    SP(double, "stein_variational_mpc.lambda", 3.0, stein_variational_mpc.lambda);
    SP(double, "stein_variational_mpc.alpha", 0.1, stein_variational_mpc.alpha);
    SP(double, "stein_variational_mpc.non_biased_sampling_rate", 0.1, stein_variational_mpc.non_biased_sampling_rate);
    SP(double, "stein_variational_mpc.steer_cov", 0.1, stein_variational_mpc.steer_cov);
    SP(int, "stein_variational_mpc.num_svgd_iteration", 3, stein_variational_mpc.num_svgd_iteration);
    SP(int, "stein_variational_mpc.sample_num_for_grad_estimation", 10, stein_variational_mpc.sample_num_for_grad_estimation);
    SP(double, "stein_variational_mpc.steer_cov_for_grad_estimation", 0.01, stein_variational_mpc.steer_cov_for_grad_estimation);
    SP(double, "stein_variational_mpc.svgd_step_size", 0.5, stein_variational_mpc.svgd_step_size);
    SP(bool, "stein_variational_mpc.is_max_posterior_estimation", false, stein_variational_mpc.is_max_posterior_estimation);

    SP(int, "svg_mppi.sample_batch_num", 8000, svg_mppi.sample_batch_num);
    SP(double, "svg_mppi.lambda", 3.0, svg_mppi.lambda);
    SP(double, "svg_mppi.alpha", 0.1, svg_mppi.alpha);
    SP(double, "svg_mppi.non_biased_sampling_rate", 0.1, svg_mppi.non_biased_sampling_rate);
    SP(double, "svg_mppi.steer_cov", 0.01, svg_mppi.steer_cov);
    SP(int, "svg_mppi.guide_sample_num", 1, svg_mppi.guide_sample_num);
    SP(double, "svg_mppi.grad_lambda", 3.0, svg_mppi.grad_lambda);
    SP(int, "svg_mppi.sample_num_for_grad_estimation", 100, svg_mppi.sample_num_for_grad_estimation);
    SP(double, "svg_mppi.steer_cov_for_grad_estimation", 0.01, svg_mppi.steer_cov_for_grad_estimation);
    SP(double, "svg_mppi.svgd_step_size", 0.005, svg_mppi.svgd_step_size);
    SP(int, "svg_mppi.num_svgd_iteration", 10, svg_mppi.num_svgd_iteration);
    SP(bool, "svg_mppi.is_use_nominal_solution", true, svg_mppi.is_use_nominal_solution);
    SP(bool, "svg_mppi.is_covariance_adaptation", true, svg_mppi.is_covariance_adaptation);
    SP(double, "svg_mppi.gaussian_fitting_lambda", 0.1, svg_mppi.gaussian_fitting_lambda);
    SP(double, "svg_mppi.min_steer_cov", 0.001, svg_mppi.min_steer_cov);
    SP(double, "svg_mppi.max_steer_cov", 0.1, svg_mppi.max_steer_cov);
#undef SP

    solver_ = make_solver(params_, mpc_mode_);
    thread_num_ = p.common.thread_num;
    steer_1st_delay_ = std::max(steer_1st_delay_param_, p.common.prediction_interval);  // same as official

    // ---------------- reference_sdf_generator.yaml ----------------
    ref_num_waypoints_ = declare_parameter<int>("reference_sdf.num_waypoints", 80);
    ref_backward_margin_num_ = declare_parameter<int>("reference_sdf.backward_margin_num", 0);
    ref_waypoint_interval_ = declare_parameter<double>("reference_sdf.waypoint_interval", 0.1);
    ref_map_resolution_ = declare_parameter<double>("reference_sdf.ref_path_map_resolution", 0.1);
    ref_map_width_ = declare_parameter<double>("reference_sdf.ref_path_map_width", 20.0);
    ref_map_height_ = declare_parameter<double>("reference_sdf.ref_path_map_height", 20.0);
    ref_speed_scale_ = declare_parameter<double>("reference_sdf.reference_speed_scale", 1.0);
    ref_max_speed_ = declare_parameter<double>("reference_sdf.max_speed", 100.0);
    ref_update_rate_ = declare_parameter<double>("reference_sdf.update_rate", 0.025);
    // [11/24] reference SDF is cheap: give it fewer OpenMP threads so localisation (EKF, particle filter) keeps CPU
    ref_threads_ = declare_parameter<int>("reference_sdf.threads", 2);
    // [11/24] pose latency compensation: the localisation pose is ~0.05-0.1 s old when it arrives (pf). Predict it forward
    // to "now" with the reported velocity and yaw rate before using it as the MPPI initial state.
    pose_latency_comp_ = declare_parameter<bool>("pose_latency_comp", true);
    pose_latency_max_ = declare_parameter<double>("pose_latency_max", 0.4);
    // [추가] 최근접 waypoint 검색 방식. false = 공식 코드(전체 경로에서 최근접), true = 직전 위치 주변만 검색
    ref_local_search_ = declare_parameter<bool>("reference_sdf.local_search", false);
    ref_search_back_ = declare_parameter<int>("reference_sdf.local_search_back_num", 10);
    ref_search_fwd_ = declare_parameter<int>("reference_sdf.local_search_forward_num", 40);
    ref_jump_warn_num_ = declare_parameter<int>("reference_sdf.jump_warn_num", 30);
    reference_sdf_ = grid_map::GridMap({kDistanceLayer, kAngleLayer, kSpeedLayer});
    reference_sdf_.setFrameId(map_frame_);
    sdf_work_ = reference_sdf_;

    // ---------------- [추가] guide (obstacle-sequence-aware reference) ----------------
    GuideCfg& g = guide_cfg_;
    g.enable = declare_parameter<bool>("guide.enable", g.enable);
    guide_d_ = {{"guide.lookahead", &GuideCfg::lookahead},       {"guide.ds", &GuideCfg::ds},
                {"guide.dd", &GuideCfg::dd},                     {"guide.wall_clear", &GuideCfg::wall_clear},
                {"guide.obs_clear_d", &GuideCfg::obs_clear_d},   {"guide.obs_clear_s", &GuideCfg::obs_clear_s},
                {"guide.obs_depth", &GuideCfg::obs_depth},       {"guide.soft_margin", &GuideCfg::soft_margin},
                {"guide.w_ref", &GuideCfg::w_ref},               {"guide.w_smooth", &GuideCfg::w_smooth},
                {"guide.w_prev", &GuideCfg::w_prev},             {"guide.w_obs", &GuideCfg::w_obs},
                {"guide.w_side", &GuideCfg::w_side},
                {"guide.max_slope", &GuideCfg::max_slope},       {"guide.static_margin", &GuideCfg::static_margin},
                {"guide.memory", &GuideCfg::memory},             {"guide.blend", &GuideCfg::blend},
                {"guide.speed_scale", &GuideCfg::speed_scale},   {"guide.max_range", &GuideCfg::max_range},
                {"guide.max_curvature", &GuideCfg::max_curvature}, {"guide.curv_ahead", &GuideCfg::curv_ahead},
                {"guide.curv_back", &GuideCfg::curv_back},       {"guide.curv_keep_obs", &GuideCfg::curv_keep_obs}};
    for (auto& [name, field] : guide_d_) g.*field = declare_parameter<double>(name, g.*field);

    // ---------------- local_costmap_generator_simulation.yaml ----------------
    cm_crop_robot_ = declare_parameter<bool>("local_costmap.is_crop_robot", true);
    bl2front_ = declare_parameter<double>("local_costmap.baselink2front", 0.47);
    bl2rear_ = declare_parameter<double>("local_costmap.baselink2rear", 0.14);
    bl2right_ = declare_parameter<double>("local_costmap.baselink2right", 0.15);
    bl2left_ = declare_parameter<double>("local_costmap.baselink2left", 0.15);
    const double mx = declare_parameter<double>("local_costmap.map_x_length", 10.0);
    const double my = declare_parameter<double>("local_costmap.map_y_length", 10.0);
    const double ox = declare_parameter<double>("local_costmap.map_center_offset_x", 3.0);
    const double oy = declare_parameter<double>("local_costmap.map_center_offset_y", 0.0);
    const double mr = declare_parameter<double>("local_costmap.map_resolution", 0.05);
    cm_max_val_ = declare_parameter<double>("local_costmap.max_val", 100.0);
    cm_inflation_circle_ = declare_parameter<std::string>("local_costmap.inflation_mode", "circle") == "circle";  // [변경 11/19] 공식 rigid
    cm_circle_radius_ = declare_parameter<double>("local_costmap.circle_radius", 0.15);
    cm_filter_isolated_ = declare_parameter<bool>("local_costmap.filter_isolated", false);  // [추가] 라이다 허상 점 제거
    cm_filter_radius_ = declare_parameter<double>("local_costmap.filter_radius", 0.10);
    cost_map_ = grid_map::GridMap({kCollisionLayer});
    cost_map_.setGeometry(grid_map::Length(mx, my), mr, grid_map::Position(ox, oy));

    // ---------------- ROS I/O ----------------
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mtx_);
          scan_ = m;
        });
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {
      std::lock_guard<std::mutex> lk(mtx_);
      const auto& q = m->pose.pose.orientation;
      robot_.x = m->pose.pose.position.x;
      robot_.y = m->pose.pose.position.y;
      robot_.yaw = wrap(yaw_from_quat(q.x, q.y, q.z, q.w));
      robot_.vel = m->twist.twist.linear.x;
      robot_yaw_rate_ = m->twist.twist.angular.z;
      robot_stamp_ = rclcpp::Time(m->header.stamp, get_clock()->get_clock_type());
      // [11/24] latency at arrival (stamp -> now) and arrival time on the steady clock. The system clock can jump
      // (WSL2: +2.94 s every ~32 s, the car does not move meanwhile), so the pose age is measured on the steady clock.
      const double lat = (now() - robot_stamp_).seconds();
      robot_recv_latency_ = (lat > 0.0 && lat < 1.0) ? lat : 0.0;  // >= 1 s: the clock jumped in between
      robot_recv_steady_ = steady_sec();
      // [guide] pose history so that scan points are projected with the pose at the scan time
      pose_hist_.push_back({rclcpp::Time(m->header.stamp).seconds(), robot_.x, robot_.y, robot_.yaw});
      while (pose_hist_.size() > 200) pose_hist_.pop_front();
      odom_frame_ = m->header.frame_id;
      if (robot_frame_.empty()) robot_frame_ = robot_frame_param_.empty() ? m->child_frame_id : robot_frame_param_;
      is_odom_ok_ = true;
    });
    sub_wpnts_ = create_subscription<f110_msgs::msg::WpntArray>(waypoints_topic_, 10,
                                                                [this](f110_msgs::msg::WpntArray::SharedPtr m) { on_waypoints(*m); });
    sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        static_map_topic_, rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) { on_static_map(*m); });

    pub_drive_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 1);
    pub_best_path_ = create_publisher<nav_msgs::msg::Path>("/mppi/best_path", 1);
    pub_candidates_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mppi/candidate_paths", 1);
    pub_costmap_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/mppi/local_costmap", 1);
    pub_calc_time_ = create_publisher<std_msgs::msg::Float32>("/mppi/calculation_time", 1);
    pub_coll_rate_ = create_publisher<std_msgs::msg::Float32>("/mppi/collision_rate", 1);
    pub_cost_ = create_publisher<std_msgs::msg::Float32>("/mppi/cost", 1);
    pub_speed_ = create_publisher<std_msgs::msg::Float32>("/mppi/speed", 1);
    pub_ref_path_ = create_publisher<nav_msgs::msg::Path>("/mppi/reference_path", 1);
    pub_guide_path_ = create_publisher<nav_msgs::msg::Path>("/mppi/guide_path", 1);
    pub_guide_obs_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mppi/guide_obstacles", 1);
    pub_guide_offset_ = create_publisher<std_msgs::msg::Float32>("/mppi/guide_offset", 1);

    param_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& ps) { return on_params(ps); });

    cg_control_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cg_sdf_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cg_rebuild_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = create_wall_timer(std::chrono::duration<double>(control_sampling_time_), [this]() { control_loop(); }, cg_control_);
    timer_sdf_ = create_wall_timer(std::chrono::duration<double>(ref_update_rate_), [this]() { sdf_loop(); }, cg_sdf_);
    timer_rebuild_ = create_wall_timer(50ms, [this]() { rebuild_loop(); }, cg_rebuild_);
    RCLCPP_INFO(get_logger(), "SVG-MPPI ready: mode=%s, %.0f Hz, threads=%d, drive_publish_enabled=%s, guide=%s, inflation=%s%s", mpc_mode_.c_str(),
                1.0 / control_sampling_time_, thread_num_.load(), drive_publish_enabled_.load() ? "true" : "false",
                guide_cfg_.enable ? "on" : "off", cm_inflation_circle_ ? "circle r=" : "rigid",
                cm_inflation_circle_ ? std::to_string(cm_circle_radius_).substr(0, 4).c_str() : "");
  }

 private:
  // ======================================================================
  // solver construction / live parameters
  // ======================================================================
  using Setter = std::function<void(mppi::Params&, const rclcpp::Parameter&)>;

  template <class T, class F>
  void sp(const std::string& name, const T& def, F set) {
    const T v = declare_parameter<T>(name, def);
    set(params_, v);
    solver_setters_[name] = [set](mppi::Params& q, const rclcpp::Parameter& prm) { set(q, prm.get_value<T>()); };
  }

  std::unique_ptr<mppi::cpu::MPCTemplate> make_solver(const mppi::Params& p, const std::string& mode) {
    if (mode == "forward_mppi") return std::make_unique<mppi::cpu::ForwardMPPI>(p.common, p.forward_mppi);
    if (mode == "reverse_mppi") return std::make_unique<mppi::cpu::ReverseMPPI>(p.common, p.reverse_mppi);
    if (mode == "sv_mpc") return std::make_unique<mppi::cpu::SteinVariationalMPC>(p.common, p.stein_variational_mpc);
    if (mode == "svg_mppi") return std::make_unique<mppi::cpu::SVGuidedMPPI>(p.common, p.svg_mppi);
    RCLCPP_FATAL(get_logger(), "Invalid mpc_mode: %s", mode.c_str());
    throw std::runtime_error("invalid mpc_mode");
  }

  rcl_interfaces::msg::SetParametersResult on_params(const std::vector<rclcpp::Parameter>& ps) {
    rcl_interfaces::msg::SetParametersResult r;
    r.successful = true;
    // 1) type check first (rclcpp checks after callbacks) so that nothing is applied half-way
    for (const auto& q : ps) {
      if (!has_parameter(q.get_name())) continue;
      if (get_parameter(q.get_name()).get_type() != q.get_type()) {
        r.successful = false;
        r.reason = q.get_name() + ": wrong type (e.g. write 5.0 instead of 5 for a double)";
        return r;
      }
    }
    bool rebuild = false;
    for (const auto& q : ps) {
      const std::string& n = q.get_name();
      if (n == "drive_publish_enabled") drive_publish_enabled_ = q.as_bool();
      else if (n == "visualize_candidates") visualize_candidates_ = q.as_bool();
      else if (n == "reference_sdf.local_search") ref_local_search_ = q.as_bool();
      else if (n == "reference_sdf.num_waypoints") ref_num_waypoints_ = std::max<int>(2, q.as_int());
      else if (n == "reference_sdf.reference_speed_scale") ref_speed_scale_ = q.as_double();
      else if (n == "reference_sdf.max_speed") ref_max_speed_ = q.as_double();
      else if (n == "reference_speed") reference_speed_ = q.as_double();
      else if (n == "constant_speed_mode") constant_speed_mode_ = q.as_bool();
      else if (n == "pose_latency_comp") pose_latency_comp_ = q.as_bool();
      else if (n == "pose_latency_max") pose_latency_max_ = q.as_double();
      else if (n == "steer_1st_delay") {
        std::lock_guard<std::mutex> lk(params_mtx_);
        steer_1st_delay_param_ = q.as_double();
        params_.common.steer_1st_delay = steer_1st_delay_param_;
        rebuild = true;
      } else if (n == "guide.enable") {
        std::lock_guard<std::mutex> lk(cfg_mtx_);
        guide_cfg_.enable = q.as_bool();
      } else if (guide_d_.count(n)) {
        std::lock_guard<std::mutex> lk(cfg_mtx_);
        guide_cfg_.*guide_d_.at(n) = q.as_double();
      } else if (solver_setters_.count(n)) {
        std::lock_guard<std::mutex> lk(params_mtx_);
        solver_setters_.at(n)(params_, q);
        rebuild = true;
      } else {
        continue;
      }
      RCLCPP_INFO(get_logger(), "param %s = %s%s", n.c_str(), q.value_to_string().c_str(), rebuild ? "  (solver rebuild queued)" : "");
    }
    if (rebuild) rebuild_requested_ = true;
    return r;
  }

  // builds a new solver off the control thread; control_loop swaps it in between two cycles
  void rebuild_loop() {
    if (!rebuild_requested_.exchange(false)) return;
    mppi::Params p;
    {
      std::lock_guard<std::mutex> lk(params_mtx_);
      p = params_;
    }
    const auto t0 = std::chrono::steady_clock::now();
    auto s = make_solver(p, mpc_mode_);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::lock_guard<std::mutex> lk(swap_mtx_);
    next_solver_ = std::move(s);
    next_params_ = p;
    RCLCPP_INFO(get_logger(), "new solver built in %.1f ms (samples=%d, horizon=%d x %.3f s, lambda=%.2f, steer_cov=%.4f)", ms,
                mpc_mode_ == "forward_mppi" ? p.forward_mppi.sample_batch_num : p.svg_mppi.sample_batch_num,
                p.common.prediction_step_size, p.common.prediction_interval,
                mpc_mode_ == "forward_mppi" ? p.forward_mppi.lambda : p.svg_mppi.lambda,
                mpc_mode_ == "forward_mppi" ? p.forward_mppi.steer_cov : p.svg_mppi.steer_cov);
  }

  // ======================================================================
  // waypoints  (stands in for reference_waypoint_loader)
  // ======================================================================
  void on_waypoints(const f110_msgs::msg::WpntArray& msg) {
    if (msg.wpnts.size() < 3) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "received empty waypoints");
      return;
    }
    std::vector<Pose2D> wps;
    wps.reserve(msg.wpnts.size());
    for (const auto& w : msg.wpnts) {
      Pose2D q;
      q.x = w.x_m;
      q.y = w.y_m;
      q.vel = w.vx_mps;
      q.s = w.s_m;
      q.dl = w.d_left;
      q.dr = w.d_right;
      q.kappa = w.kappa_radpm;
      wps.push_back(q);
    }
    const size_t n = wps.size();
    for (size_t i = 0; i < n; ++i) {
      const auto& a = wps[i];
      const auto& b = wps[(i + 1) % n];
      wps[i].psi = std::atan2(b.y - a.y, b.x - a.x);
    }
    const double len = wps.back().s + std::hypot(wps.front().x - wps.back().x, wps.front().y - wps.back().y);
    std::lock_guard<std::mutex> lk(mtx_);
    if (!is_waypoints_ok_) RCLCPP_INFO(get_logger(), "waypoints received: %zu points from %s", wps.size(), waypoints_topic_.c_str());
    if (wps.size() != path_.size()) path_changed_ = true;
    path_ = std::move(wps);
    track_len_ = len;
    is_waypoints_ok_ = true;
  }

  // ======================================================================
  // reference_sdf_generator.cpp  (calc_waypoints + build_reference_sdf)
  // ======================================================================
  static double dist2d(const Pose2D& a, const Pose2D& b) { return std::hypot(a.x - b.x, a.y - b.y); }

  static Pose2D get_waypoint(const std::vector<Pose2D>& path, int index) {
    Pose2D w = path[index];
    const int n = static_cast<int>(path.size());
    if (index == n - 1) {
      const auto& prev = path[index - 1];
      w.yaw = std::atan2(path[index].y - prev.y, path[index].x - prev.x);
    } else {
      const auto& next = path[index + 1];
      w.yaw = std::atan2(next.y - path[index].y, next.x - path[index].x);
    }
    return w;
  }

  // sdf thread only (last_nearest_)
  std::vector<Pose2D> calc_waypoints(const std::vector<Pose2D>& path, const Pose2D& robot) {
    const int n = static_cast<int>(path.size());
    int nearest = 0;
    double min_d = std::numeric_limits<double>::max();
    bool searched = false;
    if (ref_local_search_ && last_nearest_ >= 0 && last_nearest_ < n) {
      // [추가] 직전 최근접 인덱스 주변(뒤 back_num ~ 앞 forward_num)만 검색 -> 벽 건너편 구간으로 점프 방지
      for (int k = -ref_search_back_; k <= ref_search_fwd_; ++k) {
        const int i = ((last_nearest_ + k) % n + n) % n;
        const double d = dist2d(path[i], robot);
        if (d < min_d) {
          min_d = d;
          nearest = i;
        }
      }
      searched = min_d < 2.0;  // 너무 멀어졌으면(리스폰 등) 전체 검색으로 복귀
    }
    if (!searched) {
      min_d = std::numeric_limits<double>::max();
      for (int i = 0; i < n; ++i) {  // 공식 코드 방식
        const double d = dist2d(path[i], robot);
        if (d < min_d) {
          min_d = d;
          nearest = i;
        }
      }
    }
    if (last_nearest_ >= 0 && last_nearest_ < n) {
      int diff = ((nearest - last_nearest_) % n + n) % n;
      if (diff > n / 2) diff -= n;
      if (std::abs(diff) > ref_jump_warn_num_)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                             "reference jump: waypoint %d -> %d (%+d pts), robot (%.2f, %.2f), dist to ref %.2f m", last_nearest_,
                             nearest, diff, robot.x, robot.y, min_d);
    }
    last_nearest_ = nearest;
    const int start = ((nearest - ref_backward_margin_num_) % n + n) % n;
    std::vector<Pose2D> out;
    out.push_back(get_waypoint(path, start));
    int cur = start;
    const int num = ref_num_waypoints_;
    for (int k = 1; k < num; ++k) {
      int idx = cur;
      for (int guard = 0; guard < n; ++guard) {  // find_lookahead_index
        idx = (idx + 1) % n;
        if (dist2d(path[idx], path[cur]) >= ref_waypoint_interval_) break;
      }
      out.push_back(get_waypoint(path, idx));
      cur = idx;
    }
    return out;
  }

  void build_reference_sdf(const std::vector<Pose2D>& wps, const Pose2D& robot, grid_map::GridMap* map) {
    map->setGeometry(grid_map::Length(ref_map_width_, ref_map_height_), ref_map_resolution_,
                     grid_map::Position(robot.x, robot.y));
    grid_map::Matrix& dist_data = map->get(kDistanceLayer);
    grid_map::Matrix& angle_data = map->get(kAngleLayer);
    grid_map::Matrix& speed_data = map->get(kSpeedLayer);
    const auto size = map->getSize();
    const int linear = static_cast<int>(size.prod());
    const double scale = ref_speed_scale_, vmax = ref_max_speed_;
#pragma omp parallel for num_threads(std::max(1, ref_threads_))
    for (int i = 0; i < linear; ++i) {
      const grid_map::Index index(grid_map::getIndexFromLinearIndex(i, size));
      grid_map::Position pos;
      map->getPosition(index, pos);
      double min_d = std::numeric_limits<double>::max();
      size_t nearest = 0;
      for (size_t k = 0; k < wps.size(); ++k) {
        const double d = std::hypot(wps[k].x - pos[0], wps[k].y - pos[1]);
        if (d < min_d) {
          min_d = d;
          nearest = k;
        }
      }
      dist_data(index[0], index[1]) = min_d;
      angle_data(index[0], index[1]) = wps[nearest].yaw;
      speed_data(index[0], index[1]) = std::min(wps[nearest].vel * scale, vmax);
    }
  }

  // ======================================================================
  // [추가] guide: obstacle-sequence-aware reference
  // ======================================================================
  void on_static_map(const nav_msgs::msg::OccupancyGrid& m) {
    auto sm = std::make_shared<StaticMap>();
    sm->ox = m.info.origin.position.x;
    sm->oy = m.info.origin.position.y;
    sm->res = m.info.resolution;
    sm->w = static_cast<int>(m.info.width);
    sm->h = static_cast<int>(m.info.height);
    const int W = sm->w, H = sm->h;
    sm->dist.assign(static_cast<size_t>(W) * H, std::numeric_limits<float>::max());
    std::vector<int> site(static_cast<size_t>(W) * H, -1);
    std::deque<int> q;
    for (int i = 0; i < W * H; ++i) {
      const int8_t v = m.data[i];
      if (v < 0 || v > 50) {
        sm->dist[i] = 0.0f;
        site[i] = i;
        q.push_back(i);
      }
    }
    // brushfire with nearest-site propagation (close to exact euclidean)
    while (!q.empty()) {
      const int c = q.front();
      q.pop_front();
      const int cx = c % W, cy = c / W;
      const int sx = site[c] % W, sy = site[c] / W;
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
          const int nx = cx + dx, ny = cy + dy;
          if ((dx == 0 && dy == 0) || nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
          const int ni = ny * W + nx;
          const float d = static_cast<float>(std::hypot(nx - sx, ny - sy) * sm->res);
          if (d < sm->dist[ni]) {
            sm->dist[ni] = d;
            site[ni] = site[c];
            q.push_back(ni);
          }
        }
    }
    {
      std::lock_guard<std::mutex> lk(map_mtx_);
      static_map_ = sm;
    }
    RCLCPP_INFO(get_logger(), "static map received for guide: %dx%d @ %.3f m", W, H, sm->res);
  }

  static int nearest_index(const std::vector<Pose2D>& path, double x, double y, int hint, int win) {
    const int n = static_cast<int>(path.size());
    int best = 0;
    double bd = std::numeric_limits<double>::max();
    if (hint >= 0 && hint < n) {
      for (int k = -win; k <= win; ++k) {
        const int i = ((hint + k) % n + n) % n;
        const double d = std::hypot(path[i].x - x, path[i].y - y);
        if (d < bd) bd = d, best = i;
      }
      if (bd < 1.5) return best;
    }
    bd = std::numeric_limits<double>::max();
    for (int i = 0; i < n; ++i) {
      const double d = std::hypot(path[i].x - x, path[i].y - y);
      if (d < bd) bd = d, best = i;
    }
    return best;
  }

  double s_rel(const std::vector<Pose2D>& path, int i, int i0) const {
    double ds = path[i].s - path[i0].s;
    const double L = track_len_snapshot_;
    if (L > 0) {
      while (ds > L / 2) ds -= L;
      while (ds < -L / 2) ds += L;
    }
    return ds;
  }

  // returns true and fills *out (shifted raceline) when obstacles are in the window
  bool compute_guide(const std::vector<Pose2D>& path, const Pose2D& robot, const sensor_msgs::msg::LaserScan& scan, const GuideCfg& c,
                     std::vector<Pose2D>* out) {
    const int n = static_cast<int>(path.size());
    std::shared_ptr<const StaticMap> sm;
    {
      std::lock_guard<std::mutex> lk(map_mtx_);
      sm = static_map_;
    }
    if (!sm || n < 10) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "guide: waiting for static map %s", static_map_topic_.c_str());
      return false;
    }
    const double tnow = steady_sec();
    const int ir = nearest_index(path, robot.x, robot.y, guide_idx_, 30);
    guide_idx_ = ir;

    // [11/22] curvature rule: raceline too tight around the car -> follow the plain raceline (generic, map-derived)
    // [11/26] decided after the obstacle scan below: kept running while obstacles are close ahead (curv_keep_obs)
    double kmax = 0.0;
    if (c.max_curvature > 0.0) {
      const int nb = static_cast<int>(std::round(c.curv_back / 0.1)), na = static_cast<int>(std::round(c.curv_ahead / 0.1));
      for (int m = -nb; m <= na; ++m) kmax = std::max(kmax, std::abs(path[((ir + m) % n + n) % n].kappa));
    }
    const bool tight = c.max_curvature > 0.0 && kmax > c.max_curvature;
    const double cpsi = std::cos(path[ir].psi), spsi = std::sin(path[ir].psi);
    const double d_car = cpsi * (robot.y - path[ir].y) - spsi * (robot.x - path[ir].x);

    // --- 1. obstacle memory from this scan (cells 0.1 m, map frame) ---
    double lx = laser_offset_x_, ly = 0.0, lyaw = 0.0;
    if (have_laser_tf_) lx = laser_tf_[0], ly = laser_tf_[1], lyaw = laser_tf_[2];
    const Pose2D sp = scan_pose_;  // pose at the scan stamp (interpolated in sdf_loop)
    const double cy = std::cos(sp.yaw), sy = std::sin(sp.yaw);
    const double wx0 = sp.x + cy * lx - sy * ly, wy0 = sp.y + sy * lx + cy * ly, wyaw = sp.yaw + lyaw;
    constexpr double kCell = 0.1;
    auto key_of = [](long ix, long iy) {
      return static_cast<long>((static_cast<unsigned long>(ix) << 32) ^ (static_cast<unsigned long>(iy) & 0xffffffffUL));
    };
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const float r = scan.ranges[i];
      if (!std::isfinite(r) || r < scan.range_min || r >= scan.range_max || r > c.max_range) continue;
      const double a = wyaw + scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double px = wx0 + r * std::cos(a), py = wy0 + r * std::sin(a);
      if (sm->at(px, py) <= c.static_margin + 0.04 * r) continue;  // margin grows with range (pose/angle error)
      const long ix = static_cast<long>(std::floor(px / kCell)), iy = static_cast<long>(std::floor(py / kCell));
      auto& cell = obs_mem_[key_of(ix, iy)];
      if (cell.scan_id != scan_id_) {  // count each cell once per scan
        cell.hits = (tnow - cell.t > c.memory) ? 1 : cell.hits + 1;
        cell.scan_id = scan_id_;
      }
      cell.t = tnow;
      cell.ix = ix;
      cell.iy = iy;
    }
    ++scan_id_;
    for (auto it = obs_mem_.begin(); it != obs_mem_.end();) {
      if (tnow - it->second.t > c.memory) it = obs_mem_.erase(it);
      else ++it;
    }

    // --- 2. keep cells that have a neighbour (drop single-cell noise), project to Frenet ---
    const double L = c.lookahead;
    std::vector<ObsPt> obs;
    const int win_back = 10, win_fwd = static_cast<int>((L + 1.5) / 0.1) + 10;
    for (const auto& [k, cell] : obs_mem_) {
      if (cell.hits < kMinHits) continue;  // seen in fewer than kMinHits scans: transient (pose/scan mismatch)
      bool nb = false;
      for (int dy = -1; dy <= 1 && !nb; ++dy)
        for (int dx = -1; dx <= 1 && !nb; ++dx)
          if ((dx || dy) && obs_mem_.count(key_of(cell.ix + dx, cell.iy + dy))) nb = true;
      if (!nb) continue;
      const double px = (cell.ix + 0.5) * kCell, py = (cell.iy + 0.5) * kCell;
      int bi = -1;
      double bd = std::numeric_limits<double>::max();
      for (int m = -win_back; m <= win_fwd; ++m) {
        const int i = ((ir + m) % n + n) % n;
        const double d = std::hypot(path[i].x - px, path[i].y - py);
        if (d < bd) bd = d, bi = i;
      }
      const double tc = std::cos(path[bi].psi), ts = std::sin(path[bi].psi);
      const double ex = px - path[bi].x, ey = py - path[bi].y;
      const double s = s_rel(path, bi, ir) + tc * ex + ts * ey;
      const double d = tc * ey - ts * ex;
      if (s < -0.6 || s > L + 0.5) continue;
      if (d > path[bi].dl + 0.3 || d < -path[bi].dr - 0.3) continue;  // outside the track (e.g. behind a wall)
      obs.push_back({s, d, px, py});
    }
    publish_guide_obstacles(obs);
    if (tight) {
      bool near_obs = false;
      if (c.curv_keep_obs > 0.0)
        for (const auto& o : obs)
          if (o.s > -0.6 && o.s < c.curv_keep_obs) {
            near_obs = true;
            break;
          }
      if (!near_obs) {
        if (!prev_guide_.empty())
          RCLCPP_INFO(get_logger(), "guide paused: raceline curvature %.2f > %.2f 1/m", kmax, c.max_curvature);
        prev_guide_.clear();
        publish_f32(pub_guide_offset_, 0.0);
        return false;
      }
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "guide kept in tight curve (%.2f > %.2f 1/m): obstacle within %.1f m ahead", kmax, c.max_curvature,
                           c.curv_keep_obs);
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "guide: scan is %.1f ms newer than the last odom", scan_pose_dt_ * 1e3);
    if (obs.empty()) {
      prev_guide_.clear();
      publish_f32(pub_guide_offset_, 0.0);
      return false;
    }

    // --- 3. DP over stations k (ds) x lateral offsets j (dd) ---
    const int K = std::max(1, static_cast<int>(std::ceil(L / c.ds)));
    constexpr double Dmax = 3.0;
    const int J = 2 * static_cast<int>(std::round(Dmax / c.dd)) + 1;
    const int jmax = std::max(1, static_cast<int>(std::floor(c.max_slope * c.ds / c.dd + 1e-9)));
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<int> st_idx(K + 1);
    {
      int i = ir;
      for (int k = 0; k <= K; ++k) {
        const double target = k * c.ds;
        for (int guard = 0; guard < n && s_rel(path, i, ir) < target - 0.05; ++guard) i = (i + 1) % n;
        st_idx[k] = i;
      }
    }
    std::vector<double> cost(static_cast<size_t>(K + 1) * J, INF);
    std::vector<int> from(static_cast<size_t>(K + 1) * J, -1);
    // per obstacle point: which side the previous plan passed it on (+1 left / -1 right / 0 unknown)
    std::vector<int> prev_side(obs.size(), 0);
    if (!prev_guide_.empty()) {
      for (size_t q = 0; q < obs.size(); ++q) {
        double dp;
        if (prev_offset_at(path[ir].s + obs[q].s, &dp) && std::abs(dp - obs[q].d) > 0.1) prev_side[q] = dp > obs[q].d ? 1 : -1;
      }
    }
    // per station: obstacle points whose blocked zone covers it (so each node only looks at those)
    std::vector<std::vector<int>> st_obs(K + 1);
    for (int k = 0; k <= K; ++k) {
      const double sk = k * c.ds;
      for (size_t q = 0; q < obs.size(); ++q)
        if (sk >= obs[q].s - c.obs_clear_s && sk <= obs[q].s + c.obs_depth + c.obs_clear_s) st_obs[k].push_back(static_cast<int>(q));
    }
    std::vector<double> prev_at(K + 1, std::numeric_limits<double>::quiet_NaN());
    if (!prev_guide_.empty())
      for (int k = 0; k <= K; ++k) {
        double dp;
        if (prev_offset_at(path[ir].s + k * c.ds, &dp)) prev_at[k] = dp;
      }
    auto node_cost = [&](int k, int j) -> double {
      const double d = -Dmax + j * c.dd;
      const Pose2D& w = path[st_idx[k]];
      if (d > w.dl + 0.3 || d < -w.dr - 0.3) return INF;  // far outside the track
      const double x = w.x - std::sin(w.psi) * d, y = w.y + std::cos(w.psi) * d;
      // constraints are "very expensive" rather than forbidden so that the DP always returns a path
      double soft = 0.0;
      const double wd = sm->at(x, y);
      if (wd < 0.05) return INF;                                  // inside a wall
      if (wd < c.wall_clear) soft += 1e3 * (c.wall_clear - wd);  // too close to a wall
      for (int q : st_obs[k]) {
        const auto& o = obs[q];
        const double ad = std::abs(d - o.d);
        if (ad < c.obs_clear_d) soft += 1e4 * (c.obs_clear_d - ad);
        else if (ad < c.obs_clear_d + c.soft_margin) soft += c.w_obs * std::pow(c.obs_clear_d + c.soft_margin - ad, 2);
        if (prev_side[q] != 0 && (d > o.d ? 1 : -1) != prev_side[q]) soft += c.w_side;  // keep the side chosen before
      }
      const double prev = std::isnan(prev_at[k]) ? 0.0 : c.w_prev * (d - prev_at[k]) * (d - prev_at[k]);
      return c.w_ref * d * d + soft + prev;
    };
    const int j0 = std::clamp(static_cast<int>(std::round((d_car + Dmax) / c.dd)), 0, J - 1);
    cost[j0] = 0.0;
    for (int k = 1; k <= K; ++k) {
      bool any = false;
      for (int j = 0; j < J; ++j) {
        const double nc = node_cost(k, j);
        if (!std::isfinite(nc)) continue;
        double best = INF;
        int bi = -1;
        for (int i = std::max(0, j - jmax); i <= std::min(J - 1, j + jmax); ++i) {
          const double pc = cost[static_cast<size_t>(k - 1) * J + i];
          if (!std::isfinite(pc)) continue;
          const double slope = (j - i) * c.dd / c.ds;
          const double tc = pc + c.w_smooth * slope * slope * c.ds;
          if (tc < best) best = tc, bi = i;
        }
        if (bi < 0) continue;
        cost[static_cast<size_t>(k) * J + j] = best + nc * c.ds;
        from[static_cast<size_t>(k) * J + j] = bi;
        any = true;
      }
      if (!any) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "guide: no feasible corridor at %.2f m ahead (%zu obstacle cells)",
                             k * c.ds, obs.size());
        prev_guide_.clear();
        return false;
      }
    }
    int jb = 0;
    double cb = INF;
    for (int j = 0; j < J; ++j)
      if (cost[static_cast<size_t>(K) * J + j] < cb) cb = cost[static_cast<size_t>(K) * J + j], jb = j;
    std::vector<double> dsol(K + 1);
    for (int k = K, j = jb; k >= 0; --k) {
      dsol[k] = -Dmax + j * c.dd;
      if (k > 0) j = from[static_cast<size_t>(k) * J + j];
    }

    // --- 4. shifted raceline ---
    *out = path;
    auto offset_at = [&](double sr) {
      if (sr <= 0.0) return dsol[0];
      const double u = sr / c.ds;
      if (u >= K) return dsol[K] * std::max(0.0, 1.0 - (sr - K * c.ds) / std::max(1e-3, c.blend));
      const int k = static_cast<int>(u);
      const double f = u - k;
      return dsol[k] * (1 - f) + dsol[k + 1] * f;
    };
    const int m_end = static_cast<int>((K * c.ds + c.blend) / 0.1) + 10;
    for (int m = -15; m <= m_end && m < n - 15; ++m) {
      const int i = ((ir + m) % n + n) % n;
      const double off = offset_at(s_rel(path, i, ir));
      (*out)[i].x = path[i].x - std::sin(path[i].psi) * off;
      (*out)[i].y = path[i].y + std::cos(path[i].psi) * off;
      if (std::abs(off) > 0.1) (*out)[i].vel = path[i].vel * c.speed_scale;
    }
    prev_guide_.clear();
    for (int k = 0; k <= K; ++k) prev_guide_.push_back({path[ir].s + k * c.ds, dsol[k]});
    prev_guide_track_len_ = track_len_snapshot_;

    // --- 5. viz / log ---
    double dmax = 0.0;
    for (double d : dsol)
      if (std::abs(d) > std::abs(dmax)) dmax = d;
    publish_f32(pub_guide_offset_, dmax);
    if (pub_guide_path_->get_subscription_count() > 0) {
      nav_msgs::msg::Path gp;
      gp.header.stamp = now();
      gp.header.frame_id = map_frame_;
      for (int k = 0; k <= K; ++k) {
        const Pose2D& w = path[st_idx[k]];
        geometry_msgs::msg::PoseStamped ps;
        ps.header = gp.header;
        ps.pose.position.x = w.x - std::sin(w.psi) * dsol[k];
        ps.pose.position.y = w.y + std::cos(w.psi) * dsol[k];
        ps.pose.orientation.w = 1.0;
        gp.poses.push_back(ps);
      }
      pub_guide_path_->publish(gp);
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "guide: %zu obstacle cells ahead, offset %+.2f m (car d=%+.2f)", obs.size(),
                         dmax, d_car);
    return true;
  }

  bool prev_offset_at(double sabs, double* d) const {
    if (prev_guide_.size() < 2) return false;
    const double L = prev_guide_track_len_;
    auto rel = [&](double a) {
      double r = a - prev_guide_.front().first;
      if (L > 0) {
        while (r > L / 2) r -= L;
        while (r < -L / 2) r += L;
      }
      return r;
    };
    const double r = rel(sabs);
    const double step = prev_guide_[1].first - prev_guide_[0].first;
    if (r < 0 || step <= 0) return false;
    const double u = r / step;
    const size_t k = static_cast<size_t>(u);
    if (k + 1 >= prev_guide_.size()) return false;
    const double f = u - k;
    *d = prev_guide_[k].second * (1 - f) + prev_guide_[k + 1].second * f;
    return true;
  }

  void publish_guide_obstacles(const std::vector<ObsPt>& obs) {
    if (pub_guide_obs_->get_subscription_count() == 0) return;
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = now();
    m.ns = "guide_obstacles";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = 0.08;
    m.color.r = 1.0;
    m.color.g = 0.2;
    m.color.a = 1.0;
    m.lifetime = rclcpp::Duration::from_seconds(0.2);
    for (const auto& o : obs) {
      geometry_msgs::msg::Point p;
      p.x = o.x;
      p.y = o.y;
      m.points.push_back(p);
    }
    arr.markers.push_back(m);
    pub_guide_obs_->publish(arr);
  }

  // ======================================================================
  // local_costmap_generator.cpp  (scan -> points in base_link -> crop -> grid -> inflate)
  // ======================================================================
  bool laser_to_base(const std::string& laser_frame, double* tx, double* ty, double* tyaw) {
    if (robot_frame_.empty()) robot_frame_ = robot_frame_param_.empty() ? "base_link" : robot_frame_param_;
    if (have_laser_tf_) {
      *tx = laser_tf_[0];
      *ty = laser_tf_[1];
      *tyaw = laser_tf_[2];
      return true;
    }
    try {
      const auto t = tf_buffer_->lookupTransform(robot_frame_, laser_frame, tf2::TimePointZero);
      const auto& q = t.transform.rotation;
      laser_tf_ = {t.transform.translation.x, t.transform.translation.y, yaw_from_quat(q.x, q.y, q.z, q.w)};
      have_laser_tf_ = true;
      RCLCPP_INFO(get_logger(), "TF %s <- %s: x=%.3f y=%.3f yaw=%.3f", robot_frame_.c_str(), laser_frame.c_str(), laser_tf_[0],
                  laser_tf_[1], laser_tf_[2]);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF %s <- %s not available (%s). using laser_offset_x=%.3f",
                           robot_frame_.c_str(), laser_frame.c_str(), ex.what(), laser_offset_x_);
      *tx = laser_offset_x_;
      *ty = 0.0;
      *tyaw = 0.0;
      return true;
    }
    *tx = laser_tf_[0];
    *ty = laser_tf_[1];
    *tyaw = laser_tf_[2];
    return true;
  }

  void build_local_costmap(const sensor_msgs::msg::LaserScan& scan) {
    double tx, ty, tyaw;
    laser_to_base(scan.header.frame_id, &tx, &ty, &tyaw);
    const double c = std::cos(tyaw), s = std::sin(tyaw);

    grid_map::Matrix& data = cost_map_.get(kCollisionLayer);
    data.setZero();
    std::vector<grid_map::Index> occupied;
    occupied.reserve(scan.ranges.size());
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const float r = scan.ranges[i];
      if (!std::isfinite(r) || r < scan.range_min || r >= scan.range_max) continue;  // laser_geometry::projectLaser
      const double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double lx = r * std::cos(a), ly = r * std::sin(a);
      const double bx = tx + c * lx - s * ly;
      const double by = ty + s * lx + c * ly;
      if (cm_crop_robot_ && bx >= -bl2rear_ && bx <= bl2front_ && by >= -bl2right_ && by <= bl2left_) continue;
      const grid_map::Position pos(bx, by);
      if (!cost_map_.isInside(pos)) continue;
      grid_map::Index idx;
      cost_map_.getIndex(pos, idx);
      data(idx.x(), idx.y()) = cm_max_val_;
      occupied.push_back(idx);
    }
    const double res = cost_map_.getResolution();
    const grid_map::Size gsz = cost_map_.getSize();
    if (cm_filter_isolated_) {
      // [추가] 외톨이 점 제거: 주변 filter_radius 안에 다른 스캔 점이 하나도 없으면 허상 점으로 보고 지움
      const int rr = std::max(1, static_cast<int>(std::ceil(cm_filter_radius_ / res)));
      std::vector<grid_map::Index> kept;
      kept.reserve(occupied.size());
      for (const auto& index : occupied) {
        bool nb = false;
        for (int dx = -rr; dx <= rr && !nb; ++dx)
          for (int dy = -rr; dy <= rr && !nb; ++dy) {
            if (dx == 0 && dy == 0) continue;
            const int x = index.x() + dx, y = index.y() + dy;
            if (x >= 0 && y >= 0 && x < gsz(0) && y < gsz(1) && data(x, y) > 0) nb = true;
          }
        if (nb) kept.push_back(index);
      }
      for (const auto& index : occupied) data(index.x(), index.y()) = 0;
      for (const auto& index : kept) data(index.x(), index.y()) = cm_max_val_;
      occupied.swap(kept);
    }
    if (cm_inflation_circle_) {
      // [추가] 방향과 무관한 원형 부풀림 (기본은 아래 공식 rigid body 방식)
      const int rr = static_cast<int>(std::ceil(cm_circle_radius_ / res));
      std::vector<std::pair<int, int>> offs;
      for (int dx = -rr; dx <= rr; ++dx)
        for (int dy = -rr; dy <= rr; ++dy)
          if ((dx * dx + dy * dy) * res * res <= cm_circle_radius_ * cm_circle_radius_) offs.emplace_back(dx, dy);
      for (const auto& index : occupied)
        for (const auto& [dx, dy] : offs) {
          const int x = index.x() + dx, y = index.y() + dy;
          if (x >= 0 && y >= 0 && x < gsz(0) && y < gsz(1)) data(x, y) = cm_max_val_;
        }
      return;
    }
    // inflate_rigid_body (verbatim index arithmetic)
    const int front = static_cast<int>(std::ceil(bl2front_ / res));
    const int rear = static_cast<int>(std::ceil(bl2rear_ / res));
    const int right = static_cast<int>(std::ceil(bl2right_ / res));
    const int left = static_cast<int>(std::ceil(bl2left_ / res));
    const grid_map::Size size = cost_map_.getSize();
#pragma omp parallel for num_threads(thread_num_.load())
    for (size_t i = 0; i < occupied.size(); ++i) {
      const auto index = occupied[i];
      const int sy = std::max(index.y() - right, 0);
      const int ly = std::min(index.y() + left, static_cast<int>(size(0)));
      const int sx = std::max(index.x() - rear, 0);
      const int lx = std::min(index.x() + front, static_cast<int>(size(1)));
      for (int x = sx; x < lx; ++x)
        for (int y = sy; y < ly; ++y) data(x, y) = cm_max_val_;
    }
  }

  // ======================================================================
  // mppi_controller_ros.cpp :: timer_callback
  // ======================================================================
  void sdf_loop() {
    Pose2D robot;
    std::vector<Pose2D> path;
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!is_odom_ok_ || !is_waypoints_ok_) return;
      robot = robot_;
      path = path_;
      scan = scan_;
      track_len_snapshot_ = track_len_;
      scan_pose_ = robot;
      if (scan && !pose_hist_.empty()) {  // pose at the scan stamp (linear interpolation)
        const double ts = rclcpp::Time(scan->header.stamp).seconds();
        scan_pose_dt_ = ts - pose_hist_.back().t;
        // find the two poses around ts; if the scan is newer than the last pose (odom arrives later), extrapolate
        size_t ib = pose_hist_.size() - 1;
        while (ib > 0 && pose_hist_[ib - 1].t >= ts) --ib;
        if (ib == 0) ib = 1;
        if (pose_hist_.size() >= 2) {
          size_t ia = ib - 1;
          while (ia > 0 && pose_hist_[ib].t - pose_hist_[ia].t < 1e-3) --ia;  // skip duplicated stamps
          const auto& a = pose_hist_[ia];
          const auto& b = pose_hist_[ib];
          if (b.t - a.t > 1e-3) {
            const double f = std::clamp((ts - a.t) / (b.t - a.t), 0.0, 1.0 + 0.1 / (b.t - a.t));  // at most 0.1 s ahead
            scan_pose_.x = a.x + f * (b.x - a.x);
            scan_pose_.y = a.y + f * (b.y - a.y);
            scan_pose_.yaw = a.yaw + f * wrap(b.yaw - a.yaw);
          }
        }
      }
      if (path_changed_) {
        last_nearest_ = -1;
        guide_idx_ = -1;
        prev_guide_.clear();
        path_changed_ = false;
      }
    }
    GuideCfg gc;
    {
      std::lock_guard<std::mutex> lk(cfg_mtx_);
      gc = guide_cfg_;
    }
    std::vector<Pose2D> guided;
    bool guide_on = false;
    if (gc.enable && scan) {
      guide_on = compute_guide(path, robot, *scan, gc, &guided);
    } else if (!obs_mem_.empty()) {
      obs_mem_.clear();
      prev_guide_.clear();
    }
    const auto wps = calc_waypoints(guide_on ? guided : path, robot);
    if (!debug_dump_dir_.empty()) {
      std::lock_guard<std::mutex> lk(dbg_mtx_);
      last_wps_ = wps;
    }

    const auto t0 = std::chrono::steady_clock::now();
    build_reference_sdf(wps, robot, &sdf_work_);
    if (++sdf_cycle_ % 4 == 0 && pub_ref_path_->get_subscription_count() > 0) {
      nav_msgs::msg::Path rp;
      rp.header.stamp = now();
      rp.header.frame_id = map_frame_;
      rp.poses.resize(wps.size());
      for (size_t k = 0; k < wps.size(); ++k) {
        rp.poses[k].header = rp.header;
        rp.poses[k].pose.position.x = wps[k].x;
        rp.poses[k].pose.position.y = wps[k].y;
        rp.poses[k].pose.orientation.z = std::sin(wps[k].yaw / 2.0);
        rp.poses[k].pose.orientation.w = std::cos(wps[k].yaw / 2.0);
      }
      pub_ref_path_->publish(rp);
    }
    sdf_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::lock_guard<std::mutex> lk(sdf_mtx_);
    reference_sdf_ = sdf_work_;
    is_sdf_ok_ = true;
  }

  void control_loop() {
    const auto t0 = std::chrono::steady_clock::now();
    {  // [추가] swap in a solver rebuilt with new parameters (between two cycles)
      std::lock_guard<std::mutex> lk(swap_mtx_);
      if (next_solver_) {
        solver_ = std::move(next_solver_);
        thread_num_ = next_params_.common.thread_num;
        steer_1st_delay_ = std::max(next_params_.common.steer_1st_delay, next_params_.common.prediction_interval);
        RCLCPP_INFO(get_logger(), "solver swapped (warm start reset)");
      }
    }
    Pose2D robot;
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!is_odom_ok_ || !is_waypoints_ok_ || !scan_ || !is_sdf_ok_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Not ready: odom=%d waypoints=%d scan=%d ref_sdf=%d", is_odom_ok_,
                             is_waypoints_ok_, scan_ != nullptr, static_cast<bool>(is_sdf_ok_));
        return;
      }
      robot = robot_;
      scan = scan_;
      if (pose_latency_comp_ && robot_recv_steady_ > 0.0) {  // [11/24] predict the pose to now
        // age = latency at arrival + time since arrival (steady clock). A localisation stall (EKF missed its rate for
        // ~0.35 s: /car_state/odom stops, pf and gt alike) is extrapolated up to pose_latency_max.
        const double age = robot_recv_latency_ + (steady_sec() - robot_recv_steady_);
        const double tau = std::clamp(age, 0.0, static_cast<double>(pose_latency_max_));
        const double yaw_mid = robot.yaw + 0.5 * robot_yaw_rate_ * tau;
        robot.x += robot.vel * std::cos(yaw_mid) * tau;
        robot.y += robot.vel * std::sin(yaw_mid) * tau;
        robot.yaw = wrap(robot.yaw + robot_yaw_rate_ * tau);
        last_pose_latency_ = tau;
        if (age > pose_latency_max_)
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "pose is %.0f ms old (localisation stalled): extrapolated %.0f ms",
                               age * 1e3, tau * 1e3);
        else
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "pose latency compensation: %.0f ms", tau * 1e3);
      }
      if (odom_frame_ != map_frame_)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "odom frame '%s' != map_frame_id '%s'", odom_frame_.c_str(),
                             map_frame_.c_str());
    }
    const auto t_a = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lk(sdf_mtx_);
      ref_snapshot_ = reference_sdf_;
    }
    const auto t_b = std::chrono::steady_clock::now();
    build_local_costmap(*scan);
    const auto t_c = std::chrono::steady_clock::now();

    solver_->set_obstacle_map(cost_map_);
    solver_->set_reference_map(ref_snapshot_);

    const double prev_steer = last_cmd_steer_;
    steer_est_ = steer_est_ + (prev_steer - steer_est_) * (1.0 - std::exp(-control_sampling_time_ / steer_1st_delay_));
    mppi::cpu::State x0 = mppi::cpu::State::Zero();
    x0[mppi::STATE_SPACE::x] = robot.x;
    x0[mppi::STATE_SPACE::y] = robot.y;
    x0[mppi::STATE_SPACE::yaw] = robot.yaw;
    x0[mppi::STATE_SPACE::vel] = robot.vel;
    x0[mppi::STATE_SPACE::steer] = steer_est_;

    const auto [u_seq, collision_rate] = solver_->solve(x0);
    const auto t_d = std::chrono::steady_clock::now();
    const auto [best_seq, state_cost, collision_cost, input_error] = solver_->get_predictive_seq(x0, u_seq);
    (void)input_error;

    const double steer = wrap(u_seq(0, mppi::CONTROL_SPACE::steer));
    if (!debug_dump_dir_.empty()) debug_dump(x0, u_seq, best_seq, steer, collision_rate);
    double speed = reference_speed_;
    if (!constant_speed_mode_) {
      const grid_map::Position here(robot.x, robot.y);
      if (ref_snapshot_.isInside(here)) {
        speed = ref_snapshot_.atPosition(kSpeedLayer, here);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "robot out of reference sdf map, using reference_speed");
      }
    }
    last_cmd_steer_ = steer;

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = robot_frame_;
    cmd.drive.steering_angle = steer;
    cmd.drive.speed = speed;
    if (drive_publish_enabled_) pub_drive_->publish(cmd);

    const double calc_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    // --- visualization / metrics ---
    publish_path(best_seq);
    if (visualize_candidates_) publish_candidates();
    if (publish_costmap_every_ > 0 && (++cycle_ % publish_costmap_every_ == 0)) publish_costmap(scan->header.stamp);
    publish_f32(pub_calc_time_, calc_ms);
    publish_f32(pub_coll_rate_, collision_rate);
    publish_f32(pub_cost_, state_cost + collision_cost);
    publish_f32(pub_speed_, robot.vel);
    if (calc_ms > control_sampling_time_ * 1e3) {
      auto ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "control loop %.1f ms > period %.1f ms  (copy_sdf %.1f, costmap %.1f, solve %.1f, wait %.1f | ref_sdf thread %.1f)",
                           calc_ms, control_sampling_time_ * 1e3, ms(t_a, t_b), ms(t_b, t_c), ms(t_c, t_d),
                           ms(t0, t_a), sdf_ms_.load());
    }
  }

  // [진단] 조향 부호가 한 주기 만에 크게 뒤집히면 그 순간 solver 가 본 것을 JSON 으로 저장 (debug.dump_dir 가 비어 있으면 꺼짐)
  void debug_dump(const mppi::cpu::State& x0, const mppi::cpu::ControlSeq& u_seq, const mppi::cpu::StateSeq& best_seq, double steer,
                  double collision_rate) {
    const double prev = last_cmd_steer_;
    const double prev_coll = dbg_prev_coll_;
    dbg_prev_coll_ = collision_rate;
    // trigger: steering jumps by more than 2*threshold, or the collision rate jumps from ~0 to > 0.3 in one cycle
    const bool flip = std::abs(steer - prev) > 2.0 * debug_flip_thresh_ || (prev_coll < 0.1 && collision_rate > 0.3);
    const double t = steady_sec();
    if (!flip || t - last_dump_t_ < 2.0) return;
    last_dump_t_ = t;
    auto seq_json = [](const mppi::cpu::StateSeq& s) {
      std::string o = "[";
      for (int i = 0; i < s.rows(); ++i) {
        char b[96];
        snprintf(b, sizeof(b), "%s[%.3f,%.3f,%.3f,%.3f]", i ? "," : "", s(i, mppi::STATE_SPACE::x), s(i, mppi::STATE_SPACE::y),
                 s(i, mppi::STATE_SPACE::yaw), s(i, mppi::STATE_SPACE::steer));
        o += b;
      }
      return o + "]";
    };
    std::string j = "{";
    char b[256];
    snprintf(b, sizeof(b), "\"x0\":[%.3f,%.3f,%.3f,%.3f,%.3f],\"prev_steer\":%.4f,\"steer\":%.4f,\"collision_rate\":%.3f,", x0[0], x0[1],
             x0[2], x0[3], x0[4], prev, steer, collision_rate);
    j += b;
    j += "\"u_seq\":[";
    for (int i = 0; i < u_seq.rows(); ++i) j += (i ? "," : "") + std::to_string(u_seq(i, 0));
    j += "],\"best\":" + seq_json(best_seq);
    const auto nominal = solver_->get_control_seq();
    const auto [gseq, gcost, gcoll, gin] = solver_->get_predictive_seq(x0, nominal);
    (void)gin;
    snprintf(b, sizeof(b), ",\"guide_cost\":%.2f,\"guide_collision_cost\":%.2f,\"guide\":", gcost, gcoll);
    j += b + seq_json(gseq);
    const auto [seqs, weights] = solver_->get_state_seq_candidates(debug_num_candidates_);
    j += ",\"candidates\":[";
    for (size_t k = 0; k < seqs.size(); ++k) j += (k ? "," : "") + seq_json(seqs[k]);
    j += "],\"weights\":[";
    for (size_t k = 0; k < weights.size(); ++k) {
      snprintf(b, sizeof(b), "%s%.6g", k ? "," : "", weights[k]);
      j += b;
    }
    j += "],\"obstacles\":[";  // occupied costmap cells -> map frame
    const auto& data = cost_map_.get(kCollisionLayer);
    const double cy = std::cos(x0[2]), sy = std::sin(x0[2]);
    bool first = true;
    for (grid_map::GridMapIterator it(cost_map_); !it.isPastEnd(); ++it) {
      const auto idx = *it;
      if (data(idx(0), idx(1)) <= 0) continue;
      grid_map::Position p;
      cost_map_.getPosition(idx, p);
      snprintf(b, sizeof(b), "%s[%.3f,%.3f]", first ? "" : ",", x0[0] + cy * p.x() - sy * p.y(), x0[1] + sy * p.x() + cy * p.y());
      j += b;
      first = false;
    }
    j += "],\"ref\":[";
    {
      std::lock_guard<std::mutex> lk(dbg_mtx_);
      for (size_t k = 0; k < last_wps_.size(); ++k) {
        snprintf(b, sizeof(b), "%s[%.3f,%.3f]", k ? "," : "", last_wps_[k].x, last_wps_[k].y);
        j += b;
      }
    }
    j += "]}";
    snprintf(b, sizeof(b), "%s/flip_%.2f.json", debug_dump_dir_.c_str(), t);
    if (FILE* f = fopen(b, "w")) {
      fputs(j.c_str(), f);
      fclose(f);
      RCLCPP_WARN(get_logger(), "steer flip %.2f -> %.2f (collision rate %.2f) dumped to %s", prev, steer, collision_rate, b);
    }
  }

  void publish_f32(const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& pub, double v) {
    std_msgs::msg::Float32 m;
    m.data = static_cast<float>(v);
    pub->publish(m);
  }

  void publish_path(const mppi::cpu::StateSeq& seq) {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = map_frame_;
    for (int i = 0; i < seq.rows(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = seq(i, mppi::STATE_SPACE::x);
      ps.pose.position.y = seq(i, mppi::STATE_SPACE::y);
      const double yaw = seq(i, mppi::STATE_SPACE::yaw);
      ps.pose.orientation.z = std::sin(yaw / 2.0);
      ps.pose.orientation.w = std::cos(yaw / 2.0);
      path.poses.push_back(ps);
    }
    pub_best_path_->publish(path);
  }

  void publish_candidates() {
    const auto [seqs, weights] = solver_->get_state_seq_candidates(100);
    if (seqs.empty()) return;
    const double wmax = *std::max_element(weights.begin(), weights.end());
    visualization_msgs::msg::MarkerArray arr;
    for (size_t k = 0; k < seqs.size(); ++k) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = now();
      m.ns = "candidates";
      m.id = static_cast<int>(k);
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.01;
      m.color.a = 0.6;
      const double w = wmax > 0 ? weights[k] / wmax : 0.0;
      m.color.r = 0.2;
      m.color.g = 0.3 + 0.7 * w;
      m.color.b = 1.0 - w;
      m.lifetime = rclcpp::Duration::from_seconds(0.1);
      for (int i = 0; i < seqs[k].rows(); ++i) {
        geometry_msgs::msg::Point pt;
        pt.x = seqs[k](i, mppi::STATE_SPACE::x);
        pt.y = seqs[k](i, mppi::STATE_SPACE::y);
        m.points.push_back(pt);
      }
      arr.markers.push_back(m);
    }
    pub_candidates_->publish(arr);
  }

  void publish_costmap(const builtin_interfaces::msg::Time& stamp) {
    nav_msgs::msg::OccupancyGrid og;
    og.header.stamp = stamp;
    og.header.frame_id = robot_frame_;
    const double res = cost_map_.getResolution();
    const auto len = cost_map_.getLength();
    const auto c = cost_map_.getPosition();
    const int nx = static_cast<int>(std::round(len.x() / res));
    const int ny = static_cast<int>(std::round(len.y() / res));
    og.info.resolution = static_cast<float>(res);
    og.info.width = nx;
    og.info.height = ny;
    og.info.origin.position.x = c.x() - len.x() / 2.0;
    og.info.origin.position.y = c.y() - len.y() / 2.0;
    og.info.origin.orientation.w = 1.0;
    og.data.assign(static_cast<size_t>(nx) * ny, 0);
    const auto& data = cost_map_.get(kCollisionLayer);
    for (int row = 0; row < ny; ++row) {
      for (int col = 0; col < nx; ++col) {
        const grid_map::Position pos(og.info.origin.position.x + (col + 0.5) * res, og.info.origin.position.y + (row + 0.5) * res);
        grid_map::Index idx;
        if (!cost_map_.getIndex(pos, idx)) continue;
        og.data[row * nx + col] = static_cast<int8_t>(std::min(100.0, static_cast<double>(data(idx.x(), idx.y()))));
      }
    }
    pub_costmap_->publish(og);
  }

  // ---------------- members ----------------
  std::string scan_topic_, odom_topic_, waypoints_topic_, drive_topic_, map_frame_, robot_frame_, robot_frame_param_, odom_frame_;
  std::string static_map_topic_, mpc_mode_, debug_dump_dir_;
  double debug_flip_thresh_ = 0.12, last_dump_t_ = 0.0, dbg_prev_coll_ = 0.0;
  int debug_num_candidates_ = 400;
  std::mutex dbg_mtx_;
  std::vector<Pose2D> last_wps_;
  std::atomic<bool> drive_publish_enabled_{false}, visualize_candidates_{false}, constant_speed_mode_{false};
  int publish_costmap_every_ = 4, cycle_ = 0;
  std::atomic<int> thread_num_{12};
  double laser_offset_x_ = 0.275, control_sampling_time_ = 0.025, steer_1st_delay_ = 0.1, steer_1st_delay_param_ = 0.1;
  int ref_threads_ = 2;
  std::atomic<bool> pose_latency_comp_{true};
  std::atomic<double> pose_latency_max_{0.4};
  double robot_yaw_rate_ = 0.0, last_pose_latency_ = 0.0;
  double robot_recv_steady_ = 0.0, robot_recv_latency_ = 0.0;
  rclcpp::Time robot_stamp_{0, 0, RCL_ROS_TIME};
  std::atomic<double> reference_speed_{1.5};

  // solver + live rebuild
  mppi::Params params_{};
  std::mutex params_mtx_, swap_mtx_;
  std::map<std::string, Setter> solver_setters_;
  std::unique_ptr<mppi::cpu::MPCTemplate> solver_, next_solver_;
  mppi::Params next_params_{};
  std::atomic<bool> rebuild_requested_{false};

  grid_map::GridMap reference_sdf_, cost_map_, sdf_work_, ref_snapshot_;
  std::mutex sdf_mtx_;
  std::atomic<bool> is_sdf_ok_{false};
  std::atomic<double> sdf_ms_{0.0};
  double ref_update_rate_ = 0.025;
  rclcpp::CallbackGroup::SharedPtr cg_control_, cg_sdf_, cg_rebuild_;
  rclcpp::TimerBase::SharedPtr timer_sdf_, timer_rebuild_;
  std::atomic<int> ref_num_waypoints_{80};
  int ref_backward_margin_num_ = 0;
  double ref_waypoint_interval_ = 0.1, ref_map_resolution_ = 0.1, ref_map_width_ = 20.0, ref_map_height_ = 20.0;
  std::atomic<double> ref_speed_scale_{1.0}, ref_max_speed_{100.0};
  std::atomic<bool> ref_local_search_{false};
  int ref_search_back_ = 10, ref_search_fwd_ = 40, ref_jump_warn_num_ = 30, last_nearest_ = -1, sdf_cycle_ = 0;
  bool cm_crop_robot_ = true;
  double bl2front_ = 0.47, bl2rear_ = 0.14, bl2right_ = 0.15, bl2left_ = 0.15, cm_max_val_ = 100.0;
  bool cm_inflation_circle_ = true;
  double cm_circle_radius_ = 0.15;
  bool cm_filter_isolated_ = false;
  double cm_filter_radius_ = 0.10;

  // guide
  std::mutex cfg_mtx_, map_mtx_;
  GuideCfg guide_cfg_;
  std::map<std::string, double GuideCfg::*> guide_d_;
  std::shared_ptr<const StaticMap> static_map_;
  struct ObsCell {
    double t = -1e9;
    long ix = 0, iy = 0;
    int hits = 0;
    long scan_id = -1;
  };
  static constexpr int kMinHits = 3;
  long scan_id_ = 0;
  std::unordered_map<long, ObsCell> obs_mem_;               // sdf thread only
  std::vector<std::pair<double, double>> prev_guide_;       // (abs s, offset), sdf thread only
  double prev_guide_track_len_ = 0.0, track_len_ = 0.0, track_len_snapshot_ = 0.0;
  int guide_idx_ = -1;

  std::mutex mtx_;
  Pose2D robot_, scan_pose_;
  double scan_pose_dt_ = 0.0;
  struct PoseStamp {
    double t, x, y, yaw;
  };
  std::deque<PoseStamp> pose_hist_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  std::vector<Pose2D> path_;
  bool is_odom_ok_ = false, is_waypoints_ok_ = false, path_changed_ = false;
  double steer_est_ = 0.0, last_cmd_steer_ = 0.0;
  std::atomic<bool> have_laser_tf_{false};
  std::array<double, 3> laser_tf_{0.0, 0.0, 0.0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr sub_wpnts_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_best_path_, pub_ref_path_, pub_guide_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_candidates_, pub_guide_obs_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_costmap_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_calc_time_, pub_coll_rate_, pub_cost_, pub_speed_, pub_guide_offset_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SVGMPPINode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
