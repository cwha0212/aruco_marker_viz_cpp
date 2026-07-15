// ArUco 마커를 인식해 '마커 정면 10cm 앞'까지 자동 접근하는 ROS2 노드 (C++).
//
// ★ 검출 경로(멤버 선언순서·검출기 생성 위치·process() 검출부)는
//   init_pose_from_marker_node 와 "완전히 동일"하게 유지한다(이 환경의 OpenCV setSize
//   크래시가 노드 메모리 배치에 민감해, 동작하는 노드와 검출부를 동일하게 두는 게 중요).
//   그 뒤에 /cmd_vel 제어 로직만 얹는다.
//
// 로봇은 4족보행(홀로노믹 평면): x(전진)·y(횡이동)·yaw 만 제어. 높이(z-up)는 제어 불가라
// 전부 지면 평면에 투영해 무시한다. 접근 2단계:
//   1) ALIGN   : 전진 없이 중앙정렬(vy)+z축 정렬(yaw) 로 마커 정면 자세를 만든다.
//   2) APPROACH: 정면 유지한 채, 카메라 z거리(tvec[2]) 가 final_distance(0.10m) 될 때까지 직진.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "aruco_marker_viz_cpp/aruco_common.hpp"

using namespace aruco_common;
using std::placeholders::_1;
using std::placeholders::_2;
using Trigger = std_srvs::srv::Trigger;

static inline double wrapToPi(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

class MarkerApproachNode : public rclcpp::Node
{
public:
  MarkerApproachNode()
  : Node("marker_approach")
  {
    // ===== 여기부터 init_pose_from_marker_node 와 동일한 '검출 설정' =====
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw/compressed");
    bool uc = declare_parameter<bool>("use_compressed", true);
    bool ends = image_topic_.size() >= 10 &&
      image_topic_.compare(image_topic_.size() - 10, 10, "compressed") == 0;
    use_compressed_ = uc || ends;
    std::string dict = declare_parameter<std::string>("aruco_dict", "DICT_4X4_50");
    marker_size_ = declare_parameter<double>("marker_size", 0.185);
    std::string calib_file = declare_parameter<std::string>("calib_file", "");
    std::string cam_info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera_info");
    double fx = declare_parameter<double>("fx", 0.0);
    double fy = declare_parameter<double>("fy", 0.0);
    double cx = declare_parameter<double>("cx", 0.0);
    double cy = declare_parameter<double>("cy", 0.0);
    auto dist_coeffs = declare_parameter<std::vector<double>>(
      "dist_coeffs", {0.0, 0.0, 0.0, 0.0, 0.0});
    auto calib_res = declare_parameter<std::vector<int64_t>>("calib_resolution", {0, 0});
    auto lidar_rpy = declare_parameter<std::vector<double>>(
      "lidar_mount_rpy_deg", {0.0, 0.0, 0.0});
    auto cam_xyz = declare_parameter<std::vector<double>>("cam_mount_xyz", {0.0, 0.0, 0.0});
    auto cam_rpy = declare_parameter<std::vector<double>>("cam_mount_rpy_deg", {0.0, 0.0, 0.0});
    target_id_ = declare_parameter<int>("target_marker_id", 1);
    max_reproj_ = declare_parameter<double>("max_reproj_error", 4.0);
    max_pose_age_ = declare_parameter<double>("max_pose_age_sec", 0.5);

    detector_ = std::make_unique<MarkerDetector>(dict);
    obj_pts_ = objectPoints(marker_size_);
    intr_ = buildIntrinsics(calib_file, fx, fy, cx, cy, dist_coeffs, calib_res);
    ext_ = buildBodyCamExtrinsic(lidar_rpy, cam_xyz, cam_rpy);
    // ===== 여기까지 검출 설정 (init_pose 와 동일) =====

    // ----- 접근 제어 설정 (추가분) -----
    std::string cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    auto_start_ = declare_parameter<bool>("auto_start", false);
    double rate = declare_parameter<double>("control_rate_hz", 20.0);
    final_distance_ = declare_parameter<double>("final_distance", 0.10);
    max_lin_ = declare_parameter<double>("max_linear_speed", 0.3);
    max_yaw_ = declare_parameter<double>("max_yaw_rate", 0.6);
    max_lin_acc_ = declare_parameter<double>("max_linear_accel", 0.5);
    max_yaw_acc_ = declare_parameter<double>("max_yaw_accel", 1.0);
    min_lin_ = declare_parameter<double>("min_linear_speed", 0.02);
    kp_fwd_ = declare_parameter<double>("kp_forward", 0.8);
    kp_lat_ = declare_parameter<double>("kp_lateral", 1.0);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.2);
    dist_tol_ = declare_parameter<double>("distance_tolerance", 0.02);
    lat_tol_ = declare_parameter<double>("lateral_tolerance", 0.03);
    yaw_tol_ = deg2rad(declare_parameter<double>("yaw_tolerance_deg", 5.0));
    filter_alpha_ = declare_parameter<double>("obs_filter_alpha", 0.5);
    openloop_enabled_ = declare_parameter<bool>("openloop_final_approach", true);
    openloop_enter_dist_ = declare_parameter<double>("openloop_enter_distance", 0.35);
    openloop_speed_ = declare_parameter<double>("openloop_speed", 0.1);
    openloop_max_dist_ = declare_parameter<double>("openloop_max_distance", 0.3);
    use_odom_ = declare_parameter<bool>("use_odom", true);
    std::string odom_topic = declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    auto qos = rclcpp::SensorDataQoS();
    if (!intr_.ok) {
      sub_cinfo_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        cam_info_topic, qos, std::bind(&MarkerApproachNode::onCameraInfo, this, _1));
    }
    if (use_compressed_) {
      sub_cimg_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        image_topic_, qos, std::bind(&MarkerApproachNode::onCompressed, this, _1));
    } else {
      sub_img_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, qos, std::bind(&MarkerApproachNode::onRaw, this, _1));
    }
    if (use_odom_) {
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, qos, std::bind(&MarkerApproachNode::onOdom, this, _1));
    }
    srv_start_ = create_service<Trigger>(
      "~/start", std::bind(&MarkerApproachNode::onStart, this, _1, _2));
    srv_stop_ = create_service<Trigger>(
      "~/stop", std::bind(&MarkerApproachNode::onStop, this, _1, _2));

    if (auto_start_) {state_ = State::ALIGN;}
    rate = rate > 0.0 ? rate : 20.0;
    last_dt_ = 1.0 / rate;
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MarkerApproachNode::onControl, this));

    RCLCPP_INFO(get_logger(),
      "marker_approach(C++) 시작 | in=%s cmd->%s 캘리브=%s target_id=%d 자동시작=%s odom=%s",
      image_topic_.c_str(), cmd_topic.c_str(), intr_.ok ? "ON" : "OFF", target_id_,
      auto_start_ ? "ON" : "OFF", use_odom_ ? odom_topic.c_str() : "OFF");
  }

private:
  enum class State { IDLE, ALIGN, APPROACH, OPENLOOP, ARRIVED };

  // ---------------- 검출 경로: init_pose_from_marker_node 와 동일 ----------------
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (intr_.ok) {return;}
    intr_.K = (cv::Mat_<double>(3, 3) <<
      msg->k[0], msg->k[1], msg->k[2], msg->k[3], msg->k[4], msg->k[5],
      msg->k[6], msg->k[7], msg->k[8]);
    if (!msg->d.empty()) {
      intr_.dist = cv::Mat(1, static_cast<int>(msg->d.size()), CV_64F);
      for (size_t i = 0; i < msg->d.size(); ++i) {
        intr_.dist.at<double>(0, static_cast<int>(i)) = msg->d[i];
      }
    } else {
      intr_.dist = cv::Mat::zeros(1, 5, CV_64F);
    }
    intr_.ok = true;
    RCLCPP_INFO(get_logger(), "CameraInfo 수신 → 캘리브 활성화.");
  }

  void onCompressed(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (img.empty()) {
      RCLCPP_ERROR(get_logger(), "CompressedImage 디코드 실패");
      return;
    }
    process(img);
  }

  void onRaw(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      process(cv_bridge::toCvCopy(msg, "bgr8")->image);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "이미지 변환 실패: %s", e.what());
    }
  }

  void process(cv::Mat img)
  {
    // [임시 진단] 검출 직전 이미지 상태 출력 + detectMarkers 예외 포착(크래시 대신 로그).
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[diag] before detect: img=%dx%d ch=%d type=%d intr.ok=%d",
      img.cols, img.rows, img.channels(), img.type(), intr_.ok ? 1 : 0);
    cv::Mat K = intr_.ok ? scaleK(intr_.K, intr_.calib_w, intr_.calib_h, img.cols, img.rows) :
      cv::Mat();
    std::vector<Detection> markers;
    try {
      markers = detectMarkers(*detector_, img, nullptr);  // init_pose 와 동일 (nullptr)
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR(get_logger(), "[diag] detectMarkers 예외 (img=%dx%d ch=%d): %s",
        img.cols, img.rows, img.channels(), e.what());
      return;
    }
    auto t = now();

    for (const auto & [mid, corners] : markers) {
      if (mid != target_id_) {continue;}       // target 마커만
      if (!intr_.ok) {continue;}
      cv::Vec3d rvec, tvec;
      if (!cv::solvePnP(obj_pts_, corners, K, intr_.dist, rvec, tvec, false,
        cv::SOLVEPNP_IPPE_SQUARE))
      {
        continue;
      }
      double reproj = reprojectionError(obj_pts_, corners, rvec, tvec, K, intr_.dist);
      if (max_reproj_ > 0 && reproj > max_reproj_) {continue;}

      // ↓↓↓ 여기까지 검출 경로는 init_pose 와 동일. 이후는 접근 제어용 관측 계산. ↓↓↓
      updateObservation(rvec, tvec, t);
    }
  }

  // 카메라 프레임 pose → 바디 FLU 투영 → 제어 오차(EMA 필터) 갱신.
  void updateObservation(const cv::Vec3d & rvec, const cv::Vec3d & tvec, const rclcpp::Time & t)
  {
    tf2::Matrix3x3 R_cam_marker = cvRodriguesToMat(rvec);
    tf2::Vector3 t_cam_marker(tvec[0], tvec[1], tvec[2]);
    tf2::Matrix3x3 R_body_marker = ext_.R_body_cam * R_cam_marker;
    tf2::Vector3 t_body_marker = ext_.R_body_cam * t_cam_marker + ext_.t_body_cam;

    tf2::Vector3 n = R_body_marker.getColumn(2);   // 바디기준 마커 z축(법선)
    double raw_z = tvec[2];                          // 카메라 z거리
    double raw_center = t_body_marker.y();           // 중앙정렬 오차(바디 y)
    double raw_axis = std::atan2(-n.y(), -n.x());    // z축 정렬 오차(전방∥법선 반대일 때 0)

    double a = filter_alpha_;
    if (!obs_has_ || (t - obs_stamp_).seconds() > max_pose_age_) {  // 소실 후 재획득이면 리셋
      f_z_ = raw_z; f_center_ = raw_center; f_axis_ = raw_axis;
    } else {
      f_z_ = a * raw_z + (1.0 - a) * f_z_;
      f_center_ = a * raw_center + (1.0 - a) * f_center_;
      f_axis_ = wrapToPi(f_axis_ + a * wrapToPi(raw_axis - f_axis_));
    }
    obs_stamp_ = t;
    obs_has_ = true;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;
    odom_stamp_ = now();
    odom_has_ = true;
  }

  bool odomFresh() {return odom_has_ && (now() - odom_stamp_).seconds() < 1.0;}

  // ---------------- 서비스 ----------------
  void onStart(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> resp)
  {
    if (!intr_.ok) {
      resp->success = false;
      resp->message = "카메라 캘리브레이션이 없어 접근 불가.";
      return;
    }
    state_ = State::ALIGN;
    have_prev_ = false;
    resp->success = true;
    resp->message = "접근 시작(ALIGN).";
    RCLCPP_INFO(get_logger(), "[service] 접근 시작 → ALIGN");
  }

  void onStop(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> resp)
  {
    state_ = State::IDLE;
    publishStop();
    resp->success = true;
    resp->message = "접근 정지(IDLE).";
    RCLCPP_INFO(get_logger(), "[service] 접근 정지 → IDLE");
  }

  // ---------------- 제어 루프 ----------------
  void onControl()
  {
    if (state_ == State::IDLE || state_ == State::ARRIVED) {
      publishStop();
      return;
    }
    bool fresh = obs_has_ && (now() - obs_stamp_).seconds() <= max_pose_age_;

    // OPENLOOP: 근접에서 마커가 화면을 벗어나도 마지막 구간을 무피드백 직진으로 마무리.
    if (state_ == State::OPENLOOP) {
      if (fresh) {
        state_ = State::APPROACH;
        RCLCPP_INFO(get_logger(), "마커 재획득 → APPROACH");
      } else {
        bool done = false;
        double traveled = 0.0;
        if (ol_use_odom_ && odomFresh()) {
          traveled = std::hypot(odom_x_ - ol_start_x_, odom_y_ - ol_start_y_);
          done = traveled >= ol_remain_;
        }
        if (done || now() >= ol_deadline_) {
          publishStop();
          state_ = State::ARRIVED;
          RCLCPP_INFO(get_logger(), "열린루프 종료 → ARRIVED(이동 %.3f/%.3fm, %s)",
            traveled, ol_remain_, done ? "odom" : "timeout");
          return;
        }
        publishSmoothed(openloop_speed_, 0.0, 0.0);
        return;
      }
    }

    if (!fresh) {  // 마커 소실
      if (state_ == State::APPROACH && openloop_enabled_ && obs_has_ &&
        f_z_ <= openloop_enter_dist_)
      {
        ol_remain_ = std::clamp(f_z_ - final_distance_, 0.0, openloop_max_dist_);
        if (ol_remain_ <= 1e-3) {
          publishStop();
          state_ = State::ARRIVED;
          RCLCPP_INFO(get_logger(), "소실 시 이미 목표거리 → ARRIVED (z=%.3f)", f_z_);
          return;
        }
        ol_use_odom_ = use_odom_ && odomFresh();
        ol_start_x_ = odom_x_; ol_start_y_ = odom_y_;
        double t_lim = ol_use_odom_ ? (ol_remain_ / openloop_speed_ * 2.0 + 1.0)
                                    : (ol_remain_ / openloop_speed_);
        ol_deadline_ = now() + rclcpp::Duration::from_seconds(t_lim);
        have_prev_ = false;
        state_ = State::OPENLOOP;
        RCLCPP_WARN(get_logger(), "근접 소실(z=%.3f) → 열린루프 %.3fm 직진(%.2f m/s, %s)",
          f_z_, ol_remain_, openloop_speed_, ol_use_odom_ ? "odom" : "time");
        return;
      }
      publishStop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "마커 미검출/소실 → 정지 (target_id=%d)", target_id_);
      return;
    }

    // 폐루프 제어 (오차가 tolerance 이내면 해당 축 0)
    double vx = 0.0, vy = 0.0, wz = 0.0;
    bool center_ok = std::abs(f_center_) < lat_tol_;
    bool axis_ok = std::abs(f_axis_) < yaw_tol_;
    if (!center_ok) {vy = kp_lat_ * f_center_;}   // 중앙정렬
    if (!axis_ok) {wz = kp_yaw_ * f_axis_;}        // z축 정렬

    if (state_ == State::ALIGN) {
      if (center_ok && axis_ok) {
        state_ = State::APPROACH;
        RCLCPP_INFO(get_logger(), "정면 정렬 완료 → APPROACH (z=%.3fm)", f_z_);
      }
      // vx = 0 (ALIGN 은 전진하지 않음)
    } else {  // APPROACH
      if (std::abs(f_z_ - final_distance_) < dist_tol_) {
        publishStop();
        state_ = State::ARRIVED;
        RCLCPP_INFO(get_logger(), "도착 완료(ARRIVED): z=%.3fm (목표 %.3fm)",
          f_z_, final_distance_);
        return;
      }
      vx = kp_fwd_ * (f_z_ - final_distance_);
    }

    publishSmoothed(vx, vy, wz);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[%s] z=%.3f e_center=%.3f e_axis=%.1fdeg | cmd v=(%.2f,%.2f) w=%.2f",
      state_ == State::ALIGN ? "ALIGN" : "APPROACH",
      f_z_, f_center_, rad2deg(f_axis_), last_vx_, last_vy_, last_wz_);
  }

  // 속도 포화 → 최소속도 floor → 가속 제한(slew) 후 발행.
  void publishSmoothed(double vx, double vy, double wz)
  {
    double sp = std::hypot(vx, vy);
    if (sp > max_lin_ && sp > 0.0) {double s = max_lin_ / sp; vx *= s; vy *= s;}
    wz = std::clamp(wz, -max_yaw_, max_yaw_);

    sp = std::hypot(vx, vy);   // 데드밴드 대신 floor: 도착 전 멈춤(stall) 방지
    if (sp > 1e-6 && sp < min_lin_) {double s = min_lin_ / sp; vx *= s; vy *= s;}

    double dt = last_dt_ > 0.0 ? last_dt_ : (1.0 / 20.0);
    if (have_prev_) {
      double dv = max_lin_acc_ * dt, dw = max_yaw_acc_ * dt;
      vx = last_vx_ + std::clamp(vx - last_vx_, -dv, dv);
      vy = last_vy_ + std::clamp(vy - last_vy_, -dv, dv);
      wz = last_wz_ + std::clamp(wz - last_wz_, -dw, dw);
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.angular.z = wz;
    pub_cmd_->publish(cmd);
    last_vx_ = vx; last_vy_ = vy; last_wz_ = wz;
    have_prev_ = true;
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist cmd;
    pub_cmd_->publish(cmd);
    last_vx_ = last_vy_ = last_wz_ = 0.0;
    have_prev_ = true;
  }

  // ---- 파라미터/상수 (검출) ----
  std::string image_topic_;
  bool use_compressed_{true};
  int target_id_{1};
  double marker_size_{0.185}, max_reproj_{4.0}, max_pose_age_{0.5};

  // ---- 검출 자원 (init_pose 와 동일 순서) ----
  std::unique_ptr<MarkerDetector> detector_;
  std::vector<cv::Point3f> obj_pts_;
  Intrinsics intr_;
  Extrinsic ext_;

  // ---- 제어 파라미터 ----
  bool auto_start_{false}, openloop_enabled_{true}, use_odom_{true};
  double final_distance_{0.10};
  double max_lin_{0.3}, max_yaw_{0.6}, max_lin_acc_{0.5}, max_yaw_acc_{1.0}, min_lin_{0.02};
  double kp_fwd_{0.8}, kp_lat_{1.0}, kp_yaw_{1.2};
  double dist_tol_{0.02}, lat_tol_{0.03}, yaw_tol_{deg2rad(5.0)};
  double filter_alpha_{0.5};
  double openloop_enter_dist_{0.35}, openloop_speed_{0.1}, openloop_max_dist_{0.3};

  // ---- 상태 ----
  State state_{State::IDLE};
  bool obs_has_{false};
  double f_z_{0.0}, f_center_{0.0}, f_axis_{0.0};
  rclcpp::Time obs_stamp_;
  double last_vx_{0.0}, last_vy_{0.0}, last_wz_{0.0}, last_dt_{0.05};
  bool have_prev_{false};
  bool odom_has_{false};
  double odom_x_{0.0}, odom_y_{0.0};
  rclcpp::Time odom_stamp_;
  bool ol_use_odom_{false};
  double ol_remain_{0.0}, ol_start_x_{0.0}, ol_start_y_{0.0};
  rclcpp::Time ol_deadline_;

  // ---- ROS 인터페이스 ----
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_cimg_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cinfo_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Service<Trigger>::SharedPtr srv_start_, srv_stop_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerApproachNode>());
  rclcpp::shutdown();
  return 0;
}
