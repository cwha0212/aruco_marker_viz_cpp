// ArUco 마커를 인식해 '마커 정면 10cm 앞'까지 자동 접근하는 ROS2 노드 (C++).
// ★ marker_viz_node.cpp 를 그대로 복사해(검출 경로·멤버 배치 동일 = 크래시 없는 검증된 구조)
//   /cmd_vel 접근 제어만 얹었다. 시각화/맵/초기위치 코드는 메모리 배치 보존을 위해 남겨둔다
//   (구독자·marker_ids 없으면 실행 비용 거의 없음). 제어는 wall timer 없이 이미지 콜백에서 수행.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <map>

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

// 마커 map pose(파라미터) 및 로봇 map pose 캐시 (초기위치 추정용).
struct MapMarker
{
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
};
struct PoseBuffer
{
  std::vector<tf2::Vector3> pos;
  std::vector<tf2::Quaternion> q;
  rclcpp::Time last;
  bool has{false};
};

class MarkerApproachNode : public rclcpp::Node
{
public:
  MarkerApproachNode()
  : Node("marker_approach")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw/compressed");
    bool uc = declare_parameter<bool>("use_compressed", true);
    bool ends_compressed = image_topic_.size() >= 10 &&
      image_topic_.compare(image_topic_.size() - 10, 10, "compressed") == 0;
    use_compressed_ = uc || ends_compressed;
    std::string out_topic = declare_parameter<std::string>("output_topic", "~/image_markers");
    std::string dict = declare_parameter<std::string>("aruco_dict", "DICT_4X4_50");
    std::string ids = declare_parameter<std::string>("ids", "");
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
    max_reproj_ = declare_parameter<double>("max_reproj_error", 4.0);
    draw_distance_ = declare_parameter<bool>("draw_distance", true);
    draw_axes_ = declare_parameter<bool>("draw_axes", true);
    std::string odom_topic = declare_parameter<std::string>(
      "odom_topic", "/aft_mapped_to_init");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    auto lidar_rpy = declare_parameter<std::vector<double>>(
      "lidar_mount_rpy_deg", {0.0, 0.0, 0.0});
    auto cam_xyz = declare_parameter<std::vector<double>>("cam_mount_xyz", {0.0, 0.0, 0.0});
    auto cam_rpy = declare_parameter<std::vector<double>>("cam_mount_rpy_deg", {0.0, 0.0, 0.0});

    // 허용 ID 파싱
    for (char & c : ids) {if (c == ',') {c = ' ';}}
    std::istringstream iss(ids);
    int v;
    while (iss >> v) {allowed_ids_.insert(v);}
    has_id_filter_ = !allowed_ids_.empty();

    detector_ = std::make_unique<MarkerDetector>(dict);
    obj_pts_ = objectPoints(marker_size_);

    // 내부 파라미터
    intr_ = buildIntrinsics(calib_file, fx, fy, cx, cy, dist_coeffs, calib_res);
    if (intr_.ok) {
      RCLCPP_INFO(get_logger(), "intrinsics 로드 (calib_res=%dx%d)", intr_.calib_w, intr_.calib_h);
    }

    // 외부 파라미터
    ext_ = buildBodyCamExtrinsic(lidar_rpy, cam_xyz, cam_rpy);
    if (lidar_rpy == std::vector<double>{0, 0, 0} && cam_xyz == std::vector<double>{0, 0, 0} &&
      cam_rpy == std::vector<double>{0, 0, 0})
    {
      RCLCPP_WARN(get_logger(), "외부 파라미터가 모두 0 → 카메라가 라이다 원점에서 정면을 "
        "본다고 가정. 실제 장착값 설정 필요.");
    }

    auto qos = rclcpp::SensorDataQoS();
    // best-effort 로 발행(입력과 동일) → publish 논블로킹, 밀리면 최신만.
    pub_ = create_publisher<sensor_msgs::msg::Image>(out_topic, qos);
    pub_comp_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      out_topic + "/compressed", qos);
    // 이미지 발행 제어. 무거운 raw 는 기본 off, compressed 만 기본 on.
    publish_image_ = declare_parameter<bool>("publish_image", true);
    publish_raw_ = declare_parameter<bool>("publish_raw", false);
    publish_compressed_ = declare_parameter<bool>("publish_compressed", true);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 80);
    pub_map_ = create_publisher<geometry_msgs::msg::PoseArray>("~/marker_map_poses", 10);
    pub_pt_ = create_publisher<geometry_msgs::msg::PointStamped>("~/marker_map_point", 10);

    if (!intr_.ok) {
      sub_cinfo_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        cam_info_topic, qos, std::bind(&MarkerApproachNode::onCameraInfo, this, _1));
    }
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, qos, std::bind(&MarkerApproachNode::onOdom, this, _1));
    if (use_compressed_) {
      sub_cimg_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        image_topic_, qos, std::bind(&MarkerApproachNode::onCompressed, this, _1));
    } else {
      sub_img_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, qos, std::bind(&MarkerApproachNode::onRaw, this, _1));
    }

    // ── 초기위치 추정(init_pose) 기능 ──
    std::string init_topic = declare_parameter<std::string>("initialpose_topic", "/initialpose");
    declare_parameter<int>("target_marker_id", 1);
    ip_num_samples_ = std::max<int64_t>(1, declare_parameter<int>("num_samples", 5));
    ip_max_age_ = declare_parameter<double>("max_pose_age_sec", 1.0);
    ip_cov_diag_ = declare_parameter<std::vector<double>>(
      "pose_covariance_diag", {0.1, 0.1, 0.1, 0.05, 0.05, 0.05});
    auto mids = declare_parameter<std::vector<int64_t>>("marker_ids", std::vector<int64_t>{});
    for (int64_t id : mids) {
      std::string mp = "marker_" + std::to_string(id);
      auto pos = declare_parameter<std::vector<double>>(mp + ".position", {0.0, 0.0, 0.0});
      auto rpy = declare_parameter<std::vector<double>>(mp + ".rpy_deg", {0.0, 0.0, 0.0});
      if (pos.size() >= 3 && rpy.size() >= 3) {
        MapMarker m;
        m.t = tf2::Vector3(pos[0], pos[1], pos[2]);
        m.R = rpyDegToMat(rpy[0], rpy[1], rpy[2]);
        marker_map_[static_cast<int>(id)] = m;
      }
    }
    pub_init_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(init_topic, 10);
    srv_ = create_service<Trigger>(
      "~/estimate_init_pose", std::bind(&MarkerApproachNode::onEstimateInitPose, this, _1, _2));

    // ===== 접근 제어 (marker_viz 구조 위에 추가; 검출부는 위 그대로) =====
    // target_marker_id / max_pose_age_sec(=ip_max_age_) / odom 은 위에서 이미 선언·구독됨 → 재사용.
    target_id_ = static_cast<int>(get_parameter("target_marker_id").as_int());
    std::string cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    auto_start_ = declare_parameter<bool>("auto_start", false);
    final_distance_ = declare_parameter<double>("final_distance", 0.10);
    max_lin_ = declare_parameter<double>("max_linear_speed", 0.3);
    max_yaw_ = declare_parameter<double>("max_yaw_rate", 0.6);
    max_lin_acc_ = declare_parameter<double>("max_linear_accel", 0.5);
    max_yaw_acc_ = declare_parameter<double>("max_yaw_accel", 1.0);
    min_vx_ = declare_parameter<double>("min_vx", 0.05);       // 하드웨어 vx 데드존 하한
    min_vy_ = declare_parameter<double>("min_vy", 0.1);        // 하드웨어 vy 데드존 하한
    min_wz_ = declare_parameter<double>("min_yaw_rate", 0.02);  // 하드웨어 yaw 데드존 하한
    center_first_tol_ = declare_parameter<double>("center_first_tolerance", 0.05);
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
    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
    srv_start_ = create_service<Trigger>(
      "~/start", std::bind(&MarkerApproachNode::onStart, this, _1, _2));
    srv_stop_ = create_service<Trigger>(
      "~/stop", std::bind(&MarkerApproachNode::onStop, this, _1, _2));
    if (auto_start_) {state_ = State::CENTER;}

    RCLCPP_INFO(get_logger(),
      "marker_approach(C++) 시작 | in=%s cmd->%s 캘리브=%s target_id=%d 자동시작=%s",
      image_topic_.c_str(), cmd_topic.c_str(), intr_.ok ? "ON" : "OFF", target_id_,
      auto_start_ ? "ON" : "OFF");
  }

private:
  // 정렬 순서: CENTER(y 거친) → YAW(yaw만) → FINE(y 미세) → APPROACH(전진)
  enum class State { IDLE, CENTER, YAW, FINE, APPROACH, OPENLOOP, ARRIVED };

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & q = msg->pose.pose.orientation;
    const auto & p = msg->pose.pose.position;
    R_map_body_ = tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w));
    t_map_body_ = tf2::Vector3(p.x, p.y, p.z);
    have_odom_ = true;
  }

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
    RCLCPP_INFO(get_logger(), "CameraInfo 수신 → 거리/축 활성화.");
  }

  void onCompressed(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    auto t = std::chrono::steady_clock::now();
    cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    last_decode_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t).count();
    if (img.empty()) {
      RCLCPP_ERROR(get_logger(), "CompressedImage 디코드 실패");
      return;
    }
    process(img, msg->header);
  }

  void onRaw(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      process(cv_bridge::toCvCopy(msg, "bgr8")->image, msg->header);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "이미지 변환 실패: %s", e.what());
    }
  }

  void process(cv::Mat img, const std_msgs::msg::Header & header)
  {
    auto t0 = std::chrono::steady_clock::now();
    cv::Mat K = intr_.ok ? scaleK(intr_.K, intr_.calib_w, intr_.calib_h, img.cols, img.rows) :
      cv::Mat();
    auto markers = detectMarkers(*detector_, img, has_id_filter_ ? &allowed_ids_ : nullptr);
    auto t1 = std::chrono::steady_clock::now();

    // 이미지 발행 여부(config + 구독자). 발행 안 하면 그리기 자체를 건너뛴다 → 검출 풀스피드.
    bool want_comp = publish_image_ && publish_compressed_ &&
      pub_comp_->get_subscription_count() > 0;
    bool want_raw = publish_image_ && publish_raw_ && pub_->get_subscription_count() > 0;
    bool draw_needed = want_comp || want_raw;

    geometry_msgs::msg::PoseArray map_arr;
    map_arr.header.stamp = header.stamp;
    map_arr.header.frame_id = map_frame_;

    for (const auto & [mid, corners] : markers) {
      cv::Vec3d rvec, tvec;
      double reproj = -1.0;
      bool have_pose = false;
      if (intr_.ok) {
        have_pose = cv::solvePnP(obj_pts_, corners, K, intr_.dist, rvec, tvec, false,
            cv::SOLVEPNP_IPPE_SQUARE);
        if (have_pose) {
          reproj = reprojectionError(obj_pts_, corners, rvec, tvec, K, intr_.dist);
        }
      }
      bool bad = have_pose && max_reproj_ > 0 && reproj > max_reproj_;
      cv::Scalar color = bad ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
      if (draw_needed) {
        std::vector<cv::Point> poly(corners.begin(), corners.end());
        std::vector<std::vector<cv::Point>> polys{poly};
        cv::polylines(img, polys, true, color, 2);
      }

      cv::Point p0(cvRound(corners[0].x), cvRound(corners[0].y));
      std::string label = "id" + std::to_string(mid);
      if (have_pose && !bad) {
        // 접근 제어: target 마커면 제어 오차 관측 갱신.
        if (mid == target_id_) {updateObservation(rvec, tvec);}
        // 초기위치 추정: map 에 등록된 마커면 로봇 map pose 캐시.
        if (marker_map_.find(mid) != marker_map_.end()) {
          updateInitPoseCache(mid, rvec, tvec);
        }
        if (draw_needed && draw_axes_) {
          drawAxes(img, K, intr_.dist, rvec, tvec, marker_size_ * 0.5);
        }
        double d_m = std::sqrt(tvec[0] * tvec[0] + tvec[1] * tvec[1] + tvec[2] * tvec[2]);
        char buf[32];
        if (draw_distance_) {
          std::snprintf(buf, sizeof(buf), " %.2fm", d_m);
          label += buf;
        }
        geometry_msgs::msg::Pose pose;
        tf2::Matrix3x3 R_map_marker;
        if (markerPoseInMap(rvec, tvec, pose, R_map_marker)) {
          map_arr.poses.push_back(pose);
          char mb[48];
          std::snprintf(mb, sizeof(mb), " map(%.2f,%.2f,%.2f)",
            pose.position.x, pose.position.y, pose.position.z);
          label += mb;
        }
      } else if (bad) {
        label += " (bad)";
      }
      if (draw_needed) {
        cv::putText(img, label, cv::Point(p0.x, p0.y - 10), cv::FONT_HERSHEY_SIMPLEX,
          0.7, color, 2, cv::LINE_AA);
      }
    }

    auto t2 = std::chrono::steady_clock::now();
    // config(publish_*) + 구독자 있을 때만 발행. 없으면 위에서 그리기도 스킵됨.
    if (want_comp) {
      std::vector<unsigned char> buf;
      cv::imencode(".jpg", img, buf, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
      sensor_msgs::msg::CompressedImage cmsg;
      cmsg.header = header;
      cmsg.format = "jpeg";
      cmsg.data = std::move(buf);
      pub_comp_->publish(cmsg);
    }
    if (want_raw) {
      auto out = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
      pub_->publish(*out);
    }
    auto t3 = std::chrono::steady_clock::now();
    auto ms = [](auto a, auto b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
      };
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "timing[ms]: decode=%.1f detect=%.1f pnp/draw=%.1f encode/pub=%.1f | total(proc)=%.1f "
      "(markers=%zu, %dx%d)", last_decode_ms_, ms(t0, t1), ms(t1, t2), ms(t2, t3),
      last_decode_ms_ + ms(t0, t3), markers.size(), img.cols, img.rows);

    if (!map_arr.poses.empty()) {
      pub_map_->publish(map_arr);
      geometry_msgs::msg::PointStamped pt;
      pt.header = map_arr.header;
      pt.point = map_arr.poses[0].position;
      pub_pt_->publish(pt);
    }

    runControl();  // wall timer 대신 이미지 콜백 끝에서 제어 1스텝(marker_viz 처럼 timer 없음)
  }

  // 마커(카메라 광학) → map Pose. odom 없으면 false.
  bool markerPoseInMap(
    const cv::Vec3d & rvec, const cv::Vec3d & tvec,
    geometry_msgs::msg::Pose & pose, tf2::Matrix3x3 & R_map_marker_out)
  {
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "odom(/aft_mapped_to_init) 미수신 → map 좌표 계산 생략.");
      return false;
    }
    tf2::Matrix3x3 R_cam_marker = cvRodriguesToMat(rvec);
    tf2::Vector3 t_cam_marker(tvec[0], tvec[1], tvec[2]);

    tf2::Matrix3x3 R_map_cam = R_map_body_ * ext_.R_body_cam;
    tf2::Vector3 t_map_cam = R_map_body_ * ext_.t_body_cam + t_map_body_;
    tf2::Vector3 p_map = R_map_cam * t_cam_marker + t_map_cam;
    R_map_marker_out = R_map_cam * R_cam_marker;
    tf2::Quaternion q = matToQuat(R_map_marker_out);

    pose.position.x = p_map.x();
    pose.position.y = p_map.y();
    pose.position.z = p_map.z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return true;
  }

  // ── 초기위치 추정 헬퍼 ──
  void updateInitPoseCache(int mid, const cv::Vec3d & rvec, const cv::Vec3d & tvec)
  {
    // 로봇(body) map pose = T(map<-marker)[param] * inv( T(body<-cam)*T(cam<-marker) )
    tf2::Matrix3x3 R_cam_marker = cvRodriguesToMat(rvec);
    tf2::Vector3 t_cam_marker(tvec[0], tvec[1], tvec[2]);
    tf2::Matrix3x3 R_body_marker = ext_.R_body_cam * R_cam_marker;
    tf2::Vector3 t_body_marker = ext_.R_body_cam * t_cam_marker + ext_.t_body_cam;
    tf2::Matrix3x3 R_marker_body = R_body_marker.transpose();
    tf2::Vector3 t_marker_body = -(R_marker_body * t_body_marker);
    const MapMarker & m = marker_map_.at(mid);
    tf2::Matrix3x3 R_map_body = m.R * R_marker_body;
    tf2::Vector3 pos = m.R * t_marker_body + m.t;
    tf2::Quaternion quat = matToQuat(R_map_body);

    PoseBuffer & b = ip_cache_[mid];
    auto t = now();
    if (b.has && (t - b.last).seconds() > ip_max_age_) {b.pos.clear(); b.q.clear();}
    b.pos.push_back(pos);
    b.q.push_back(quat);
    while (static_cast<int64_t>(b.pos.size()) > ip_num_samples_) {
      b.pos.erase(b.pos.begin());
      b.q.erase(b.q.begin());
    }
    b.last = t;
    b.has = true;
  }

  void onEstimateInitPose(
    const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> resp)
  {
    int target = static_cast<int>(get_parameter("target_marker_id").as_int());
    if (marker_map_.find(target) == marker_map_.end()) {
      resp->success = false;
      resp->message = "marker map 에 id=" + std::to_string(target) + " 없음.";
      return;
    }
    auto it = ip_cache_.find(target);
    if (it == ip_cache_.end() || !it->second.has || it->second.pos.empty() ||
      (now() - it->second.last).seconds() > ip_max_age_)
    {
      resp->success = false;
      resp->message = "현재 id=" + std::to_string(target) +
        " 마커가 검출되지 않음(마커를 카메라에 보이게 한 뒤 호출).";
      RCLCPP_WARN(get_logger(), "[init_pose] %s", resp->message.c_str());
      return;
    }
    const PoseBuffer & b = it->second;
    tf2::Vector3 avg = ipMeanPos(b.pos);
    tf2::Quaternion aq = ipAvgQuat(b.q);
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.pose.pose.position.x = avg.x();
    msg.pose.pose.position.y = avg.y();
    msg.pose.pose.position.z = avg.z();
    msg.pose.pose.orientation.x = aq.x();
    msg.pose.pose.orientation.y = aq.y();
    msg.pose.pose.orientation.z = aq.z();
    msg.pose.pose.orientation.w = aq.w();
    for (int i = 0; i < 6; ++i) {
      msg.pose.covariance[i * 6 + i] =
        (i < static_cast<int>(ip_cov_diag_.size())) ? ip_cov_diag_[i] : 0.0;
    }
    pub_init_->publish(msg);
    auto rpy = matToRpyDeg(tf2::Matrix3x3(aq));
    char buf[176];
    std::snprintf(buf, sizeof(buf),
      "id=%d init_pose 발행 pos=(%.3f, %.3f, %.3f) yaw=%.1fdeg (n=%zu)",
      target, avg.x(), avg.y(), avg.z(), rpy[2], b.pos.size());
    resp->success = true;
    resp->message = buf;
    RCLCPP_INFO(get_logger(), "[init_pose] %s", buf);
  }

  static tf2::Vector3 ipMeanPos(const std::vector<tf2::Vector3> & v)
  {
    tf2::Vector3 s(0, 0, 0);
    for (const auto & p : v) {s += p;}
    return s / static_cast<double>(v.size());
  }

  static tf2::Quaternion ipAvgQuat(const std::vector<tf2::Quaternion> & v)
  {
    tf2::Quaternion ref = v.front();
    double x = 0, y = 0, z = 0, w = 0;
    for (auto q : v) {
      if (q.dot(ref) < 0) {q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());}
      x += q.x(); y += q.y(); z += q.z(); w += q.w();
    }
    tf2::Quaternion out(x, y, z, w);
    double n = out.length();
    return n > 0 ? out / n : ref;
  }

  // ================= 접근 제어 =================
  // 카메라 프레임 pose → 바디 FLU 투영 → 제어 오차(EMA 필터) 갱신.
  void updateObservation(const cv::Vec3d & rvec, const cv::Vec3d & tvec)
  {
    tf2::Matrix3x3 R_cam_marker = cvRodriguesToMat(rvec);
    tf2::Vector3 t_cam_marker(tvec[0], tvec[1], tvec[2]);
    tf2::Matrix3x3 R_body_marker = ext_.R_body_cam * R_cam_marker;
    tf2::Vector3 t_body_marker = ext_.R_body_cam * t_cam_marker + ext_.t_body_cam;
    tf2::Vector3 n = R_body_marker.getColumn(2);   // 바디기준 마커 z축(법선)
    double raw_z = tvec[2];                          // 카메라 z거리
    double raw_center = t_body_marker.y();           // 중앙정렬 오차(바디 y)
    double raw_axis = std::atan2(-n.y(), -n.x());    // z축 정렬 오차(전방∥법선 반대일 때 0)
    auto t = now();
    double a = filter_alpha_;
    if (!obs_has_ || (t - obs_stamp_).seconds() > ip_max_age_) {
      f_z_ = raw_z; f_center_ = raw_center; f_axis_ = raw_axis;
    } else {
      f_z_ = a * raw_z + (1.0 - a) * f_z_;
      f_center_ = a * raw_center + (1.0 - a) * f_center_;
      f_axis_ = wrapToPi(f_axis_ + a * wrapToPi(raw_axis - f_axis_));
    }
    obs_stamp_ = t;
    obs_has_ = true;
  }

  void onStart(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> resp)
  {
    if (!intr_.ok) {
      resp->success = false; resp->message = "카메라 캘리브레이션이 없어 접근 불가.";
      return;
    }
    state_ = State::CENTER; have_prev_ = false;
    resp->success = true; resp->message = "접근 시작(CENTER).";
    RCLCPP_INFO(get_logger(), "[approach] 시작 → CENTER(1차 중앙정렬)");
  }

  void onStop(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> resp)
  {
    state_ = State::IDLE; publishStop();
    resp->success = true; resp->message = "접근 정지(IDLE).";
    RCLCPP_INFO(get_logger(), "[approach] 정지 → IDLE");
  }

  // 이미지 콜백 끝에서 호출되는 제어 1스텝(wall timer 없음).
  void runControl()
  {
    if (state_ == State::IDLE || state_ == State::ARRIVED) {publishStop(); return;}

    auto tnow = std::chrono::steady_clock::now();
    double dt = ctrl_time_init_ ?
      std::chrono::duration<double>(tnow - last_ctrl_time_).count() : 0.05;
    if (dt <= 0.0 || dt > 1.0) {dt = 0.05;}
    last_ctrl_time_ = tnow; ctrl_time_init_ = true;

    bool fresh = obs_has_ && (now() - obs_stamp_).seconds() <= ip_max_age_;

    // OPENLOOP: 근접에서 마커가 화면 벗어나도 마지막 구간을 무피드백 직진으로 마무리.
    if (state_ == State::OPENLOOP) {
      if (fresh) {
        state_ = State::APPROACH;
        RCLCPP_INFO(get_logger(), "마커 재획득 → APPROACH");
      } else {
        bool done = false; double traveled = 0.0;
        if (ol_use_odom_ && have_odom_) {  // odom(map) 진행거리
          traveled = std::hypot(t_map_body_.x() - ol_start_x_, t_map_body_.y() - ol_start_y_);
          done = traveled >= ol_remain_;
        }
        if (done || now() >= ol_deadline_) {
          publishStop(); state_ = State::ARRIVED;
          RCLCPP_INFO(get_logger(), "열린루프 종료 → ARRIVED(이동 %.3f/%.3fm, %s)",
            traveled, ol_remain_, done ? "odom" : "time");
          return;
        }
        publishSmoothed(openloop_speed_, 0.0, 0.0, dt);
        return;
      }
    }

    if (!fresh) {  // 마커 소실
      if (state_ == State::APPROACH && openloop_enabled_ && obs_has_ &&
        f_z_ <= openloop_enter_dist_)
      {
        ol_remain_ = std::clamp(f_z_ - final_distance_, 0.0, openloop_max_dist_);
        if (ol_remain_ <= 1e-3) {
          publishStop(); state_ = State::ARRIVED;
          RCLCPP_INFO(get_logger(), "소실 시 이미 목표거리 → ARRIVED (z=%.3f)", f_z_);
          return;
        }
        ol_use_odom_ = have_odom_;
        ol_start_x_ = t_map_body_.x(); ol_start_y_ = t_map_body_.y();
        double t_lim = ol_use_odom_ ? (ol_remain_ / openloop_speed_ * 2.0 + 1.0)
                                    : (ol_remain_ / openloop_speed_);
        ol_deadline_ = now() + rclcpp::Duration::from_seconds(t_lim);
        have_prev_ = false; state_ = State::OPENLOOP;
        RCLCPP_WARN(get_logger(), "근접 소실(z=%.3f) → 열린루프 %.3fm 직진(%s)",
          f_z_, ol_remain_, ol_use_odom_ ? "odom" : "time");
        return;
      }
      publishStop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "마커 미검출/소실 → 정지 (target_id=%d)", target_id_);
      return;
    }

    // 폐루프 제어: 축을 순차 분리. CENTER(y거친) → YAW(yaw만) → FINE(y미세) → APPROACH(x+보정)
    double vx = 0.0, vy = 0.0, wz = 0.0;
    bool center_ok = std::abs(f_center_) < lat_tol_;
    bool axis_ok = std::abs(f_axis_) < yaw_tol_;

    if (state_ == State::CENTER) {          // 1) y축 거친 정렬 (vy만)
      if (std::abs(f_center_) >= center_first_tol_) {
        vy = kp_lat_ * f_center_;
      } else {
        state_ = State::YAW;
        RCLCPP_INFO(get_logger(), "1차 중앙정렬 완료 → YAW(정면 회전)");
      }
    } else if (state_ == State::YAW) {       // 2) yaw 정렬 (wz만)
      if (!axis_ok) {
        wz = kp_yaw_ * f_axis_;
      } else {
        state_ = State::FINE;
        RCLCPP_INFO(get_logger(), "yaw 정렬 완료 → FINE(y 미세정렬)");
      }
    } else if (state_ == State::FINE) {       // 3) y축 미세 정렬 (vy만; yaw로 틀어진 중앙 보정)
      if (!center_ok) {
        vy = kp_lat_ * f_center_;
      } else {
        state_ = State::APPROACH;
        RCLCPP_INFO(get_logger(), "미세 중앙정렬 완료 → APPROACH (z=%.3fm)", f_z_);
      }
    } else {  // 4) APPROACH (전진 + 정렬 유지 보정)
      if (std::abs(f_z_ - final_distance_) < dist_tol_) {
        publishStop(); state_ = State::ARRIVED;
        RCLCPP_INFO(get_logger(), "도착 완료(ARRIVED): z=%.3fm (목표 %.3fm)", f_z_, final_distance_);
        return;
      }
      vx = kp_fwd_ * (f_z_ - final_distance_);
      if (!center_ok) {vy = kp_lat_ * f_center_;}
      if (!axis_ok) {wz = kp_yaw_ * f_axis_;}
    }

    publishSmoothed(vx, vy, wz, dt);
    const char * sn = state_ == State::CENTER ? "CENTER" :
      (state_ == State::YAW ? "YAW" : (state_ == State::FINE ? "FINE" : "APPROACH"));
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[%s] z=%.3f e_center=%.3f e_axis=%.1fdeg | cmd v=(%.2f,%.2f) w=%.2f",
      sn, f_z_, f_center_, rad2deg(f_axis_), last_vx_, last_vy_, last_wz_);
  }

  void publishSmoothed(double vx, double vy, double wz, double dt)
  {
    // 1) 선속도 벡터 포화 + 각속도 클램프
    double sp = std::hypot(vx, vy);
    if (sp > max_lin_ && sp > 0.0) {double s = max_lin_ / sp; vx *= s; vy *= s;}
    wz = std::clamp(wz, -max_yaw_, max_yaw_);
    // 의도적으로 0 이 아닌 축 기록 → floor 는 이 축에만. (정렬 완료로 0 인 축은 0 으로 수렴)
    bool nz_vx = std::abs(vx) > 1e-9, nz_vy = std::abs(vy) > 1e-9, nz_wz = std::abs(wz) > 1e-9;
    // 2) 가속 제한(slew)
    if (have_prev_) {
      double dv = max_lin_acc_ * dt, dw = max_yaw_acc_ * dt;
      vx = last_vx_ + std::clamp(vx - last_vx_, -dv, dv);
      vy = last_vy_ + std::clamp(vy - last_vy_, -dv, dv);
      wz = last_wz_ + std::clamp(wz - last_wz_, -dw, dw);
    }
    // 3) 하드웨어 최소속도 floor(축별). 데드존 아래로 못 가므로 명령을 최소값까지 끌어올림.
    //    원래 0 이던 축은 건드리지 않아 정지 시 0 으로 수렴.
    if (nz_vx && std::abs(vx) < min_vx_) {vx = std::copysign(min_vx_, vx);}
    if (nz_vy && std::abs(vy) < min_vy_) {vy = std::copysign(min_vy_, vy);}
    if (nz_wz && std::abs(wz) < min_wz_) {wz = std::copysign(min_wz_, wz);}

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx; cmd.linear.y = vy; cmd.angular.z = wz;
    pub_cmd_->publish(cmd);
    last_vx_ = vx; last_vy_ = vy; last_wz_ = wz; have_prev_ = true;
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist cmd;
    pub_cmd_->publish(cmd);
    last_vx_ = last_vy_ = last_wz_ = 0.0; have_prev_ = true;
  }

  std::string image_topic_, map_frame_;
  bool use_compressed_{true}, has_id_filter_{false}, draw_distance_{true}, draw_axes_{true};
  double marker_size_{0.185}, max_reproj_{4.0};
  std::set<int> allowed_ids_;

  std::unique_ptr<MarkerDetector> detector_;
  std::vector<cv::Point3f> obj_pts_;
  Intrinsics intr_;
  Extrinsic ext_;

  bool have_odom_{false};
  tf2::Matrix3x3 R_map_body_;
  tf2::Vector3 t_map_body_;
  double last_decode_ms_{0.0};

  bool publish_image_{true}, publish_raw_{false}, publish_compressed_{true};
  int jpeg_quality_{80};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_comp_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_map_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_pt_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_cimg_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cinfo_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  // 초기위치 추정
  std::map<int, MapMarker> marker_map_;
  std::map<int, PoseBuffer> ip_cache_;
  int64_t ip_num_samples_{5};
  double ip_max_age_{1.0};
  std::vector<double> ip_cov_diag_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_init_;
  rclcpp::Service<Trigger>::SharedPtr srv_;

  // ── 접근 제어 ──
  int target_id_{1};
  bool auto_start_{false}, openloop_enabled_{true};
  double final_distance_{0.10};
  double max_lin_{0.15}, max_yaw_{0.6}, max_lin_acc_{0.5}, max_yaw_acc_{1.0};
  double min_vx_{0.05}, min_vy_{0.1}, min_wz_{0.02}, center_first_tol_{0.05};
  double kp_fwd_{0.8}, kp_lat_{1.0}, kp_yaw_{1.2};
  double dist_tol_{0.02}, lat_tol_{0.03}, yaw_tol_{deg2rad(5.0)};
  double filter_alpha_{0.5};
  double openloop_enter_dist_{0.35}, openloop_speed_{0.1}, openloop_max_dist_{0.3};
  State state_{State::IDLE};
  bool obs_has_{false};
  double f_z_{0.0}, f_center_{0.0}, f_axis_{0.0};
  rclcpp::Time obs_stamp_;
  double last_vx_{0.0}, last_vy_{0.0}, last_wz_{0.0};
  bool have_prev_{false}, ctrl_time_init_{false};
  std::chrono::steady_clock::time_point last_ctrl_time_;
  bool ol_use_odom_{false};
  double ol_remain_{0.0}, ol_start_x_{0.0}, ol_start_y_{0.0};
  rclcpp::Time ol_deadline_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Service<Trigger>::SharedPtr srv_start_, srv_stop_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerApproachNode>());
  rclcpp::shutdown();
  return 0;
}
