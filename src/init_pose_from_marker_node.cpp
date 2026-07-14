// 알려진 ArUco 마커로 로봇 초기위치를 추정해 /initialpose 로 발행하는 ROS2 노드 (C++).
// std_srvs/Trigger 호출 시 target_marker_id 마커를 인식할 때까지 탐색 → 평균 → 발행.
//   init_pose(body in map) = T(map<-marker)[YAML] * inv( T(body<-cam)*T(cam<-marker) )
//
// ★ 검출은 반드시 '메인 스레드'에서 실행한다(marker_viz 와 동일). 이 커스텀 OpenCV
//   빌드에서 detectMarkers 를 워커 스레드에서 돌리면 힙 손상이 나기 때문.
//   구조: 이미지 콜백(default group)은 메인 스레드 실행기가, 서비스(별도 group)는
//   별도 스레드 실행기가 spin. 서비스는 조건변수로 대기(블로킹), 이미지 콜백(메인
//   스레드)이 마커를 인식하면 발행 후 서비스를 깨운다.
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "aruco_marker_viz_cpp/aruco_common.hpp"

using namespace aruco_common;
using std::placeholders::_1;
using std::placeholders::_2;
using Trigger = std_srvs::srv::Trigger;

struct MarkerPose
{
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
};

class InitPoseNode : public rclcpp::Node
{
public:
  InitPoseNode()
  : Node("init_pose_from_marker")
  {
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
    std::string marker_map_file = declare_parameter<std::string>("marker_map_file", "");
    declare_parameter<int>("target_marker_id", 1);
    std::string init_topic = declare_parameter<std::string>("initialpose_topic", "/initialpose");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    max_reproj_ = declare_parameter<double>("max_reproj_error", 4.0);
    num_samples_ = std::max<int64_t>(1, declare_parameter<int>("num_samples", 5));
    search_timeout_ = declare_parameter<double>("search_timeout_sec", 0.0);
    cov_diag_ = declare_parameter<std::vector<double>>(
      "pose_covariance_diag", {0.1, 0.1, 0.1, 0.05, 0.05, 0.05});

    detector_ = std::make_unique<MarkerDetector>(dict);
    obj_pts_ = objectPoints(marker_size_);
    intr_ = buildIntrinsics(calib_file, fx, fy, cx, cy, dist_coeffs, calib_res);
    ext_ = buildBodyCamExtrinsic(lidar_rpy, cam_xyz, cam_rpy);

    if (marker_map_file.empty()) {
      RCLCPP_ERROR(get_logger(), "marker_map_file 파라미터가 비어있습니다.");
    } else if (!loadMarkerMap(marker_map_file)) {
      RCLCPP_ERROR(get_logger(), "marker_map 로드 실패: %s", marker_map_file.c_str());
    }

    pub_init_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(init_topic, 10);

    // 서비스는 별도 콜백그룹 → 별도 스레드 실행기(main() 에서 분배).
    srv_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto qos = rclcpp::SensorDataQoS();
    // 이미지/카메라인포는 default group → 메인 스레드 실행기가 처리(검출 메인 스레드).
    if (use_compressed_) {
      sub_cimg_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        image_topic_, qos, std::bind(&InitPoseNode::onCompressed, this, _1));
    } else {
      sub_img_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, qos, std::bind(&InitPoseNode::onRaw, this, _1));
    }
    if (!intr_.ok) {
      sub_cinfo_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        cam_info_topic, qos, std::bind(&InitPoseNode::onCameraInfo, this, _1));
    }
    srv_ = create_service<Trigger>(
      "~/estimate_init_pose", std::bind(&InitPoseNode::onService, this, _1, _2),
      rmw_qos_profile_services_default, srv_cbg_);

    RCLCPP_INFO(get_logger(), "init_pose_from_marker(C++) 시작 | in=%s initialpose->%s 캘리브=%s",
      image_topic_.c_str(), init_topic.c_str(), intr_.ok ? "ON" : "OFF");
  }

  rclcpp::CallbackGroup::SharedPtr serviceCallbackGroup() {return srv_cbg_;}

private:
  bool loadMarkerMap(const std::string & path)
  {
    YAML::Node root;
    try {
      root = YAML::LoadFile(path);
    } catch (...) {
      return false;
    }
    if (!root["markers"]) {return false;}
    for (const auto & m : root["markers"]) {
      int id = m["id"].as<int>();
      auto pos = m["position"];
      auto rpy = m["orientation_rpy_deg"];
      MarkerPose mp;
      mp.t = tf2::Vector3(pos["x"].as<double>(), pos["y"].as<double>(), pos["z"].as<double>());
      mp.R = rpyDegToMat(rpy["roll"].as<double>(), rpy["pitch"].as<double>(),
          rpy["yaw"].as<double>());
      marker_map_[id] = mp;
    }
    RCLCPP_INFO(get_logger(), "마커 map 로드: %zu개", marker_map_.size());
    return true;
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (intr_.ok) {return;}
    intr_.K = (cv::Mat_<double>(3, 3) <<
      msg->k[0], msg->k[1], msg->k[2], msg->k[3], msg->k[4], msg->k[5],
      msg->k[6], msg->k[7], msg->k[8]);
    intr_.dist = msg->d.empty() ? cv::Mat::zeros(1, 5, CV_64F) :
      cv::Mat(1, static_cast<int>(msg->d.size()), CV_64F);
    for (size_t i = 0; i < msg->d.size(); ++i) {
      intr_.dist.at<double>(0, static_cast<int>(i)) = msg->d[i];
    }
    intr_.ok = true;
    RCLCPP_INFO(get_logger(), "CameraInfo 수신 → 캘리브 활성화.");
  }

  void onCompressed(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lk(m_);
      if (!search_active_) {return;}
    }
    cv::Mat img;
    try {
      img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "imdecode 예외: %s", e.what());
      return;
    }
    if (!img.empty()) {processFrame(img);}
  }

  void onRaw(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lk(m_);
      if (!search_active_) {return;}
    }
    try {
      processFrame(cv_bridge::toCvCopy(msg, "bgr8")->image);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "이미지 변환 실패: %s", e.what());
    }
  }

  // 메인 스레드에서 실행되는 검출 + 누적 + (충분하면) 발행.
  void processFrame(cv::Mat img)
  {
    int target;
    {
      std::lock_guard<std::mutex> lk(m_);
      if (!search_active_) {return;}
      target = target_;
    }
    tf2::Vector3 pos;
    tf2::Quaternion quat;
    const char * step = "?";
    try {
      if (img.empty() || !intr_.ok || intr_.K.empty() || intr_.dist.empty()) {return;}
      step = "scaleK";
      cv::Mat K = scaleK(intr_.K, intr_.calib_w, intr_.calib_h, img.cols, img.rows);
      step = "detect";
      std::set<int> want{target};
      auto markers = detectMarkers(*detector_, img, &want);
      if (markers.empty()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "id=%d 아직 미검출...", target);
        return;
      }
      const auto & corners = markers[0].second;
      if (corners.size() != 4) {return;}
      step = "solvePnP";
      cv::Vec3d rvec, tvec;
      if (!cv::solvePnP(obj_pts_, corners, K, intr_.dist, rvec, tvec, false,
        cv::SOLVEPNP_IPPE_SQUARE))
      {
        return;
      }
      step = "reproj";
      double reproj = reprojectionError(obj_pts_, corners, rvec, tvec, K, intr_.dist);
      if (max_reproj_ > 0 && reproj > max_reproj_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "재투영오차 %.2fpx 초과 → 샘플 제외", reproj);
        return;
      }
      step = "robotPoseInMap";
      robotPoseInMap(target, rvec, tvec, pos, quat);
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "processFrame 예외 @%s: %s", step, e.what());
      return;
    }

    tf2::Vector3 avg_pos;
    tf2::Quaternion avg_q;
    {
      std::lock_guard<std::mutex> lk(m_);
      if (!search_active_) {return;}
      samples_pos_.push_back(pos);
      samples_q_.push_back(quat);
      if (static_cast<int64_t>(samples_pos_.size()) < num_samples_) {return;}
      avg_pos = meanPos(samples_pos_);
      avg_q = avgQuat(samples_q_);
      search_active_ = false;
    }
    publishInitialPose(avg_pos, avg_q, now());
    auto rpy = matToRpyDeg(tf2::Matrix3x3(avg_q));
    char buf[160];
    std::snprintf(buf, sizeof(buf),
      "id=%d 인식 -> init_pose 발행 pos=(%.3f, %.3f, %.3f) yaw=%.1fdeg (n=%ld)",
      target, avg_pos.x(), avg_pos.y(), avg_pos.z(), rpy[2], (long)num_samples_);
    {
      std::lock_guard<std::mutex> lk(m_);
      result_success_ = true;
      result_msg_ = buf;
      done_ = true;
    }
    cv_.notify_all();
    RCLCPP_INFO(get_logger(), "[service] %s", buf);
  }

  // 서비스 콜백(별도 스레드): 탐색 시작 → 조건변수로 대기 → 결과 반환.
  void onService(
    const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> resp)
  {
    int target = static_cast<int>(get_parameter("target_marker_id").as_int());
    if (!intr_.ok) {
      resp->success = false;
      resp->message = "카메라 캘리브레이션이 없어 pose 계산 불가.";
      return;
    }
    if (marker_map_.find(target) == marker_map_.end()) {
      resp->success = false;
      resp->message = "marker_map 에 id=" + std::to_string(target) + " 없음.";
      return;
    }
    {
      std::lock_guard<std::mutex> lk(m_);
      if (search_active_) {
        resp->success = false;
        resp->message = "이미 다른 탐색이 진행 중입니다.";
        return;
      }
      target_ = target;
      samples_pos_.clear();
      samples_q_.clear();
      result_success_ = false;
      result_msg_.clear();
      done_ = false;
      search_active_ = true;
    }
    RCLCPP_INFO(get_logger(), "[service] id=%d 마커 탐색 시작(인식 시까지 대기)", target);

    std::unique_lock<std::mutex> lk(m_);
    bool got;
    if (search_timeout_ > 0) {
      got = cv_.wait_for(lk, std::chrono::duration<double>(search_timeout_),
          [this] {return done_;});
    } else {
      cv_.wait(lk, [this] {return done_;});
      got = true;
    }
    if (!got) {
      search_active_ = false;
      resp->success = false;
      resp->message = "타임아웃: id=" + std::to_string(target) + " 마커 미인식.";
      return;
    }
    resp->success = result_success_;
    resp->message = result_msg_;
  }

  void robotPoseInMap(
    int marker_id, const cv::Vec3d & rvec, const cv::Vec3d & tvec,
    tf2::Vector3 & pos, tf2::Quaternion & quat)
  {
    tf2::Matrix3x3 R_cam_marker = cvRodriguesToMat(rvec);
    tf2::Vector3 t_cam_marker(tvec[0], tvec[1], tvec[2]);

    tf2::Matrix3x3 R_body_marker = ext_.R_body_cam * R_cam_marker;
    tf2::Vector3 t_body_marker = ext_.R_body_cam * t_cam_marker + ext_.t_body_cam;
    tf2::Matrix3x3 R_marker_body = R_body_marker.transpose();
    tf2::Vector3 t_marker_body = -(R_marker_body * t_body_marker);

    const MarkerPose & mp = marker_map_.at(marker_id);
    tf2::Matrix3x3 R_map_body = mp.R * R_marker_body;
    pos = mp.R * t_marker_body + mp.t;
    quat = matToQuat(R_map_body);
  }

  static tf2::Vector3 meanPos(const std::vector<tf2::Vector3> & v)
  {
    tf2::Vector3 s(0, 0, 0);
    for (const auto & p : v) {s += p;}
    return s / static_cast<double>(v.size());
  }

  static tf2::Quaternion avgQuat(const std::vector<tf2::Quaternion> & v)
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

  void publishInitialPose(
    const tf2::Vector3 & pos, const tf2::Quaternion & q, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.pose.pose.position.x = pos.x();
    msg.pose.pose.position.y = pos.y();
    msg.pose.pose.position.z = pos.z();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    for (int i = 0; i < 6; ++i) {
      msg.pose.covariance[i * 6 + i] =
        (i < static_cast<int>(cov_diag_.size())) ? cov_diag_[i] : 0.0;
    }
    pub_init_->publish(msg);
  }

  std::string image_topic_, map_frame_;
  bool use_compressed_{true};
  double marker_size_{0.185}, max_reproj_{4.0}, search_timeout_{0.0};
  int64_t num_samples_{5};
  std::vector<double> cov_diag_;

  std::unique_ptr<MarkerDetector> detector_;
  std::vector<cv::Point3f> obj_pts_;
  Intrinsics intr_;
  Extrinsic ext_;
  std::map<int, MarkerPose> marker_map_;

  std::mutex m_;
  std::condition_variable cv_;
  bool search_active_{false}, done_{false}, result_success_{false};
  int target_{1};
  std::string result_msg_;
  std::vector<tf2::Vector3> samples_pos_;
  std::vector<tf2::Quaternion> samples_q_;

  rclcpp::CallbackGroup::SharedPtr srv_cbg_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_init_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_cimg_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cinfo_;
  rclcpp::Service<Trigger>::SharedPtr srv_;
};

int main(int argc, char ** argv)
{
  cv::setNumThreads(1);  // marker_viz 와 동일: OpenCV 내부 병렬처리 off.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InitPoseNode>();
  auto base = node->get_node_base_interface();

  // 서비스는 별도 스레드 실행기에서 (블로킹 대기).
  rclcpp::executors::SingleThreadedExecutor srv_exec;
  srv_exec.add_callback_group(node->serviceCallbackGroup(), base);
  std::thread srv_thread([&srv_exec]() {srv_exec.spin();});

  // 이미지/카메라인포(default group)는 메인 스레드 실행기에서 → 검출이 메인 스레드.
  rclcpp::executors::SingleThreadedExecutor main_exec;
  main_exec.add_callback_group(base->get_default_callback_group(), base);
  main_exec.spin();

  srv_exec.cancel();
  if (srv_thread.joinable()) {srv_thread.join();}
  rclcpp::shutdown();
  return 0;
}
