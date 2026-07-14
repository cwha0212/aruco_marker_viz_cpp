// 이미지 토픽을 받아 ArUco 마커를 검출·표시한 이미지를 발행하는 ROS2 노드 (C++).
// map 좌표(위치 + rpy)는 /aft_mapped_to_init + 외부파라미터가 있을 때 계산·로그·발행한다.
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

#include "aruco_marker_viz_cpp/aruco_common.hpp"

using namespace aruco_common;
using std::placeholders::_1;

class MarkerVizNode : public rclcpp::Node
{
public:
  MarkerVizNode()
  : Node("aruco_marker_viz")
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
    pub_ = create_publisher<sensor_msgs::msg::Image>(out_topic, 5);
    pub_map_ = create_publisher<geometry_msgs::msg::PoseArray>("~/marker_map_poses", 10);
    pub_pt_ = create_publisher<geometry_msgs::msg::PointStamped>("~/marker_map_point", 10);

    if (!intr_.ok) {
      sub_cinfo_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        cam_info_topic, qos, std::bind(&MarkerVizNode::onCameraInfo, this, _1));
    }
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, qos, std::bind(&MarkerVizNode::onOdom, this, _1));
    if (use_compressed_) {
      sub_cimg_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        image_topic_, qos, std::bind(&MarkerVizNode::onCompressed, this, _1));
    } else {
      sub_img_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, qos, std::bind(&MarkerVizNode::onRaw, this, _1));
    }

    RCLCPP_INFO(get_logger(), "marker_viz(C++) 시작 | in=%s 거리/축=%s",
      image_topic_.c_str(), intr_.ok ? "ON" : "OFF(캘리브 없음)");
  }

private:
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
    cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
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
    cv::Mat K = intr_.ok ? scaleK(intr_.K, intr_.calib_w, intr_.calib_h, img.cols, img.rows) :
      cv::Mat();
    auto markers = detectMarkers(*detector_, img, has_id_filter_ ? &allowed_ids_ : nullptr);

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
      std::vector<cv::Point> poly(corners.begin(), corners.end());
      std::vector<std::vector<cv::Point>> polys{poly};
      cv::polylines(img, polys, true, color, 2);

      cv::Point p0(cvRound(corners[0].x), cvRound(corners[0].y));
      std::string label = "id" + std::to_string(mid);
      if (have_pose && !bad) {
        if (draw_axes_) {drawAxes(img, K, intr_.dist, rvec, tvec, marker_size_ * 0.5);}
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
          auto rpy = matToRpyDeg(R_map_marker);
          char mb[48];
          std::snprintf(mb, sizeof(mb), " map(%.2f,%.2f,%.2f)",
            pose.position.x, pose.position.y, pose.position.z);
          label += mb;
          RCLCPP_INFO(get_logger(),
            "[id=%d] cam dist=%.3fm -> map position=(%.3f, %.3f, %.3f) "
            "orientation_rpy_deg=(%.1f, %.1f, %.1f)", mid, d_m,
            pose.position.x, pose.position.y, pose.position.z, rpy[0], rpy[1], rpy[2]);
        }
      } else if (bad) {
        label += " (bad)";
      }
      cv::putText(img, label, cv::Point(p0.x, p0.y - 10), cv::FONT_HERSHEY_SIMPLEX,
        0.7, color, 2, cv::LINE_AA);
    }

    auto out = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
    pub_->publish(*out);

    if (!map_arr.poses.empty()) {
      pub_map_->publish(map_arr);
      geometry_msgs::msg::PointStamped pt;
      pt.header = map_arr.header;
      pt.point = map_arr.poses[0].position;
      pub_pt_->publish(pt);
    }
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

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_map_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_pt_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_cimg_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cinfo_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerVizNode>());
  rclcpp::shutdown();
  return 0;
}
