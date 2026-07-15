// ArUco 검출·투영·좌표변환·캘리브/외부파라미터 공용 헬퍼 (Python aruco_common.py 포팅).
#pragma once

#include <array>
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>
// OpenCV 4.7+ 는 aruco 가 objdetect(메인)로 이동, 그 이전은 contrib aruco.hpp.
#if (CV_VERSION_MAJOR > 4) || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#define ARUCO_NEW_API 1
#include <opencv2/objdetect/aruco_detector.hpp>
#else
#define ARUCO_NEW_API 0
#include <opencv2/aruco.hpp>
#endif

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <yaml-cpp/yaml.h>

namespace aruco_common
{

// --------------------------------------------------------------------------
// 회전 유틸 (tf2::Matrix3x3 사용; setRPY = Rz(yaw)Ry(pitch)Rx(roll) = Python 규약)
// --------------------------------------------------------------------------
inline double deg2rad(double d) { return d * M_PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / M_PI; }

inline tf2::Matrix3x3 rpyDegToMat(double roll, double pitch, double yaw)
{
  tf2::Matrix3x3 m;
  m.setRPY(deg2rad(roll), deg2rad(pitch), deg2rad(yaw));
  return m;
}

inline std::array<double, 3> matToRpyDeg(const tf2::Matrix3x3 & m)
{
  double r, p, y;
  m.getRPY(r, p, y);
  return {rad2deg(r), rad2deg(p), rad2deg(y)};
}

inline tf2::Quaternion matToQuat(const tf2::Matrix3x3 & m)
{
  tf2::Quaternion q;
  m.getRotation(q);
  return q;
}

// 카메라 광학프레임(x右 y下 z前) → ROS 바디(x前 y左 z上) 고정 변환.
inline tf2::Matrix3x3 opticalToBody()
{
  return tf2::Matrix3x3(0, 0, 1, -1, 0, 0, 0, -1, 0);
}

inline tf2::Matrix3x3 cvRodriguesToMat(const cv::Vec3d & rvec)
{
  cv::Matx33d R;
  cv::Rodrigues(rvec, R);
  return tf2::Matrix3x3(
    R(0, 0), R(0, 1), R(0, 2),
    R(1, 0), R(1, 1), R(1, 2),
    R(2, 0), R(2, 1), R(2, 2));
}

// --------------------------------------------------------------------------
// ArUco 검출
// --------------------------------------------------------------------------
inline int dictId(const std::string & name)
{
  static const std::map<std::string, int> m = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
  };
  auto it = m.find(name);
  return it != m.end() ? it->second : cv::aruco::DICT_4X4_50;
}

// ArUco 검출 래퍼: OpenCV 버전차(신 ArucoDetector / 구 함수형 API)를 흡수.
// 서브픽셀 정밀화 + 오검출 억제(흰 테두리 잡힘/배경 허위 마커 완화) 적용.
struct MarkerDetector
{
#if ARUCO_NEW_API
  cv::aruco::ArucoDetector detector;
  explicit MarkerDetector(const std::string & name)
  : detector(cv::aruco::getPredefinedDictionary(dictId(name)), makeParams()) {}
  static cv::aruco::DetectorParameters makeParams()
  {
    cv::aruco::DetectorParameters p;
    // ★ 모든 필드를 명시적으로 설정한다. 이 커스텀 OpenCV 4.8 빌드에서 기본생성자가
    //   일부 필드를 초기화하지 않으면(쓰레기값) detectMarkers 내부 크기 계산이 음수가
    //   되어 setSize<0 로 죽는다(노드별 메모리 배치에 따라 재현). 명시 설정으로 제거.
    p.adaptiveThreshWinSizeMin = 3;
    p.adaptiveThreshWinSizeMax = 23;
    p.adaptiveThreshWinSizeStep = 10;
    p.adaptiveThreshConstant = 7.0;
    p.minMarkerPerimeterRate = 0.03;
    p.maxMarkerPerimeterRate = 4.0;
    p.polygonalApproxAccuracyRate = 0.03;
    p.minCornerDistanceRate = 0.05;
    p.minDistanceToBorder = 3;
    p.minMarkerDistanceRate = 0.05;
    p.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    p.cornerRefinementWinSize = 5;
    p.cornerRefinementMaxIterations = 50;
    p.cornerRefinementMinAccuracy = 0.01;
    p.markerBorderBits = 1;
    p.perspectiveRemovePixelPerCell = 4;
    p.perspectiveRemoveIgnoredMarginPerCell = 0.13;
    p.maxErroneousBitsInBorderRate = 0.2;
    p.minOtsuStdDev = 5.0;
    p.errorCorrectionRate = 0.4;
    p.detectInvertedMarker = false;
    return p;
  }
  void detect(
    const cv::Mat & img, std::vector<int> & ids,
    std::vector<std::vector<cv::Point2f>> & corners)
  {
    std::vector<std::vector<cv::Point2f>> rej;
    detector.detectMarkers(img, corners, ids, rej);
  }
#else
  cv::Ptr<cv::aruco::Dictionary> dict;
  cv::Ptr<cv::aruco::DetectorParameters> params;
  explicit MarkerDetector(const std::string & name)
  : dict(cv::aruco::getPredefinedDictionary(dictId(name))),
    params(cv::aruco::DetectorParameters::create())
  {
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    params->cornerRefinementWinSize = 5;
    params->cornerRefinementMaxIterations = 50;
    params->cornerRefinementMinAccuracy = 0.01;
    params->errorCorrectionRate = 0.4;
    params->maxErroneousBitsInBorderRate = 0.2;
  }
  void detect(
    const cv::Mat & img, std::vector<int> & ids,
    std::vector<std::vector<cv::Point2f>> & corners)
  {
    std::vector<std::vector<cv::Point2f>> rej;
    cv::aruco::detectMarkers(img, dict, corners, ids, params, rej);
  }
#endif
};

using Detection = std::pair<int, std::vector<cv::Point2f>>;  // (id, corners(4))

inline std::vector<Detection> detectMarkers(
  MarkerDetector & det, const cv::Mat & img, const std::set<int> * allowed_ids = nullptr)
{
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  det.detect(img, ids, corners);
  std::vector<Detection> out;
  for (size_t i = 0; i < ids.size(); ++i) {
    if (allowed_ids && allowed_ids->find(ids[i]) == allowed_ids->end()) {
      continue;
    }
    out.emplace_back(ids[i], corners[i]);
  }
  return out;
}

// SOLVEPNP_IPPE_SQUARE 규약 순서(Y 위). 어기면 tiny-Z 비물리 해로 붕괴.
inline std::vector<cv::Point3f> objectPoints(double size)
{
  float h = static_cast<float>(size / 2.0);
  return {{-h, h, 0.f}, {h, h, 0.f}, {h, -h, 0.f}, {-h, -h, 0.f}};
}

inline double reprojectionError(
  const std::vector<cv::Point3f> & obj, const std::vector<cv::Point2f> & corners,
  const cv::Vec3d & rvec, const cv::Vec3d & tvec, const cv::Mat & K, const cv::Mat & dist)
{
  std::vector<cv::Point2f> proj;
  cv::projectPoints(obj, rvec, tvec, K, dist, proj);
  double s = 0.0;
  for (size_t i = 0; i < proj.size(); ++i) {
    double dx = proj[i].x - corners[i].x, dy = proj[i].y - corners[i].y;
    s += dx * dx + dy * dy;
  }
  return std::sqrt(s / proj.size());
}

inline void drawAxes(
  cv::Mat & img, const cv::Mat & K, const cv::Mat & dist,
  const cv::Vec3d & rvec, const cv::Vec3d & tvec, double length)
{
  std::vector<cv::Point3f> axis = {
    {0, 0, 0}, {(float)length, 0, 0}, {0, (float)length, 0}, {0, 0, (float)length}};
  std::vector<cv::Point2f> pts;
  cv::projectPoints(axis, rvec, tvec, K, dist, pts);
  for (const auto & p : pts) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
      std::abs(p.x) > 1e6 || std::abs(p.y) > 1e6)
    {
      return;
    }
  }
  cv::Point o(cvRound(pts[0].x), cvRound(pts[0].y));
  cv::Scalar colors[3] = {{0, 0, 255}, {0, 255, 0}, {255, 0, 0}};
  for (int i = 0; i < 3; ++i) {
    cv::Point e(cvRound(pts[i + 1].x), cvRound(pts[i + 1].y));
    cv::arrowedLine(img, o, e, colors[i], 3, cv::LINE_8, 0, 0.25);
  }
  cv::circle(img, o, 4, cv::Scalar(0, 255, 255), -1);
}

// --------------------------------------------------------------------------
// 카메라 내부/외부 파라미터
// --------------------------------------------------------------------------
struct Intrinsics
{
  bool ok = false;
  cv::Mat K;            // 3x3 double
  cv::Mat dist;         // 1x5 double
  int calib_w = 0;      // 캘리브 해상도 (0 = 미지정)
  int calib_h = 0;
};

// Kalibr camchain yaml → Intrinsics. 형식 아니면 ok=false.
inline Intrinsics loadKalibr(const std::string & path)
{
  Intrinsics r;
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (...) {
    return r;
  }
  YAML::Node cam;
  if (root["intrinsics"]) {
    cam = root;
  } else {
    for (auto it = root.begin(); it != root.end(); ++it) {
      if (it->second["intrinsics"]) {cam = it->second; break;}
    }
  }
  if (!cam || !cam["intrinsics"]) {return r;}
  auto in = cam["intrinsics"].as<std::vector<double>>();
  if (in.size() < 4) {return r;}
  r.K = (cv::Mat_<double>(3, 3) << in[0], 0, in[2], 0, in[1], in[3], 0, 0, 1);
  std::vector<double> dc;
  if (cam["distortion_coeffs"]) {dc = cam["distortion_coeffs"].as<std::vector<double>>();}
  while (dc.size() < 4) {dc.push_back(0.0);}
  r.dist = (cv::Mat_<double>(1, 5) << dc[0], dc[1], dc[2], dc[3], 0.0);
  if (cam["resolution"]) {
    auto res = cam["resolution"].as<std::vector<int>>();
    if (res.size() == 2) {r.calib_w = res[0]; r.calib_h = res[1];}
  }
  r.ok = true;
  return r;
}

// calib_file(Kalibr) > fx/fy 순으로 Intrinsics 구성. 없으면 ok=false.
inline Intrinsics buildIntrinsics(
  const std::string & calib_file, double fx, double fy, double cx, double cy,
  const std::vector<double> & dist_coeffs, const std::vector<int64_t> & calib_res)
{
  if (!calib_file.empty()) {
    Intrinsics r = loadKalibr(calib_file);
    if (r.ok) {return r;}
  }
  Intrinsics r;
  if (fx > 0 && fy > 0) {
    r.K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    bool any = false;
    for (double v : dist_coeffs) {if (v != 0.0) {any = true;}}
    if (any) {
      r.dist = cv::Mat(1, static_cast<int>(dist_coeffs.size()), CV_64F);
      for (size_t i = 0; i < dist_coeffs.size(); ++i) {
        r.dist.at<double>(0, static_cast<int>(i)) = dist_coeffs[i];
      }
    } else {
      r.dist = cv::Mat::zeros(1, 5, CV_64F);
    }
    if (calib_res.size() == 2 && calib_res[0] > 0 && calib_res[1] > 0) {
      r.calib_w = static_cast<int>(calib_res[0]);
      r.calib_h = static_cast<int>(calib_res[1]);
    }
    r.ok = true;
  }
  return r;
}

inline cv::Mat scaleK(const cv::Mat & K, int calib_w, int calib_h, int w, int h)
{
  if (calib_w <= 0 || calib_h <= 0 || (calib_w == w && calib_h == h)) {return K;}
  cv::Mat s = K.clone();
  double sx = static_cast<double>(w) / calib_w, sy = static_cast<double>(h) / calib_h;
  s.at<double>(0, 0) *= sx; s.at<double>(0, 2) *= sx;
  s.at<double>(1, 1) *= sy; s.at<double>(1, 2) *= sy;
  return s;
}

struct Extrinsic
{
  tf2::Matrix3x3 R_body_cam;  // 카메라광학 → 라이다바디
  tf2::Vector3 t_body_cam;
};

// 정면(FLU) 기준 입력 → (R_body_cam, t_body_cam).
inline Extrinsic buildBodyCamExtrinsic(
  const std::vector<double> & lidar_rpy, const std::vector<double> & cam_xyz,
  const std::vector<double> & cam_rpy)
{
  tf2::Matrix3x3 R_F_L = rpyDegToMat(lidar_rpy[0], lidar_rpy[1], lidar_rpy[2]);
  tf2::Matrix3x3 R_L_F = R_F_L.transpose();
  tf2::Matrix3x3 R_F_cam = rpyDegToMat(cam_rpy[0], cam_rpy[1], cam_rpy[2]) * opticalToBody();
  Extrinsic e;
  e.R_body_cam = R_L_F * R_F_cam;
  e.t_body_cam = R_L_F * tf2::Vector3(cam_xyz[0], cam_xyz[1], cam_xyz[2]);
  return e;
}

}  // namespace aruco_common
