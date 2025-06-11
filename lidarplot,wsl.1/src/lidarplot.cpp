#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

// 라디안을 도(degree)로 변환하는 매크로
#define RAD2DEG(x) ((x) * 180. / M_PI)

using std::placeholders::_1;

class LidarVisualizer : public rclcpp::Node {
public:
  LidarVisualizer() : Node("sllidar_client") {
    // "/scan" 토픽으로부터 LaserScan 메시지 구독, 콜백 함수 등록
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), std::bind(&LidarVisualizer::scanCallback, this, _1));

    // 영상 크기 설정 (500x500 픽셀)
    img_size_ = 500;
    // 1픽셀 당 실제 거리 (2cm) 설정, 즉 500픽셀 = 10미터 영역 표현
    meter_per_pixel_ = 0.02;
    // 영상 중심 좌표 계산 (중심점 기준 좌표)
    center_ = cv::Point(img_size_ / 2, img_size_ / 2);

    // OpenCV 윈도우 생성 (자동 크기 조절)
    cv::namedWindow("Lidar Scan", cv::WINDOW_AUTOSIZE);

    // 동영상 저장용 VideoWriter 초기화 (MJPG 코덱, 10fps)
    video_writer_.open("lidar_scan.avi",
                       cv::VideoWriter::fourcc('M','J','P','G'),
                       10, // 초당 프레임 수
                       cv::Size(img_size_, img_size_)); // 영상 크기
    if (!video_writer_.isOpened()) {
      RCLCPP_WARN(this->get_logger(), "Failed to open video writer, video will not be saved.");
    }
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // 수신 데이터 포인트 개수 계산 (scan_time / time_increment)
    int count = static_cast<int>(scan->scan_time / scan->time_increment);
    if (count <= 0) 
      count = scan->ranges.size();  // 만약 계산 실패하면 범위 벡터 크기로 대체

    // 수신 정보 출력 (프레임 아이디, 포인트 개수, 각도 범위)
    RCLCPP_INFO(this->get_logger(), "I heard a laser scan %s [%d]:",
                scan->header.frame_id.c_str(), count);
    RCLCPP_INFO(this->get_logger(), "angle_range : [%f, %f]",
                RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    // 영상 초기화: 흰 배경 (500x500, 3채널 컬러)
    cv::Mat img(img_size_, img_size_, CV_8UC3, cv::Scalar(255, 255, 255));

    // 중심 십자선 그리기 (검정색, 두께 2)
    cv::line(img, cv::Point(center_.x - 5, center_.y), 
                  cv::Point(center_.x + 5, center_.y), 
                  cv::Scalar(0, 0, 0), 2);
    cv::line(img, cv::Point(center_.x, center_.y - 5), 
                  cv::Point(center_.x, center_.y + 5), 
                  cv::Scalar(0, 0, 0), 2);

    // 각 라이다 데이터 점에 대해 처리
    for (int i = 0; i < count; ++i) {
     // 현재 점의 각도 계산 (rad)
      float angle = scan->angle_min + i * scan->angle_increment;
      // 현재 점의 거리
      float range = scan->ranges[i]; 

      RCLCPP_DEBUG(this->get_logger(), "angle-distance : [%f, %f]", RAD2DEG(angle), range);

      // 거리 유효성 검사: 최소/최대 범위 내, 그리고 NaN 아니어야 함
      if (range < scan->range_min || range > scan->range_max || std::isnan(range)) 
        continue;

      // 극좌표 -> 직교좌표 변환 (x: 앞쪽, y: 왼쪽)
      float x = range * cos(angle);
      float y = range * sin(angle);

      // 90도 오른쪽(시계 방향) 회전 변환 적용
      // (x,y) → (y, -x)
      float x_rot = y;
      float y_rot = -x;

      // 좌표 → 픽셀 좌표 변환 (y축 반전 처리)
      int px = static_cast<int>(center_.x + x_rot / meter_per_pixel_);
      int py = static_cast<int>(center_.y - y_rot / meter_per_pixel_);

      // 영상 영역 내에 있는 점만 그리기 (빨간색 원, 반지름 2, 채움)
      if (px >= 0 && px < img_size_ && py >= 0 && py < img_size_) {
        cv::circle(img, cv::Point(px, py), 2, cv::Scalar(0, 0, 255), -1);
      }
    }

    // 영상 화면에 표시
    cv::imshow("Lidar Scan", img);
    cv::waitKey(1);

    // 동영상 저장 (옵션)
    if (video_writer_.isOpened()) {
      video_writer_.write(img);
    }
  }

  // 멤버 변수 선언
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;  // LaserScan 구독자
  int img_size_;                  // 영상 크기 (픽셀 단위)
  float meter_per_pixel_;         // 픽셀 당 실제 거리 (m)
  cv::Point center_;              // 영상 중심 좌표
  cv::VideoWriter video_writer_;  // 동영상 저장용 객체
};

int main(int argc, char **argv) {
  // ROS2 초기화
  rclcpp::init(argc, argv);
  // LidarVisualizer 노드 생성 및 실행
  rclcpp::spin(std::make_shared<LidarVisualizer>());
  // 종료 전 ROS2 종료 처리 및 OpenCV 윈도우 닫기
  rclcpp::shutdown();
  cv::destroyAllWindows();
  return 0;
}
