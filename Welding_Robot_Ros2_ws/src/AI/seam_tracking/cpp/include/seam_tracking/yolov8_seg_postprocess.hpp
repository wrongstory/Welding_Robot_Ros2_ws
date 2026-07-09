// YOLOv8-seg 후처리 (C++ 포팅) — welding_robot/hailo/scripts/infer_hailo_seg.py 의
// dfl_decode / classify_outputs / decode / nms / build_masks 를 bit-parity 목표로 이식.
//
// 입력 계약: 카메라 프레임을 중앙 640×640 crop 한 것을 HEF 입력으로 사용(letterbox 아님).
// 따라서 박스/마스크는 640 좌표계 그대로 반환한다(scale=1, pad=0).
#ifndef SEAM_TRACKING_YOLOV8_SEG_POSTPROCESS_HPP
#define SEAM_TRACKING_YOLOV8_SEG_POSTPROCESS_HPP

#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace seam_tracking
{

// HEF 상수 (welding yolov8s-seg, 1 class) — infer_hailo_seg.py 와 동일
constexpr int kImgSize = 640;
constexpr int kRegLen = 16;
constexpr int kNumClasses = 1;
constexpr int kProtoChannels = 32;
constexpr int kProtoSize = 160;  // kImgSize / 4
static const int kStrides[3] = {8, 16, 32};

// 하나의 raw 출력 텐서 (NHWC, dequantized float, row-major h*w*c)
struct RawTensor
{
  int height = 0;
  int width = 0;
  int channels = 0;
  std::vector<float> data;  // size = height*width*channels

  inline const float * at(int y, int x) const  // 채널 벡터 시작 포인터
  {
    return data.data() + (static_cast<size_t>(y) * width + x) * channels;
  }
};

struct Detection
{
  float x1 = 0.f, y1 = 0.f, x2 = 0.f, y2 = 0.f;  // 640 좌표계 xyxy
  float score = 0.f;
  int cls = 0;
  cv::Mat mask;  // 640×640 CV_8U (0/1)
  // 중심선(ADR 0010): 유효 시 line_p1~line_p2 가 640 좌표계 직선 끝점
  bool has_line = false;
  cv::Point2f line_p1{0.f, 0.f}, line_p2{0.f, 0.f};
};

// 출력 텐서 맵(vstream 이름 → RawTensor)을 후처리해 검출 리스트 반환.
// conf/iou/mask_thr 는 infer_hailo_seg.py 기본값과 동일 의미.
std::vector<Detection> postprocess(
  const std::map<std::string, RawTensor> & outputs,
  float conf_thr = 0.25f, float iou_thr = 0.45f, float mask_thr = 0.5f);

}  // namespace seam_tracking

#endif  // SEAM_TRACKING_YOLOV8_SEG_POSTPROCESS_HPP
