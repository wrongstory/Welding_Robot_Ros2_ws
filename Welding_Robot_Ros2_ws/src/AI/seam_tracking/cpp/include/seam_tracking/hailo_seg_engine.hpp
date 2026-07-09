// HailoSegEngine — Hailo-8 에서 welding YOLOv8s-seg HEF 를 구동하는 C++ 코어.
// 입력: 중앙 640×640 crop RGB uint8. 출력: 640 좌표계 Detection(박스+마스크).
// 후처리(DFL/NMS/mask)는 yolov8_seg_postprocess 로 위임. HailoRT InferVStreams(동기) 사용.
//
// 향후 ROS2 노드는 이 클래스를 그대로 소유·호출한다(ADR 0009 §7).
#ifndef SEAM_TRACKING_HAILO_SEG_ENGINE_HPP
#define SEAM_TRACKING_HAILO_SEG_ENGINE_HPP

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "seam_tracking/yolov8_seg_postprocess.hpp"

// 전방 선언으로 HailoRT 헤더를 공개 인터페이스에서 숨김(빌드 경계 최소화).
namespace hailort
{
class VDevice;
class ConfiguredNetworkGroup;
class InferVStreams;
}  // namespace hailort

namespace seam_tracking
{

// 초기화/추론 실패 시 std::runtime_error 를 던진다(C 반환코드 → 예외 변환, stack.md §4).
class HailoSegEngine
{
public:
  explicit HailoSegEngine(
    const std::string & hef_path,
    float conf_thr = 0.25f, float iou_thr = 0.45f, float mask_thr = 0.5f);
  ~HailoSegEngine();

  HailoSegEngine(const HailoSegEngine &) = delete;
  HailoSegEngine & operator=(const HailoSegEngine &) = delete;

  // rgb640: 640*640*3 uint8 (RGB, row-major). 크기 불일치 시 예외.
  std::vector<Detection> infer(const uint8_t * rgb640, size_t num_bytes);

  int input_size() const { return kImgSize; }

private:
  struct OutputSpec
  {
    std::string name;
    int height, width, channels;
    size_t float_count;  // height*width*channels
  };

  std::unique_ptr<hailort::VDevice> vdevice_;
  std::shared_ptr<hailort::ConfiguredNetworkGroup> network_group_;
  std::unique_ptr<hailort::InferVStreams> pipeline_;
  std::string input_name_;
  std::vector<OutputSpec> output_specs_;
  float conf_thr_, iou_thr_, mask_thr_;
};

}  // namespace seam_tracking

#endif  // SEAM_TRACKING_HAILO_SEG_ENGINE_HPP
