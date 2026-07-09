#include "seam_tracking/hailo_seg_engine.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#include <hailo/hailort.hpp>

namespace seam_tracking
{
using hailort::ConfiguredNetworkGroup;
using hailort::Hef;
using hailort::InferVStreams;
using hailort::MemoryView;
using hailort::VDevice;

namespace
{
[[noreturn]] void fail(const std::string & what, hailo_status st)
{
  throw std::runtime_error("[HailoSegEngine] " + what + " (status=" + std::to_string(st) + ")");
}
}  // namespace

HailoSegEngine::HailoSegEngine(
  const std::string & hef_path, float conf_thr, float iou_thr, float mask_thr)
: conf_thr_(conf_thr), iou_thr_(iou_thr), mask_thr_(mask_thr)
{
  auto hef_exp = Hef::create(hef_path);
  if (!hef_exp) {
    fail("Hef::create('" + hef_path + "')", hef_exp.status());
  }
  Hef hef = hef_exp.release();

  // 스케줄러(ROUND_ROBIN) 활성 → 수동 activate 불필요 (Python infer 경로와 동일).
  hailo_vdevice_params_t params;
  hailo_status ps = hailo_init_vdevice_params(&params);
  if (ps != HAILO_SUCCESS) {
    fail("hailo_init_vdevice_params", ps);
  }
  params.scheduling_algorithm = HAILO_SCHEDULING_ALGORITHM_ROUND_ROBIN;
  auto vdev_exp = VDevice::create(params);
  if (!vdev_exp) {
    fail("VDevice::create", vdev_exp.status());
  }
  vdevice_ = vdev_exp.release();

  auto cfg_exp = vdevice_->configure(hef);
  if (!cfg_exp) {
    fail("VDevice::configure", cfg_exp.status());
  }
  auto groups = cfg_exp.release();
  if (groups.empty()) {
    throw std::runtime_error("[HailoSegEngine] HEF 에 network group 이 없음");
  }
  network_group_ = groups[0];

  auto in_params = network_group_->make_input_vstream_params(
    false, HAILO_FORMAT_TYPE_UINT8, HAILO_DEFAULT_VSTREAM_TIMEOUT_MS,
    HAILO_DEFAULT_VSTREAM_QUEUE_SIZE);
  if (!in_params) {
    fail("make_input_vstream_params", in_params.status());
  }
  auto out_params = network_group_->make_output_vstream_params(
    false, HAILO_FORMAT_TYPE_FLOAT32, HAILO_DEFAULT_VSTREAM_TIMEOUT_MS,
    HAILO_DEFAULT_VSTREAM_QUEUE_SIZE);
  if (!out_params) {
    fail("make_output_vstream_params", out_params.status());
  }

  auto pipe_exp = InferVStreams::create(*network_group_, in_params.value(), out_params.value());
  if (!pipe_exp) {
    fail("InferVStreams::create", pipe_exp.status());
  }
  pipeline_.reset(new InferVStreams(pipe_exp.release()));

  auto in_infos = network_group_->get_input_vstream_infos();
  if (!in_infos || in_infos->empty()) {
    fail("get_input_vstream_infos", in_infos.status());
  }
  input_name_ = in_infos->at(0).name;

  auto out_infos = network_group_->get_output_vstream_infos();
  if (!out_infos) {
    fail("get_output_vstream_infos", out_infos.status());
  }
  for (const auto & info : out_infos.value()) {
    OutputSpec spec;
    spec.name = info.name;
    spec.height = static_cast<int>(info.shape.height);
    spec.width = static_cast<int>(info.shape.width);
    spec.channels = static_cast<int>(info.shape.features);
    spec.float_count =
      static_cast<size_t>(spec.height) * spec.width * spec.channels;
    output_specs_.push_back(std::move(spec));
  }
}

HailoSegEngine::~HailoSegEngine() = default;

std::vector<Detection> HailoSegEngine::infer(const uint8_t * rgb640, size_t num_bytes)
{
  const size_t expected = static_cast<size_t>(kImgSize) * kImgSize * 3;
  if (num_bytes != expected) {
    throw std::runtime_error(
      "[HailoSegEngine] 입력 크기 불일치: " + std::to_string(num_bytes) +
      " != " + std::to_string(expected) + " (640*640*3 RGB uint8 필요)");
  }

  std::map<std::string, MemoryView> input_data;
  input_data.emplace(
    input_name_, MemoryView(const_cast<uint8_t *>(rgb640), num_bytes));

  std::vector<std::vector<float>> buffers(output_specs_.size());
  std::map<std::string, MemoryView> output_data;
  for (size_t i = 0; i < output_specs_.size(); ++i) {
    buffers[i].resize(output_specs_[i].float_count);
    output_data.emplace(
      output_specs_[i].name,
      MemoryView(buffers[i].data(), buffers[i].size() * sizeof(float)));
  }

  const hailo_status st = pipeline_->infer(input_data, output_data, 1);
  if (st != HAILO_SUCCESS) {
    fail("InferVStreams::infer", st);
  }

  std::map<std::string, RawTensor> outputs;
  for (size_t i = 0; i < output_specs_.size(); ++i) {
    RawTensor t;
    t.height = output_specs_[i].height;
    t.width = output_specs_[i].width;
    t.channels = output_specs_[i].channels;
    t.data = std::move(buffers[i]);
    outputs.emplace(output_specs_[i].name, std::move(t));
  }

  return postprocess(outputs, conf_thr_, iou_thr_, mask_thr_);
}

}  // namespace seam_tracking
