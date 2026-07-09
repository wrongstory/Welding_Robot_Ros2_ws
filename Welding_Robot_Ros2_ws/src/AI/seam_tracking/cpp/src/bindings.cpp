// pybind11 바인딩 — C++ HailoSegEngine 을 파이썬 모듈 seam_tracking_cpp 로 노출.
// UI(PyQt5)가 numpy uint8[640,640,3] 를 넘기면 검출 리스트(dict)를 돌려받는다.
#include <cstring>
#include <vector>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "seam_tracking/hailo_seg_engine.hpp"
#include "seam_tracking/seam_centerline.hpp"

namespace py = pybind11;
using seam_tracking::Detection;
using seam_tracking::HailoSegEngine;

namespace
{
// 디바이스 없이 중심선 알고리즘을 단위 검증하기 위한 노출 헬퍼.
// mask: (H,W) uint8 → line (x1,y1,x2,y2) 또는 None.
py::object fit_centerline_np(
  py::array_t<uint8_t, py::array::c_style | py::array::forcecast> arr)
{
  if (arr.ndim() != 2) {
    throw std::runtime_error("mask 는 (H,W) uint8 여야 함");
  }
  cv::Mat mask(static_cast<int>(arr.shape(0)), static_cast<int>(arr.shape(1)),
    CV_8U, const_cast<uint8_t *>(arr.data()));
  const seam_tracking::SeamLine sl = seam_tracking::fit_seam_centerline(mask);
  if (!sl.valid) {
    return py::none();
  }
  return py::make_tuple(sl.p1.x, sl.p1.y, sl.p2.x, sl.p2.y);
}
}  // namespace

namespace
{

// rgb640: (640,640,3) uint8 C-contiguous. 반환: list[dict{box,score,cls,mask}]
py::list infer_np(
  HailoSegEngine & engine,
  py::array_t<uint8_t, py::array::c_style | py::array::forcecast> arr)
{
  if (arr.ndim() != 3 || arr.shape(0) != seam_tracking::kImgSize ||
    arr.shape(1) != seam_tracking::kImgSize || arr.shape(2) != 3)
  {
    throw std::runtime_error("입력은 (640,640,3) uint8 RGB 여야 함");
  }

  std::vector<Detection> dets;
  {
    py::gil_scoped_release release;  // 추론 동안 GIL 해제 (stack.md §4)
    dets = engine.infer(arr.data(), static_cast<size_t>(arr.size()));
  }

  py::list out;
  for (const auto & d : dets) {
    py::array_t<uint8_t> mask({d.mask.rows, d.mask.cols});
    std::memcpy(mask.mutable_data(), d.mask.data, d.mask.total());

    py::dict item;
    item["box"] = py::make_tuple(d.x1, d.y1, d.x2, d.y2);
    item["score"] = d.score;
    item["cls"] = d.cls;
    item["mask"] = std::move(mask);
    // 중심선(ADR 0010): 640 좌표계 (x1,y1,x2,y2) 또는 None
    if (d.has_line) {
      item["line"] = py::make_tuple(d.line_p1.x, d.line_p1.y, d.line_p2.x, d.line_p2.y);
    } else {
      item["line"] = py::none();
    }
    out.append(std::move(item));
  }
  return out;
}

}  // namespace

PYBIND11_MODULE(seam_tracking_cpp, m)
{
  m.doc() = "Hailo-8 YOLOv8-seg 용접선 추적 추론 엔진 (C++ 코어 바인딩)";

  py::class_<HailoSegEngine>(m, "Engine")
    .def(
      py::init<const std::string &, float, float, float>(),
      py::arg("hef_path"), py::arg("conf") = 0.25f,
      py::arg("iou") = 0.45f, py::arg("mask_thr") = 0.5f,
      "HEF 로 엔진 초기화 (Hailo-8 디바이스 필요)")
    .def(
      "infer", &infer_np, py::arg("rgb640"),
      "중앙 640x640 crop RGB uint8 → [{box,score,cls,mask}]")
    .def_property_readonly(
      "input_size", &HailoSegEngine::input_size, "입력 한 변 크기(640)");

  m.def(
    "fit_centerline", &fit_centerline_np, py::arg("mask"),
    "마스크(HxW uint8) → 중심선 직선 끝점 (x1,y1,x2,y2) 또는 None (ADR 0010)");
}
