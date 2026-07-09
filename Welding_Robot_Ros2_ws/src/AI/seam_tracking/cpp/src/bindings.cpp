// pybind11 바인딩 — C++ HailoSegEngine 을 파이썬 모듈 seam_tracking_cpp 로 노출.
// UI(PyQt5)가 numpy uint8[640,640,3] 를 넘기면 검출 리스트(dict)를 돌려받는다.
#include <cstring>
#include <vector>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "seam_tracking/hailo_seg_engine.hpp"

namespace py = pybind11;
using seam_tracking::Detection;
using seam_tracking::HailoSegEngine;

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
}
