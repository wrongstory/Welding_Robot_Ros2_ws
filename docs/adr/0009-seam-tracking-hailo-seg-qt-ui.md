# ADR 0009 — seam_tracking: Hailo-8 YOLOv8-seg C++ 코어 + PyQt5 UI (독립 실행형, ROS2-ready)

- **Status**: Accepted (2026-07-09)
- **작성 계기**: user_instructions 2026-07-09 "AI 폴더에 카메라 영상 받아 segment 하는 UI(Qt5/Python)+C++(Hailo) 구현 계획", "UI+C++ 구현 계획부터 나중에 ros2 연동", "seam_tracking : 용접선 추적", "카메라 이미지 중심 기준 640x640 입력"
- **관련**: code_review/welding_robot/2026-07-09.md(데이터·HEF·crop 계약), welding_robot/hailo/README.md(HEF 사양), welding_robot/hailo/scripts/infer_hailo_seg.py(후처리 레퍼런스)

## Context

용접선(seam) 추적을 위해 카메라 프레임을 실시간 인스턴스 세그멘테이션한다. 자산은 이미 존재:
- 배포 HEF: `welding_robot/hailo/deploy/welding_yolov8s_seg_int8.hef` (YOLOv8s-seg, 1 class `Welding`, INT8, 640×640 입력, 106 FPS, on-chip NMS 없음 → raw 텐서 10개 출력).
- 후처리 레퍼런스: `infer_hailo_seg.py` (DFL decode + NMS + mask 조립, numpy).
- HailoRT 4.23.0 dev (`find_package(HailoRT)`), pybind11 2.9.1, PyQt5/Qt 5.15.3, 카메라 `/dev/video0`.

요구 순서: **① 독립 실행형(카메라→C++ Hailo→Qt UI) 먼저, ② ROS2 연동 나중.** 학습은 1280×720 **중앙 640×640 crop**(`crop_center_640.py`, x 320~960 / y 40~680) 기반이므로 추론 입력도 동일 crop 계약을 지켜야 한다(letterbox 아님).

## Decision

### 1. 패키지 — `Welding_Robot_Ros2_ws/src/AI/seam_tracking/` (ament_cmake)
지금은 독립 실행형이지만 **처음부터 ament_cmake 패키지**로 두어 `colcon build` 호환. ROS2 노드는 나중에 같은 트리에 추가(재구조화 0).

### 2. 3계층 아키텍처 (stack.md 계층 분리)
- **C++ 코어** `HailoSegEngine` — HailoRT VDevice 추론 + **후처리 전부 C++**(DFL+NMS+mask). Hailo·알고리즘 = C/C++ 원칙.
- **pybind11 경계** — C++ 코어를 파이썬 확장모듈 `seam_tracking_cpp` 로 빌드, UI가 직접 import(인프로세스, zero-copy).
- **PyQt5 UI** — 카메라 캡처(QThread)+중앙 crop+오버레이 표시. 로직(`.py`)과 레이아웃(`.ui`) 분리.

### 3. 입력 계약 = 중앙 640×640 crop (letterbox 금지)
UI가 1280×720 프레임을 `x[320:960], y[40:680]` 로 crop → uint8[640,640,3] RGB 를 엔진에 전달. 엔진은 640 좌표계 순수 처리(sc=1, pad=0). 학습 crop 계약과 동일 → 정합 보장. crop 상수는 UI·(향후)ROS2 가 공유하는 단일 상수(`CROP_X0=320, CROP_Y0=40, CROP=640`).

### 4. 후처리 C++ 포팅 (bit-parity 게이트)
`infer_hailo_seg.py` 의 `dfl_decode/classify_outputs/decode/nms/build_masks` 를 C++ 로 이식. HEF 텐서 사양(고정):
```
stride 8  → reg(80,80,64) cls(80,80,1) coef(80,80,32)
stride 16 → reg(40,40,64) cls(40,40,1) coef(40,40,32)
stride 32 → reg(20,20,64) cls(20,20,1) coef(20,20,32)
proto     → (160,160,32) ;  cls 는 on-chip sigmoid 적용됨
```
**검증**: `welding_robot/hailo/ref_pytorch/*.npz`(val 43장) 또는 기존 Python HEF 경로 출력과 C++ 출력을 box/mask IoU 로 대조 → Python 경로(IoU 0.978/0.979)와 동등 통과가 병합 게이트. never-self-approve.

### 5. C++ 공개 API (경계 = 공개표면, §3 대상)
```cpp
namespace seam_tracking {
struct Detection { float x1,y1,x2,y2, score; int cls; cv::Mat mask; }; // mask 640×640 uint8(0/1)
class HailoSegEngine {
public:
  HailoSegEngine(const std::string& hef_path, float conf=0.25f, float iou=0.45f, float mask_thr=0.5f);
  std::vector<Detection> infer(const uint8_t* rgb640);   // 640×640×3 RGB uint8
};
}
```
pybind11: `infer(np.uint8[640,640,3]) -> list[dict]`. 메모리 소유권 — 입력 버퍼 Python 소유(GC 보호, 호출 동안 유지), 반환 마스크는 C++ 생성분을 numpy 로 복사 이전(단일 소유). 긴 추론 호출은 `py::gil_scoped_release` 로 GIL 해제.

### 6. 의존성 추가 — pybind11
| 항목 | 값 |
|---|---|
| License | BSD-3-Clause (허용적, 상용 무제한) |
| 취약점 | 헤더-온리 빌드시간 바인딩, 런타임 공격면 최소. 2.9.1(시스템 `pybind11-dev`) 사용 |
| 대안 | ctypes(C 한정·시그니처 런타임만 체크·segfault 위험), Cython(빌드 복잡), 별도 프로세스+IPC(지연↑) → pybind11 채택 |

### 7. ROS2 연동 (나중, 범위 밖 — 훅만)
동일 `HailoSegEngine` 를 `rclcpp` 노드가 감싸 `sensor_msgs/Image` 구독 → 마스크/추적선 발행. 코어 재작성 없음. 인터페이스(.msg) 신설 시 별도 ADR.

## Rollback Plan

되돌림 비가역 아님(신규 파일 추가만, 기존 코드 무수정). 문제 시 `seam_tracking/` 디렉토리 삭제 + ADR Status→Superseded. HEF·데이터·기존 welding_robot 자산 무영향(읽기만).

## Consequences

- (+) Hailo·후처리가 C++ 코어에 캡슐화 → ROS2 노드가 그대로 재사용(포팅 재작업 0).
- (+) 중앙 crop 계약 일치 → 학습-추론 정합, letterbox 오차 제거.
- (+) 기존 npz 기준으로 C++ 정합 회귀 검증 체계 확보.
- (−) 후처리 C++ 이식 1회 비용(numpy→Eigen/수동 루프). numeric-coding(수치 안정 softmax/sigmoid) 준수 필요.
- (−) pybind11 빌드 의존 추가(ABI·Python 버전 결합) → CMake 일원화로 완화.
- (−) 카메라 직접 캡처(OpenCV)라 지금은 ROS2 image 파이프라인과 분리 — 의도된 1단계.
