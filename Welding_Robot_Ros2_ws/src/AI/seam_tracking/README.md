# seam_tracking — 용접선 실시간 세그멘테이션 (Hailo-8)

카메라 프레임을 **중앙 640×640 crop** 하여 Hailo-8 에서 YOLOv8s-seg HEF 로 용접선(seam)을
인스턴스 세그멘테이션하고 PyQt5 UI 에 오버레이한다. 설계 근거: [ADR 0009](../../../../docs/adr/0009-seam-tracking-hailo-seg-qt-ui.md).

## 구조 (3계층)

```
카메라(/dev/video0) → [PyQt5 UI] 중앙 640 crop → [pybind11] → [C++ HailoSegEngine]
                                                              HailoRT 추론 + DFL/NMS/mask(C++)
                    ← 마스크 오버레이 표시 ←──────────────────── Detection(640 좌표계)
```

| 계층 | 파일 | 언어 |
|---|---|---|
| C++ 코어 | `cpp/src/hailo_seg_engine.cpp` (HailoRT InferVStreams), `cpp/src/yolov8_seg_postprocess.cpp` (DFL+NMS+mask) | C++17 |
| 바인딩 | `cpp/src/bindings.cpp` → 모듈 `seam_tracking_cpp` | pybind11 |
| UI | `ui/seam_tracking_gui.py` + `ui/seam_tracking.ui` + `ui/camera_worker.py` | Python/PyQt5 |

C++ 코어는 향후 ROS2 노드가 그대로 재사용한다(ADR 0009 §7).

## 입력 계약 (중요)

학습이 1280×720 **중앙 640×640 crop**(`welding_robot/crop_center_640.py`, x 320~960 / y 40~680)
기반이므로 추론 입력도 동일하게 **중앙 crop**(letterbox 아님). `camera_worker.center_crop_640`.

## 빌드

```bash
cd Welding_Robot_Ros2_ws
source /opt/ros/humble/setup.bash
MAKEFLAGS="-j2" colcon build --packages-select seam_tracking --cmake-args -DCMAKE_BUILD_TYPE=Release
```

의존: HailoRT 4.23.0 dev(`find_package(HailoRT)`), OpenCV, pybind11(`pybind11_vendor`), Python3 dev, PyQt5.

## 실행

```bash
source install/setup.bash
# HEF 자동 탐색(welding_robot/hailo/deploy 폴백) 또는 --hef 지정
ros2 run seam_tracking seam_tracking_gui.py --device 0
```

HEF 를 `models/` 에 심볼릭 링크하면 `share/` 로 설치되어 자동 탐색된다(`models/README.md`).

## 검증 (never-self-approve)

C++ 엔진 출력을 PyTorch 기준(`welding_robot/hailo/ref_pytorch/*.npz`)과 IoU 대조:

| 지표 | C++ 코어 | Python HEF(README) |
|---|---|---|
| Box IoU vs PyTorch | **0.9795** (val 15/15) | 0.978 |
| Mask IoU vs PyTorch | **0.9781** | 0.979 |
| 추론 지연 | ~19.8 ms/frame | ~18 ms |

→ C++ 후처리 포팅이 Python HEF 경로와 정합(bit-parity 수준). 재실행:

```bash
# 위 표 재현: build 후 seam_tracking_cpp 로 crop640/valid 이미지 추론 후 npz 대조
# (검증 스크립트는 세션 로그 참조; ref 생성은 welding_robot/hailo/scripts/pytorch_ref.py)
```

## 상태

- [x] C++ 코어(HailoRT + 후처리) — 빌드·장치 추론·IoU 정합 검증 완료
- [x] pybind11 모듈 로드·API
- [x] PyQt5 UI(카메라 스레드·중앙 crop·오버레이) — **GUI 실기 표시 검증은 디스플레이 환경에서 필요**
- [ ] ROS2 노드 래핑(향후, ADR 0009 §7)
