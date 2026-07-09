#!/usr/bin/env python3
"""Seam Tracking GUI — 카메라 → 중앙 640 crop → Hailo-8 YOLOv8-seg → 오버레이 표시.

C++ 코어(seam_tracking_cpp.Engine)를 pybind11 로 인프로세스 호출. 레이아웃은
seam_tracking.ui(Qt Designer) 로 분리(로직/UI 분리 원칙, stack.md §3).

실행:
    ros2 run seam_tracking seam_tracking_gui.py            # 설치 후
    python3 ui/seam_tracking_gui.py --hef <HEF> --device 0  # 소스 트리
"""
import argparse
import os
import sys

import cv2
import numpy as np
from PyQt5 import uic
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)  # co-located seam_tracking_cpp(.so) + camera_worker

from camera_worker import InferenceWorker, center_crop_640  # noqa: E402

MASK_COLOR = (40, 40, 230)   # BGR (빨강) - 마스크
LINE_COLOR = (60, 220, 60)   # BGR (초록) - 중심선
MASK_ALPHA = 0.45
IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp")


def find_resource(rel_candidates):
    """설치(share/lib) 또는 소스 트리에서 리소스 경로 탐색."""
    for p in rel_candidates:
        if p and os.path.isfile(p):
            return p
    return None


def default_ui_path():
    cands = []
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(
            get_package_share_directory("seam_tracking"), "ui", "seam_tracking.ui"))
    except Exception:  # noqa: BLE001 - ament 미가용(소스 실행) 시 폴백
        pass
    cands += [
        os.path.join(HERE, "seam_tracking.ui"),                       # co-located
        os.path.join(HERE, "..", "ui", "seam_tracking.ui"),           # 소스 트리
    ]
    return find_resource(cands)


def default_image_dir():
    """이미지 열기 대화상자 기본 위치 — 원본 이미지(welding_robot 데이터셋) 폴더 우선."""
    cands = [
        os.path.expanduser(
            "~/Project/kkw/Welding_Robot_Ros2_ws/src/AI/welding_robot/dataset_original"),
        os.path.expanduser(
            "~/Project/kkw/Welding_Robot_Ros2_ws/src/AI/welding_robot"),
    ]
    for p in cands:
        if os.path.isdir(p):
            return p
    return os.path.expanduser("~")


def default_hef_path():
    cands = []
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(
            get_package_share_directory("seam_tracking"), "models",
            "welding_yolov8s_seg_int8.hef"))
    except Exception:  # noqa: BLE001
        pass
    cands += [
        os.path.join(HERE, "..", "models", "welding_yolov8s_seg_int8.hef"),
        os.path.expanduser(
            "~/Project/kkw/Welding_Robot_Ros2_ws/src/AI/welding_robot/"
            "hailo/deploy/welding_yolov8s_seg_int8.hef"),
    ]
    return find_resource(cands)


def render_overlay(crop_bgr, dets):
    """검출 마스크(반투명)+박스 오버레이."""
    out = crop_bgr.copy()
    if dets:
        overlay = crop_bgr.copy()
        for d in dets:
            overlay[d["mask"] > 0] = MASK_COLOR
        cv2.addWeighted(overlay, MASK_ALPHA, out, 1 - MASK_ALPHA, 0, out)
        for d in dets:
            x1, y1, x2, y2 = (int(v) for v in d["box"])
            cv2.rectangle(out, (x1, y1), (x2, y2), MASK_COLOR, 2)
            cv2.putText(out, f"Welding:{d['score']:.2f}", (x1, max(y1 - 5, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, MASK_COLOR, 1)
        # 중심선(ADR 0010) — 초록 직선
        for d in dets:
            line = d.get("line")
            if line is not None:
                lx1, ly1, lx2, ly2 = (int(v) for v in line)
                cv2.line(out, (lx1, ly1), (lx2, ly2), LINE_COLOR, 2)
    return out


def bgr_to_qpixmap(bgr):
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    rgb = np.ascontiguousarray(rgb)
    h, w = rgb.shape[:2]
    img = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
    return QPixmap.fromImage(img.copy())  # copy: numpy 버퍼 수명과 분리


class SeamTrackingWindow(QMainWindow):
    def __init__(self, hef_path):
        super().__init__()
        ui_path = default_ui_path()
        if ui_path is None:
            raise FileNotFoundError("seam_tracking.ui 를 찾을 수 없음")
        uic.loadUi(ui_path, self)

        self._hef_path = hef_path
        self._engine = None
        self._engine_conf = None  # 엔진 생성 시점 conf (변경 시 재생성)
        self._worker = None
        self._last_image_dir = default_image_dir()  # 파일 대화상자 위치 기억
        self._image_list = []   # 선택 폴더의 이미지 경로 목록
        self._image_idx = 0     # 현재 표시 인덱스
        self._last_full_frame = None  # 카메라 원본 프레임(저장용)
        self._save_dir = os.path.expanduser("~/seam_captures")

        self.startButton.clicked.connect(self.on_start)
        self.stopButton.clicked.connect(self.on_stop)
        self.openButton.clicked.connect(self.on_open_folder)
        self.saveButton.clicked.connect(self.on_save_frame)
        self.sourceCombo.currentIndexChanged.connect(self.on_source_changed)
        self.on_source_changed(self.sourceCombo.currentIndex())

    # ---- 엔진 ----
    def _ensure_engine(self):
        """현재 conf 로 엔진을 준비(없거나 conf 변경 시 생성). 실패 시 None."""
        import seam_tracking_cpp  # 지연 import: 초기화 실패를 UI 로 보고
        if self._hef_path is None or not os.path.isfile(str(self._hef_path)):
            QMessageBox.critical(self, "HEF 없음", f"HEF 경로를 찾을 수 없음:\n{self._hef_path}")
            return None
        conf = float(self.confSpin.value())
        if self._engine is not None and self._engine_conf == conf:
            return self._engine
        try:
            self._engine = seam_tracking_cpp.Engine(self._hef_path, conf)
            self._engine_conf = conf
        except Exception as exc:  # noqa: BLE001
            self._engine = None
            QMessageBox.critical(self, "엔진 초기화 실패", str(exc))
            return None
        return self._engine

    # ---- 소스 선택 ----
    def is_camera_source(self):
        return self.sourceCombo.currentIndex() == 0

    def on_source_changed(self, _index):
        cam = self.is_camera_source()
        if not cam:
            self.on_stop()  # 이미지 모드로 전환 시 카메라 정지
        self.startButton.setEnabled(cam)
        self.stopButton.setEnabled(False)
        self.deviceSpin.setEnabled(cam)
        self.openButton.setEnabled(not cam)
        self.statusLabel.setText("카메라 모드" if cam else "이미지 파일 모드")
        # 결과 표시 전 안내 텍스트를 모드에 맞게 갱신(오해 방지)
        if self._worker is None:
            self.videoLabel.setText(
                "카메라 대기 중…  Start 를 누르세요" if cam
                else "이미지 폴더 모드…  Open Folder… 로 폴더 선택 후 ←/→ 로 이동")

    # ---- 이미지 폴더 입력 (←/→ 순회) ----
    def on_open_folder(self):
        from PyQt5.QtWidgets import QFileDialog
        folder = QFileDialog.getExistingDirectory(
            self, "이미지 폴더 선택", self._last_image_dir)
        if not folder:
            return
        self._last_image_dir = folder
        files = sorted(
            os.path.join(folder, f) for f in os.listdir(folder)
            if f.lower().endswith(IMAGE_EXTS))
        if not files:
            QMessageBox.warning(self, "이미지 없음", f"폴더에 이미지가 없음:\n{folder}")
            return
        self._image_list = files
        self._image_idx = 0
        self.setFocus()  # ←/→ 키 입력 수신
        self._show_current_image()

    def _show_current_image(self):
        """현재 인덱스 이미지 추론·표시."""
        if not self._image_list:
            return
        engine = self._ensure_engine()
        if engine is None:
            return
        path = self._image_list[self._image_idx]
        img = cv2.imread(path)
        if img is None:
            self.statusLabel.setText(f"읽기 실패: {os.path.basename(path)}")
            return
        crop_bgr, _, _ = center_crop_640(img)
        rgb = np.ascontiguousarray(cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB))
        import time
        t0 = time.time()
        try:
            dets = engine.infer(rgb)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.warning(self, "추론 실패", str(exc))
            return
        self.on_result(img, crop_bgr, dets, 1.0 / max(time.time() - t0, 1e-6))
        n = len(self._image_list)
        self.statusLabel.setText(
            f"[{self._image_idx + 1}/{n}] {os.path.basename(path)} · 검출 {len(dets)}  (←/→ 이동)")

    def _step_image(self, delta):
        if not self._image_list:
            return
        self._image_idx = (self._image_idx + delta) % len(self._image_list)
        self._show_current_image()

    def keyPressEvent(self, event):
        key = event.key()
        # 카메라 라이브 중 S → 원본 저장
        if self.is_camera_source() and self._worker is not None and key == Qt.Key_S:
            self.on_save_frame()
            return
        # 이미지 폴더 모드에서 ←/→(및 A/D) 로 순회
        if self._image_list and not self.is_camera_source():
            if key in (Qt.Key_Right, Qt.Key_Down, Qt.Key_D, Qt.Key_Space):
                self._step_image(1)
                return
            if key in (Qt.Key_Left, Qt.Key_Up, Qt.Key_A):
                self._step_image(-1)
                return
        super().keyPressEvent(event)

    # ---- 카메라 제어 ----
    def on_start(self):
        engine = self._ensure_engine()
        if engine is None:
            return

        device = int(self.deviceSpin.value())
        self._worker = InferenceWorker(engine, device)
        self._worker.result_ready.connect(self.on_result)
        self._worker.error.connect(self.on_error)
        self._worker.start()
        self.startButton.setEnabled(False)
        self.stopButton.setEnabled(True)
        self.confSpin.setEnabled(False)
        self.openButton.setEnabled(False)
        self.saveButton.setEnabled(True)  # 카메라 라이브 중 원본 저장 가능
        self.statusLabel.setText("실행 중…  (S: 원본 저장)")

    def on_stop(self):
        """카메라 워커만 정지. 엔진은 캐시 유지(conf 변경 시 _ensure_engine 이 재생성)."""
        if self._worker is not None:
            self._worker.stop()
            self._worker = None
        self._last_full_frame = None
        cam = self.is_camera_source()
        self.startButton.setEnabled(cam)
        self.stopButton.setEnabled(False)
        self.openButton.setEnabled(not cam)
        self.saveButton.setEnabled(False)  # 라이브 정지 → 저장 불가
        self.confSpin.setEnabled(True)

    # ---- 콜백 ----
    def on_result(self, full_bgr, crop_bgr, dets, fps):
        self._last_full_frame = full_bgr  # 저장용 원본(오버레이 없음)
        vis = render_overlay(crop_bgr, dets)
        self.videoLabel.setPixmap(bgr_to_qpixmap(vis).scaled(
            self.videoLabel.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.statusLabel.setText(f"검출 {len(dets)} · {fps:.1f} FPS")

    # ---- 원본 저장 (카메라 라이브 전용) ----
    def on_save_frame(self):
        if not self.is_camera_source() or self._worker is None:
            return
        if self._last_full_frame is None:
            self.statusLabel.setText("저장할 프레임 없음")
            return
        from datetime import datetime
        os.makedirs(self._save_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = os.path.join(self._save_dir, f"capture_{ts}.jpg")
        if cv2.imwrite(path, self._last_full_frame):
            self.statusLabel.setText(f"저장: {path}")
        else:
            QMessageBox.warning(self, "저장 실패", f"저장 실패:\n{path}")

    def on_error(self, msg):
        self.on_stop()
        QMessageBox.warning(self, "오류", msg)

    def closeEvent(self, event):
        self.on_stop()
        super().closeEvent(event)


def main():
    ap = argparse.ArgumentParser(description="Seam Tracking GUI (Hailo-8 YOLOv8-seg)")
    ap.add_argument("--hef", default=None, help="HEF 경로(미지정 시 자동 탐색)")
    ap.add_argument("--device", type=int, default=0, help="카메라 인덱스 기본값")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    win = SeamTrackingWindow(args.hef or default_hef_path())
    win.deviceSpin.setValue(args.device)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
