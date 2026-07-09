"""카메라 캡처 + 중앙 640x640 crop + Hailo 추론 워커 (QThread).

학습이 1280x720 중앙 640 crop(crop_center_640.py) 기반이므로 추론 입력도 동일하게
중앙 crop 한다(letterbox 아님). 추론은 C++ 코어(seam_tracking_cpp.Engine)가 수행하며
GIL 을 해제하므로 이 워커 스레드에서 돌려 UI 를 막지 않는다.
"""
import time

import cv2
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal

CROP = 640  # 입력 한 변 (HEF 640x640 고정)


def center_crop_640(frame_bgr):
    """프레임을 중앙 기준 640x640 으로 crop. 640 보다 작으면 먼저 확대.

    반환: (crop_bgr, x0, y0) — x0,y0 는 원본 프레임 내 crop 좌상단(오버레이 매핑용).
    """
    h, w = frame_bgr.shape[:2]
    if w < CROP or h < CROP:
        frame_bgr = cv2.resize(frame_bgr, (max(w, CROP), max(h, CROP)))
        h, w = frame_bgr.shape[:2]
    x0 = (w - CROP) // 2
    y0 = (h - CROP) // 2
    return frame_bgr[y0:y0 + CROP, x0:x0 + CROP].copy(), x0, y0


class InferenceWorker(QThread):
    """카메라 프레임마다 crop→추론→결과 방출."""

    result_ready = pyqtSignal(np.ndarray, list, float)  # crop_bgr, dets, fps
    error = pyqtSignal(str)

    def __init__(self, engine, device=0, parent=None):
        super().__init__(parent)
        self._engine = engine
        self._device = device
        self._running = False

    def run(self):
        cap = cv2.VideoCapture(self._device)
        if not cap.isOpened():
            self.error.emit(f"카메라 열기 실패: {self._device}")
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self._running = True
        try:
            while self._running:
                ok, frame = cap.read()
                if not ok:
                    self.error.emit("프레임 읽기 실패")
                    break
                crop_bgr, _, _ = center_crop_640(frame)
                rgb = np.ascontiguousarray(cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB))
                t0 = time.time()
                try:
                    dets = self._engine.infer(rgb)
                except Exception as exc:  # noqa: BLE001 - 추론 실패는 UI 로 보고
                    self.error.emit(f"추론 실패: {exc}")
                    break
                fps = 1.0 / max(time.time() - t0, 1e-6)
                self.result_ready.emit(crop_bgr, dets, fps)
        finally:
            cap.release()

    def stop(self):
        self._running = False
        self.wait(2000)
