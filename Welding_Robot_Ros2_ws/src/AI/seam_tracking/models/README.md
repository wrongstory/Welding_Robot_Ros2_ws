# models

배포 HEF 를 여기에 두면 `share/seam_tracking/models/` 로 설치되어 GUI 가 자동 탐색한다.
대용량(17MB) 이므로 git 커밋 대신 **심볼릭 링크** 권장:

```bash
ln -s ../../../welding_robot/hailo/deploy/welding_yolov8s_seg_int8.hef \
      welding_yolov8s_seg_int8.hef
```

비워 두면 GUI 는 `welding_robot/hailo/deploy/welding_yolov8s_seg_int8.hef` 를 폴백 탐색한다
(ADR 0009 §3, seam_tracking_gui.py `default_hef_path`).
