# welding_robot 코드 리뷰 타임라인

리뷰 대상: `Welding_Robot_Ros2_ws/src/AI/welding_robot/` (YOLOv8s-seg 용접선 세그멘테이션 데이터셋·학습·Hailo 배포 파이프라인)

| 날짜 | 코드 버전 | Verdict | 핵심 |
| --- | --- | --- | --- |
| 2026-07-09 | 파일해시(비-git) | COMMENT | temporal leakage 위험(High) 1 · 데이터 규모/절대경로/OMC creep/대용량 산출물(Medium) 4 |
