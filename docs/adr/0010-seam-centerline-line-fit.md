# ADR 0010 — seam_tracking: segment 마스크 중심선 추출 (per-scanline 중심점 + 최소자승 직선 피팅)

- **Status**: Accepted (2026-07-10)
- **작성 계기**: user_instructions 2026-07-10 "segment 영역의 중심을 추출해서 직선의 방정식을 이용하여 중심선을 구할 수 있도록 (C++ OpenCV)"
- **관련**: ADR 0009(seam_tracking 코어), yolov8_seg_postprocess(마스크 산출), numeric-coding(부동소수·0 나눗셈 방어)

## Context

용접선 추적을 위해 세그멘테이션 마스크(640×640, 0/1)에서 **중심선(centerline)**을 직선 방정식으로 구해야 한다. 마스크는 대체로 한 방향으로 긴 띠(용접선). 요구: 영역 중심점들을 추출 → 직선 피팅.

## Decision

### 1. 알고리즘 — per-scanline 중심점 + `cv::fitLine` (L2 최소자승)
1. `cv::findNonZero(mask)` → 전경 픽셀. 없으면 `valid=false`(빈 마스크 방어).
2. `cv::boundingRect` 로 주방향 판정: **height ≥ width → 수직 seam**(행 스캔), 아니면 수평 seam(열 스캔). (`==` 대신 부등호, numeric §3.)
3. 주방향 각 스캔라인의 **전경 픽셀 좌표 평균**(centroid)을 중심점으로 수집:
   - 수직: 각 행 `y` 에서 전경 `x` 평균 → 점 `(x̄, y)`.
   - 수평: 각 열 `x` 에서 전경 `y` 평균 → 점 `(x, ȳ)`.
4. 중심점 < 2개면 `valid=false`.
5. `cv::fitLine(centers, line, cv::DIST_L2, 0, 0.01, 0.01)` → 방향 `(vx,vy)` + 통과점 `(x0,y0)`.
6. **직선 방정식**은 파라메트릭 `(x0,y0)+t(vx,vy)`. 표시용 두 끝점은 주방향 극단 스캔라인 좌표에서 직선으로 교차좌표 계산:
   - 수직: `y=ymin, ymax` 에서 `x = x0 + (vx/vy)(y−y0)` — `|vy|<eps` 면 수직선으로 `x=x0`(0 나눗셈 방어).
   - 수평: 대칭.

### 2. 좌표계·단위
- 모든 좌표는 **640 crop 픽셀 좌표계**(입력 마스크와 동일). 정수 픽셀 → float. 프레임 변환 없음(UI 가 표시 시 crop offset 만 적용).

### 3. 통합·노출
- `Detection` 에 `has_line`(bool) + 끝점 `l1,l2`(cv::Point2f, 640 좌표) 추가. `postprocess` 가 마스크 생성 직후 채운다.
- pybind: dict 에 `"line"` 키 = `(x1,y1,x2,y2)` 또는 `None`. UI 는 초록 선으로 오버레이.

### 4. 대안 검토
- **PCA 주축**: 블롭 주축은 굵은 띠에서 중심을 벗어날 수 있음 → per-scanline centroid 가 "중심선" 의미에 더 부합.
- **skeleton/thinning**: 곡선 seam 엔 유리하나 직선 방정식 요구 + 비용↑ → 1차는 직선 피팅. (곡선 확장은 후속 debt.)

## Rollback Plan

되돌림 비가역 아님(신규 파일 `seam_centerline.{hpp,cpp}` + Detection 필드 추가 + dict 키 추가 — 모두 additive). 문제 시 해당 파일·필드·키 제거로 원복, 기존 검출/마스크 동작 무영향.

## Consequences

- (+) 용접선 중심선을 직선 방정식으로 제공 → 추적 제어 입력(각도·오프셋) 산출 기반.
- (+) `has_line/None` 로 검출 없거나 피팅 실패 시 안전.
- (−) 직선 가정 — 굽은 seam 은 오차. 곡선 대응은 후속(debt).
- (−) per-scanline 스캔 O(H·W) — 640² 에서 무시 가능(추론 대비 미미).
