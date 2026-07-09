// 용접선 중심선 추출 — segment 마스크(640 좌표계)에서 per-scanline 중심점을
// 모아 최소자승(cv::fitLine, L2) 직선으로 피팅한다. (ADR 0010)
//
// 좌표계: 입력 마스크와 동일한 640×640 crop 픽셀 좌표계. 프레임 변환 없음.
#ifndef SEAM_TRACKING_SEAM_CENTERLINE_HPP
#define SEAM_TRACKING_SEAM_CENTERLINE_HPP

#include <opencv2/core.hpp>

namespace seam_tracking
{

// 중심선 직선 방정식: 통과점 (x0,y0) + 방향 (vx,vy) [정규화], 표시용 끝점 p1,p2.
struct SeamLine
{
  bool valid = false;
  cv::Point2f p1{0.f, 0.f};  // 끝점 1 (640 좌표)
  cv::Point2f p2{0.f, 0.f};  // 끝점 2
  float vx = 0.f, vy = 0.f;  // 방향(단위벡터)
  float x0 = 0.f, y0 = 0.f;  // 통과점
};

// mask: CV_8U (0/1 또는 0/255). 전경 픽셀 중심선을 직선으로 피팅.
// 전경 없음/중심점 2개 미만이면 valid=false.
SeamLine fit_seam_centerline(const cv::Mat & mask);

}  // namespace seam_tracking

#endif  // SEAM_TRACKING_SEAM_CENTERLINE_HPP
