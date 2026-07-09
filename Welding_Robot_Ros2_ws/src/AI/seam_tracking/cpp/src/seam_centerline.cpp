#include "seam_tracking/seam_centerline.hpp"

#include <cmath>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace seam_tracking
{
namespace
{
constexpr float kEps = 1e-6f;  // 0 나눗셈 방어용 (numeric-coding §3)
}  // namespace

SeamLine fit_seam_centerline(const cv::Mat & mask)
{
  SeamLine out;
  if (mask.empty() || mask.type() != CV_8U) {
    return out;
  }

  std::vector<cv::Point> fg;
  cv::findNonZero(mask, fg);
  if (fg.size() < 2) {
    return out;  // 전경 없음/부족
  }
  const cv::Rect bb = cv::boundingRect(fg);
  const bool vertical = bb.height >= bb.width;  // 주방향 판정(== 대신 부등호)

  // 주방향 각 스캔라인의 전경 좌표 평균(centroid)을 중심점으로 수집
  std::vector<cv::Point2f> centers;
  if (vertical) {
    centers.reserve(bb.height);
    for (int y = bb.y; y < bb.y + bb.height; ++y) {
      const uchar * row = mask.ptr<uchar>(y);
      long sum_x = 0;
      int count = 0;
      for (int x = bb.x; x < bb.x + bb.width; ++x) {
        if (row[x]) {
          sum_x += x;
          ++count;
        }
      }
      if (count > 0) {
        centers.emplace_back(static_cast<float>(sum_x) / count, static_cast<float>(y));
      }
    }
  } else {
    centers.reserve(bb.width);
    for (int x = bb.x; x < bb.x + bb.width; ++x) {
      long sum_y = 0;
      int count = 0;
      for (int y = bb.y; y < bb.y + bb.height; ++y) {
        if (mask.ptr<uchar>(y)[x]) {
          sum_y += y;
          ++count;
        }
      }
      if (count > 0) {
        centers.emplace_back(static_cast<float>(x), static_cast<float>(sum_y) / count);
      }
    }
  }
  if (centers.size() < 2) {
    return out;
  }

  cv::Vec4f line;  // (vx, vy, x0, y0)
  cv::fitLine(centers, line, cv::DIST_L2, 0, 0.01, 0.01);
  out.vx = line[0];
  out.vy = line[1];
  out.x0 = line[2];
  out.y0 = line[3];

  // 표시용 끝점: 주방향 극단 스캔라인 좌표에서 직선 교차좌표 계산
  if (vertical) {
    const float y_lo = static_cast<float>(bb.y);
    const float y_hi = static_cast<float>(bb.y + bb.height - 1);
    if (std::fabs(out.vy) < kEps) {  // 완전 수평 방향 → 수직선으로 폴백(0 나눗셈 방어)
      out.p1 = cv::Point2f(out.x0, y_lo);
      out.p2 = cv::Point2f(out.x0, y_hi);
    } else {
      const float slope = out.vx / out.vy;  // dx/dy
      out.p1 = cv::Point2f(out.x0 + slope * (y_lo - out.y0), y_lo);
      out.p2 = cv::Point2f(out.x0 + slope * (y_hi - out.y0), y_hi);
    }
  } else {
    const float x_lo = static_cast<float>(bb.x);
    const float x_hi = static_cast<float>(bb.x + bb.width - 1);
    if (std::fabs(out.vx) < kEps) {
      out.p1 = cv::Point2f(x_lo, out.y0);
      out.p2 = cv::Point2f(x_hi, out.y0);
    } else {
      const float slope = out.vy / out.vx;  // dy/dx
      out.p1 = cv::Point2f(x_lo, out.y0 + slope * (x_lo - out.x0));
      out.p2 = cv::Point2f(x_hi, out.y0 + slope * (x_hi - out.x0));
    }
  }
  out.valid = true;
  return out;
}

}  // namespace seam_tracking
