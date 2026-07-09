#include "seam_tracking/yolov8_seg_postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <opencv2/imgproc.hpp>

namespace seam_tracking
{
namespace
{

inline float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }

// 후보 검출(NMS 이전) — 640 좌표계 박스 + 마스크 계수
struct Candidate
{
  float x1, y1, x2, y2;
  float score;
  int cls;
  float coef[kProtoChannels];
};

// DFL: reg 채널 64 = 4변(ltrb) × 16 bin. 각 변에 대해 softmax 후 기대값(=거리, grid 셀 단위).
// infer_hailo_seg.py dfl_decode 와 동일. reg_ptr 는 한 grid 셀의 64 채널 벡터.
void dfl_decode_cell(const float * reg_ptr, float ltrb[4])
{
  for (int side = 0; side < 4; ++side) {
    const float * bins = reg_ptr + side * kRegLen;
    float maxv = bins[0];
    for (int i = 1; i < kRegLen; ++i) {
      maxv = std::max(maxv, bins[i]);
    }
    float sum = 0.f;
    float exps[kRegLen];
    for (int i = 0; i < kRegLen; ++i) {
      exps[i] = std::exp(bins[i] - maxv);
      sum += exps[i];
    }
    float acc = 0.f;
    for (int i = 0; i < kRegLen; ++i) {
      acc += (exps[i] / sum) * static_cast<float>(i);
    }
    ltrb[side] = acc;
  }
}

// 출력 텐서를 stride 별 reg/cls/coef + proto 로 분류 (shape 기반). classify_outputs 와 동일.
void classify_outputs(
  const std::map<std::string, RawTensor> & outputs,
  std::map<int, const RawTensor *> & reg,
  std::map<int, const RawTensor *> & cls,
  std::map<int, const RawTensor *> & coef,
  const RawTensor *& proto)
{
  proto = nullptr;
  for (const auto & kv : outputs) {
    const RawTensor & t = kv.second;
    const int h = t.height;
    const int c = t.channels;
    if (h == kProtoSize && c == kProtoChannels) {  // 160×160×32 = proto
      proto = &t;
      continue;
    }
    if (h == 0) {
      continue;
    }
    const int st = kImgSize / h;  // 640/80=8, /40=16, /20=32
    if (st != 8 && st != 16 && st != 32) {
      continue;
    }
    if (c == 4 * kRegLen) {
      reg[st] = &t;
    } else if (c == kNumClasses) {
      cls[st] = &t;
    } else if (c == kProtoChannels) {
      coef[st] = &t;
    }
  }
}

// 클래스 무관 NMS (인덱스 반환). nms_numpy 와 동일 로직.
std::vector<int> nms(const std::vector<Candidate> & cands, float iou_thr)
{
  std::vector<int> order(cands.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
    [&](int a, int b) { return cands[a].score > cands[b].score; });

  std::vector<int> keep;
  std::vector<char> removed(cands.size(), 0);
  for (size_t oi = 0; oi < order.size(); ++oi) {
    const int i = order[oi];
    if (removed[i]) {
      continue;
    }
    keep.push_back(i);
    const float ai = std::max(0.f, cands[i].x2 - cands[i].x1) *
      std::max(0.f, cands[i].y2 - cands[i].y1);
    for (size_t oj = oi + 1; oj < order.size(); ++oj) {
      const int j = order[oj];
      if (removed[j]) {
        continue;
      }
      const float xx1 = std::max(cands[i].x1, cands[j].x1);
      const float yy1 = std::max(cands[i].y1, cands[j].y1);
      const float xx2 = std::min(cands[i].x2, cands[j].x2);
      const float yy2 = std::min(cands[i].y2, cands[j].y2);
      const float inter = std::max(0.f, xx2 - xx1) * std::max(0.f, yy2 - yy1);
      const float aj = std::max(0.f, cands[j].x2 - cands[j].x1) *
        std::max(0.f, cands[j].y2 - cands[j].y1);
      const float iou = inter / (ai + aj - inter + 1e-9f);
      if (iou >= iou_thr) {
        removed[j] = 1;
      }
    }
  }
  return keep;
}

// 인스턴스 마스크 조립: sigmoid(coef · proto) → 160×160 → 박스 crop → 640 업샘플 → 이진화.
// build_masks + postprocess 의 마스크 처리(letterbox 없음)와 동일.
cv::Mat build_mask(const Candidate & c, const RawTensor & proto, float mask_thr)
{
  const int mh = proto.height;  // 160
  const int mw = proto.width;   // 160
  cv::Mat m(mh, mw, CV_32F);
  for (int y = 0; y < mh; ++y) {
    float * row = m.ptr<float>(y);
    for (int x = 0; x < mw; ++x) {
      const float * pv = proto.at(y, x);  // 32 채널
      float acc = 0.f;
      for (int k = 0; k < kProtoChannels; ++k) {
        acc += c.coef[k] * pv[k];
      }
      row[x] = sigmoid(acc);
    }
  }

  // 박스로 crop (proto 해상도 기준, rx=ry=160/640)
  const float rx = static_cast<float>(mw) / kImgSize;
  const float ry = static_cast<float>(mh) / kImgSize;
  int cx1 = static_cast<int>(std::floor(c.x1 * rx));
  int cy1 = static_cast<int>(std::floor(c.y1 * ry));
  int cx2 = static_cast<int>(std::ceil(c.x2 * rx));
  int cy2 = static_cast<int>(std::ceil(c.y2 * ry));
  cx1 = std::max(cx1, 0);
  cy1 = std::max(cy1, 0);
  cx2 = std::min(cx2, mw);
  cy2 = std::min(cy2, mh);

  cv::Mat crop = cv::Mat::zeros(mh, mw, CV_32F);
  if (cx2 > cx1 && cy2 > cy1) {
    m(cv::Rect(cx1, cy1, cx2 - cx1, cy2 - cy1))
      .copyTo(crop(cv::Rect(cx1, cy1, cx2 - cx1, cy2 - cy1)));
  }

  cv::Mat up;
  cv::resize(crop, up, cv::Size(kImgSize, kImgSize), 0, 0, cv::INTER_LINEAR);
  cv::Mat bin;
  cv::compare(up, mask_thr, bin, cv::CMP_GT);  // CV_8U 0/255
  bin /= 255;                                  // 0/1
  return bin;
}

}  // namespace

std::vector<Detection> postprocess(
  const std::map<std::string, RawTensor> & outputs,
  float conf_thr, float iou_thr, float mask_thr)
{
  std::map<int, const RawTensor *> reg, cls, coef;
  const RawTensor * proto = nullptr;
  classify_outputs(outputs, reg, cls, coef, proto);

  std::vector<Candidate> cands;
  for (const int st : kStrides) {
    if (!reg.count(st) || !cls.count(st) || !coef.count(st)) {
      continue;
    }
    const RawTensor & rt = *reg.at(st);
    const RawTensor & ct = *cls.at(st);
    const RawTensor & ft = *coef.at(st);
    const int H = rt.height;
    const int W = rt.width;
    for (int gy = 0; gy < H; ++gy) {
      for (int gx = 0; gx < W; ++gx) {
        const float * cls_ptr = ct.at(gy, gx);  // NC(=1) 채널, sigmoid 이미 적용됨
        int best_c = 0;
        float best_s = cls_ptr[0];
        for (int k = 1; k < kNumClasses; ++k) {
          if (cls_ptr[k] > best_s) {
            best_s = cls_ptr[k];
            best_c = k;
          }
        }
        if (best_s <= conf_thr) {
          continue;
        }
        float ltrb[4];
        dfl_decode_cell(rt.at(gy, gx), ltrb);
        const float cx = (gx + 0.5f) * st;
        const float cy = (gy + 0.5f) * st;
        Candidate cand;
        cand.x1 = cx - ltrb[0] * st;
        cand.y1 = cy - ltrb[1] * st;
        cand.x2 = cx + ltrb[2] * st;
        cand.y2 = cy + ltrb[3] * st;
        cand.score = best_s;
        cand.cls = best_c;
        const float * co = ft.at(gy, gx);
        for (int k = 0; k < kProtoChannels; ++k) {
          cand.coef[k] = co[k];
        }
        cands.push_back(cand);
      }
    }
  }

  std::vector<Detection> dets;
  if (cands.empty() || proto == nullptr) {
    return dets;
  }

  const std::vector<int> keep = nms(cands, iou_thr);
  dets.reserve(keep.size());
  for (const int idx : keep) {
    const Candidate & c = cands[idx];
    const float x1 = std::clamp(c.x1, 0.f, static_cast<float>(kImgSize));
    const float y1 = std::clamp(c.y1, 0.f, static_cast<float>(kImgSize));
    const float x2 = std::clamp(c.x2, 0.f, static_cast<float>(kImgSize));
    const float y2 = std::clamp(c.y2, 0.f, static_cast<float>(kImgSize));
    if (x2 <= x1 || y2 <= y1) {
      continue;
    }
    Detection d;
    d.x1 = x1;
    d.y1 = y1;
    d.x2 = x2;
    d.y2 = y2;
    d.score = c.score;
    d.cls = c.cls;
    d.mask = build_mask(c, *proto, mask_thr);
    dets.push_back(std::move(d));
  }
  return dets;
}

}  // namespace seam_tracking
