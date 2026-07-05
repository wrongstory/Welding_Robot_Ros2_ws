#ifndef AMR_MOTION_CONTROL__RECURSIVE_MOVING_AVERAGE_HPP_
#define AMR_MOTION_CONTROL__RECURSIVE_MOVING_AVERAGE_HPP_

#include <vector>

namespace amr_motion_control
{

/// Recursive Moving Average filter — O(1) per update
/// Reference: kuks2309/aMAP-Henes-Powerpack RecursiveMovingAverage
class RecursiveMovingAverage
{
public:
  explicit RecursiveMovingAverage(int window_size = 5)
  : window_size_(window_size), data_(window_size, 0.0),
    avg_(0.0), count_(0), index_(0)
  {
  }

  double update(double value)
  {
    if (count_ < window_size_) {
      // Fill phase: accumulate then compute full average
      data_[index_] = value;
      index_ = (index_ + 1) % window_size_;
      count_++;
      double sum = 0.0;
      for (int i = 0; i < count_; i++) {
        sum += data_[i];
      }
      avg_ = sum / count_;
    } else {
      // Recursive phase: avg_new = avg_old + (newest - oldest) / N
      double old_value = data_[index_];
      data_[index_] = value;
      index_ = (index_ + 1) % window_size_;
      avg_ = avg_ + (value - old_value) / window_size_;
    }
    return avg_;
  }

  void reset()
  {
    for (int i = 0; i < window_size_; i++) {
      data_[i] = 0.0;
    }
    avg_ = 0.0;
    count_ = 0;
    index_ = 0;
  }

  int windowSize() const { return window_size_; }

private:
  int window_size_;
  std::vector<double> data_;
  double avg_;
  int count_;
  int index_;
};

}  // namespace amr_motion_control

#endif  // AMR_MOTION_CONTROL__RECURSIVE_MOVING_AVERAGE_HPP_
