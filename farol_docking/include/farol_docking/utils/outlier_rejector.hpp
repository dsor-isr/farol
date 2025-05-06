#pragma once

#include <deque>
#include <Eigen/Dense>
#include <farol_docking/utils/median_utils.hpp>  

template<typename T>
class OutlierRejector {
public:
  OutlierRejector(size_t window_size, double threshold_sigma = 3.0)
    : window_size_(window_size), threshold_sigma_(threshold_sigma) {}

  void add(const T& measurement) {
    if (buffer_.size() >= window_size_) {
      buffer_.pop_front();
    }
    buffer_.push_back(measurement);
  }

  bool isOutlier(const T& measurement) const {
    if (buffer_.size() < window_size_) {
      return false;  // Not enough data to reliably decide
    }

    // Compute the median
    auto median_value = median(buffer_);

    // Compute deviation
    std::vector<typename T::Scalar> deviations;
    for (const auto& m : buffer_) {
      deviations.push_back((m - median_value).cwiseAbs().maxCoeff());
    }

    // Median Absolute Deviation (MAD)
    double mad_value = median(deviations);

    // Scale MAD for Gaussian assumptions
    double scaled_mad = 1.4826 * mad_value;

    // Compute deviation of the new measurement
    double measurement_deviation = (measurement - median_value).cwiseAbs().maxCoeff();

    return measurement_deviation > threshold_sigma_ * scaled_mad;
  }

private:
  size_t window_size_;
  double threshold_sigma_;
  std::deque<T> buffer_;
};
