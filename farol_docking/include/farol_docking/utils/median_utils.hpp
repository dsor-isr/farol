#pragma once

#include <vector>
#include <algorithm>
#include <type_traits>
#include <stdexcept>
#include <Eigen/Dense>

/// @brief Helper to detect if a type is Eigen-like (has .size() method)
template<typename T, typename = void>
struct is_eigen_type : std::false_type {};

template<typename T>
struct is_eigen_type<T, decltype((void) std::declval<T>().size(), void())> : std::true_type {};

/// @brief Median for Eigen vector types (element-wise)
template<typename Iterable>
typename Iterable::value_type median(const Iterable& data) {
  using VectorType = typename Iterable::value_type;
  using Scalar = typename VectorType::Scalar;

  if (data.empty()) {
    throw std::runtime_error("Cannot compute median of empty data");
  }

  size_t dim = std::begin(data)->size();
  VectorType median_result;
  median_result.resize(dim);         
  median_result.setZero();

  for (size_t d = 0; d < dim; ++d) {
    std::vector<Scalar> elements;
    for (const auto& vec : data) {
      elements.push_back(vec[d]);
    }

    size_t n = elements.size();
    auto mid = elements.begin() + n / 2;
    std::nth_element(elements.begin(), mid, elements.end());

    if (n % 2 == 0) {
      auto mid_prev = std::max(elements.begin(), mid - 1);
      median_result[d] = 0.5 * (*mid_prev + *mid);
    } else {
      median_result[d] = *mid;
    }
  }

  return median_result;
}


/// @brief Median for scalar containers
template<typename Iterable>
typename std::enable_if<std::is_arithmetic<typename Iterable::value_type>::value,
                        typename Iterable::value_type>::type
median(const Iterable& data) {
  using Scalar = typename Iterable::value_type;

  if (data.empty()) {
    throw std::runtime_error("Cannot compute median of empty data");
  }

  std::vector<Scalar> elements(data.begin(), data.end());

  size_t n = elements.size();
  auto mid = elements.begin() + n/2;
  std::nth_element(elements.begin(), mid, elements.end());

  if (n % 2 == 0) {
    auto mid_prev = std::max(elements.begin(), mid-1);
    return 0.5 * (*mid_prev + *mid);
  } else {
    return *mid;
  }
}
