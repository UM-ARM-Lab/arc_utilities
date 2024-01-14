#include <fcntl.h>
#include <omp.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cassert>
#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <random>
#include <thread>
#include <type_traits>
#include <unordered_map>

#include <eigen3/Eigen/Core>

namespace arm_helpers {

template <typename T>
inline bool CheckAlignment(const T& item, const uint64_t desired_alignment) {
  const T* item_ptr = &item;
  const uintptr_t item_ptr_val = (uintptr_t)item_ptr;
  return (item_ptr_val % desired_alignment) == 0;
}

template <typename T>
inline void RequireAlignment(const T& item, const uint64_t desired_alignment) {
  if (CheckAlignment(item, desired_alignment) == false) {
    std::cout << "Item not aligned at desired alignment of " << desired_alignment << std::endl;
    assert(false);
  }
}

template <typename T>
inline T SetBit(const T current, const uint32_t bit_position, const bool bit_value) {
  // Safety check on the type we've been called with
  static_assert((std::is_same<T, uint8_t>::value || std::is_same<T, uint16_t>::value ||
                 std::is_same<T, uint32_t>::value || std::is_same<T, uint64_t>::value),
                "Type must be a fixed-size unsigned integral type");
  // Do it
  T update_mask = 1;
  update_mask = update_mask << bit_position;
  if (bit_value) {
    return (current | update_mask);
  } else {
    update_mask = (~update_mask);
    return (current & update_mask);
  }
}

template <typename T>
inline bool GetBit(const T current, const uint32_t bit_position) {
  // Type safety checks are performed in the SetBit() function
  const uint32_t mask = arm_helpers::SetBit((T)0, bit_position, true);
  return (mask & current) > 0;
}


template <class T>
inline T ClampValue(const T& val, const T& min, const T& max) {
  assert(min <= max);
  return std::min(max, std::max(min, val));
}

template <class T>
inline T ClampValueAndWarn(const T& val, const T& min, const T& max) {
  assert(min <= max);
  if (val < min) {
    const std::string msg = "Clamping " + std::to_string(val) + " to min " + std::to_string(min) + "\n";
    std::cerr << msg << std::flush;
    return min;
  } else if (val > max) {
    const std::string msg = "Clamping " + std::to_string(val) + " to max " + std::to_string(max) + "\n";
    std::cerr << msg << std::flush;
    return max;
  }
  return val;
}

//// Color Related Functionality ///////////////////////////////////////////

inline constexpr float ColorChannelFromHex(uint8_t hexval) { return (float)hexval / 255.0f; }

inline float TrimColorValue(const float val) { return ClampValue<float>(val, 0.0f, 1.0f); }

inline uint8_t ColorChannelToHex(float colorval) { return (uint8_t)round(TrimColorValue(colorval) * 255.0f); }

class RGBAColor {
 public:
  float r;
  float g;
  float b;
  float a;

  RGBAColor(const float in_r, const float in_g, const float in_b, const float in_a)
      : r(TrimColorValue(in_r)), g(TrimColorValue(in_g)), b(TrimColorValue(in_b)), a(TrimColorValue(in_a)) {}

  RGBAColor(const float in_r, const float in_g, const float in_b)
      : r(TrimColorValue(in_r)), g(TrimColorValue(in_g)), b(TrimColorValue(in_b)), a(1.0f) {}

  RGBAColor(const uint8_t in_r, const uint8_t in_g, const uint8_t in_b, const uint8_t in_a)
      : r(ColorChannelFromHex(in_r)),
        g(ColorChannelFromHex(in_g)),
        b(ColorChannelFromHex(in_b)),
        a(ColorChannelFromHex(in_a)) {}

  RGBAColor(const uint8_t in_r, const uint8_t in_g, const uint8_t in_b, const float in_a)
      : r(ColorChannelFromHex(in_r)),
        g(ColorChannelFromHex(in_g)),
        b(ColorChannelFromHex(in_b)),
        a(TrimColorValue(in_a)) {}

  RGBAColor(const uint8_t in_r, const uint8_t in_g, const uint8_t in_b)
      : r(ColorChannelFromHex(in_r)), g(ColorChannelFromHex(in_g)), b(ColorChannelFromHex(in_b)), a(1.0f) {}

  RGBAColor() : r(0.0f), g(0.0f), b(0.0f), a(0.0f) {}

  inline float GetR() const { return r; }

  inline float GetG() const { return g; }

  inline float GetB() const { return b; }

  inline float GetA() const { return a; }

  inline void SetR(const float new_r) { r = TrimColorValue(new_r); }

  inline void SetG(const float new_g) { g = TrimColorValue(new_g); }

  inline void SetB(const float new_b) { b = TrimColorValue(new_b); }

  inline void SetA(const float new_a) { a = TrimColorValue(new_a); }

  inline uint8_t GetRHex() const { return ColorChannelToHex(r); }

  inline uint8_t GetGHex() const { return ColorChannelToHex(g); }

  inline uint8_t GetBHex() const { return ColorChannelToHex(b); }

  inline uint8_t GetAHex() const { return ColorChannelToHex(a); }

  inline void SetRHex(const uint8_t hex_r) { r = ColorChannelFromHex(hex_r); }

  inline void SetGHex(const uint8_t hex_g) { g = ColorChannelFromHex(hex_g); }

  inline void SetBHex(const uint8_t hex_b) { b = ColorChannelFromHex(hex_b); }

  inline void SetAHex(const uint8_t hex_a) { a = ColorChannelFromHex(hex_a); }
};

template <typename ColorType>
class RGBAColorBuilder {
 private:
  RGBAColorBuilder() {}

 public:
  static inline ColorType MakeFromFloatColors(const float r, const float g, const float b, const float a = 1.0f) {
    ColorType color;
    color.r = TrimColorValue(r);
    color.g = TrimColorValue(g);
    color.b = TrimColorValue(b);
    color.a = TrimColorValue(a);
    return color;
  }

  static inline ColorType MakeFromHexColors(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t a = 0xff) {
    return MakeFromFloatColors(ColorChannelFromHex(r), ColorChannelFromHex(g), ColorChannelFromHex(b),
                               ColorChannelFromHex(a));
  }

  static inline ColorType MakeFromMixedColors(const uint8_t r, const uint8_t g, const uint8_t b, const float a = 1.0f) {
    return MakeFromFloatColors(ColorChannelFromHex(r), ColorChannelFromHex(g), ColorChannelFromHex(b),
                               TrimColorValue(a));
  }

  static inline ColorType InterpolateHotToCold(const double value, const double min_value = 0.0,
                                               const double max_value = 1.0) {
    assert(min_value < max_value);
    const double real_value = ClampValue(value, min_value, max_value);
    const double range = max_value - min_value;
    // Start with white
    double r = 1.0;
    double g = 1.0;
    double b = 1.0;
    // Interpolate
    if (real_value < (min_value + (0.25 * range))) {
      r = 0.0;
      g = 4.0 * (real_value - min_value) / range;
    } else if (real_value < (min_value + (0.5 * range))) {
      r = 0.0;
      b = 1.0 + 4.0 * (min_value + 0.25 * range - real_value) / range;
    } else if (real_value < (min_value + (0.75 * range))) {
      r = 4.0 * (real_value - min_value - 0.5 * range) / range;
      b = 0.0;
    } else {
      g = 1.0 + 4.0 * (min_value + 0.75 * range - real_value) / range;
      b = 0.0;
    }
    return MakeFromFloatColors((float)r, (float)g, (float)b, 1.0f);
  }
};

template <typename ColorTypeA, typename ColorTypeB>
inline ColorTypeB ConvertColor(const ColorTypeA& color) {
  ColorTypeB cvt_color;
  cvt_color.r = TrimColorValue(color.r);
  cvt_color.g = TrimColorValue(color.g);
  cvt_color.b = TrimColorValue(color.b);
  cvt_color.a = TrimColorValue(color.a);
  return cvt_color;
}

template <typename ColorType>
inline ColorType GenerateUniqueColor(const uint32_t color_code, const float alpha = 1.0f) {
  // Note: sdf_tools relies on this particular color (with alpha = 0) for color_code 0
  if (color_code == 0) {
    return RGBAColorBuilder<ColorType>::MakeFromFloatColors(1.0f, 1.0f, 1.0f, 0.0f);
  }

  static const std::map<uint32_t, std::vector<uint8_t>> color_map{
      {1, {0xff, 0x00, 0xb3}},  {2, {0x80, 0x75, 0x3e}},  {3, {0xff, 0x00, 0x68}},  {4, {0xa6, 0xd7, 0xbd}},
      {5, {0xc1, 0x20, 0x00}},  {6, {0xce, 0x62, 0xa2}},  {7, {0x81, 0x66, 0x70}},  {8, {0x00, 0x34, 0x7d}},
      {9, {0xf6, 0x8e, 0x76}},  {10, {0x00, 0x8a, 0x53}}, {11, {0xff, 0x5c, 0x7a}}, {12, {0x53, 0x7a, 0x37}},
      {13, {0xff, 0x00, 0x8e}}, {14, {0xb3, 0x51, 0x28}}, {15, {0xf4, 0x00, 0xc8}}, {16, {0x7f, 0x0d, 0x18}},
      {17, {0x93, 0x00, 0xaa}}, {18, {0x59, 0x15, 0x33}}, {19, {0xf1, 0x13, 0x3a}}, {20, {0x23, 0x16, 0x2c}}};

  const auto itr = color_map.find(color_code);
  if (itr == color_map.end()) {
    return RGBAColorBuilder<ColorType>::MakeFromFloatColors(0.0f, 0.0f, 0.0f, alpha);
  }
  const auto& c = itr->second;
  return RGBAColorBuilder<ColorType>::MakeFromMixedColors(c[0], c[1], c[2], alpha);
}

/**
 * @brief UniqueColorGenerator Calls to `next` return a sequence of unique colors
 *
 */
template <typename ColorType>
class UniqueColorGenerator {
  uint32_t i = 1;
  float alpha;

 public:
  UniqueColorGenerator(float alpha_ = 1.0f) : alpha(alpha_) {}

  ColorType next() { return GenerateUniqueColor<ColorType>(i++, alpha); }
};

/**
 * @brief Multiply Multiples the color channels (r, g, b) of @color by
 *        @factor, ensuring the result stays in the range [0, 1]
 */
template <typename ColorType>
inline ColorType Multiply(const ColorType& color, const float factor) {
  ColorType ret;
  ret.r = TrimColorValue(factor * color.r);
  ret.g = TrimColorValue(factor * color.g);
  ret.b = TrimColorValue(factor * color.b);
  ret.a = color.a;
  return ret;
}

/**
 * @brief InterpolateColor Interpolates between c1 and c2 directly in RGBA space.
 *  @ColorType is intended to be a ros std_msgs::ColorRGBA type, or the RGBAColor
 *  defined in this file
 * @param c1 Start color for interpolation
 * @param c2 End color for interpolation
 * @param ratio Value between 0 and 1, indicating how far to move from c1 towards c2
 * @return
 */
template <typename ColorType>
inline ColorType InterpolateColor(const ColorType& c1, const ColorType& c2, const float& ratio) {
  // Safety check ratio
  // TODO: use SafetyCheckRatio from eigen_helpers.hpp (or move that function to this file)
  float real_ratio = ratio;
  if (real_ratio < 0.0) {
    real_ratio = 0.0;
    std::cerr << "Interpolation ratio < 0.0, set to 0.0" << std::endl;
  } else if (real_ratio > 1.0) {
    real_ratio = 1.0;
    std::cerr << "Interpolation ratio > 1.0, set to 1.0" << std::endl;
  }
  // Interpolate
  // This is the numerically stable version, rather than  (c1 + (c2 - c1) * real_ratio)
  ColorType ret;
  ret.r = TrimColorValue(c1.r * (1.0 - real_ratio) + c2.r * real_ratio);
  ret.g = TrimColorValue(c1.g * (1.0 - real_ratio) + c2.g * real_ratio);
  ret.b = TrimColorValue(c1.b * (1.0 - real_ratio) + c2.b * real_ratio);
  ret.a = TrimColorValue(c1.a * (1.0 - real_ratio) + c2.a * real_ratio);
  return ret;
}

////////////////////////////////////////////////////////////////////////////

inline int GetNumOMPThreads() {
#if defined(_OPENMP)
  int num_threads = 0;
#pragma omp parallel
  { num_threads = omp_get_num_threads(); }
  return num_threads;
#else
  return 1;
#endif
}

template <typename Datatype, typename Allocator = std::allocator<Datatype>>
inline Eigen::MatrixXd BuildDistanceMatrixParallel(
    const std::vector<Datatype, Allocator>& data,
    const std::function<double(const Datatype&, const Datatype&)>& distance_fn) {
  Eigen::MatrixXd distance_matrix(data.size(), data.size());
#if defined(_OPENMP)
#pragma omp parallel for
#endif
  for (size_t idx = 0; idx < data.size(); idx++) {
    for (size_t jdx = idx; jdx < data.size(); jdx++) {
      if (idx != jdx) {
        const double distance = distance_fn(data[idx], data[jdx]);
        distance_matrix((ssize_t)idx, (ssize_t)jdx) = distance;
        distance_matrix((ssize_t)jdx, (ssize_t)idx) = distance;
      } else {
        distance_matrix((ssize_t)idx, (ssize_t)jdx) = 0.0;
        distance_matrix((ssize_t)jdx, (ssize_t)idx) = 0.0;
      }
    }
  }
  return distance_matrix;
}

template <typename FirstDatatype, typename SecondDatatype, typename FirstAllocator = std::allocator<FirstDatatype>,
          typename SecondAllocator = std::allocator<SecondDatatype>>
inline Eigen::MatrixXd BuildDistanceMatrixParallel(
    const std::vector<FirstDatatype, FirstAllocator>& data1, const std::vector<SecondDatatype, SecondAllocator>& data2,
    const std::function<double(const FirstDatatype&, const SecondDatatype&)>& distance_fn) {
  Eigen::MatrixXd distance_matrix(data1.size(), data1.size());
#if defined(_OPENMP)
#pragma omp parallel for
#endif
  for (size_t idx = 0; idx < data1.size(); idx++) {
    for (size_t jdx = 0; jdx < data2.size(); jdx++) {
      const double distance = distance_fn(data1[idx], data2[jdx]);
      distance_matrix((ssize_t)idx, (ssize_t)jdx) = distance;
      distance_matrix((ssize_t)jdx, (ssize_t)idx) = distance;
    }
  }
  return distance_matrix;
}

template <typename Datatype, typename Allocator = std::allocator<Datatype>>
inline Eigen::MatrixXd BuildDistanceMatrixSerial(
    const std::vector<Datatype, Allocator>& data,
    const std::function<double(const Datatype&, const Datatype&)>& distance_fn) {
  Eigen::MatrixXd distance_matrix(data.size(), data.size());
  for (size_t idx = 0; idx < data.size(); idx++) {
    for (size_t jdx = idx; jdx < data.size(); jdx++) {
      if (idx != jdx) {
        const double distance = distance_fn(data[idx], data[jdx]);
        distance_matrix((ssize_t)idx, (ssize_t)jdx) = distance;
        distance_matrix((ssize_t)jdx, (ssize_t)idx) = distance;
      } else {
        distance_matrix((ssize_t)idx, (ssize_t)jdx) = 0.0;
        distance_matrix((ssize_t)jdx, (ssize_t)idx) = 0.0;
      }
    }
  }
  return distance_matrix;
}

template <typename FirstDatatype, typename SecondDatatype, typename FirstAllocator = std::allocator<FirstDatatype>,
          typename SecondAllocator = std::allocator<SecondDatatype>>
inline Eigen::MatrixXd BuildDistanceMatrixSerial(
    const std::vector<FirstDatatype, FirstAllocator>& data1, const std::vector<SecondDatatype, SecondAllocator>& data2,
    const std::function<double(const FirstDatatype&, const SecondDatatype&)>& distance_fn) {
  Eigen::MatrixXd distance_matrix(data1.size(), data1.size());
  for (size_t idx = 0; idx < data1.size(); idx++) {
    for (size_t jdx = 0; jdx < data2.size(); jdx++) {
      const double distance = distance_fn(data1[idx], data2[jdx]);
      distance_matrix((ssize_t)idx, (ssize_t)jdx) = distance;
      distance_matrix((ssize_t)jdx, (ssize_t)idx) = distance;
    }
  }
  return distance_matrix;
}

template <typename Item, typename Value, typename ItemAlloc = std::allocator<Item>>
std::vector<std::pair<int64_t, double>> GetKNearestNeighborsParallel(
    const std::vector<Item, ItemAlloc>& items, const Value& current,
    const std::function<double(const Item&, const Value&)>& distance_fn, const size_t K) {
  if (K == 0) {
    return std::vector<std::pair<int64_t, double>>();
  }
  if (items.size() > K) {
    std::function<bool(const std::pair<int64_t, double>&, const std::pair<int64_t, double>&)> compare_fn =
        [](const std::pair<int64_t, double>& index1, const std::pair<int64_t, double>& index2) {
          return index1.second < index2.second;
        };
    std::vector<std::vector<std::pair<int64_t, double>>> per_thread_nearests(
        GetNumOMPThreads(),
        std::vector<std::pair<int64_t, double>>(K, std::make_pair(-1, std::numeric_limits<double>::infinity())));
#if defined(_OPENMP)
#pragma omp parallel for
#endif
    for (size_t idx = 0; idx < items.size(); idx++) {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
#if defined(_OPENMP)
      const size_t thread_num = (size_t)omp_get_thread_num();
#else
      const size_t thread_num = 0;
#endif
      std::vector<std::pair<int64_t, double>>& current_thread_nearests = per_thread_nearests[thread_num];
      auto itr = std::max_element(current_thread_nearests.begin(), current_thread_nearests.end(), compare_fn);
      const double worst_distance = itr->second;
      if (worst_distance > distance) {
        itr->first = (int64_t)idx;
        itr->second = distance;
      }
    }
    std::vector<std::pair<int64_t, double>> k_nearests;
    k_nearests.reserve(K);
    for (size_t thread_idx = 0; thread_idx < per_thread_nearests.size(); thread_idx++) {
      const std::vector<std::pair<int64_t, double>>& thread_nearests = per_thread_nearests[thread_idx];
      for (size_t nearest_idx = 0; nearest_idx < thread_nearests.size(); nearest_idx++) {
        const std::pair<int64_t, double> current_ith_nearest = thread_nearests[nearest_idx];
        if (!std::isinf(current_ith_nearest.second) && current_ith_nearest.first != -1) {
          if (k_nearests.size() < K) {
            k_nearests.push_back(current_ith_nearest);
          } else {
            auto itr = std::max_element(k_nearests.begin(), k_nearests.end(), compare_fn);
            const double worst_distance = itr->second;
            if (worst_distance > current_ith_nearest.second) {
              itr->first = current_ith_nearest.first;
              itr->second = current_ith_nearest.second;
            }
          }
        }
      }
    }
    k_nearests.shrink_to_fit();
    return k_nearests;
  } else {
    std::vector<std::pair<int64_t, double>> k_nearests(items.size(),
                                                       std::make_pair(-1, std::numeric_limits<double>::infinity()));
#if defined(_OPENMP)
#pragma omp parallel for
#endif
    for (size_t idx = 0; idx < items.size(); idx++) {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
      k_nearests[idx] = std::make_pair((int64_t)idx, distance);
    }
    return k_nearests;
  }
}

template <typename Item, typename Value, typename ItemAlloc = std::allocator<Item>>
std::vector<std::pair<int64_t, double>> GetKNearestNeighborsSerial(
    const std::vector<Item, ItemAlloc>& items, const Value& current,
    const std::function<double(const Item&, const Value&)>& distance_fn, const size_t K) {
  if (K == 0) {
    return std::vector<std::pair<int64_t, double>>();
  }
  if (items.size() > K) {
    std::function<bool(const std::pair<int64_t, double>&, const std::pair<int64_t, double>&)> compare_fn =
        [](const std::pair<int64_t, double>& index1, const std::pair<int64_t, double>& index2) {
          return index1.second < index2.second;
        };
    std::vector<std::pair<int64_t, double>> k_nearests(K, std::make_pair(-1, std::numeric_limits<double>::infinity()));
    for (size_t idx = 0; idx < items.size(); idx++) {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
      auto itr = std::max_element(k_nearests.begin(), k_nearests.end(), compare_fn);
      const double worst_distance = itr->second;
      if (worst_distance > distance) {
        itr->first = (int64_t)idx;
        itr->second = distance;
      }
    }
    return k_nearests;
  } else {
    std::vector<std::pair<int64_t, double>> k_nearests(items.size(),
                                                       std::make_pair(-1, std::numeric_limits<double>::infinity()));
    for (size_t idx = 0; idx < items.size(); idx++) {
      const Item& item = items[idx];
      const double distance = distance_fn(item, current);
      k_nearests[idx] = std::make_pair((int64_t)idx, distance);
    }
    return k_nearests;
  }
}

class AstarPQueueElement {
 protected:
  int64_t node_id_;
  int64_t backpointer_;
  double cost_to_come_;
  double value_;

 public:
  AstarPQueueElement(const int64_t node_id, const int64_t backpointer, const double cost_to_come, const double value)
      : node_id_(node_id), backpointer_(backpointer), cost_to_come_(cost_to_come), value_(value) {}

  inline int64_t id() const { return node_id_; }

  inline int64_t backpointer() const { return backpointer_; }

  inline double costToCome() const { return cost_to_come_; }

  inline double value() const { return value_; }
};

class CompareAstarPQueueElementFn {
 public:
  bool operator()(const AstarPQueueElement& lhs, const AstarPQueueElement& rhs) const {
    return lhs.value() > rhs.value();
  }
};

// Return is a pair<path, cost>
// Path is a vector of node indices in the provided graph
// Cost is the computed cost-to-come of the goal node
typedef std::pair<std::vector<int64_t>, double> AstarResult;

inline AstarResult ExtractAstarResult(const std::unordered_map<int64_t, std::pair<int64_t, double>>& explored,
                                      const int64_t start_index, const int64_t goal_index) {
  // Check if a solution was found
  const auto goal_index_itr = explored.find(goal_index);
  // If no solution found
  if (goal_index_itr == explored.end()) {
    return std::make_pair(std::vector<int64_t>(), std::numeric_limits<double>::infinity());
  }
  // If a solution was found
  // Extract the path indices in reverse order
  std::vector<int64_t> solution_path_indices;
  solution_path_indices.push_back(goal_index);
  int64_t backpointer = goal_index_itr->second.first;
  // Any backpointer >= 0 is a valid node in the graph
  // The backpointer for start_index is -1
  while (backpointer >= 0) {
    const int64_t current_index = backpointer;
    solution_path_indices.push_back(current_index);
    if (current_index == start_index) {
      break;
    }

    // This provides bounds safety check
    const auto current_index_data = explored.at(current_index);
    backpointer = current_index_data.first;
  }
  // Reverse
  std::reverse(solution_path_indices.begin(), solution_path_indices.end());
  // Get the cost of the path
  const double solution_path_cost = goal_index_itr->second.second;
  return std::make_pair(solution_path_indices, solution_path_cost);
}

inline AstarResult GenericAstarSearch(const int64_t start_id, const int64_t goal_id,
                                      const std::function<std::vector<int64_t>(const int64_t)>& generate_children_fn,
                                      const std::function<bool(const int64_t, const int64_t)>& edge_validity_check_fn,
                                      const std::function<double(const int64_t, const int64_t)>& distance_fn,
                                      const std::function<double(const int64_t, const int64_t)>& heuristic_fn,
                                      const bool limit_pqueue_duplicates) {
  // Enforced sanity checks
  if (start_id == goal_id) {
    throw std::invalid_argument("Start and goal ID must be different");
  }
  // Make helper function
  const auto heuristic_function = [&](const int64_t node_index) { return heuristic_fn(node_index, goal_id); };
  // Setup
  std::priority_queue<AstarPQueueElement, std::vector<AstarPQueueElement>, CompareAstarPQueueElementFn> queue;
  // Optional map to reduce the number of duplicate items added to the pqueue
  // Key is the node ID
  // Value is cost-to-come
  std::unordered_map<int64_t, double> queue_members_map;
  // Key is the node ID
  // Value is a pair<backpointer, cost-to-come>
  // backpointer is the parent node ID
  std::unordered_map<int64_t, std::pair<int64_t, double>> explored;
  // Initialize
  queue.push(AstarPQueueElement(start_id, -1, 0.0, heuristic_function(start_id)));
  if (limit_pqueue_duplicates) {
    queue_members_map[start_id] = 0.0;
  }
  // Search
  while (queue.size() > 0) {
    // Get the top of the priority queue
    const AstarPQueueElement top_node = queue.top();
    queue.pop();
    // Remove from queue map if necessary
    if (limit_pqueue_duplicates) {
      queue_members_map.erase(top_node.id());
    }
    // Check if the node has already been discovered
    const auto node_explored_find_itr = explored.find(top_node.id());
    // We have not been here before, or it is cheaper now
    const bool node_in_explored = (node_explored_find_itr != explored.end());
    const bool node_explored_is_better =
        (node_in_explored) ? (top_node.costToCome() >= node_explored_find_itr->second.second) : false;
    if (!node_explored_is_better) {
      // Add to the explored list
      explored[top_node.id()] = std::make_pair(top_node.backpointer(), top_node.costToCome());
      // Check if we have reached the goal
      if (top_node.id() == goal_id) {
        break;
      }
      // Generate possible children
      const std::vector<int64_t> candidate_children = generate_children_fn(top_node.id());
      // Loop through potential child nodes
      for (const int64_t child_node_id : candidate_children) {
        // Check if the top node->child edge is valid
        if (edge_validity_check_fn(top_node.id(), child_node_id)) {
          // Compute the cost-to-come for the new child
          const double parent_cost_to_come = top_node.costToCome();
          const double parent_to_child_cost = distance_fn(top_node.id(), child_node_id);
          const double child_cost_to_come = parent_cost_to_come + parent_to_child_cost;
          // Check if the child state has already been explored
          const auto child_explored_find_itr = explored.find(child_node_id);
          // It is not in the explored list, or is there with a higher cost-to-come
          const bool child_in_explored = (child_explored_find_itr != explored.end());
          const bool explored_child_is_better =
              (child_in_explored) ? (child_cost_to_come >= child_explored_find_itr->second.second) : false;
          // Check if the child state is already in the queue
          bool queue_is_better = false;
          if (limit_pqueue_duplicates) {
            const auto queue_members_map_itr = queue_members_map.find(child_node_id);
            const bool in_queue = (queue_members_map_itr != queue_members_map.end());
            queue_is_better = (in_queue) ? (child_cost_to_come >= queue_members_map_itr->second) : false;
          }
          // Only add the new state if we need to
          if (!explored_child_is_better && !queue_is_better) {
            // Compute the heuristic for the child
            const double child_heuristic = heuristic_function(child_node_id);
            // Compute the child value
            const double child_value = child_cost_to_come + child_heuristic;
            queue.push(AstarPQueueElement(child_node_id, top_node.id(), child_cost_to_come, child_value));
          }
        }
      }
    }
  }
  return ExtractAstarResult(explored, start_id, goal_id);
}

}