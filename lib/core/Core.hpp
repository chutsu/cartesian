#pragma once

/*****************************************************************************
 * Contents:
 * - MACROS
 * - DATA TYPE
 * - FILESYSTEM
 * - DATA
 * - ALGEBRA
 * - LINEAR ALGEBRA
 * - GEOMETRY
 * - STATISTICS
 * - TRANSFORM
 * - TIME
 * - INTERPOLATION
 * - MEASUREMENTS
 * - PARAMETERS
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <inttypes.h>
#include <dirent.h>
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <random>
#include <set>
#include <list>
#include <deque>
#include <vector>
#include <regex>
#include <unordered_map>
#include <unordered_set>
#include <type_traits>
#include <filesystem>

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace cartesian {

namespace fs = std::filesystem;

/******************************************************************************
 *                                MACROS
 *****************************************************************************/

#define KNRM "\x1B[1;0m"
#define KRED "\x1B[1;31m"
#define KGRN "\x1B[1;32m"
#define KYEL "\x1B[1;33m"
#define KBLU "\x1B[1;34m"
#define KMAG "\x1B[1;35m"
#define KCYN "\x1B[1;36m"
#define KWHT "\x1B[1;37m"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define __FILENAME__                                                           \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LOG_ERROR(M, ...)                                                      \
  fprintf(stderr,                                                              \
          "\033[31m[ERROR] [%s:%d] " M "\033[0m\n",                            \
          __FILENAME__,                                                        \
          __LINE__,                                                            \
          ##__VA_ARGS__)

#define LOG_INFO(M, ...) fprintf(stdout, "[INFO] " M "\n", ##__VA_ARGS__)
#define LOG_WARN(M, ...)                                                       \
  fprintf(stdout, "\033[33m[WARN] " M "\033[0m\n", ##__VA_ARGS__)

#define FATAL(M, ...)                                                          \
  {                                                                            \
    char msg[9046] = {0};                                                      \
    sprintf(msg,                                                               \
            "\033[31m[FATAL] [%s:%d] " M "\033[0m\n",                          \
            __FILENAME__,                                                      \
            __LINE__,                                                          \
            ##__VA_ARGS__);                                                    \
    throw std::runtime_error(msg);                                             \
  }

#define FATAL_ASSERT(X, M, ...)                                                \
  if (!(X)) {                                                                  \
    char msg[9046] = {0};                                                      \
    sprintf(msg,                                                               \
            "\033[31m[FATAL] [%s:%d] " M "\033[0m\n",                          \
            __FILENAME__,                                                      \
            __LINE__,                                                          \
            ##__VA_ARGS__);                                                    \
    throw std::runtime_error(msg);                                             \
  }

// #ifdef NDEBUG
// #define DEBUG(M, ...)
// #else
// #define DEBUG(M, ...) fprintf(stdout, "[DEBUG] " M "\n", ##__VA_ARGS__)
// #endif

#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

#ifndef CHECK
#define CHECK(A)                                                               \
  if (!(A)) {                                                                  \
    char msg[9046] = {0};                                                      \
    sprintf(msg,                                                               \
            "\033[31m[CHECK FAILED] [%s:%d] " TOSTRING(A) "\033[0m\n",         \
            __FILENAME__,                                                      \
            __LINE__);                                                         \
    throw std::runtime_error(msg);                                             \
  }
#endif

/******************************************************************************
 *                                DATA TYPE
 *****************************************************************************/

// -- TIMESTAMP ----------------------------------------------------------------

typedef int64_t timestamp_t;
typedef std::vector<timestamp_t> timestamps_t;

// -- VECTOR ------------------------------------------------------------------

using Vec2i = Eigen::Matrix<int, 2, 1>;
using Vec3i = Eigen::Matrix<int, 3, 1>;
using Vec4i = Eigen::Matrix<int, 4, 1>;
using Vec5i = Eigen::Matrix<int, 5, 1>;
using Vec6i = Eigen::Matrix<int, 6, 1>;
using Vec7i = Eigen::Matrix<int, 7, 1>;
using Vec8i = Eigen::Matrix<int, 8, 1>;
using Vec9i = Eigen::Matrix<int, 9, 1>;
using VecXi = Eigen::Matrix<int, Eigen::Dynamic, 1>;

using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec4 = Eigen::Matrix<double, 4, 1>;
using Vec5 = Eigen::Matrix<double, 5, 1>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec7 = Eigen::Matrix<double, 7, 1>;
using Vec8 = Eigen::Matrix<double, 8, 1>;
using Vec9 = Eigen::Matrix<double, 9, 1>;
using VecX = Eigen::Matrix<double, Eigen::Dynamic, 1>;

using Vec2s = std::vector<Vec2>;
using Vec3s = std::vector<Vec3>;
using Vec4s = std::vector<Vec4>;
using Vec5s = std::vector<Vec5>;
using Vec6s = std::vector<Vec6>;
using Vec7s = std::vector<Vec7>;
using Vec8s = std::vector<Vec8>;
using Vec9s = std::vector<Vec9>;
using VecXs = std::vector<VecX>;

template <int LENGTH, Eigen::StorageOptions STRIDE_TYPE = Eigen::ColMajor>
using Vec = Eigen::Matrix<double, LENGTH, 1, STRIDE_TYPE>;

// -- MATRIX -------------------------------------------------------------------

using Mat2 = Eigen::Matrix<double, 2, 2>;
using Mat3 = Eigen::Matrix<double, 3, 3>;
using Mat4 = Eigen::Matrix<double, 4, 4>;
using MatX = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using MatXRowMajor =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Mat34 = Eigen::Matrix<double, 3, 4>;

using Mat2s = std::vector<Mat2>;
using Mat3s = std::vector<Mat3>;
using Mat4s = std::vector<Mat4>;
using MatXs = std::vector<MatX>;
using MatXsRowMajor = std::vector<MatXRowMajor>;

template <int ROWS,
          int COLS,
          Eigen::StorageOptions STRIDE_TYPE = Eigen::ColMajor>
using Mat = Eigen::Matrix<double, ROWS, COLS, STRIDE_TYPE>;

// -- GEOMETRY -----------------------------------------------------------------

using Quat = Eigen::Quaternion<double>;
using Quats = std::vector<Quat>;
using AngleAxis = Eigen::AngleAxis<double>;
using ArrayX = Eigen::Array<double, Eigen::Dynamic, 1>;

/******************************************************************************
 *                                FILESYSTEM
 *****************************************************************************/

/**
 * Open file in `path` with `mode` and set `nb_rows`.
 * @returns File pointer on success, nullptr on failure.
 */
FILE *file_open(const std::string &path,
                const std::string &mode,
                int *nb_rows = nullptr);

/**
 * Skip file line in file `fp`.
 */
void skip_line(FILE *fp);

/**
 * Get number of rows in file.
 * @returns Number of rows in file else -1 for failure.
 */
int file_rows(const std::string &file_path);

/**
 * Copy file from path `src` to path `dest.
 *
 * @returns 0 for success else -1 if `src` file could not be opend, or -2 if
 * `dest` file could not be opened.
 */
int file_copy(const std::string &src, const std::string &dest);

/*****************************************************************************
 *                               CONFIG
 *****************************************************************************/

/** Configuration **/
struct config_t {
  std::string file_path;
  YAML::Node root;
  bool ok = false;

  config_t() = default;
  config_t(const std::string &file_path_);
  ~config_t() = default;
};

/**
 * Load YAML file.
 * @returns 0 for success or -1 for failure.
 */
int yaml_load_file(const std::string file_path, YAML::Node &root);

/**
 * Get YAML node containing the parameter value.
 * @returns 0 for success or -1 for failure.
 */
int yaml_get_node(const config_t &config,
                  const std::string &key,
                  const bool optional,
                  YAML::Node &node);

/**
 * Check if yaml file has `key`.
 * @returns 0 for success or -1 for failure.
 */
int yaml_has_key(const config_t &config, const std::string &key);

/**
 * Check if yaml file has `key`.
 * @returns 0 for success or -1 for failure.
 */
int yaml_has_key(const std::string &file_path, const std::string &key);

/**
 * Check size of vector in config file and returns the size.
 */
template <typename T>
size_t yaml_check_vector(const YAML::Node &node,
                         const std::string &key,
                         const bool optional);

/**
 * Check matrix fields.
 */
void yaml_check_matrix_fields(const YAML::Node &node,
                              const std::string &key,
                              size_t &rows,
                              size_t &cols);

/**
 * Check matrix to make sure that the parameter has the data field "rows",
 * "cols" and "data". It also checks to make sure the number of values is the
 * same size as the matrix.
 */
template <typename T>
void yaml_check_matrix(const YAML::Node &node,
                       const std::string &key,
                       const bool optional,
                       size_t &rows,
                       size_t &cols);

template <typename T>
void yaml_check_matrix(const YAML::Node &node,
                       const std::string &key,
                       const bool optional);

template <typename T>
int parse(const config_t &config,
          const std::string &key,
          T &out,
          const bool optional = false);

template <typename T>
int parse(const config_t &config,
          const std::string &key,
          std::vector<T> &out,
          const bool optional);

int parse(const config_t &config,
          const std::string &key,
          fs::path &path,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          Vec2 &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          Vec3 &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          Vec4 &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          Vec5 &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          Vec6 &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          Vec7 &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          VecX &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          Vec2i &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          Vec3i &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          Vec4i &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          VecXi &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          Mat2 &mat,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          Mat3 &mat,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          Mat4 &mat,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          MatX &mat,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          cv::Mat &mat,
          const bool optional = false);

template <typename T>
int parse(const config_t &config,
          const std::string &key,
          const int rows,
          const int cols,
          T &mat,
          const bool optional = false);

////////// CONFIG IMPLEMENTATION

template <typename T>
size_t yaml_check_vector(const YAML::Node &node,
                         const std::string &key,
                         const bool optional) {
  UNUSED(optional);
  assert(node);

  // Get expected vector size
  size_t vector_size = 0;
  if (std::is_same<T, Vec2>::value) {
    vector_size = 2;
  } else if (std::is_same<T, Vec3>::value) {
    vector_size = 3;
  } else if (std::is_same<T, Vec4>::value) {
    vector_size = 4;
  } else if (std::is_same<T, Vec5>::value) {
    vector_size = 5;
  } else if (std::is_same<T, VecX>::value) {
    vector_size = node.size();
    return vector_size; // Don't bother, it could be anything
  } else if (std::is_same<T, Vec2i>::value) {
    vector_size = 2;
  } else if (std::is_same<T, Vec3i>::value) {
    vector_size = 3;
  } else if (std::is_same<T, Vec4i>::value) {
    vector_size = 4;
  } else if (std::is_same<T, Vec5i>::value) {
    vector_size = 5;
  } else if (std::is_same<T, VecXi>::value) {
    vector_size = node.size();
    return vector_size; // Don't bother, it could be anything
  } else {
    LOG_ERROR("Failed to check vector [%s]!", key.c_str());
    FATAL("Unsupported vector type!");
  }

  // Check number of values in the param
  if (node.size() == 0 && node.size() != vector_size) {
    FATAL("Vector [%s] should have %d values but config has %d!",
          key.c_str(),
          static_cast<int>(vector_size),
          static_cast<int>(node.size()));
  }

  return vector_size;
}

template <typename T>
void yaml_check_matrix(const YAML::Node &node,
                       const std::string &key,
                       const bool optional,
                       size_t &rows,
                       size_t &cols) {
  UNUSED(optional);
  assert(node);
  yaml_check_matrix_fields(node, key, rows, cols);

  // Check number of elements
  size_t nb_elements = 0;
  if (std::is_same<T, Mat2>::value) {
    nb_elements = 4;
  } else if (std::is_same<T, Mat3>::value) {
    nb_elements = 9;
  } else if (std::is_same<T, Mat4>::value) {
    nb_elements = 16;
  } else if (std::is_same<T, MatX>::value) {
    nb_elements = node["data"].size();

  } else if (std::is_same<T, cv::Mat>::value) {
    nb_elements = node["data"].size();
  } else {
    FATAL("Unsportted matrix type!");
  }
  if (node["data"].size() != nb_elements) {
    FATAL("Matrix [%s] rows and cols do not match number of values!",
          key.c_str());
  }
}

template <typename T>
void yaml_check_matrix(const YAML::Node &node,
                       const std::string &key,
                       const bool optional) {
  size_t rows;
  size_t cols;
  yaml_check_matrix<T>(node, key, optional, rows, cols);
}

template <typename T>
int parse(const config_t &config,
          const std::string &key,
          T &out,
          const bool optional) {
  try {
    // Get node
    YAML::Node node;
    if (yaml_get_node(config, key, optional, node) != 0) {
      return -1;
    }

    // Parse
    out = node.as<T>();
  } catch (const std::exception &e) {
    FATAL("Failed to parse [%s]: %s", key.c_str(), e.what());
  }

  return 0;
}

template <typename T>
int parse(const config_t &config,
          const std::string &key,
          std::vector<T> &out,
          const bool optional) {
  // Get node
  try {
    YAML::Node node;
    if (yaml_get_node(config, key, optional, node) != 0) {
      return -1;
    }

    // Parse
    std::vector<T> array;
    for (auto n : node) {
      out.push_back(n.as<T>());
    }
  } catch (const std::exception &e) {
    FATAL("Failed to parse [%s]: %s", key.c_str(), e.what());
  }

  return 0;
}

/*****************************************************************************
 *                                  DATA
 *****************************************************************************/

/**
 * Convert bytes to signed 8bit number
 */
int8_t int8(const uint8_t *data, const size_t offset);

/**
 * Convert bytes to unsigned 8bit number
 */
uint8_t uint8(const uint8_t *data, const size_t offset);

/**
 * Convert bytes to signed 16bit number
 */
int16_t int16(const uint8_t *data, const size_t offset);

/**
 * Convert bytes to unsigned 16bit number
 */
uint16_t uint16(const uint8_t *data, const size_t offset);

/**
 * Convert bytes to signed 32bit number
 */
int32_t int32(const uint8_t *data, const size_t offset);

/**
 * Convert bytes to unsigned 32bit number
 */
uint32_t uint32(const uint8_t *data, const size_t offset);

/**
 * Allocate memory for a C-style string
 */
char *malloc_string(const char *s);

/**
 * Search and replace substring
 */
std::string replace(const std::string &in,
                    const std::string &from,
                    const std::string &to);

/**
 * Get number of rows in CSV file.
 * @returns Number of rows in CSV file else -1 for failure.
 */
int csv_rows(const char *fp);

/**
 * Get number of cols in CSV file.
 * @returns Number of cols in CSV file else -1 for failure.
 */
int csv_cols(const char *fp);

/**
 * Return csv fields as strings and number of fields in csv file.
 */
char **csv_fields(const char *fp, int *nb_fields);

/**
 * Load data in csv file `fp`. Assumming the data are doubles. Also returns
 * number of rows and cols in `nb_rows` and `nb_cols` respectively.
 */
double **csv_data(const char *fp, int *nb_rows, int *nb_cols);

/**
 * Load integer arrays in csv file located at `csv_path`. The number of arrays
 * is returned in `nb_arrays`.
 */
int **load_iarrays(const char *csv_path, int *nb_arrays);

/**
 * Load double arrays in csv file located at `csv_path`. The number of arrays
 * is returned in `nb_arrays`.
 */
double **load_darrays(const char *csv_path, int *nb_arrays);

/**
 * Get number of rows in CSV file.
 * @returns Number of rows in CSV file else -1 for failure.
 */
int csv_rows(const std::string &file_path);

/**
 * Get number of columns in CSV file.
 * @returns Number of columns in CSV file else -1 for failure.
 */
int csv_cols(const std::string &file_path);

/**
 * Convert CSV file to matrix.
 * @returns 0 for success, -1 for failure
 */
int csv2mat(const std::string &file_path, const bool header, MatX &data);

/**
 * Convert CSV file to vector.
 * @returns 0 for success, -1 for failure
 */
int csv2vec(const std::string &file_path, const bool header, VecX &data);

/**
 * Convert matrix to csv file.
 * @returns 0 for success, -1 for failure
 */
int mat2csv(const std::string &file_path, const MatX &data);

/**
 * Convert vector to csv file.
 * @returns 0 for success, -1 for failure
 */
int vec2csv(const std::string &file_path, const std::deque<Vec3> &data);

/**
 * Convert timestamps to csv file.
 * @returns 0 for success, -1 for failure
 */
int ts2csv(const std::string &file_path, const std::deque<timestamp_t> &data);

/**
 * Print progress to screen
 */
void print_progress(const double percentage, const std::string &prefix = "");

/**
 * Check if vector `x` is all true.
 */
bool all_true(const std::vector<bool> x);

/**
 * Pop front of an `std::vector`.
 */
template <typename T>
void pop_front(std::vector<T> &vec) {
  assert(!vec.empty());
  vec.front() = std::move(vec.back());
  vec.pop_back();
}

/**
 * Pop front of an `std::vector`.
 */
template <typename T1, typename T2>
void pop_front(std::vector<T1, T2> &vec) {
  assert(!vec.empty());
  vec.front() = std::move(vec.back());
  vec.pop_back();
}

/**
 * Extend `std::vector`.
 */
template <typename T>
void extend(std::vector<T> &x, const std::vector<T> &add) {
  x.reserve(x.size() + add.size());
  x.insert(x.end(), add.begin(), add.end());
}

/**
 * Extend `std::vector`.
 */
template <typename T1, typename T2>
void extend(std::vector<T1, T2> &x, std::vector<T1, T2> &add) {
  x.reserve(x.size() + add.size());
  x.insert(x.end(), add.begin(), add.end());
}

/**
 * Slice `std::vector`.
 */
template <typename T>
std::vector<T> slice(std::vector<T> const &v, int m, int n) {
  auto first = v.cbegin() + m;
  auto last = v.cbegin() + n + 1;

  std::vector<T> vec(first, last);
  return vec;
}

/**
 * Slice `std::vector`.
 */
template <typename T1, typename T2>
std::vector<T1, T2> slice(std::vector<T1, T2> const &v, int m, int n) {
  auto first = v.cbegin() + m;
  auto last = v.cbegin() + n + 1;

  std::vector<T1, T2> vec(first, last);
  return vec;
}

/**
 * Get raw pointer of a value in a `std::map`.
 */
template <typename K, typename V>
const V *lookup(const std::map<K, V> &map, K key) {
  typename std::map<K, V>::const_iterator iter = map.find(key);
  if (iter != map.end()) {
    return &iter->second;
  } else {
    return nullptr;
  }
}

/**
 * Get raw pointer of a value in a `std::map`.
 */
template <typename K, typename V>
V *lookup(std::map<K, V> &map, K key) {
  return const_cast<V *>(lookup(const_cast<const std::map<K, V> &>(map), key));
}

/**
 * Get raw pointer of a value in a `std::map`.
 */
template <typename K, typename V>
const V *lookup(const std::unordered_map<K, V> &map, K key) {
  typename std::unordered_map<K, V>::const_iterator iter = map.find(key);
  if (iter != map.end()) {
    return &iter->second;
  } else {
    return nullptr;
  }
}

/**
 * Get raw pointer of a value in a `std::map`.
 */
template <typename K, typename V>
V *lookup(std::unordered_map<K, V> &map, K key) {
  return const_cast<V *>(
      lookup(const_cast<const std::unordered_map<K, V> &>(map), key));
}

/**
 * Union between set `a` and set `b`.
 */
template <typename T>
T set_union(const T &s1, const T &s2) {
  T result = s1;
  result.insert(s2.begin(), s2.end());
  return result;
}

/**
 * Difference between `a` and set `b`.
 */
template <typename T>
T set_diff(const T &a, const T &b) {
  T results;
  std::set_difference(a.begin(),
                      a.end(),
                      b.begin(),
                      b.end(),
                      std::inserter(results, results.end()));
  return results;
}

/**
 * Symmetric difference between `a` and `b`.
 */
template <typename T>
T set_symmetric_diff(const T &a, const T &b) {
  T results;
  std::set_symmetric_difference(a.begin(),
                                a.end(),
                                b.begin(),
                                b.end(),
                                std::back_inserter(results));
  return results;
}

/**
 * Intersection between std::vectors `vecs`.
 * @returns Number of common elements
 */
template <typename T>
std::set<T> intersection(const std::list<std::vector<T>> &vecs) {
  // Obtain element count across all vectors
  std::unordered_map<T, size_t> counter;
  for (const auto &vec : vecs) { // Loop over all vectors
    for (const auto &p : vec) {  // Loop over elements in vector
      counter[p] += 1;
    }
  }

  // Build intersection result
  std::set<T> retval;
  for (const auto &el : counter) {
    if (el.second == vecs.size()) {
      retval.insert(el.first);
    }
  }

  return retval;
}

/**
 * Ordered Set
 */
template <class T>
class ordered_set_t {
public:
  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;

  iterator begin() { return vector.begin(); }
  iterator end() { return vector.end(); }
  const_iterator begin() const { return vector.begin(); }
  const_iterator end() const { return vector.end(); }
  const T &at(const size_t i) const { return vector.at(i); }
  const T &front() const { return vector.front(); }
  const T &back() const { return vector.back(); }
  void insert(const T &item) {
    if (set.insert(item).second)
      vector.push_back(item);
  }
  size_t count(const T &item) const { return set.count(item); }
  bool empty() const { return set.empty(); }
  size_t size() const { return set.size(); }
  void clear() {
    vector.clear();
    set.clear();
  }

private:
  std::vector<T> vector;
  std::set<T> set;
};

/* Save vector of size 3 `y` with timestamps `ts` to `save_path`. */
void save_data(const std::string &save_path,
               const timestamps_t &ts,
               const Vec3s &y);

/* Save quaternions with timestamps `ts` to `save_path`. */
void save_data(const std::string &save_path,
               const timestamps_t &ts,
               const Quats &y);

/**
 * Save 3D features to csv file defined in `path`.
 */
void save_features(const std::string &path, const Vec3s &features);

/**
 * Save poses to csv file in `path`.
 */
void save_poses(const std::string &path,
                const timestamps_t &timestamps,
                const Mat4s &poses);

/**
 * Save extrinsics to csv file in `path`.
 */
void save_extrinsics(const std::string &path, const Mat4 &ext);

/**
 * Save IMU measurements
 */
void save_imu_data(const std::string &path,
                   const timestamps_t &imu_ts,
                   const Vec3s &imu_acc,
                   const Vec3s &imu_gyr);

/** Load pose */
Mat4 load_pose(const std::string &fpath);

/** Load poses */
void load_poses(const std::string &fpath,
                timestamps_t &timestamps,
                Mat4s &poses);

/******************************************************************************
 *                                  ALGEBRA
 *****************************************************************************/

/**
 * Sign of number
 *
 * @param[in] x Number to check sign
 * @return
 *    - 0: Number is zero
 *    - 1: Positive number
 *    - -1: Negative number
 */
int sign(const double x);

/**
 * Floating point comparator
 *
 * @param[in] f1 First value
 * @param[in] f2 Second value
 * @return
 *    - 0: if equal
 *    - 1: if f1 > f2
 *    - -1: if f1 < f2
 */
int fltcmp(const double f1, const double f2);

/**
 * Calculate binomial coefficient
 *
 * @param[in] n
 * @param[in] k
 * @returns Binomial coefficient
 */
double binomial(const double n, const double k);

/**
 * Return evenly spaced numbers over a specified interval.
 */
template <typename T>
std::vector<T> linspace(const T start, const T end, const int num) {
  std::vector<T> linspaced;

  if (num == 0) {
    return linspaced;
  }
  if (num == 1) {
    linspaced.push_back(start);
    return linspaced;
  }

  const double diff = static_cast<double>(end - start);
  const double delta = diff / static_cast<double>(num - 1);
  for (int i = 0; i < num - 1; ++i) {
    linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end);
  return linspaced;
}

/******************************************************************************
 *                              LINEAR ALGEBRA
 *****************************************************************************/

/**
 * Print shape of a matrix
 *
 * @param[in] name Name of matrix
 * @param[in] A Matrix
 */
void print_shape(const std::string &name, const MatX &A);

/**
 * Print shape of a vector
 *
 * @param[in] name Name of vector
 * @param[in] v Vector
 */
void print_shape(const std::string &name, const VecX &v);

/**
 * Print array
 *
 * @param[in] name Name of array
 * @param[in] array Target array
 * @param[in] size Size of target array
 */
void print_array(const std::string &name,
                 const double *array,
                 const size_t size);

/**
 * Print vector `v` with a `name`.
 */
void print_vector(const std::string &name, const VecX &v);

/**
 * Print vector `v` with a `name`.
 */
void print_vector(const std::string &name, const double *v, const int N);

/**
 * Print matrix `m` with a `name`.
 */
void print_matrix(const std::string &name,
                  const MatX &m,
                  const std::string &indent = "");

/**
 * Print quaternion `q` with a `name`.
 */
void print_quaternion(const std::string &name, const Quat &q);

/**
 * Array to string
 *
 * @param[in] array Target array
 * @param[in] size Size of target array
 * @returns String of array
 */
std::string array2str(const double *array, const size_t size);

/**
 * Convert double array to Eigen::Vector
 *
 * @param[in] x Input array
 * @param[in] size Size of input array
 * @param[out] y Output vector
 */
void array2vec(const double *x, const size_t size, VecX &y);

/**
 * Vector to array
 *
 * @param[in] v Vector
 * @returns Array
 */
double *vec2array(const VecX &v);

/**
 * Matrix to array
 *
 * @param[in] m Matrix
 * @returns Array
 */
double *mat2array(const MatX &m);

/**
 * Quaternion to array
 *
 * *VERY IMPORTANT*: The returned array is (x, y, z, w).
 *
 * @param[in] q Quaternion
 * @returns Array
 */
double *quat2array(const Quat &q);

/**
 * Vector to array
 *
 * @param[in] v Vector
 * @param[out] out Output array
 */
void vec2array(const VecX &v, double *out);

/**
 * Matrix to array
 *
 * @param[in] m Matrix
 * @param[in] out Output array
 */
void mat2array(const MatX &m, double *out);

/**
 * Matrix to list of vectors
 *
 * @param[in] m Matrix
 * @param[in] row_wise Row wise
 * @returns Vectors
 */
std::vector<VecX> mat2vec(const MatX &m, const bool row_wise = true);

/**
 * Matrix to list of vectors of size 3
 *
 * @param[in] m Matrix
 * @param[in] row_wise Row wise
 * @returns Vectors
 */
Vec3s mat2vec3(const MatX &m, const bool row_wise = true);

/**
 * Matrix to list of vectors of size 3
 *
 * @param[in] m Matrix
 * @param[in] row_wise Row wise
 * @returns Vectors
 */
Vec2s mat2vec2(const MatX &m, const bool row_wise = true);

/**
 * Vectors to matrix
 */
MatX vecs2mat(const Vec3s &vs);

/**
 * Vector to string
 *
 * @param[in] v Vector
 * @param[in] brackets Brakcets around vector string
 * @param[in] max_digits Max digits
 * @returns Vector as a string
 */
std::string vec2str(const VecX &v,
                    const bool brackets = true,
                    const bool max_digits = false);

/**
 * Vector to string
 *
 * @param[in] v Vector
 * @param[in] brackets Brakcets around vector string
 * @param[in] max_digits Max digits
 * @returns Vector as a string
 */
template <typename T>
std::string vec2str(const std::vector<T> &v,
                    const bool brackets = true,
                    const bool max_digits = false) {
  std::ostringstream ss;

  if (max_digits) {
    typedef std::numeric_limits<double> numeric_limits;
    ss << std::setprecision(numeric_limits::max_digits10);
  }

  if (brackets) {
    ss << "[";
  }

  for (int i = 0; i < v.size(); i++) {
    ss << v.at(i);
    if ((i + 1) != v.size()) {
      ss << ", ";
    }
  }

  if (brackets) {
    ss << "]";
  }

  return ss.str();
}

/**
 * Array to string
 *
 * @param[in] arr Array
 * @param[in] len Length of array
 * @param[in] brackets Brakcets around vector string
 * @returns Array as a string
 */
std::string arr2str(const double *arr, const size_t len, bool brackets = true);

/**
 * Matrix to string
 *
 * @param[in] m Matrix
 * @param[in] indent Indent string
 * @returns Array as a string
 */
std::string mat2str(const MatX &m,
                    const std::string &indent = "  ",
                    const bool max_digits = false);

/**
 * Normalize vector x
 */
Vec2 normalize(const Vec2 &x);

/**
 * Normalize vector `v`.
 */
Vec3 normalize(const Vec3 &v);

/**
 * Condition number of `A`.
 */
double cond(const MatX &A);

/**
 * Zeros-matrix
 *
 * @param rows Number of rows
 * @param cols Number of cols
 * @returns Zeros matrix
 */
MatX zeros(const int rows, const int cols);

/**
 * Zeros square matrix
 *
 * @param size Square size of matrix
 * @returns Zeros matrix
 */
MatX zeros(const int size);

/**
 * Identity-matrix
 *
 * @param rows Number of rows
 * @param cols Number of cols
 * @returns Identity matrix
 */
MatX eye(const int rows, const int cols);

/**
 * Identity square matrix
 *
 * @param size Square size of matrix
 * @returns Identity square matrix
 */
MatX eye(const int size);

/**
 * Ones-matrix
 *
 * @param rows Number of rows
 * @param cols Number of cols
 * @returns Ones square matrix
 */
MatX ones(const int rows, const int cols);

/**
 * Ones square matrix
 *
 * @param size Square size of matrix
 * @returns Ones square matrix
 */
MatX ones(const int size);

/**
 * Horizontally stack matrices A and B
 *
 * @param A Matrix A
 * @param B Matrix B
 * @returns Stacked matrix
 */
MatX hstack(const MatX &A, const MatX &B);

/**
 * Vertically stack matrices A and B
 *
 * @param A Matrix A
 * @param B Matrix B
 * @returns Stacked matrix
 */
MatX vstack(const MatX &A, const MatX &B);

/**
 * Diagonally stack matrices A and B
 *
 * @param A Matrix A
 * @param B Matrix B
 * @returns Stacked matrix
 */
MatX dstack(const MatX &A, const MatX &B);

/**
 * Skew symmetric-matrix
 *
 * @param w Input vector
 * @returns Skew symmetric matrix
 */
Mat3 skew(const Vec3 &w);

/**
 * Skew symmetric-matrix squared
 *
 * @param w Input vector
 * @returns Skew symmetric matrix squared
 */
Mat3 skewsq(const Vec3 &w);

/**
 * Enforce Positive Semi-Definite
 *
 * @param A Input matrix
 * @returns Positive semi-definite matrix
 */
MatX enforce_psd(const MatX &A);

/**
 * Null-space of A
 *
 * @param A Input matrix
 * @returns Null space of A
 */
MatX nullspace(const MatX &A);

/**
 * Check if two matrices `A` and `B` are equal.
 */
bool equals(const MatX &A, const MatX &B);

/**
 * Load std::vector of doubles to an Eigen::Matrix
 *
 * @param[in] x Matrix values
 * @param[in] rows Number of matrix rows
 * @param[in] cols Number of matrix colums
 * @param[out] y Output matrix
 */
void load_matrix(const std::vector<double> &x,
                 const int rows,
                 const int cols,
                 MatX &y);

/**
 * Load an Eigen::Matrix into a std::vector of doubles
 *
 * @param[in] A Matrix
 * @param[out] x Output vector of matrix values
 */
void load_matrix(const MatX A, std::vector<double> &x);

/** Pseudo Inverse via SVD **/
MatX pinv(const MatX &A, const double tol = 1e-4);

/** Rank of matrix A **/
long int rank(const MatX &A);

/** Check if matrix A is full rank */
bool full_rank(const MatX &A);

/**
 * Perform Schur's Complement
 */
int schurs_complement(MatX &H,
                      VecX &b,
                      const size_t m,
                      const size_t r,
                      const bool precond = false);

/******************************************************************************
 *                                 GEOMETRY
 *****************************************************************************/

/**
 * Sinc function.
 */
double sinc(const double x);

/**
 * Degrees to radians
 *
 * @param[in] d Degree to be converted
 * @return Degree in radians
 */
double deg2rad(const double d);

/**
 * Degrees to radians
 *
 * @param[in] d Degree to be converted
 * @return Degree in radians
 */
Vec3 deg2rad(const Vec3 d);

/**
 * Radians to degree
 *
 * @param[in] r Radian to be converted
 * @return Radian in degrees
 */
double rad2deg(const double r);

/**
 * Radians to degree
 *
 * @param[in] r Radian to be converted
 * @return Radian in degrees
 */
Vec3 rad2deg(const Vec3 &r);

/**
 * Wrap angle in degrees to 180
 *
 * @param[in] d Degrees
 * @return Angle wraped to 180
 */
double wrap180(const double d);

/**
 * Wrap angle in degrees to 360
 *
 * @param[in] d Degrees
 * @return Angle wraped to 360
 */
double wrap360(const double d);

/**
 * Wrap angle in radians to PI
 *
 * @param[in] r Radians
 * @return Angle wraped to PI
 */
double wrap_pi(const double r);

/**
 * Wrap angle in radians to 2 PI
 *
 * @param[in] r Radians
 * @return Angle wraped to 2 PI
 */
double wrap_2pi(const double r);

/**
 * Create a circle point of radius `r` at angle `theta` radians.
 */
Vec2 circle(const double r, const double theta);

/**
 * Create the sphere point with sphere radius `rho` at longitude `theta`
 * [radians] and latitude `phi` [radians].
 */
Vec3 sphere(const double rho, const double theta, const double phi);

/**
 * Create look at matrix.
 */
Mat4 lookat(const Vec3 &cam_pos,
            const Vec3 &target,
            const Vec3 &up_axis = Vec3{0.0, -1.0, 0.0});

/**
 * Cross-Track error based on waypoint line between p1, p2, and robot position
 *
 * @param[in] p1 Waypoint 1
 * @param[in] p2 Waypoint 2
 * @param[in] pos Robot position
 * @return Cross track error
 */
double cross_track_error(const Vec2 &p1, const Vec2 &p2, const Vec2 &pos);

/**
 * Check if point `pos` is left or right of line formed by `p1` and `p2`
 *
 * @param[in] p1 Waypoint 1
 * @param[in] p2 Waypoint 2
 * @param[in] pos Robot position
 * @returns
 *    - 1: Point is left of waypoint line formed by `p1` and `p2`
 *    - 2: Point is right of waypoint line formed by `p1` and `p2`
 *    - 0: Point is colinear with waypoint line formed by `p1` and `p2`
 */
int point_left_right(const Vec2 &p1, const Vec2 &p2, const Vec2 &pos);

/**
 * Calculate closest point given waypoint line between `p1`, `p2` and robot
 * position
 *
 * @param[in] p1 Waypoint 1
 * @param[in] p2 Waypoint 2
 * @param[in] p3 Robot position
 * @param[out] closest Closest point
 * @returns
 *    Unit number denoting where the closest point is on waypoint line. For
 *    example, a return value of 0.5 denotes the closest point is half-way
 *    (50%) of the waypoint line, alternatively a negative number denotes the
 *    closest point is behind the first waypoint.
 */
double
closest_point(const Vec2 &p1, const Vec2 &p2, const Vec2 &p3, Vec2 &closest);

/**
 * Fit a circle from a list of points.
 */
void fit_circle(const Vec2s &points, double &cx, double &cy, double &radius);

/**
 * Find the intersect of two circles.
 */
Vec2s intersect_circles(const double cx0,
                        const double cy0,
                        const double r0,
                        const double cx1,
                        const double cy1,
                        const double r1);

#define EARTH_RADIUS_M 6378137.0

/**
 * Calculate new latitude and logitude coordinates with an offset in North and
 * East direction.
 *
 * IMPORTANT NOTE: This function is only an approximation. As such do not rely
 * on this function for precise latitude, longitude offsets.
 *
 * @param lat_ref Latitude of origin (decimal format)
 * @param lon_ref Longitude of origin (decimal format)
 * @param offset_N Offset in North direction (meters)
 * @param offset_E Offset in East direction (meters)
 * @param lat_new New latitude (decimal format)
 * @param lon_new New longitude (decimal format)
 */
void latlon_offset(double lat_ref,
                   double lon_ref,
                   double offset_N,
                   double offset_E,
                   double *lat_new,
                   double *lon_new);

/**
 * Calculate difference in distance in North and East from two GPS coordinates
 *
 * IMPORTANT NOTE: This function is only an approximation. As such do not rely
 * on this function for precise latitude, longitude diffs.
 *
 * @param lat_ref Latitude of origin (decimal format)
 * @param lon_ref Longitude of origin (decimal format)
 * @param lat Latitude of point of interest (decimal format)
 * @param lon Longitude of point of interest (decimal format)
 * @param dist_N Distance of point of interest in North axis [m]
 * @param dist_E Distance of point of interest in East axis [m]
 */
void latlon_diff(double lat_ref,
                 double lon_ref,
                 double lat,
                 double lon,
                 double *dist_N,
                 double *dist_E);

/**
 * Calculate Euclidean distance between two GPS coordintes
 *
 * IMPORTANT NOTE: This function is only an approximation. As such do not rely
 * on this function for precise latitude, longitude distance.
 *
 * @param lat_ref Latitude of origin (decimal format)
 * @param lon_ref Longitude of origin (decimal format)
 * @param lat Latitude of point of interest (decimal format)
 * @param lon Longitude of point of interest (decimal format)
 *
 * @returns Euclidean distance between two GPS coordinates [m]
 */
double latlon_dist(double lat_ref, double lon_ref, double lat, double lon);

/******************************************************************************
 *                                STATISTICS
 *****************************************************************************/

/**
 * Create random integer
 *
 * @param[in] ub Upper bound
 * @param[in] lb Lower bound
 * @return Random integer
 */
int randi(const int ub, const int lb);

/**
 * Create random double
 *
 * @param[in] ub Upper bound
 * @param[in] lb Lower bound
 * @return Random floating point
 */
double randf(const double ub, const double lb);

/**
 * Sum values in vector.
 *
 * @param[in] x Array of numbers
 * @return Sum of vector
 */
double sum(const std::vector<double> &x);

/** Calculate median given an array of numbers */
double median(const std::vector<double> &v);

/** Max */
double max(const std::vector<double> &x);

/** Mean */
double mean(const std::vector<double> &x);

/** Mean */
Vec2 mean(const Vec2s &x);

/** Mean */
Vec3 mean(const Vec3s &x);

/** Variance */
double var(const std::vector<double> &x);

/** Variance */
Vec2 var(const Vec2s &vecs);

/** Variance */
Vec3 var(const Vec3s &vecs);

/** Standard Deviation */
double stddev(const std::vector<double> &x);

/** Standard Deviation */
Vec2 stddev(const Vec2s &x);

/** Standard Deviation */
Vec3 stddev(const Vec3s &x);

/** Root Mean Squared Error.  */
double rmse(const std::vector<double> &x);

/** Root Mean Squared Error.  */
Vec2 rmse(const Vec2s &vecs);

/** Root Mean Squared Error.  */
Vec3 rmse(const Vec3s &vecs);

/**
 * Multivariate normal.
 */
Vec3 mvn(std::default_random_engine &engine,
         const Vec3 &mu = Vec3{0.0, 0.0, 0.0},
         const Vec3 &stdev = Vec3{1.0, 1.0, 1.0});

/**
 * Gassian normal.
 * http://c-faq.com/lib/gaussian.html
 */
double gauss_normal();

/*****************************************************************************
 *                               TRANSFORM
 *****************************************************************************/

/**
 * Form a 4x4 homogeneous transformation matrix from a
 * rotation matrix `C` and translation vector `r`.
 */
template <typename T>
Eigen::Matrix<T, 4, 4> tf(const Eigen::Matrix<T, 3, 3> &C,
                          const Eigen::Matrix<T, 3, 1> &r) {
  Eigen::Matrix<T, 4, 4> transform = Eigen::Matrix<T, 4, 4>::Identity();
  transform.block(0, 0, 3, 3) = C;
  transform.block(0, 3, 3, 1) = r;
  return transform;
}

/**
 * Form a 4x4 homogeneous transformation matrix from a pointer to double array
 * containing (translation + rotation) 7 elements: (x, y, z, qx, qy, qz, qw)
 */
Mat4 tf(const double *params);

/**
 * Form a 4x4 homogeneous transformation matrix from a pointer to double array
 * containing (translation + rotation) 7 elements: (x, y, z, qx, qy, qz, qw)
 */
Mat4 tf(const VecX &params);

/**
 * Form a 4x4 homogeneous transformation matrix from a
 * rotation matrix `C` and translation vector `r`.
 */
Mat4 tf(const Mat3 &C, const Vec3 &r);

/**
 * Form a 4x4 homogeneous transformation matrix from a
 * Hamiltonian quaternion `q` and translation vector `r`.
 */
Mat4 tf(const Quat &q, const Vec3 &r);

/**
 * Invert a 4x4 homogeneous transformation matrix
 */
Mat4 tf_inv(const Mat4 &T);

/**
 * Form a 7x1 pose vector
 */
VecX tf_vec(const Mat4 &T = eye(4));

/**
 * Extract rotation from transform
 */
Mat3 tf_rot(const Mat4 &tf);

/**
 * Extract rotation and convert to quaternion from transform
 */
Quat tf_quat(const Mat4 &tf);

/**
 * Extract rotation and convert to quaternion from transform
 */
Vec4 tf_quat_vec(const Mat4 &tf);

/**
 * Extract translation from transform
 */
Vec3 tf_trans(const Mat4 &tf);

/**
 * Perturb the rotation element in the tranform `T` by `step_size` at index
 * `i`. Where i = 0 for x-axis, i = 1 for y-axis, and i = 2 for z-axis.
 */
Mat4 tf_perturb_rot(const Mat4 &T, double step_size, const int i);

/**
 * Perturb the translation element in the tranform `T` by `step_size` at index
 * `i`. Where i = 0 for x-axis, i = 1 for y-axis, and i = 2 for z-axis.
 */
Mat4 tf_perturb_trans(const Mat4 &T, double step_size, const int i);

/**
 * Perturb the translation and rotation component.
 */
Mat4 tf_perturb(const Mat4 &T, const double dr, const double drot);

/**
 * Transform point `p` with transform `T`.
 */
Vec3 tf_point(const Mat4 &T, const Vec3 &p);

/**
 * Pose difference between pose0 and pose1.
 */
void pose_diff(const double pose0[7],
               const double pose1[7],
               double *dr,
               double *drot);

/**
 * Rotation matrix around x-axis (counter-clockwise, right-handed).
 * @returns Rotation matrix
 */
Mat3 rotx(const double theta);

/**
 * Rotation matrix around y-axis (counter-clockwise, right-handed).
 * @returns Rotation matrix
 */
Mat3 roty(const double theta);

/**
 * Rotation matrix around z-axis (counter-clockwise, right-handed).
 * @returns Rotation matrix
 */
Mat3 rotz(const double theta);

/**
 * Convert euler sequence 123 to rotation matrix R
 * This function assumes we are performing a body fixed intrinsic rotation.
 *
 * Source:
 *
 *     Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
 *     Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
 *     Princeton University Press, 1999. Print.
 *
 *     Page 86.
 *
 * @returns Rotation matrix
 */
Mat3 euler123(const Vec3 &euler);

/**
 * Convert euler sequence 321 to rotation matrix R
 * This function assumes we are performing a body fixed intrinsic rotation.
 *
 * Source:
 *
 *     Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
 *     Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
 *     Princeton University Press, 1999. Print.
 *
 *     Page 86.
 *
 * @returns Rotation matrix
 */
Mat3 euler321(const Vec3 &euler);

/**
 * Convert roll, pitch and yaw to quaternion.
 */
Quat euler2quat(const Vec3 &euler);

/**
 * Convert rotation vectors to rotation matrix using measured acceleration
 * `a_m` from an IMU and gravity vector `g`.
 */
Mat3 vecs2rot(const Vec3 &a_m, const Vec3 &g);

/**
 * Convert rotation vector `rvec` to rotation matrix.
 */
Mat3 rvec2rot(const Vec3 &rvec, const double eps = 1e-5);

/**
 * Convert quaternion to euler angles.
 */
Vec3 quat2euler(const Quat &q);

/**
 * Convert quaternion to rotation matrix.
 */
Mat3 quat2rot(const Quat &q);

/**
 * Convert small angle euler angle to quaternion.
 */
Quat quat_delta(const Vec3 &dalpha);

/**
 * Left quaternion product matrix
 */
Mat4 quat_left(const Quat &q);

/**
 * Right quaternion product matrix
 */
Mat4 quat_right(const Quat &q);

/**
 * Return left quaternion product matrix.
 */
Mat4 quat_lmul(const Quat &q);

/**
 * Return left quaternion product matrix (but only for x, y, z components).
 */
Mat3 quat_lmul_xyz(const Quat &q);

/**
 * Return right quaternion product matrix.
 */
Mat4 quat_rmul(const Quat &q);

/**
 * Return right quaternion product matrix (but only for x, y, z components).
 */
Mat3 quat_rmul_xyz(const Quat &q);

/**
 * Return only the x, y, z, components of a quaternion matrix.
 */
Mat3 quat_mat_xyz(const Mat4 &Q);

/**
 * Average quaternion
 */
Eigen::Quaterniond
quat_average(const std::vector<Eigen::Quaterniond> &quaternions);

/**
 * Add noise to rotation matrix `rot`, where noise `n` is in degrees.
 */
Mat3 add_noise(const Mat3 &rot, const double n);

/**
 * Add noise to position vector `pos`, where noise `n` is in meters.
 */
Vec3 add_noise(const Vec3 &pos, const double n);

/**
 * Add noise to transform `pose`, where `pos_n` is in meters and `rot_n` is in
 * degrees.
 */
Mat4 add_noise(const Mat4 &pose, const double pos_n, const double rot_n);

/**
 * Initialize attitude using IMU gyroscope `w_m` and accelerometer `a_m`
 * measurements. The calculated attitude outputted into to `C_WS`. Note: this
 * function does not calculate initial yaw angle in the world frame. Only the
 * roll, and pitch are inferred from IMU measurements.
 */
void imu_init_attitude(const Vec3s w_m,
                       const Vec3s a_m,
                       Mat3 &C_WS,
                       const size_t buffer_size = 50);

/*****************************************************************************
 *                                    TIME
 *****************************************************************************/

/**
 * Print timestamp.
 */
void timestamp_print(const timestamp_t &ts, const std::string &prefix = "");

/**
 * Convert seconds to timestamp.
 */
timestamp_t sec2ts(const double sec);

/**
 * Convert ts to second.
 */
double ts2sec(const timestamp_t &ts);

/**
 * Decompose timestamp to seconds and nano-seconds.
 */
void tsdecomp(const timestamp_t &ts, long int &sec, long int &nsec);

/**
 * Form timestamp from seconds and nano-seconds.
 */
timestamp_t tsform(const long int sec, const long int &nsec);

/**
 * Convert nano-second to second.
 */
double ns2sec(const int64_t ns);

/**
 * Start timer.
 */
struct timespec tic();

/**
 * Stop timer and return number of seconds.
 */
float toc(struct timespec *tic);

/**
 * Stop timer and return miliseconds elasped.
 */
float mtoc(struct timespec *tic);

/**
 * Get time now in milliseconds since epoch
 */
double time_now();

/**
 * Profiler
 */
struct profiler_t {
  std::map<std::string, timespec> timers;
  std::map<std::string, std::vector<double>> record;

  profiler_t() {}

  void start(const std::string &key) { timers[key] = tic(); }

  float stop(const std::string &key) {
    if (timers.count(key) == 0) {
      FATAL("Key [%s] does not exist in profiler!", key.c_str());
    }

    record[key].push_back(toc(&timers[key]));
    return record[key].back();
  }

  void print(const std::string &key, const bool show_last = true) {
    if (record.count(key) == 0) {
      FATAL("Key [%s] does not exist in profiler!", key.c_str());
    }

    if (show_last) {
      printf("[%s]: %.4fs\n", key.c_str(), record[key].back());
    } else {
      printf("Total [%s]: %.4fs\n", key.c_str(), sum(record[key]));
    }
  }
};

/******************************************************************************
 *                              INTERPOLATION
 *****************************************************************************/

/**
 * Linear interpolation between two points.
 *
 * @param[in] a First point
 * @param[in] b Second point
 * @param[in] t Unit number
 * @returns Linear interpolation
 */
template <typename T>
T lerp(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}

/**
 * Slerp
 */
Quat slerp(const Quat &q_start, const Quat &q_end, const double alpha);

/**
 * Interpolate between two poses `p0` and `p1` with parameter `alpha`.
 */
Mat4 interp_pose(const Mat4 &p0, const Mat4 &p1, const double alpha);

/**
 * Interpolate `poses` where each pose has a timestamp in `timestamps` and the
 * interpolation points in time are in `interp_ts`. The results are recorded
 * in `interp_poses`.
 * @returns 0 for success, -1 for failure
 */
void interp_poses(const timestamps_t &timestamps,
                  const Mat4s &poses,
                  const timestamps_t &interp_ts,
                  Mat4s &interped_poses,
                  const double threshold = 0.001);

/**
 * Get the closest pose in `poses` where each pose has a timestamp in
 * `timestamps` and the target points in time are in `target_ts`. The results
 * are recorded in `result`.
 * @returns 0 for success, -1 for failure
 */
void closest_poses(const timestamps_t &timestamps,
                   const Mat4s &poses,
                   const timestamps_t &interp_ts,
                   Mat4s &result);

/**
 * Let `t0` and `t1` be timestamps from two signals. If one of them is measured
 * at a higher rate, the goal is to interpolate the lower rate signal so that
 * it aligns with the higher rate one.
 *
 * This function will determine which timestamp deque will become the reference
 * signal and the other will become the target signal. Based on this the
 * interpolation points will be based on the reference signal.
 *
 * Additionally, this function ensures the interpolation timestamps are
 * achievable by:
 *
 * - interp start > target start
 * - interp end < target end
 *
 * **Note**: This function will not include timestamps from the target
 * (lower-rate) signal. The returned interpolation timestamps only returns
 * **interpolation points** to match the reference signal (higher-rate).
 *
 * @returns Interpolation timestamps from two timestamp deques `t0` and `t1`.
 */
std::deque<timestamp_t> lerp_timestamps(const std::deque<timestamp_t> &t0,
                                        const std::deque<timestamp_t> &t1);

/**
 * Given the interpolation timestamps `lerp_ts`, target timestamps
 * `target_ts` and target data `target_data`. This function will:
 *
 * 1: Interpolate the `target_data` at the interpolation points defined by
 *    `target_ts`.
 * 2: Disgard data that are not in the target timestamp
 */
void lerp_data(const std::deque<timestamp_t> &lerp_ts,
               std::deque<timestamp_t> &target_ts,
               std::deque<Vec3> &target_data,
               const bool keep_old = false);

/** Lerp pose */
Mat4 lerp_pose(const timestamp_t &t0,
               const Mat4 &pose0,
               const timestamp_t &t1,
               const Mat4 &pose1,
               const timestamp_t &t_lerp);

/**
 * Given two data signals with timestamps `ts0`, `vs0`, `ts1`, and `vs1`, this
 * function determines which data signal is at a lower rate and performs linear
 * interpolation inorder to synchronize against the higher rate data signal.
 *
 * The outcome of this function is that both data signals will have:
 *
 * - Same number of timestamps.
 * - Lower-rate data will be interpolated against the higher rate data.
 *
 * **Note**: This function will drop values from the start and end of both
 * signals inorder to synchronize them.
 */
void lerp_data(std::deque<timestamp_t> &ts0,
               std::deque<Vec3> &vs0,
               std::deque<timestamp_t> &ts1,
               std::deque<Vec3> &vs1);

/**
 * SIM IMU
 */
struct sim_imu_t {
  // IMU parameters
  double rate = 0.0;       // IMU rate [Hz]
  double tau_a = 3600.0;   // Reversion time constant for accel [s]
  double tau_g = 3600.0;   // Reversion time constant for gyro [s]
  double sigma_g_c = 0.0;  // Gyro noise density [rad/s/sqrt(Hz)]
  double sigma_a_c = 0.0;  // Accel noise density [m/s^s/sqrt(Hz)]
  double sigma_gw_c = 0.0; // Gyro drift noise density [rad/s^s/sqrt(Hz)]
  double sigma_aw_c = 0.0; // Accel drift noise density [m/s^2/sqrt(Hz)]
  double g = 9.81;         // Gravity vector [ms-2]

  // IMU flags and biases
  bool started = false;
  Vec3 b_g = zeros(3, 1);
  Vec3 b_a = zeros(3, 1);
  timestamp_t ts_prev = 0;
};

/**
 * Reset IMU
 */
void sim_imu_reset(sim_imu_t &imu);

/**
 * Simulate IMU measurement
 */
void sim_imu_measurement(sim_imu_t &imu,
                         std::default_random_engine &rndeng,
                         const timestamp_t &ts,
                         const Mat4 &T_WS_W,
                         const Vec3 &w_WS_W,
                         const Vec3 &a_WS_W,
                         Vec3 &a_WS_S,
                         Vec3 &w_WS_S);

/******************************************************************************
 *                                  OPENCV
 *****************************************************************************/

/**
 * Convert gray-scale image to rgb image
 * @param image
 * @returns RGB image
 */
cv::Mat gray2rgb(const cv::Mat &image);

/**
 * Convert rgb image to gray-scale image
 * @param image
 * @returns Gray-scale image
 */
cv::Mat rgb2gray(const cv::Mat &image);

/**
 * Convert cv::Mat type to string
 */
std::string cvtype2str(const int cvtype);

/**
 * Convert std::vector<cv::KeyPoint> to std::vector<cv::Point2f>
 */
std::vector<cv::Point2f> kps2pts(const std::vector<cv::KeyPoint> &kps);

/**
 * Print Keypoint
 */
void print_keypoint(const cv::KeyPoint &kp);

/**
 * Sort Keypoints
 */
void sort_keypoints(std::vector<cv::KeyPoint> &kps);

/**
 * Given a set of keypoints `kps` make sure they are atleast `min_dist` pixels
 * away from each other, if they are not remove them.
 */
std::vector<cv::KeyPoint> spread_keypoints(
    const cv::Mat &image,
    const std::vector<cv::KeyPoint> &kps,
    const int min_dist = 10,
    const std::vector<cv::KeyPoint> &kps_prev = std::vector<cv::KeyPoint>(),
    const bool debug = false);

/**
 * Returns number of inliers, outliers and total from inlier vector.
 */
void inlier_stats(const std::vector<uchar> inliers,
                  size_t &num_inliers,
                  size_t &num_outliers,
                  size_t &num_total,
                  float &inlier_ratio);

/**
 * Filter Outliers
 */
void filter_outliers(std::vector<cv::KeyPoint> &kps_i,
                     std::vector<cv::KeyPoint> &kps_j,
                     const std::vector<uchar> &inliers);

/**
 * Track keypoints `pts_i` from image `img_i` to image `img_j` using optical
 * flow. Returns a tuple of `(pts_i, pts_j, inliers)` points in image i, j and a
 * vector of inliers.
 */
void optflow_track(const cv::Mat &img_i,
                   const cv::Mat &img_j,
                   const std::vector<cv::KeyPoint> &kps_i,
                   std::vector<cv::KeyPoint> &kps_j,
                   std::vector<uchar> &inliers,
                   const int patch_size = 50,
                   const int max_iter = 50,
                   const int max_level = 3,
                   const double epsilon = 0.001,
                   const bool debug = false);

// /**
//  * Perform RANSAC on keypoints `kps0` and `kps1` to reject outliers.
//  */
// void ransac(const std::vector<cv::KeyPoint> &kps0,
//             const std::vector<cv::KeyPoint> &kps1,
//             const undistort_func_t cam0_undist,
//             const undistort_func_t cam1_undist,
//             const double cam0_params[8],
//             const double cam1_params[8],
//             std::vector<uchar> &inliers,
//             const double reproj_threshold = 0.5,
//             const double confidence = 0.99);
//
// /**
//  * Perform RANSAC on keypoints `kps0` and `kps1` to reject outliers.
//  */
// void ransac(const std::vector<cv::KeyPoint> &kps0,
//             const std::vector<cv::KeyPoint> &kps1,
//             const undistort_func_t undist_func,
//             const double cam_params[8],
//             std::vector<uchar> &inliers,
//             const double reproj_threshold = 0.75,
//             const double confidence = 0.99);
//
// /**
//  * Check parallax
//  */
// void check_parallax(const camera_params_t &cam0_params,
//                     const camera_params_t &cam1_params,
//                     const extrinsic_t &cam0_ext,
//                     const extrinsic_t &cam1_ext,
//                     const std::vector<cv::KeyPoint> &kps0,
//                     const std::vector<cv::KeyPoint> &kps1,
//                     const double parallax_threshold,
//                     std::vector<uchar> &inliers);
//
// /**
//  * Filter features by triangulating them via a stereo-pair and see if the
//  * reprojection error is reasonable.
//  */
// void reproj_filter(const project_func_t cam_i_proj_func,
//                    const int cam_i_res[2],
//                    const double *cam_i_params,
//                    const double *cam_j_params,
//                    const double T_C0Ci[4 * 4],
//                    const double T_C0Cj[4 * 4],
//                    const std::vector<cv::KeyPoint> &kps_i,
//                    const std::vector<cv::KeyPoint> &kps_j,
//                    std::vector<cv::Point3d> &points,
//                    std::vector<bool> &inliers,
//                    const double reproj_threshold = 0.5);

} //  namespace cartesian
