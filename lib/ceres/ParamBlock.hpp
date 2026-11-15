#pragma once
#include "../core/Core.hpp"

namespace cartesian {

/** Parameter Block **/
struct ParamBlock {
  enum Type {
    POSE,
    EXTRINSIC,
    POINT,
    VELOCITY,
    BIAS,
    TIME_DELAY,
    INTRINSIC8,
    INTRINSIC9,
    SPEED_BIASES,
    FIDUCIAL,
  };

  /** Get Parameter Size */
  static int get_param_size(const ParamBlock::Type type);

  /** Get Local Size */
  static int get_local_size(const ParamBlock::Type type);

  /** Perturb parameter based on type */
  static void perturb(const ParamBlock::Type type,
                      const int i,
                      const double step,
                      double *ptr);
};

} // namespace cartesian
