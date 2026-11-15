#include "DetectorTarget.hpp"

namespace cartesian {

int DetectorTarget::meters2px(const double meters) {
  const double px = meters * PPM;
  if (std::abs(px - std::round(px)) > 1e-6) {
    FATAL("%f [m] is not a whole number of pixels at %f ppm!", meters, PPM);
  }
  return static_cast<int>(std::round(px));
}

cv::Mat DetectorTarget::render(const double margin) const {
  const int m = meters2px(margin);
  cv::Mat texture;
  cv::copyMakeBorder(render_board(),
                     texture,
                     m,
                     m,
                     m,
                     m,
                     cv::BORDER_CONSTANT,
                     cv::Scalar{255});
  return texture;
}

} // namespace cartesian
