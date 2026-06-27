#include <gtest/gtest.h>

#include "FeatureGrid.hpp"

namespace cartesian {

TEST(FeatureGrid, test) {
  const int image_width = 640;
  const int image_height = 480;
  const int grid_rows = 3;
  const int grid_cols = 4;
  FeatureGrid grid{image_width, image_height, grid_rows, grid_cols};

  const float dx = (static_cast<double>(image_width) / grid_cols);
  const float dy = (static_cast<double>(image_height) / grid_rows);
  for (int i = 0; i < grid_rows; i++) {
    for (int j = 0; j < grid_cols; j++) {
      const int pixel_x = dx * j + dx / 2;
      const int pixel_y = dy * i + dy / 2;
      grid.add(pixel_x, pixel_y);
    }
  }

  grid.debug(true);
}

} // namespace cartesian
