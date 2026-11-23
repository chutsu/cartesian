#include <gtest/gtest.h>

#include "ceres/MarginalError.hpp"
#include "core/Core.hpp"

namespace cartesian {

/** Linear factor, e = x1 - x0 - d, or e = x0 - d if there is only one param */
struct LinearFactor : ResidualBlock {
  Vec3 d;

  LinearFactor(const std::vector<double *> &ptrs, const Vec3 &d_)
      : ResidualBlock{"LinearFactor",
                      ptrs,
                      std::vector<ParamBlock::Type>(ptrs.size(),
                                                    ParamBlock::POINT),
                      3},
        d{d_} {}

  bool eval(double const *const *params,
            double *res,
            double **jacs) const override {
    Eigen::Map<Vec3> e(res);
    const Eigen::Map<const Vec3> x_last(params[param_ptrs_.size() - 1]);
    e = x_last - d;
    if (param_ptrs_.size() == 2) {
      e -= Eigen::Map<const Vec3>(params[0]);
    }
    if (jacs) {
      const auto sign = [](const int i, const int n) {
        return (i == n - 1) ? 1.0 : -1.0;
      };
      for (size_t i = 0; i < param_ptrs_.size(); i++) {
        if (jacs[i]) {
          Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J(jacs[i]);
          J = sign(i, param_ptrs_.size()) * Mat3::Identity();
        }
      }
    }
    return true;
  }
};

TEST(MarginalError, linear_chain) {
  // min over x0 of 0.5 * (|x0 - a|^2 + |x1 - x0 - d|^2) = 0.25 * |x1 - a - d|^2
  // so the prior on x1 is e = (x1 - a - d) / sqrt(2), whatever the
  // linearization point is
  Vec3 x0{0.3, -0.2, 0.1};
  Vec3 x1{1.0, 2.0, 3.0};
  const Vec3 a{0.5, 0.5, 0.5};
  const Vec3 d{1.0, -1.0, 2.0};

  ResidualBlockPtrs factors;
  factors.push_back(std::make_shared<LinearFactor>(
      std::vector<double *>{x0.data()},
      a));
  factors.push_back(std::make_shared<LinearFactor>(
      std::vector<double *>{x0.data(), x1.data()},
      d));
  auto prior = MarginalError::create(factors, {x0.data()});
  ASSERT_NE(prior, nullptr);
  ASSERT_EQ(prior->size(), 3);
  ASSERT_EQ(prior->get_param_ptrs().size(), 1u);
  ASSERT_EQ(prior->get_param_ptrs()[0], x1.data());

  for (const Vec3 &x : {x1, Vec3{-2.0, 0.4, 5.0}}) {
    Vec3 e;
    Mat3 J;
    double *jac_ptrs[1] = {J.data()};
    const double *params[1] = {x.data()};
    ASSERT_TRUE(prior->eval(params, e.data(), jac_ptrs));
    ASSERT_NEAR(e.squaredNorm(), 0.5 * (x - a - d).squaredNorm(), 1e-9);
    // J is row major, it is symmetric here
    ASSERT_NEAR((J.transpose() * J - 0.5 * Mat3::Identity()).norm(), 0.0, 1e-9);
  }
}

/** Dense linear factor over points, e = sum_k A_k * x_k - c */
struct DenseFactor : ResidualBlock {
  std::vector<MatX> A;
  VecX c;

  DenseFactor(const std::vector<double *> &ptrs,
              const std::vector<MatX> &A_,
              const VecX &c_)
      : ResidualBlock{"DenseFactor",
                      ptrs,
                      std::vector<ParamBlock::Type>(ptrs.size(),
                                                    ParamBlock::POINT),
                      static_cast<int>(c_.size())},
        A{A_}, c{c_} {}

  bool eval(double const *const *params,
            double *res,
            double **jacs) const override {
    Eigen::Map<VecX> e(res, c.size());
    e = -c;
    for (size_t k = 0; k < A.size(); k++) {
      e += A[k] * Eigen::Map<const Vec3>(params[k]);
    }
    if (jacs) {
      for (size_t k = 0; k < A.size(); k++) {
        if (jacs[k]) {
          Eigen::Map<MatXRowMajor>(jacs[k], c.size(), 3) = A[k];
        }
      }
    }
    return true;
  }
};

TEST(MarginalError, dense_random_with_pivoting) {
  // Random factors over 6 points, marginalize 2 of them. The prior must be the
  // cost of the factors minimized over the marginalized points, for any value
  // of the points that remain. This needs a pivoted LDLT with real pivoting.
  srand(1);
  const int num_points = 6;
  std::vector<Vec3> x(num_points);
  std::vector<double *> ptrs;
  for (auto &p : x) {
    p = Vec3::Random();
    ptrs.push_back(p.data());
  }

  ResidualBlockPtrs factors;
  std::vector<std::shared_ptr<DenseFactor>> dense;
  for (int i = 0; i < 12; i++) {
    const int rows = 2 + i % 3;
    std::vector<double *> p;
    std::vector<MatX> A;
    for (int k = 0; k < num_points; k++) {
      if ((i + k) % 3 != 0) { // Not all factors see all the points
        p.push_back(ptrs[k]);
        A.push_back(100.0 * MatX::Random(rows, 3) * std::pow(10.0, k % 3));
      }
    }
    auto factor = std::make_shared<DenseFactor>(p, A, 100.0 * VecX::Random(rows));
    factors.push_back(factor);
    dense.push_back(factor);
  }
  const std::vector<double *> marg{ptrs[0], ptrs[1]};
  auto prior = MarginalError::create(factors, marg);
  ASSERT_NE(prior, nullptr);
  ASSERT_EQ(prior->size(), 12);

  // Brute force, the cost of the factors with the marginalized points solved
  auto cost_of = [&](const std::vector<Vec3> &xs) {
    double c = 0.0;
    for (const auto &factor : dense) {
      std::vector<const double *> params;
      for (auto ptr : factor->get_param_ptrs()) {
        for (int k = 0; k < num_points; k++) {
          if (ptr == ptrs[k]) {
            params.push_back(xs[k].data());
          }
        }
      }
      VecX e(factor->c.size());
      factor->eval(params.data(), e.data(), nullptr);
      c += e.squaredNorm();
    }
    return c;
  };
  auto marginal_cost = [&](std::vector<Vec3> xs) {
    // Minimize over x0 and x1, the cost is quadratic so 1 Newton step is exact
    VecX z(6);
    z << xs[0], xs[1];
    auto f = [&](const VecX &zz) {
      auto t = xs;
      t[0] = zz.head<3>();
      t[1] = zz.tail<3>();
      return cost_of(t);
    };
    MatX H(6, 6);
    VecX g(6);
    const double h = 1e-3;
    for (int i = 0; i < 6; i++) {
      VecX zp = z, zm = z;
      zp(i) += h;
      zm(i) -= h;
      g(i) = (f(zp) - f(zm)) / (2 * h);
      for (int j = 0; j < 6; j++) {
        VecX zpp = zp, zpm = zp, zmp = zm, zmm = zm;
        zpp(j) += h;
        zpm(j) -= h;
        zmp(j) += h;
        zmm(j) -= h;
        H(i, j) = (f(zpp) - f(zpm) - f(zmp) + f(zmm)) / (4 * h * h);
      }
    }
    const VecX z_opt = z - H.ldlt().solve(g);
    return f(z_opt);
  };

  // The prior at the linearization point, and at other values of the remain
  for (int trial = 0; trial < 3; trial++) {
    std::vector<Vec3> xs = x;
    if (trial > 0) {
      for (int k = 2; k < num_points; k++) {
        xs[k] += 0.1 * Vec3::Random();
      }
    }
    std::vector<const double *> params;
    for (auto ptr : prior->get_param_ptrs()) {
      for (int k = 0; k < num_points; k++) {
        if (ptr == ptrs[k]) {
          params.push_back(xs[k].data());
        }
      }
    }
    ASSERT_EQ(params.size(), 4u);
    VecX e(prior->size());
    ASSERT_TRUE(prior->eval(params.data(), e.data(), nullptr));
    const double expected = marginal_cost(xs);
    ASSERT_NEAR(e.squaredNorm(), expected, 1e-4 * std::max(1.0, expected));
  }
}

TEST(MarginalError, fixed_params) {
  // With x0 fixed, the marginalization of nothing leaves x1 with its factor
  Vec3 x0{0.3, -0.2, 0.1};
  Vec3 x1{1.0, 2.0, 3.0};
  const Vec3 d{1.0, -1.0, 2.0};

  ResidualBlockPtrs factors;
  factors.push_back(std::make_shared<LinearFactor>(
      std::vector<double *>{x0.data(), x1.data()},
      d));
  auto prior = MarginalError::create(factors, {}, {x0.data()});
  ASSERT_NE(prior, nullptr);
  ASSERT_EQ(prior->get_param_ptrs().size(), 1u);

  Vec3 e;
  const double *params[1] = {x1.data()};
  ASSERT_TRUE(prior->eval(params, e.data(), nullptr));
  ASSERT_NEAR(e.squaredNorm(), (x1 - x0 - d).squaredNorm(), 1e-9);
}

TEST(MarginalError, nothing_remains) {
  Vec3 x0{0.3, -0.2, 0.1};
  ResidualBlockPtrs factors;
  factors.push_back(std::make_shared<LinearFactor>(
      std::vector<double *>{x0.data()},
      Vec3::Zero()));
  ASSERT_EQ(MarginalError::create(factors, {x0.data()}), nullptr);
}

} // namespace cartesian
