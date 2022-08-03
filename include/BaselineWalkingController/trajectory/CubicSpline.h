#pragma once

#include <Eigen/Core>

#include <mc_rtc/logging.h>

#include <BaselineWalkingController/trajectory/Func.h>

namespace BWC
{
/** \brief Type of boundary constraint. */
enum class BoundaryConstraintType
{
  Velocity,
  Acceleration
};

/** \brief Boundary constraint.
    \tparam T value type
*/
template<class T>
struct BoundaryConstraint
{
  //! Type of boundary constraint
  BoundaryConstraintType type;

  //! Value of boundary constraint
  T value;

  /** \brief Constructor.
      \param _type type of boundary constraint
      \param _value value of boundary constraint
  */
  BoundaryConstraint(const BoundaryConstraintType & _type, const T & _value) : type(_type), value(_value) {}
};

/** \brief Cubic spline.
    \tparam T value type
*/
template<class T>
class CubicSpline : public PiecewiseFunc<T>
{
public:
  /** \brief Constructor.
      \param dim dimension of value
      \param points way points
      \param bcStart boundary constraint of start point
      \param bcEnd boundary constraint of end point
  */
  CubicSpline(int dim,
              const std::map<double, T> & points,
              const BoundaryConstraint<T> & bcStart,
              const BoundaryConstraint<T> & bcEnd)
  : dim_(dim), points_(points), bcStart_(bcStart), bcEnd_(bcEnd)
  {
  }

  /** \brief Access points. */
  const std::map<double, T> & points() const noexcept
  {
    return points_;
  }

  /** \brief Clear points. */
  void clearPoints()
  {
    points_.clear();
  }

  /** \brief Add point.
      \param point way point
  */
  void appendPoint(const std::pair<double, T> & point)
  {
    points_.insert(point);
  }

  /** \brief Calculate coefficients.

      See https://academiccommons.columbia.edu/doi/10.7916/D82Z1DMQ for the algorithm
  */
  void calcCoeff()
  {
    this->funcs_.clear();

    size_t n = points_.size();
    if(n < 2)
    {
      mc_rtc::log::error_and_throw("[CubicSpline] Invalid points size: {}.", n);
    }

    std::array<Eigen::MatrixXd, 4> coeffMatAll;
    for(auto & coeffMat : coeffMatAll)
    {
      coeffMat.resize(dim_, n);
    }

    // Calculate coefficients for each dimension
    for(int dimIdx = 0; dimIdx < dim_; dimIdx++)
    {
      // tridiagonalSystem represents the coefficients (A and b) of linear
      // system A x = b where A is tridiagonal matrix. In each row, first three
      // elements are the tridiagonal elements of A and the last element is the
      // element of b.
      Eigen::MatrixXd tridiagonalSystem(n, 4);
      Eigen::VectorXd y(n);
      Eigen::VectorXd yd(n);
      Eigen::VectorXd deltaT(n - 1);
      Eigen::VectorXd deltaY(n - 1);
      Eigen::VectorXd h(n - 1);
      Eigen::VectorXd r(n - 1);

      // Set y, deltaT, deltaY, h, r
      {
        auto pointIt = points_.begin();
        for(size_t k = 0; k < n; k++, pointIt++)
        {
          y(k) = pointIt->second(dimIdx);
          if(k != n - 1)
          {
            deltaT(k) = std::next(pointIt)->first - pointIt->first;
            deltaY(k) = std::next(pointIt)->second(dimIdx) - pointIt->second(dimIdx);
            h(k) = deltaT(k);
            r(k) = deltaY(k) / deltaT(k);
          }
        }
      }

      // Set tridiagonalSystem
      for(size_t k = 0; k < n - 2; k++)
      {
        // First and last rows are set later
        tridiagonalSystem.row(k + 1) << h(k + 1), 2 * (h(k) + h(k + 1)), h(k), 3 * (r(k) * h(k + 1) + r(k + 1) * h(k));
      }

      // Set tridiagonalSystem of boundary
      if(bcStart_.type == BoundaryConstraintType::Velocity)
      {
        tridiagonalSystem.row(0) << 0, 1, 0, bcStart_.value(dimIdx);
      }
      else if(bcStart_.type == BoundaryConstraintType::Acceleration)
      {
        tridiagonalSystem.row(0) << 0, 1, 0.5,
            (3 * deltaY(0)) / (2 * deltaT(0)) - bcStart_.value(dimIdx) / (4 * deltaT(0));
      }
      else
      {
        mc_rtc::log::error_and_throw("[CubicSpline] Unsupported type of boundary constraint: {}", bcStart_.type);
      }
      if(bcEnd_.type == BoundaryConstraintType::Velocity)
      {
        tridiagonalSystem.row(n - 1) << 0, 1, 0, bcEnd_.value(dimIdx);
      }
      else if(bcEnd_.type == BoundaryConstraintType::Acceleration)
      {
        tridiagonalSystem.row(n - 1) << 2, 4, 0,
            (6 * deltaY(n - 2)) / deltaT(n - 2) + bcEnd_.value(dimIdx) * deltaT(n - 2);
      }
      else
      {
        mc_rtc::log::error_and_throw("[CubicSpline] Unsupported type of boundary constraint: {}", bcEnd_.type);
      }

      // Calculate yd by solving tridiagonalSystem
      // See https://en.wikipedia.org/wiki/Tridiagonal_matrix_algorithm#Method
      for(size_t i = 1; i < n; i++)
      {
        double w = tridiagonalSystem(i, 0) / tridiagonalSystem(i - 1, 1);
        tridiagonalSystem(i, 1) -= w * tridiagonalSystem(i - 1, 2);
        tridiagonalSystem(i, 3) -= w * tridiagonalSystem(i - 1, 3);
      }
      yd(n - 1) = tridiagonalSystem(n - 1, 3) / tridiagonalSystem(n - 1, 1);
      for(int i = static_cast<int>(n - 2); i >= 0; i--)
      {
        yd(i) = (tridiagonalSystem(i, 3) - tridiagonalSystem(i, 2) * yd(i + 1)) / tridiagonalSystem(i, 1);
      }

      // Set coeffMatAll
      for(size_t k = 0; k < n - 1; k++)
      {
        coeffMatAll[0](dimIdx, k) = y(k);
        coeffMatAll[1](dimIdx, k) = yd(k);
        coeffMatAll[2](dimIdx, k) = (3 * deltaY(k) / deltaT(k) - 2 * yd(k) - yd(k + 1)) / deltaT(k);
        coeffMatAll[3](dimIdx, k) = (-2 * deltaY(k) / deltaT(k) + yd(k) + yd(k + 1)) / (deltaT(k) * deltaT(k));
      }
    }

    // Set piecewise CubicPolynomial
    {
      auto pointIt = points_.begin();
      for(size_t k = 0; k < n - 1; k++, pointIt++)
      {
        double t0 = pointIt->first;
        std::array<T, 4> coeff;
        for(size_t i = 0; i < coeffMatAll.size(); i++)
        {
          coeff[i] = coeffMatAll[i].col(k);
        }
        std::shared_ptr<CubicPolynomial<T>> cubicFunc = std::make_shared<CubicPolynomial<T>>(coeff, t0);
        this->funcs_.emplace(std::next(pointIt)->first, cubicFunc);
      }
    }

    // Set tLowerLimit_
    this->tLowerLimit_ = points_.begin()->first;
  }

protected:
  //! Dimension of value
  int dim_;

  //! Way points
  std::map<double, T> points_;

  //! Boundary constraint of start point
  BoundaryConstraint<T> bcStart_;

  //! Boundary constraint of end point
  BoundaryConstraint<T> bcEnd_;
};
} // namespace BWC
