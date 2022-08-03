#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

#include <BaselineWalkingController/trajectory/CubicHermiteSpline.h>

namespace BWC
{
/** \brief Calculate the value interpolating from start to end.
    \tparam T value type
    \param start start value
    \param end end value
    \param ratio interpolation ratio
*/
template<class T>
inline T interpolate(const T & start, const T & end, double ratio)
{
  return (1 - ratio) * start + ratio * end;
}

/** \brief Calculate the Quaternion interpolating from start to end.
    \param start start value
    \param end end value
    \param ratio interpolation ratio
*/
template<>
inline Eigen::Quaterniond interpolate(const Eigen::Quaterniond & start, const Eigen::Quaterniond & end, double ratio)
{
  return start.slerp(ratio, end);
}

/** \brief Calculate the 3D matrix interpolating from start to end.
    \param start start value
    \param end end value
    \param ratio interpolation ratio
*/
template<>
inline Eigen::Matrix3d interpolate(const Eigen::Matrix3d & start, const Eigen::Matrix3d & end, double ratio)
{
  return interpolate<Eigen::Quaterniond>(Eigen::Quaterniond(start), Eigen::Quaterniond(end), ratio).toRotationMatrix();
}

/** \brief Calculate the pose interpolating from start to end.
    \param start start value
    \param end end value
    \param ratio interpolation ratio
*/
template<>
inline sva::PTransformd interpolate(const sva::PTransformd & start, const sva::PTransformd & end, double ratio)
{
  return sva::interpolate(start, end, ratio);
}

/** \brief Calculate the derivative value interpolating from start to end.
    \tparam T value type
    \tparam U derivative type
    \param start start value
    \param end end value
    \param ratio interpolation ratio
    \param order derivative order
*/
template<class T, class U = T>
inline U interpolateDerivative(const T & start, const T & end, double ratio, int order = 1)
{
  mc_rtc::log::error_and_throw("[interpolateDerivative] Not implemented.");
  return U();
}

/** \brief Calculate the derivative of 3D vector interpolating from start to end.
    \param start start value
    \param end end value
    \param ratio interpolation ratio
    \param order derivative order
*/
template<>
inline Eigen::Vector3d interpolateDerivative(const Eigen::Vector3d & start,
                                             const Eigen::Vector3d & end,
                                             double, // ratio,
                                             int order)
{
  if(order == 1)
  {
    return end - start;
  }
  else
  {
    return Eigen::Vector3d::Zero();
  }
}

/** \brief Calculate the derivative of Quaternion interpolating from start to end.
    \param start start value
    \param end end value
    \param ratio interpolation ratio
    \param order derivative order
*/
template<>
inline Eigen::Vector3d interpolateDerivative(const Eigen::Quaterniond & start,
                                             const Eigen::Quaterniond & end,
                                             double, // ratio,
                                             int order)
{
  if(order == 1)
  {
    Eigen::AngleAxisd aa(start.inverse() * end);
    return aa.angle() * aa.axis();
  }
  else
  {
    return Eigen::Vector3d::Zero();
  }
}

/** \brief Calculate the derivative of 3D matrix interpolating from start to end.
    \param start start value
    \param end end value
    \param ratio interpolation ratio
    \param order derivative order
*/
template<>
inline Eigen::Vector3d interpolateDerivative(const Eigen::Matrix3d & start,
                                             const Eigen::Matrix3d & end,
                                             double, // ratio,
                                             int order)
{
  if(order == 1)
  {
    // \todo why inverse?
    Eigen::AngleAxisd aa(end.transpose() * start);
    return aa.angle() * aa.axis();
  }
  else
  {
    return Eigen::Vector3d::Zero();
  }
}

/** \brief Cubic interpolator.
    \tparam T value type
    \tparam U derivative type

    The velocity of each waypoint is assumed to be zero.
*/
template<class T, class U = T>
class CubicInterpolator
{
protected:
  //! 1D vector
  using Vector1d = Eigen::Matrix<double, 1, 1>;

public:
  /** \brief Constructor.
      \param points times and values to be interpolated
  */
  CubicInterpolator(const std::map<double, T> & points = {}) : points_(points)
  {
    func_ = std::make_shared<CubicHermiteSpline<Vector1d>>(1, std::map<double, std::pair<Vector1d, Vector1d>>{});

    if(points_.size() >= 2)
    {
      calcCoeff();
    }
  }

  /** \brief Clear points. */
  void clearPoints()
  {
    points_.clear();
  }

  /** \brief Add point.
      \param point time and value

      \note ::calcCoeff should be called before calling ::operator().
  */
  void appendPoint(const std::pair<double, T> & point)
  {
    points_.insert(point);
  }

  /** \brief Calculate coefficients. */
  void calcCoeff()
  {
    if(points_.size() < 2)
    {
      mc_rtc::log::error_and_throw<std::out_of_range>("[CubicInterpolator] Number of points should be 2 or more: {}.",
                                                      points_.size());
    }

    func_->clearPoints();

    auto it = points_.begin();
    for(size_t i = 0; i < points_.size(); i++)
    {
      func_->appendPoint(std::make_pair(
          it->first, std::make_pair((Vector1d() << static_cast<double>(i)).finished(), Vector1d::Zero())));
      it++;
    }

    func_->setDomainLowerLimit(startTime());
    func_->calcCoeff();
  }

  /** \brief Calculate interpolated value.
      \param t time
  */
  T operator()(double t) const
  {
    double ratio = getRatio(t);
    size_t idx = getIdx(ratio);
    if(!(idx + 1 <= points_.size() - 1))
    {
      mc_rtc::log::error_and_throw("[CubicInterpolator] Invalid idx {}. Should be {} <= idx <= {}", idx, 0,
                                   points_.size() - 2);
    }
    return interpolate<T>(std::next(points_.begin(), idx)->second, std::next(points_.begin(), idx + 1)->second,
                          std::min(std::max(ratio - static_cast<double>(idx), 0.0), 1.0));
  }

  /** \brief Calculate the derivative of interpolated value.
      \param t time
      \param order derivative order

      It is assumed that interpolateDerivative() returns zero if derivative order is greater than or equal to 2.
  */
  virtual U derivative(double t, int order = 1) const
  {
    double ratio = getRatio(t);
    size_t idx = getIdx(ratio);
    return func_->derivative(t, order)[0]
           * interpolateDerivative<T, U>(std::next(points_.begin(), idx)->second,
                                         std::next(points_.begin(), idx + 1)->second,
                                         std::min(std::max(ratio - static_cast<double>(idx), 0.0), 1.0), 1);
  }

  /** \brief Get ratio of interpolation points.
      \param t time
  */
  double getRatio(double t) const
  {
    return (*func_)(t)[0];
  }

  /** \brief Get start time. */
  double startTime() const
  {
    return points_.begin()->first;
  }

  /** \brief Get end time. */
  double endTime() const
  {
    return points_.rbegin()->first;
  }

  /** \brief Get points. */
  const std::map<double, T> & points() const
  {
    return points_;
  }

protected:
  /** \brief Get index of interpolation points.
      \param ratio Interpolation ratio
  */
  size_t getIdx(double ratio) const
  {
    int idx = static_cast<int>(std::floor(ratio));
    if(static_cast<size_t>(idx) == points_.size() - 1)
    {
      idx--;
    }
    if(idx < 0 || points_.size() - 2 < static_cast<size_t>(idx))
    {
      mc_rtc::log::error_and_throw<std::out_of_range>("[CubicInterpolator] Index {} is out of range [{}, {}].", idx, 0,
                                                      points_.size() - 2);
    }
    return static_cast<size_t>(idx);
  }

protected:
  //! Times and values to be interpolated
  std::map<double, T> points_;

  //! Function to calculate the ratio of interpolation points
  std::shared_ptr<CubicHermiteSpline<Vector1d>> func_;
};
} // namespace BWC
