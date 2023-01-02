#pragma once

#include <Eigen/Core>

#include <mc_rtc/logging.h>

#include <BaselineWalkingController/trajectory/Func.h>

namespace BWC
{
/** \brief Cubic Hermite spline.
    \tparam T function value type
*/
template<class T>
class CubicHermiteSpline : public PiecewiseFunc<T>
{
public:
  /** \brief Constructor.
      \param dim dimension of value
      \param points times, positions, and velocities in way points
  */
  CubicHermiteSpline(int dim, const std::map<double, std::pair<T, T>> & points) : dim_(dim), points_(points) {}

  /** \brief Access points. */
  const std::map<double, std::pair<T, T>> & points() const noexcept
  {
    return points_;
  }

  /** \brief Clear points. */
  void clearPoints()
  {
    points_.clear();
  }

  /** \brief Add point.
      \param point time, position, and velocity of way point
  */
  void appendPoint(const std::pair<double, std::pair<T, T>> & point)
  {
    points_.insert(point);
  }

  /** \brief Calculate coefficients.

      See https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Interpolation_on_an_arbitrary_interval for the algorithm
  */
  void calcCoeff()
  {
    this->funcs_.clear();

    // Set piecewise CubicPolynomial
    size_t n = points_.size();
    if(n < 2)
    {
      mc_rtc::log::error_and_throw("[CubicHermiteSpline] Invalid points size: {}.", n);
    }

    {
      auto pointIt = points_.begin();
      for(size_t k = 0; k < n - 1; k++, pointIt++)
      {
        const T & pk = pointIt->second.first;
        const T & pk1 = std::next(pointIt)->second.first;
        const T & vk = pointIt->second.second;
        const T & vk1 = std::next(pointIt)->second.second;
        double deltaT = std::next(pointIt)->first - pointIt->first;

        // clang-format off
        std::array<T, 4> coeff = {
          pk,
          deltaT * vk,
          -3 * pk - 2 * deltaT * vk + 3 * pk1 - deltaT * vk1,
          2 * pk + deltaT * vk - 2 * pk1 + deltaT * vk1};
        // clang-format on
        std::shared_ptr<CubicPolynomial<T>> cubicFunc = std::make_shared<CubicPolynomial<T>>(coeff);
        this->funcs_.emplace(std::next(pointIt)->first, cubicFunc);
      }
    }

    // Set tLowerLimit_
    this->tLowerLimit_ = points_.begin()->first;
  }

  /** \brief Evaluate function value.
      \param t arugment of function
  */
  virtual T operator()(double t) const override
  {
    this->checkArg(t);
    auto funcIt = this->funcs_.lower_bound(t);

    const auto & seg = argSegment(t);
    double scaledT = (t - seg.first) / (seg.second - seg.first);
    return (*(funcIt->second))(scaledT);
  }

  /** \brief Evaluate function derivative value.
      \param t arugment of function
      \param order derivative order
  */
  virtual T derivative(double t, int order = 1) const override
  {
    this->checkArg(t);
    auto funcIt = this->funcs_.lower_bound(t);

    const auto & seg = argSegment(t);
    double scaledT = (t - seg.first) / (seg.second - seg.first);
    return (funcIt->second)->derivative(scaledT, order) / std::pow(seg.second - seg.first, order);
  }

  /** \brief Overwrite the velocity with keeping the time and position for making cubic Hermite spline monotone.
      \param keepStartVel Whether to try to keep the start velocity which is stored in points_ (just an attempt, so
      actually changed) \param keepEndVel Whether to try to keep the end velocity which is stored in points_ (just an
      attempt, so actually changed)

      See https://en.wikipedia.org/wiki/Monotone_cubic_interpolation#Interpolant_selection for the algorithm
  */
  void calcMonotoneVelocity(bool keepStartVel = false, bool keepEndVel = false)
  {
    std::vector<T> delta;
    std::vector<T> alpha;
    std::vector<T> beta;

    // 1.
    for(auto it = points_.begin(); it != std::prev(points_.end()); it++)
    {
      delta.push_back((std::next(it)->second.first - it->second.first) / (std::next(it)->first - it->first));
    }

    // 2.
    {
      int i = 0;
      for(auto it = points_.begin(); it != points_.end(); it++)
      {
        if(it == points_.begin())
        {
          if(!keepStartVel)
          {
            it->second.second = delta[i];
          }
        }
        else if(it == std::prev(points_.end()))
        {
          if(!keepEndVel)
          {
            it->second.second = delta[i - 1];
          }
        }
        else
        {
          it->second.second = (delta[i - 1] + delta[i]) / 2;
          for(int j = 0; j < dim_; j++)
          {
            if(delta[i - 1][j] * delta[i][j] < 0)
            {
              it->second.second[j] = 0;
            }
          }
        }
        i++;
      }
    }

    // 3.
    {
      int i = 0;
      for(auto it = points_.begin(); it != std::prev(points_.end()); it++)
      {
        for(int j = 0; j < dim_; j++)
        {
          if(std::abs(delta[i][j]) < std::numeric_limits<double>::min())
          {
            it->second.second[j] = 0;
            std::next(it)->second.second[j] = 0;
          }
        }
        i++;
      }
    }

    // 4.
    {
      int i = 0;
      for(auto it = points_.begin(); it != std::prev(points_.end()); it++)
      {
        alpha.push_back(it->second.second.cwiseProduct(delta[i].cwiseInverse()));
        beta.push_back(std::next(it)->second.second.cwiseProduct(delta[i].cwiseInverse()));
        for(int j = 0; j < dim_; j++)
        {
          if(alpha[i][j] < 0)
          {
            it->second.second[j] = 0;
          }
          if(beta[i][j] < 0)
          {
            std::next(it)->second.second[j] = 0;
          }
        }
        i++;
      }
    }

    // 5.
    {
      int i = 0;
      for(auto it = points_.begin(); it != std::prev(points_.end()); it++)
      {
        for(int j = 0; j < dim_; j++)
        {
          if(std::abs(delta[i][j]) < std::numeric_limits<double>::min())
          {
            continue;
          }
          if(!(alpha[i][j] - std::pow(2 * alpha[i][j] + beta[i][j] - 3, 2) / (3 * (alpha[i][j] + beta[i][j] - 2)) > 0
               || alpha[i][j] + 2 * beta[i][j] - 3 <= 0 || 2 * alpha[i][j] + beta[i][j] - 3 <= 0))
          {
            double tau = 3 / std::sqrt(std::pow(alpha[i][j], 2) + std::pow(beta[i][j], 2));
            it->second.second[j] = tau * alpha[i][j] * delta[i][j];
            std::next(it)->second.second[j] = tau * beta[i][j] * delta[i][j];
          }
        }
        i++;
      }
    }
  }

protected:
  /** \brief Get the piecewise segment.
      \param t arugment of function
  */
  std::pair<double, double> argSegment(double t) const
  {
    auto funcIt = points_.lower_bound(t);
    if(funcIt == points_.begin())
    {
      funcIt++;
    }
    return {std::prev(funcIt)->first, funcIt->first};
  }

protected:
  //! Dimension of value
  int dim_;

  //! Times, positions, and velocities in way points
  std::map<double, std::pair<T, T>> points_;
};
} // namespace BWC
