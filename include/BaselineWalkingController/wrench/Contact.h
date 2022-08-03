#pragma once

#include <unordered_map>

#include <mc_rtc/Configuration.h>
#include <SpaceVecAlg/SpaceVecAlg>

namespace BWC
{
/** \brief Friction pyramid. */
class FrictionPyramid
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Constructor.
      \param fricCoeff friction coefficient
      \param divideNum number of vertices on the base of the polygonal pyramid
   */
  FrictionPyramid(double fricCoeff, int divideNum = 4);

  /** \brief Calculate ridge vector list in global coordinates. */
  std::vector<Eigen::Vector3d> calcGlobalRidgeList(const Eigen::Matrix3d & rot) const;

  /** \brief Number of ridges. */
  inline int divideNum() const
  {
    return localRidgeList_.size();
  }

public:
  //! Local ridge list
  std::vector<Eigen::Vector3d> localRidgeList_;
};

/** \brief Contact constraint. */
class Contact
{
public:
  /** \brief Vertex with ridges. */
  struct VertexWithRidge
  {
    //! Vertex
    Eigen::Vector3d vertex;

    //! Global ridge list
    std::vector<Eigen::Vector3d> ridgeList;

    /** \brief Constructor.
        \param _vertex vertex
        \param _ridgeList global ridge list
     */
    VertexWithRidge(const Eigen::Vector3d & _vertex, const std::vector<Eigen::Vector3d> & _ridgeList)
    : vertex(_vertex), ridgeList(_ridgeList)
    {
    }
  };

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Constructor.
      \param name name of contact
      \param fricCoeff friction coefficient
      \param localVertexList vertices of surface in local coordinates
      \param pose pose of contact
   */
  Contact(const std::string & name,
          double fricCoeff,
          const std::vector<Eigen::Vector3d> & localVertexList,
          const sva::PTransformd & pose);

  /** \brief Calculate wrench.
      \param wrenchRatio wrench ratio of each ridge
      \param momentOrigin moment origin
      \returns contact wrench
   */
  sva::ForceVecd calcWrench(const Eigen::VectorXd & wrenchRatio,
                            const Eigen::Vector3d & momentOrigin = Eigen::Vector3d::Zero()) const;

  /** \brief Calculate vertex forces represented in local coordinates.
      \param wrenchRatio wrench ratio of each ridge
      \returns vertex forces represented in local coordinates
   */
  std::vector<Eigen::Vector3d> calcLocalVertexForceList(const Eigen::VectorXd & wrenchRatio) const;

public:
  //! Name of contact
  std::string name_;

  //! Grasp matrix
  Eigen::Matrix<double, 6, Eigen::Dynamic> graspMat_;

  //! Grasp matrix list of vertex forces represented in local coordinates
  std::vector<Eigen::Matrix<double, 3, Eigen::Dynamic>> localGraspMatList_;

  //! List of vertex with ridges
  std::vector<VertexWithRidge> vertexWithRidgeList_;
};
} // namespace BWC
