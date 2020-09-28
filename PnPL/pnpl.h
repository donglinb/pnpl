#pragma once
#ifndef PNPL_H_
#define PNPL_H_

#include <eigen3/Eigen/Core>
#include <vector>
#include "PoseLib/types.h"

namespace Eigen
{
    typedef Matrix<double,6,1> Vector6d;
}

namespace pnpl
{

using pose_lib::CameraPose;

void PnPL(const std::vector<Eigen::Vector3d>& points_2d, const std::vector<Eigen::Vector3d>& points_3d, 
          const std::vector<Eigen::Vector3d>& lines_2d_coeff, const std::vector<Eigen::Vector3d>& lines_3d_midpoint, 
          const std::vector<Eigen::Vector3d>& lines_3d_direction, CameraPose& pose, 
          std::vector<double>* weights = nullptr, double threshold = 1.0, double focal = 1.0);

void PnPL(const std::vector<Eigen::Vector3d>& pts3d, const std::vector<Eigen::Vector2d>& pts2d, 
          const std::vector<Eigen::Vector6d>& lns3d, const std::vector<Eigen::Vector4d>& lns2d, 
          const Eigen::Matrix3d& K, Eigen::Matrix3d& R, Eigen::Vector3d& t, 
          std::vector<double>* weights = nullptr, double threshold = 1.0);

}  // namespace pnpl

#endif  // PNPL_H_
