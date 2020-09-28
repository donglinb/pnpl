#include "pnpl.h"
#include "PoseComputer/AdapterPointLine.h"
#include "RANSAC/AbsolutePoseSacProblem.hpp"
#include "RANSAC/Ransac.hpp"

#include <memory>

namespace pnpl
{

using pose_lib::AdapterPointLine;
using opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem;
using opengv::sac::Ransac;

void PnPL(const std::vector<Eigen::Vector3d>& points_2d, const std::vector<Eigen::Vector3d>& points_3d, 
          const std::vector<Eigen::Vector3d>& lines_2d_coeff, const std::vector<Eigen::Vector3d>& lines_3d_midpoint, 
          const std::vector<Eigen::Vector3d>& lines_3d_direction, CameraPose& pose, 
          std::vector<double>* weights, double threshold, double focal)
{
    bool useWeights = false;
    if(weights)
        useWeights = true;
        
    // create adapter
    AdapterPointLine adapter(useWeights);
    int idx_adapter = 0;
    if(useWeights)
    {
        for(int i=0;i<points_2d.size();i++)
        {
            adapter.addPointPair(points_2d[i], points_3d[i], (*weights)[idx_adapter++]);
        }
        for(int i=0;i<lines_2d_coeff.size();i++)
        {
            adapter.addLinePair(lines_2d_coeff[i], lines_3d_midpoint[i], lines_3d_direction[i], (*weights)[idx_adapter++]);
        }
    }
    else
    {
        for(int i=0;i<points_2d.size();i++)
        {
            adapter.addPointPair(points_2d[i], points_3d[i]);
        }
        for(int i=0;i<lines_2d_coeff.size();i++)
        {
            adapter.addLinePair(lines_2d_coeff[i], lines_3d_midpoint[i], lines_3d_direction[i]);
        }
    }
    
    // create sac_problem
    std::shared_ptr<AbsolutePoseSacProblem> sac_problem(new AbsolutePoseSacProblem(adapter, useWeights));
    
    // create ransac
    Ransac<AbsolutePoseSacProblem> ransac;
    ransac.sac_model_ = sac_problem;
    ransac.threshold_ = AdapterPointLine::PointThresholdFromPixel(threshold, focal);
    ransac.max_iterations_ = 50;
    
    // compute model
    ransac.computeModel(2);
    
    pose = ransac.model_coefficients_;
}

static Eigen::Vector3d UnProject2NormalPlane(const Eigen::Vector2d& x_p, const Eigen::Matrix3d& K)
{
    Eigen::Vector3d x;
    x[0] = (x_p[0] - K(0,2)) / K(0,0);
    x[1] = (x_p[1] - K(1,2)) / K(1,1);
    x[2] = 1;
    
    return x;
}
static Eigen::Vector3d UnProject2UnitBearing(const Eigen::Vector2d& x_p, const Eigen::Matrix3d& K)
{
    Eigen::Vector3d x = UnProject2NormalPlane(x_p,K);
    return x/x.norm();
}

void PnPL(const std::vector<Eigen::Vector3d>& pts3d, const std::vector<Eigen::Vector2d>& pts2d, 
          const std::vector<Eigen::Vector6d>& lns3d, const std::vector<Eigen::Vector4d>& lns2d, 
          const Eigen::Matrix3d& K, Eigen::Matrix3d& R, Eigen::Vector3d& t, 
          std::vector<double>* weights, double threshold)
{
    bool useWeights = false;
    if(weights)
        useWeights = true;
        
    // create adapter
    AdapterPointLine adapter(useWeights);
    int idx_adapter = 0;
    if(useWeights)
    {
        for(int i=0;i<pts3d.size();i++)
        {
            adapter.addPointPair(UnProject2UnitBearing(pts2d[i], K), pts3d[i], (*weights)[idx_adapter++]);
        }
        for(int i=0;i<lns3d.size();i++)
        {
            Eigen::Vector3d spl = UnProject2NormalPlane(lns2d[i].head(2), K);
            Eigen::Vector3d epl = UnProject2NormalPlane(lns2d[i].tail(2), K);
            Eigen::Vector3d le = spl.cross(epl);
            Eigen::Vector3d X = 0.5*(lns3d[i].head(3) + lns3d[i].tail(3));
            Eigen::Vector3d V = lns3d[i].tail(3) - lns3d[i].head(3);
            adapter.addLinePair(le, X, V, (*weights)[idx_adapter++]);
        }
    }
    else
    {
        for(int i=0;i<pts3d.size();i++)
        {
            adapter.addPointPair(UnProject2UnitBearing(pts2d[i], K), pts3d[i]);
        }
        for(int i=0;i<lns3d.size();i++)
        {
            Eigen::Vector3d spl = UnProject2NormalPlane(lns2d[i].head(2), K);
            Eigen::Vector3d epl = UnProject2NormalPlane(lns2d[i].tail(2), K);
            Eigen::Vector3d le = spl.cross(epl);
            Eigen::Vector3d X = 0.5*(lns3d[i].head(3) + lns3d[i].tail(3));
            Eigen::Vector3d V = lns3d[i].tail(3) - lns3d[i].head(3);
            adapter.addLinePair(le, X, V);
        }
    }
    
    // create sac_problem
    std::shared_ptr<AbsolutePoseSacProblem> sac_problem(new AbsolutePoseSacProblem(adapter, useWeights));
    
    // create ransac
    Ransac<AbsolutePoseSacProblem> ransac;
    ransac.sac_model_ = sac_problem;
    ransac.threshold_ = AdapterPointLine::PointThresholdFromPixel(threshold, K(0,0));
    ransac.max_iterations_ = 50;
    
    // compute model
    ransac.computeModel(2);
    
    R = ransac.model_coefficients_.R;
    t = ransac.model_coefficients_.t;
}

}  // namespace pnpl
