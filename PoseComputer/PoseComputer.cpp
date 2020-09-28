#include "PoseComputer.h"
#include "PoseLib/poselib.h"

namespace pose_lib
{

void PoseComputer::ComputePose(AdapterPointLine& adapter, const std::vector<int>& indices, std::vector<CameraPose>& poses)
{
    static algorithm_t algorithm_options[4] = {P3P, P2P1LL, P1P2LL, P3LL};
    
    if(adapter.getNumberCorrespondences()<3 || indices.size()<3)
    {
        throw std::runtime_error("[PoseComputer::ComputePose] Not Enough Pairs, 3 is required.");
    }
    
    // point line pairs
    std::vector<Eigen::Vector3d> points_2d;
    std::vector<Eigen::Vector3d> points_3d;
    std::vector<Eigen::Vector3d> lines_2d_coeff;
    std::vector<Eigen::Vector3d> lines_3d_midpoint;
    std::vector<Eigen::Vector3d> lines_3d_direction;
    
    algorithm_t algorithm;
    int nlines = 0;
    for(int i=0;i<3;i++)
    {
        int idx = indices[i];
        if(adapter.getPairType(idx) == AdapterPointLine::POINT)
        {
            points_2d.push_back(adapter.getPoint2D(idx));
            points_3d.push_back(adapter.getPoint3D(idx));
        }
        else
        {
            lines_2d_coeff.push_back(adapter.getLine2D(idx));
            lines_3d_midpoint.push_back(adapter.getLine3DMidPoint(idx));
            lines_3d_direction.push_back(adapter.getLine3DDirection(idx));
            nlines++;
        }
    }
    algorithm = algorithm_options[nlines];
    
    switch(algorithm)
    {
    case P3P:
        {
            p3p(points_2d, points_3d, &poses);
            break;
        }
    case P2P1LL:
        {
            p2p1ll(points_2d, points_3d, lines_2d_coeff, lines_3d_midpoint, lines_3d_direction, &poses);
            break;
        }
    case P1P2LL:
        {
            p1p2ll(points_2d, points_3d, lines_2d_coeff, lines_3d_midpoint, lines_3d_direction, &poses);
            break;
        }
    case P3LL:
        {
            p3ll(lines_2d_coeff, lines_3d_midpoint, lines_3d_direction, &poses);
            break;
        }
    default:
        {
            throw std::runtime_error("[PoseComputer::ComputePose] Unrecognized algorithm");
            break;
        }   
    }
}

}  // namespace pose_lib
