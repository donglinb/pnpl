#include "AdapterPointLine.h"

namespace pose_lib
{

AdapterPointLine::AdapterPointLine(bool useWeights): useWeights(useWeights)
{
    Reset();
}

void AdapterPointLine::Reset()
{
    // point line pairs and weights
    points_2d.clear();
    points_3d.clear();
    lines_2d_coeff.clear();
    lines_3d_midpoint.clear();
    lines_3d_direction.clear();
    weights.clear();
    // indices
    PairTypes.clear();
    indices.clear();
    n_points = 0;
    n_lines = 0;
}

int AdapterPointLine::getNumberCorrespondences()
{
    return n_points + n_lines;
}

void AdapterPointLine::addPointPair(const Eigen::Vector3d& pt2d, const Eigen::Vector3d& pt3d, double w)
{
    points_2d.push_back(pt2d);
    points_3d.push_back(pt3d);
    
    if(useWeights)
    {
        weights.push_back(w);
    }
    
    PairTypes.push_back(POINT);
    indices.push_back(n_points);
    n_points++;
}

void AdapterPointLine::addLinePair(const Eigen::Vector3d& l, const Eigen::Vector3d& X, const Eigen::Vector3d& V, double w)
{
    lines_2d_coeff.push_back(l);
    lines_3d_midpoint.push_back(X);
    lines_3d_direction.push_back(V);
    
    if(useWeights)
    {
        weights.push_back(w);
    }
    
    PairTypes.push_back(LINE);
    indices.push_back(n_lines);
    n_lines++;
}

const Eigen::Vector3d& AdapterPointLine::getPoint2D(const int idx)
{
    assert(PairTypes[idx] == POINT);
    int real_idx = indices[idx];
    return points_2d[real_idx];
}

const Eigen::Vector3d& AdapterPointLine::getPoint3D(const int idx)
{
    assert(PairTypes[idx] == POINT);
    int real_idx = indices[idx];
    return points_3d[real_idx];
}

const Eigen::Vector3d& AdapterPointLine::getLine2D(const int idx)
{
    assert(PairTypes[idx] == LINE);
    int real_idx = indices[idx];
    return lines_2d_coeff[real_idx];
}
const Eigen::Vector3d& AdapterPointLine::getLine3DMidPoint(const int idx)
{
    assert(PairTypes[idx] == LINE);
    int real_idx = indices[idx];
    return lines_3d_midpoint[real_idx];
}

const Eigen::Vector3d& AdapterPointLine::getLine3DDirection(const int idx)
{
    assert(PairTypes[idx] == LINE);
    int real_idx = indices[idx];
    return lines_3d_direction[real_idx];
}

double AdapterPointLine::ReprojectionErrorPoint(const Eigen::Vector3d& pt2d, const Eigen::Vector3d& pt3d, const CameraPose& pose)
{
    Eigen::Vector3d Pc = pose.R * pt3d + pose.t;
    double product = Pc.dot(pt2d);
    double norm = Pc.norm() * pt2d.norm();
    double error = 1.0 - product / norm;
}

double AdapterPointLine::ReprojectionErrorLine(const Eigen::Vector3d& l, const Eigen::Vector3d& X, const Eigen::Vector3d& V, const CameraPose& pose)
{
    Eigen::Vector3d Xc = pose.R * X + pose.t;
    Eigen::Vector3d Vc = pose.R * V;
    double l_norm = l.norm();
    double err1 = l.dot(Xc) / (l_norm*Xc.norm());
    double err2 = l.dot(Vc) / (l_norm*Vc.norm());
    double error = std::fabs(err1) + std::fabs(err2);
    
    return error;
}

double AdapterPointLine::ReprojectionError(const int idx, const CameraPose& pose)
{
    int real_idx = indices[idx];
    double error = 0.0;
    if(PairTypes[idx] == POINT)
    {
        error = ReprojectionErrorPoint(points_2d[real_idx],points_3d[real_idx],pose);
    }
    else
    {
        error = ReprojectionErrorLine(lines_2d_coeff[real_idx],lines_3d_midpoint[real_idx],lines_3d_direction[real_idx],pose);
    }
    return error;
}

double AdapterPointLine::PointThresholdFromPixel(double th_p, double focal)
{
    double q = atan(th_p/focal);
    double th = 1.0 - cos(q);
    return th;
}
    
}  // namespace pose_lib
