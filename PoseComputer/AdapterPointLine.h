#pragma once
#ifndef ADAPTER_POINT_LINE_H_
#define ADAPTER_POINT_LINE_H_

#include "AdapterBase.h"
#include "PoseLib/types.h"

#include <vector>
#include <eigen3/Eigen/Core>

namespace pose_lib
{

class AdapterPointLine: public AdapterBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    AdapterPointLine(bool useWeights = false);
    void Reset();
    bool isWeightsUsed() {return useWeights;}
    
    virtual int getNumberCorrespondences();
    void addPointPair(const Eigen::Vector3d& pt2d, const Eigen::Vector3d& pt3d, double w = 1.0);
    void addLinePair(const Eigen::Vector3d& l, const Eigen::Vector3d& X, const Eigen::Vector3d& V, double w = 1.0);
    
    correspondance_t getPairType(const int idx) {return PairTypes[idx];}
    const Eigen::Vector3d& getPoint2D(const int idx);
    const Eigen::Vector3d& getPoint3D(const int idx);
    const Eigen::Vector3d& getLine2D(const int idx);
    const Eigen::Vector3d& getLine3DMidPoint(const int idx);
    const Eigen::Vector3d& getLine3DDirection(const int idx);
    double getWeight(const int idx) {return weights[idx];}
    std::vector<double>* getWeights() {return &weights;}
    
    static double ReprojectionErrorPoint(const Eigen::Vector3d& pt2d, const Eigen::Vector3d& pt3d, const CameraPose& pose);
    static double ReprojectionErrorLine(const Eigen::Vector3d& l, const Eigen::Vector3d& X, const Eigen::Vector3d& V, const CameraPose& pose);
    double ReprojectionError(const int idx, const CameraPose& pose);
    static double PointThresholdFromPixel(double th_p, double focal);
private:
    // point line pairs
    std::vector<Eigen::Vector3d> points_2d;
    std::vector<Eigen::Vector3d> points_3d;
    std::vector<Eigen::Vector3d> lines_2d_coeff;
    std::vector<Eigen::Vector3d> lines_3d_midpoint;
    std::vector<Eigen::Vector3d> lines_3d_direction;
    // weights
    bool useWeights;
    std::vector<double> weights;
    // indices
    std::vector<correspondance_t> PairTypes;
    std::vector<int> indices;
    int n_points, n_lines;
    
};  // AdapterPointLine

}  // namespace pose_lib

#endif  // ADAPTER_POINT_LINE_H_
