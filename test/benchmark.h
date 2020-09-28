#pragma once

#include "PoseLib/poselib.h"

#include "problem_generator.h"
#include <Eigen/Dense>
#include <stdint.h>
#include <string>
#include <vector>

namespace pose_lib 
{

struct BenchmarkResult 
{
  std::string name_;
  ProblemOptions options_;
  int instances_ = 0;
  int solutions_ = 0;
  int valid_solutions_ = 0;
  int found_gt_pose_ = 0;
  int runtime_ns_ = 0;
};

// Wrappers for the Benchmarking code

struct SolverP3P 
{
  static inline int solve(const AbsolutePoseProblemInstance &instance, pose_lib::CameraPoseVector *solutions) 
  {
    return p3p(instance.x_point_, instance.X_point_, solutions);
  }
  typedef CalibPoseValidator validator;
  static std::string name() { return "p3p"; }
};

struct SolverP2P1LL 
{
  static inline int solve(const AbsolutePoseProblemInstance &instance, pose_lib::CameraPoseVector *solutions) 
  {
    return p2p1ll(instance.x_point_, instance.X_point_, instance.l_line_line_, instance.X_line_line_, instance.V_line_line_, solutions);
  }
  typedef CalibPoseValidator validator;
  static std::string name() { return "p2p1ll"; }
};

struct SolverP1P2LL 
{
  static inline int solve(const AbsolutePoseProblemInstance &instance, pose_lib::CameraPoseVector *solutions) 
  {
    return p1p2ll(instance.x_point_, instance.X_point_, instance.l_line_line_, instance.X_line_line_, instance.V_line_line_, solutions);
  }
  typedef CalibPoseValidator validator;
  static std::string name() { return "p1p2ll"; }
};

struct SolverP3LL 
{
  static inline int solve(const AbsolutePoseProblemInstance &instance, pose_lib::CameraPoseVector *solutions) 
  {
    return p3ll(instance.l_line_line_, instance.X_line_line_, instance.V_line_line_, solutions);
  }
  typedef CalibPoseValidator validator;
  static std::string name() { return "p3ll"; }
};

} // namespace pose_lib
