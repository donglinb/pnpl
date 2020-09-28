#pragma once
#ifndef POSE_COMPUTER_H_
#define POSE_COMPUTER_H_

#include "AdapterPointLine.h"

namespace pose_lib
{

class PoseComputer
{
public:
    typedef enum Algorithm
    {
        P3P = 0,
        P2P1LL,
        P1P2LL,
        P3LL
    } algorithm_t;
    static void ComputePose(AdapterPointLine& adapter, const std::vector<int>& indices, std::vector<CameraPose>& poses);
    
};  // PoseComputer

}  // namespace poselib

#endif  // POSE_COMPUTER_H_
