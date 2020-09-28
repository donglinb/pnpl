#pragma once
#ifndef WEIGHTED_SAMPLER_H_
#define WEIGHTED_SAMPLER_H_

#include <random>
#include <vector>

namespace pose_lib
{

class WeightedSampler
{
public:
    WeightedSampler(const std::vector<double>& weights);
    std::vector<int> GetSamples(int N);
private:
    std::vector<double> cummulative_probability;
    std::mt19937 gen;
    std::uniform_real_distribution<> dis;
    
};  // WeightedSampler

}  // namespace poselib

#endif  // WEIGHTED_SAMPLER_H_
