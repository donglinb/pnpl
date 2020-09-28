#include "WeightedSampler.h"
#include <algorithm>
#include <set>

namespace pose_lib
{

WeightedSampler::WeightedSampler(const std::vector<double>& weights)
{
    cummulative_probability.clear();
    double sum = 0.0;
    for(size_t i = 0; i<weights.size(); i++)
    {
        sum += weights[i];
        cummulative_probability.push_back(sum);
    }
    
    for(size_t i=0; i<cummulative_probability.size(); i++)
    {
        cummulative_probability[i] /= sum;
    }
    
    std::random_device rd;
    gen = std::mt19937(rd());
    dis = std::uniform_real_distribution<>(0.0,1.0);
}

std::vector<int> WeightedSampler::GetSamples(int N)
{
    std::set<int> indices;
    int maxCount = cummulative_probability.size();
    std::vector<double>::iterator it_begin = cummulative_probability.begin();
    while(indices.size()<N && indices.size()<maxCount)
    {
        double w = dis(gen);
        std::vector<double>::iterator it = std::lower_bound(cummulative_probability.begin(), cummulative_probability.end(), w);
        indices.insert(it-it_begin);
    }
    
    return std::vector<int>(indices.begin(),indices.end());
}

}  // namespace poselib
