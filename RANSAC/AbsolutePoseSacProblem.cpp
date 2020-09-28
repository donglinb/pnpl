/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/


#include "AbsolutePoseSacProblem.hpp"
#include "PoseComputer/PoseComputer.h"

bool
opengv::sac_problems::
    absolute_pose::AbsolutePoseSacProblem::computeModelCoefficients(
    const std::vector<int> &indices,
    model_t & outModel) const
{
  pose_lib::CameraPoseVector solutions;
  pose_lib::PoseComputer::ComputePose(_adapter, indices, solutions);
  
  //now compute reprojection error of fourth point, in order to find the right one
  double minScore = 1000000.0;
  int minIndex = -1;
  for(size_t i = 0; i < solutions.size(); i++)
  {
    //compute the score
    double score = _adapter.ReprojectionError(indices[3], solutions[i]);

    //check for best solution
    if( score < minScore )
    {
      minScore = score;
      minIndex = i;
    }
  }

  if(minIndex == -1)
    return false;
  outModel = solutions[minIndex];
  return true;
}

void
opengv::sac_problems::
    absolute_pose::AbsolutePoseSacProblem::getSelectedDistancesToModel(
    const model_t & model,
    const std::vector<int> & indices,
    std::vector<double> & scores) const
{
  //compute the reprojection error of all points
  for(size_t i = 0; i < indices.size(); i++)
  {
    scores.push_back(_adapter.ReprojectionError(indices[i], model));
  }
}

void
opengv::sac_problems::
    absolute_pose::AbsolutePoseSacProblem::optimizeModelCoefficients(
    const std::vector<int> & inliers,
    const model_t & model,
    model_t & optimized_model)
{
  fprintf(stderr,"AbsolutePoseSacProblem::optimizeModelCoefficients is not implemented.\n");
}

int
opengv::sac_problems::
    absolute_pose::AbsolutePoseSacProblem::getSampleSize() const
{
  int sampleSize = 4;
  return sampleSize;
}

void
opengv::sac_problems::
    absolute_pose::AbsolutePoseSacProblem::getSamples(
    int &iterations, std::vector<int> &samples)
{
  // We're assuming that indices_ have already been set in the constructor
  if (indices_->size() < (size_t)getSampleSize())
  {
    fprintf( stderr,
        "[sm::SampleConsensusModel::getSamples] Can not select %zu unique points out of %zu!\n",
        (size_t) getSampleSize(), indices_->size() );
    // one of these will make it stop :)
    samples.clear();
    iterations = std::numeric_limits<int>::max();
    return;
  }

  // Get a second point which is different than the first
  for( int iter = 0; iter < max_sample_checks_; ++iter )
  {
    if(useWeights)
    {
        samples = wsampler_->GetSamples(getSampleSize());
    }
    else
    {
        samples.resize( getSampleSize() );
        drawIndexSample(samples);
    }
    // If it's a good sample, stop here
    if(isSampleGood(samples))
      return;
  }
  fprintf( stdout,
      "[sm::SampleConsensusModel::getSamples] WARNING: Could not select %d sample points in %d iterations!\n",
      getSampleSize(), max_sample_checks_ );
  samples.clear();

}

void
opengv::sac_problems::
    absolute_pose::AbsolutePoseSacProblem::setIndices(
    const std::vector<int> & indices )
{
  indices_.reset( new std::vector<int>(indices) );
  shuffled_indices_ = *indices_;
  
  if(useWeights)
  {
    weights_.clear();
    for(int i=0;i<indices.size();i++)
    {
        weights_.push_back(_adapter.getWeight(indices[i]));
    }
    wsampler_.reset(new WeightedSampler(weights_));
  }
}
