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

/**
 * \file AbsolutePoseSacProblem.hpp
 * \brief Functions for fitting an absolute-pose model to a set of
 *        bearing-vector-point correspondences, using different algorithms
 *        (central and non-central one). Used in a sample-consenus paradigm for
 *        rejecting outlier correspondences.
 */

#ifndef OPENGV_SAC_PROBLEMS_ABSOLUTE_POSE_ABSOLUTEPOSESACPROBLEM_HPP_
#define OPENGV_SAC_PROBLEMS_ABSOLUTE_POSE_ABSOLUTEPOSESACPROBLEM_HPP_

#include "SampleConsensusProblem.hpp"
#include "PoseLib/types.h"
#include "PoseComputer/AdapterPointLine.h"
#include "WeightedSampler.h"

/**
 * \brief The namespace of this library.
 */
namespace opengv
{
/**
 * \brief The namespace for the sample consensus problems.
 */
namespace sac_problems
{
/**
 * \brief The namespace for the absolute pose methods.
 */
namespace absolute_pose
{

using pose_lib::CameraPose;
using pose_lib::AdapterPointLine;
using pose_lib::WeightedSampler;

/**
 * Provides functions for fitting an absolute-pose model to a set of
 * bearing-vector to point correspondences, using different algorithms (central
 * and non-central ones). Used in a sample-consenus paradigm for rejecting
 * outlier correspondences.
 */
class AbsolutePoseSacProblem :
    public sac::SampleConsensusProblem<CameraPose>
{
public:
  /** The model we are trying to fit (transformation) */
  typedef CameraPose model_t;
  /** The type of adapter that is expected by the methods */
  typedef AdapterPointLine adapter_t;

  /**
   * \brief Constructor.
   * \param[in] adapter Visitor holding bearing vectors, world points, etc.
   * \param[in] algorithm The algorithm we want to use.
   * \param[in] randomSeed Whether to seed the random number generator with
   *            the current time.
   */
  AbsolutePoseSacProblem(adapter_t & adapter, bool useWeights = false, 
      bool randomSeed = true) :
      sac::SampleConsensusProblem<model_t> (randomSeed),
      _adapter(adapter),
      useWeights(useWeights)
  {
    setUniformIndices(adapter.getNumberCorrespondences());
    
    if(useWeights)
    {
        weights_.clear();
        if(adapter.isWeightsUsed())
        {
            weights_ = *adapter.getWeights();
            wsampler_.reset(new WeightedSampler(weights_));
        }
        else
        {
            this->useWeights = false;
            wsampler_ = nullptr;
            fprintf(stdout, "AbsolutePoseSacProblem Constructor] Adapter doesn't have weights, ignoring parameter: useWeights\n.");
        }
    }
  };

  /**
   * \brief Constructor.
   * \param[in] adapter Visitor holding bearing vectors, world points, etc.
   * \param[in] algorithm The algorithm we want to use.
   * \param[in] indices A vector of indices to be used from all available
   *                    correspondences.
   * \param[in] randomSeed Whether to seed the random number generator with
   *            the current time.
   */
  AbsolutePoseSacProblem(
      adapter_t & adapter, 
      const std::vector<int> & indices, 
      bool useWeights = false, 
      bool randomSeed = true) :
      sac::SampleConsensusProblem<model_t> (randomSeed),
      _adapter(adapter),
      useWeights(useWeights)
  {
    setIndices(indices);
    
    if(useWeights)
    {
        weights_.clear();
        if(adapter.isWeightsUsed())
        {
            for(int i=0;i<indices.size();i++)
            {
                int idx = indices[i];
                weights_.push_back(adapter.getWeight(idx));
            }
            wsampler_.reset(new WeightedSampler(weights_));
        }
        else
        {
            this->useWeights = false;
            wsampler_ = nullptr;
            fprintf(stdout, "AbsolutePoseSacProblem Constructor] Adapter doesn't have weights, ignoring parameter: useWeights\n.");
        }
    }
  };

  /**
   * Destructor.
   */
  virtual ~AbsolutePoseSacProblem() {};

  /**
   * \brief See parent-class.
   */
  virtual bool computeModelCoefficients(
      const std::vector<int> & indices,
      model_t & outModel) const;

  /**
   * \brief See parent-class.
   */
  virtual void getSelectedDistancesToModel(
      const model_t & model,
      const std::vector<int> & indices,
      std::vector<double> & scores) const;

  /**
   * \brief See parent-class.
   */
  virtual void optimizeModelCoefficients(
      const std::vector<int> & inliers,
      const model_t & model,
      model_t & optimized_model);

  /**
   * \brief See parent-class.
   */
  virtual int getSampleSize() const;
  
  /**
   * \brief See parent-class.
   */
  virtual void getSamples( int &iterations, std::vector<int> &samples );
  
  /**
   * \brief See parent-class.
   */
  virtual void setIndices( const std::vector<int> & indices );

protected:
  /** The adapter holding all input data */
  adapter_t & _adapter;
  /** Sample weights for weighted RANSAC sampling */
  std::vector<double> weights_;
  /** Flag whether the weights are used */
  bool useWeights;
  /** Weighted sampler for RANSAC sampling */
  std::shared_ptr<WeightedSampler> wsampler_;
  
};  // AbsolutePoseSacProblem

}  // namespace absolute_pose
}  // namespace sac_problems
}  // namespace opengv

#endif  //#ifndef OPENGV_SAC_PROBLEMS_ABSOLUTE_POSE_ABSOLUTEPOSESACPROBLEM_HPP_
