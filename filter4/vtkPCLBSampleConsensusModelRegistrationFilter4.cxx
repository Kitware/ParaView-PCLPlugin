//==============================================================================
//
// Copyright 2012-2018 Kitware, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//==============================================================================

#include "vtkPCLBSampleConsensusModelRegistrationFilter4.h"
#include "vtkPCLConversions.h"
#include "FeatureExtractor.h"

#include "vtkObjectFactory.h"

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>



//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLBSampleConsensusModelRegistrationFilter4);

//------------------------------------------------------------------------------
vtkPCLBSampleConsensusModelRegistrationFilter4::vtkPCLBSampleConsensusModelRegistrationFilter4()
{
}

//------------------------------------------------------------------------------
vtkPCLBSampleConsensusModelRegistrationFilter4::~vtkPCLBSampleConsensusModelRegistrationFilter4()
{
}

//------------------------------------------------------------------------------
void vtkPCLBSampleConsensusModelRegistrationFilter4::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
int vtkPCLBSampleConsensusModelRegistrationFilter4::ApplyPCLFilter4(
  vtkPolyData * sourcePointsPD,
  vtkPolyData * sourceFeaturesPD,
  vtkPolyData * targetPointsPD,
  vtkPolyData * targetFeaturesPD,
  vtkPolyData * output
)
{
  // Determine the index of the spatial data point type.
  int index = vtkPCLConversions::GetPointTypeIndex(sourcePointsPD);
#define _statement(PointType) return this->InternalApplyPCLFilter4<PointType>(sourcePointsPD, sourceFeaturesPD, targetPointsPD, targetFeaturesPD, output);
  PCLP_INVOKE_WITH_PCL_XYZ_POINT_TYPE(index, _statement)
#undef _statement
  vtkErrorMacro(<< "no XYZ data in source points input")
  return 0;

  // return this->InternalInternalApplyPCLFilter4<pcl::PointNormal,pcl::FPFHSignature33>(sourcePointsPD, sourceFeaturesPD, targetPointsPD, targetFeaturesPD, output);
}

//------------------------------------------------------------------------------
template <typename PointType>
int vtkPCLBSampleConsensusModelRegistrationFilter4::InternalApplyPCLFilter4(
  vtkPolyData * sourcePointsPD,
  vtkPolyData * sourceFeaturesPD,
  vtkPolyData * targetPointsPD,
  vtkPolyData * targetFeaturesPD,
  vtkPolyData * output
)
{
  // Determine the index of the feature data point type.
  int index = vtkPCLConversions::GetPointTypeIndex(sourceFeaturesPD);
#define _statement(FeatureType) return this->InternalInternalApplyPCLFilter4<PointType,FeatureType>(sourcePointsPD, sourceFeaturesPD, targetPointsPD, targetFeaturesPD, output);
  PCLP_INVOKE_WITH_PCL_FEATURE_POINT_TYPE(index, _statement)
#undef _statement
  vtkErrorMacro(<< "no feature data in source features input")
  return 0;
  
  // return this->InternalInternalApplyPCLFilter4<PointType,pcl::FPFHSignature33>(sourcePointsPD, sourceFeaturesPD, targetPointsPD, targetFeaturesPD, output);
}

//------------------------------------------------------------------------------
template <typename PointType, typename FeatureType>
int vtkPCLBSampleConsensusModelRegistrationFilter4::InternalInternalApplyPCLFilter4(
  vtkPolyData * sourcePointsPD,
  vtkPolyData * sourceFeaturesPD,
  vtkPolyData * targetPointsPD,
  vtkPolyData * targetFeaturesPD,
  vtkPolyData * output
)
{
  typedef pcl::PointCloud<PointType> PCloudT;
  typedef pcl::PointCloud<FeatureType> FCloudT;

  // Convert the source cloud.
  typename PCloudT::Ptr sourcePointsPC(new PCloudT);
  vtkPCLConversions::PointCloudFromPolyData(sourcePointsPD, sourcePointsPC);

  typename PCloudT::Ptr outputCloud(new PCloudT);

  // Perform the alignment and get the transformation matrix if missing or reuse
  // is not requested.
  if (! (this->HasTransformation && this->ReuseTransformation))
  {
    typename PCloudT::Ptr targetPointsPC(new PCloudT);
    vtkPCLConversions::PointCloudFromPolyData(targetPointsPD, targetPointsPC);

    typename FCloudT::Ptr sourceFeaturesPC(new FCloudT);
    vtkPCLConversions::PointCloudFromPolyData(sourceFeaturesPD, sourceFeaturesPC);

    typename FCloudT::Ptr targetFeaturesPC(new FCloudT);
    vtkPCLConversions::PointCloudFromPolyData(targetFeaturesPD, targetFeaturesPC);

    pcl::Correspondences correspondences;
    pcl::registration::CorrespondenceEstimation<FeatureType,FeatureType> correspondenceEstimator;
    correspondenceEstimator.setInputCloud(sourceFeaturesPC);
    correspondenceEstimator.setInputTarget(targetFeaturesPC);
    correspondenceEstimator.initCompute();
    correspondenceEstimator.determineCorrespondences(correspondences);

    auto nr_correspondences = correspondences.size();
    std::vector<int> sourceIndices (nr_correspondences);
    std::vector<int> targetIndices (nr_correspondences);
    for (decltype(nr_correspondences) i = 0; i < nr_correspondences; ++i)
    {
      sourceIndices[i] = correspondences[i].index_query;
      targetIndices[i] = correspondences[i].index_match;
    }

    typename pcl::SampleConsensusModelRegistration<PointType>::Ptr ransacModel;
    ransacModel.reset(new pcl::SampleConsensusModelRegistration<PointType>(sourcePointsPC, sourceIndices));
    ransacModel->setInputTarget(targetPointsPC, targetIndices);

    pcl::RandomSampleConsensus<PointType> ransacFilter(ransacModel);
    ransacFilter.setMaxIterations(this->MaxIterations);
    ransacFilter.setDistanceThreshold(this->DistanceThreshold);
    ransacFilter.setProbability(this->Probability);
    if (! (ransacFilter.computeModel() && ransacFilter.refineModel()))
    {
      vtkErrorMacro(<< "failed to compute random sample consensus")
      return 0;
    }

    Eigen::VectorXf modelCoefficients;
    ransacFilter.getModelCoefficients(modelCoefficients);
    this->TransformationMatrix.row (0) = modelCoefficients.segment<4>(0);
    this->TransformationMatrix.row (1) = modelCoefficients.segment<4>(4);
    this->TransformationMatrix.row (2) = modelCoefficients.segment<4>(8);
    this->TransformationMatrix.row (3) = modelCoefficients.segment<4>(12);
    this->HasTransformation = true;
  }

  pcl::transformPointCloud<PointType>((* sourcePointsPC), (* outputCloud), this->TransformationMatrix);
  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
  return 1;
}

