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

#include "vtkPCLSampleConsensusModelRegistrationFilter4.h"
#include "vtkPCLConversions.h"
#include "FeatureExtractor.h"

#include "vtkObjectFactory.h"

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>



//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLSampleConsensusModelRegistrationFilter4);

//------------------------------------------------------------------------------
vtkPCLSampleConsensusModelRegistrationFilter4::vtkPCLSampleConsensusModelRegistrationFilter4()
{
  this->DistanceThreshold = 0.02;
  this->MaxIterations = 2500;
  this->Probability = 0.99;

  this->HasTransformation = false;
  this->ReuseTransformation = false;
}

//------------------------------------------------------------------------------
vtkPCLSampleConsensusModelRegistrationFilter4::~vtkPCLSampleConsensusModelRegistrationFilter4()
{
}

//------------------------------------------------------------------------------
void vtkPCLSampleConsensusModelRegistrationFilter4::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
int vtkPCLSampleConsensusModelRegistrationFilter4::ApplyPCLFilter4(
  vtkPolyData * pointsA,
  vtkPolyData * featuresA,
  vtkPolyData * pointsB,
  vtkPolyData * featuresB,
  vtkPolyData * output
)
{
  // Determine the index of the spatial data point type.
  int index = vtkPCLConversions::GetPointTypeIndex(pointsA);
#define _statement(PointType) return this->InternalApplyPCLFilter4<PointType>(pointsA, featuresA, pointsB, featuresB, output);
  PCLP_INVOKE_WITH_XYZ_NORMAL_POINT_TYPE(index, _statement)
#undef _statement
  return 0;

  // return this->InternalInternalApplyPCLFilter4<pcl::PointNormal,pcl::FPFHSignature33>(pointsA, featuresA, pointsB, featuresB, output);
}

//------------------------------------------------------------------------------
template <typename PointType>
int vtkPCLSampleConsensusModelRegistrationFilter4::InternalApplyPCLFilter4(
  vtkPolyData * pointsA,
  vtkPolyData * featuresA,
  vtkPolyData * pointsB,
  vtkPolyData * featuresB,
  vtkPolyData * output
)
{
  // Determine the index of the feature data point type.
  int index = vtkPCLConversions::GetPointTypeIndex(featuresA);
#define _statement(FeatureType) return this->InternalInternalApplyPCLFilter4<PointType,FeatureType>(pointsA, featuresA, pointsB, featuresB, output);
  PCLP_INVOKE_WITH_FEATURE_POINT_TYPE(index, _statement)
#undef _statement
  return 0;
  
  // return this->InternalInternalApplyPCLFilter4<PointType,pcl::FPFHSignature33>(pointsA, featuresA, pointsB, featuresB, output);
}

//------------------------------------------------------------------------------
template <typename PointType, typename FeatureType>
int vtkPCLSampleConsensusModelRegistrationFilter4::InternalInternalApplyPCLFilter4(
  vtkPolyData * pointsA,
  vtkPolyData * featuresA,
  vtkPolyData * pointsB,
  vtkPolyData * featuresB,
  vtkPolyData * output
)
{
  typedef pcl::PointCloud<PointType> PCloudT;
  typedef pcl::PointCloud<FeatureType> FCloudT;

  // Convert the source cloud.
  typename PCloudT::Ptr pointsACloud(new PCloudT);
  vtkPCLConversions::PointCloudFromPolyData(pointsA, pointsACloud);

  typename PCloudT::Ptr outputCloud(new PCloudT);

  // Perform the alignment and get the transformation matrix if missing or reuse
  // is not requested.
  if (! (this->HasTransformation && this->ReuseTransformation))
  {
    typename PCloudT::Ptr pointsBCloud(new PCloudT);
    vtkPCLConversions::PointCloudFromPolyData(pointsB, pointsBCloud);

    typename FCloudT::Ptr featuresACloud(new FCloudT);
    typename FCloudT::Ptr featuresBCloud(new FCloudT);

    vtkPCLConversions::PointCloudFromPolyData(featuresA, featuresACloud);
    vtkPCLConversions::PointCloudFromPolyData(featuresB, featuresBCloud);

    pcl::Correspondences correspondences;
    pcl::registration::CorrespondenceEstimation<FeatureType,FeatureType> correspondenceEstimator;
    correspondenceEstimator.setInputCloud(featuresACloud);
    correspondenceEstimator.setInputTarget(featuresBCloud);
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
    ransacModel.reset(new pcl::SampleConsensusModelRegistration<PointType>(pointsACloud, sourceIndices));
    ransacModel->setInputTarget(pointsBCloud, targetIndices);

    pcl::RandomSampleConsensus<PointType> ransacFilter(ransacModel);
    ransacFilter.setMaxIterations(this->MaxIterations);
    ransacFilter.setDistanceThreshold(this->DistanceThreshold);
    ransacFilter.setProbability(this->Probability);
    if (! (ransacFilter.computeModel() && ransacFilter.refineModel()))
    {
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

  pcl::transformPointCloud<PointType>((* pointsACloud), (* outputCloud), this->TransformationMatrix);
  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
  return 1;
}

