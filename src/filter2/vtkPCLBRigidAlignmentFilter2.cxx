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

#include "vtkPCLBRigidAlignmentFilter2.h"
#include "vtkPCLConversions.h"
#include "FeatureExtractor.h"

#include "vtkObjectFactory.h"

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>



//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLBRigidAlignmentFilter2);

//------------------------------------------------------------------------------
vtkPCLBRigidAlignmentFilter2::vtkPCLBRigidAlignmentFilter2()
{
}

//------------------------------------------------------------------------------
vtkPCLBRigidAlignmentFilter2::~vtkPCLBRigidAlignmentFilter2()
{
}

//------------------------------------------------------------------------------
void vtkPCLBRigidAlignmentFilter2::PrintSelf(ostream & os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "NormalRadius: " << this->NormalRadius << "\n";
  os << indent << "FeatureRadius: " << this->FeatureRadius << "\n";
  os << indent << "DistanceThreshold: " << this->DistanceThreshold << "\n";
  os << indent << "MaxIterations: " << this->MaxIterations << "\n";
  os << indent << "Probability: " << this->Probability << "\n";
  os << indent << "TransformationMatrix: " << this->TransformationMatrix << "\n";
  os << indent << "HasTransformation: " << (this->HasTransformation ? "yes" : "no") << "\n";
  os << indent << "ReuseTransformation: " << (this->ReuseTransformation ? "yes" : "no") << "\n";
}

//------------------------------------------------------------------------------
int vtkPCLBRigidAlignmentFilter2::ApplyPCLFilter2(
  vtkPolyData * input,
  vtkPolyData * target,
  vtkPolyData * output
)
{
  int index = vtkPCLConversions::GetPointTypeIndex(input);
#define _statement(PointType) return this->InternalApplyPCLFilter2<PointType,pcl::Normal,pcl::FPFHSignature33>(input, target, output);
  PCLP_INVOKE_WITH_PCL_XYZ_POINT_TYPE(index, _statement)
#undef _statement
  vtkErrorMacro(<< "no XYZ point data in input")
  return 0;
}

//------------------------------------------------------------------------------
template <typename PointT, typename NormalT, typename FeatureT>
int vtkPCLBRigidAlignmentFilter2::InternalApplyPCLFilter2(
  vtkPolyData * input,
  vtkPolyData * target,
  vtkPolyData * output
)
{
  // The transformation will be applied to the input cloud to preserve its
  // attributes even if the model used is purely geometric.
  typedef pcl::PointCloud<PointT> CloudT;
  typename CloudT::Ptr inputCloud(new CloudT);
  typename CloudT::Ptr outputCloud(new CloudT);

  vtkPCLConversions::PointCloudFromPolyData(input, inputCloud);

  // Perform the alignment and get the transformation matrix if missing or reuse
  // is not requested.
  if (! (this->HasTransformation && this->ReuseTransformation))
  {
    // The model functions require PointNormal clouds. These are used to
    // calculate the transformation, which will then be applied to the input
    // type to preserve attributes.
    //
    // TODO: check if this is can be generalized to all point types.
    typedef pcl::PointNormal PointNormalT;
    typedef pcl::PointCloud<PointNormalT> NormalCloudT;
    typename NormalCloudT::Ptr sourceCloud(new NormalCloudT);
    typename NormalCloudT::Ptr targetCloud(new NormalCloudT);
    vtkPCLConversions::PointCloudFromPolyData(input, sourceCloud);
    vtkPCLConversions::PointCloudFromPolyData(target, targetCloud);

    FeatureExtractor<PointNormalT, NormalT, FeatureT>
      feSource(sourceCloud, this->NormalRadius, this->FeatureRadius),
      feTarget(targetCloud, this->NormalRadius, this->FeatureRadius);

    pcl::Correspondences correspondences;
    pcl::registration::CorrespondenceEstimation<FeatureT,FeatureT> correspondenceEstimator;
    correspondenceEstimator.setInputCloud(feSource.Features);
    correspondenceEstimator.setInputTarget(feTarget.Features);
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

    pcl::SampleConsensusModelRegistration<PointNormalT>::Ptr ransacModel;
    ransacModel.reset(new pcl::SampleConsensusModelRegistration<PointNormalT>(sourceCloud, sourceIndices));
    ransacModel->setInputTarget (targetCloud, targetIndices);

    pcl::RandomSampleConsensus<PointNormalT> ransacFilter(ransacModel);
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

  pcl::transformPointCloud<PointT>((* inputCloud), (* outputCloud), this->TransformationMatrix);
  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
  return 1;
}

