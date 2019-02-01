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

#include "vtkPCLSampleConsensusInitialAlignmentFilter2.h"
#include "vtkPCLConversions.h"
#include "FeatureExtractor.h"

#include "vtkObjectFactory.h"

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>



//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLSampleConsensusInitialAlignmentFilter2);

//------------------------------------------------------------------------------
vtkPCLSampleConsensusInitialAlignmentFilter2::vtkPCLSampleConsensusInitialAlignmentFilter2()
{
}

//------------------------------------------------------------------------------
vtkPCLSampleConsensusInitialAlignmentFilter2::~vtkPCLSampleConsensusInitialAlignmentFilter2()
{
}

//------------------------------------------------------------------------------
void vtkPCLSampleConsensusInitialAlignmentFilter2::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
int vtkPCLSampleConsensusInitialAlignmentFilter2::ApplyPCLFilter2(
  vtkPolyData * input,
  vtkPolyData * target,
  vtkPolyData * output
)
{
  int index = vtkPCLConversions::GetPointTypeIndex(input);
#define _statement(PointType) return this->InternalApplyPCLFilter2<PointType>(input, target, output);
  PCLP_INVOKE_WITH_PCL_XYZ_POINT_TYPE(index, _statement)
#undef _statement
  return 0;
}

//------------------------------------------------------------------------------
template <
  typename PointT=pcl::PointXYZ,
  typename NormalT=pcl::Normal,
  typename FeatureT=pcl::FPFHSignature33
>
int vtkPCLSampleConsensusInitialAlignmentFilter2::InternalApplyPCLFilter2(
  vtkPolyData * input,
  vtkPolyData * target,
  vtkPolyData * output
)
{
  typedef pcl::PointCloud<PointT> CloudT;
  typename CloudT::Ptr inputCloud(new CloudT);
  typename CloudT::Ptr outputCloud(new CloudT);

  vtkPCLConversions::PointCloudFromPolyData(input, inputCloud);

  // Perform the alignment if a cached transformation was not applied.
  if (! this->MaybeApplyCachedTransformation((* inputCloud), (* outputCloud)))
  {
    // The Sample Consensus Initial Alignment (SCIA) filter requires the input
    // and target cloud along with their respective feature vectors.
    typename CloudT::Ptr targetCloud(new CloudT);
    vtkPCLConversions::PointCloudFromPolyData(target, targetCloud);

    FeatureExtractor<PointT, NormalT, FeatureT>
      fe_input(inputCloud, this->NormalRadius, this->FeatureRadius),
      fe_target(targetCloud, this->NormalRadius, this->FeatureRadius);

    pcl::SampleConsensusInitialAlignment<PointT, PointT, FeatureT> scia;

    scia.setInputCloud(fe_input.Cloud);
    scia.setSourceFeatures(fe_input.Features);

    scia.setInputTarget(fe_target.Cloud);
    scia.setTargetFeatures(fe_target.Features);

    // SCIA-specific options.
    scia.setMinSampleDistance(this->MinSampleDistance);

    // Set common registration options and align.
    this->ConfigureAndAlign<PointT,PointT>(scia, (* outputCloud));
  }

  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
}

