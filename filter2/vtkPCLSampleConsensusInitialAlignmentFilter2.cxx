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

#include "vtkObjectFactory.h"

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>



//------------------------------------------------------------------------------
/*!
 * @brief  Surface normal and feature extractor.
 * @tparam PointT   The PCL point type.
 * @tparam NormalT  The PCL normal type.
 * @tparam FeatureT The PCL feature type.
 */
template <
  typename PointT=pcl::PointXYZ,
  typename NormalT=pcl::Normal,
  typename FeatureT=pcl::FPFHSignature33
>
struct FeatureExtractor
{
  public:
    typedef pcl::PointCloud<PointT> XYZCloudT;
    typedef pcl::PointCloud<NormalT> NormalCloudT;
    typedef pcl::PointCloud<FeatureT> FeatureCloudT;
    typedef pcl::search::KdTree<PointT> SearchT;

    typename XYZCloudT::ConstPtr Cloud;
    typename NormalCloudT::Ptr   Normals;
    typename FeatureCloudT::Ptr  Features;
    typename SearchT::Ptr        SearchMethod;
		float               NormalRadius;
    float               FeatureRadius;

    FeatureExtractor(
      typename XYZCloudT::ConstPtr inputCloud,
      float normalRadius=0.02f,
      float featureRadius=0.02f
    )
      : Cloud { inputCloud }
      , Normals { new NormalCloudT }
      , Features { new FeatureCloudT }
      , SearchMethod { new SearchT }
      , NormalRadius { normalRadius }
      , FeatureRadius { featureRadius }
    {
      this->computeSurfaceNormals();
      this->computeLocalFeatures();
    }

    ~FeatureExtractor()
    {}

    void
    computeSurfaceNormals()
    {
      pcl::NormalEstimation<PointT, NormalT> norm_est;
      norm_est.setInputCloud (this->Cloud);
      norm_est.setSearchMethod (this->SearchMethod);
      norm_est.setRadiusSearch (this->NormalRadius);
      norm_est.compute (*(this->Normals));
    }

    void
    computeLocalFeatures()
    {
      pcl::FPFHEstimation<PointT, NormalT, FeatureT> fpfh_est;
      fpfh_est.setInputCloud(this->Cloud);
      fpfh_est.setInputNormals(this->Normals);
      fpfh_est.setSearchMethod(this->SearchMethod);
      fpfh_est.setRadiusSearch(this->FeatureRadius);
      fpfh_est.compute (*(this->Features));
    }
};



//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLSampleConsensusInitialAlignmentFilter2);

//------------------------------------------------------------------------------
vtkPCLSampleConsensusInitialAlignmentFilter2::vtkPCLSampleConsensusInitialAlignmentFilter2()
{
  this->MinSampleDistance = 0.05;
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
  PCLP_INVOKE_WITH_POINT_TYPE(index, this->InternalApplyPCLFilter2, input, target, output)
  return 1;
}

//------------------------------------------------------------------------------
template <
  typename PointT=pcl::PointXYZ,
  typename NormalT=pcl::Normal,
  typename FeatureT=pcl::FPFHSignature33
>
void vtkPCLSampleConsensusInitialAlignmentFilter2::InternalApplyPCLFilter2(
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

