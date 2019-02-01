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

#include "vtkPCLRSDEstimationFilter2.h"
#include "vtkPCLConversions.h"
#include "vtkObjectFactory.h"

// #define PCL_NO_PRECOMPILE
#include <pcl/features/rsd.h>

vtkStandardNewMacro(vtkPCLRSDEstimationFilter2);

//------------------------------------------------------------------------------
vtkPCLRSDEstimationFilter2::vtkPCLRSDEstimationFilter2()
{
  this->SecondPortOptional = true;
}

//------------------------------------------------------------------------------
vtkPCLRSDEstimationFilter2::~vtkPCLRSDEstimationFilter2()
{
}

//------------------------------------------------------------------------------
void vtkPCLRSDEstimationFilter2::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
int vtkPCLRSDEstimationFilter2::ApplyPCLFilter2(
  vtkPolyData * points,
  vtkPolyData * normals,
  vtkPolyData * features
)
{
  // If a normals cloud is not provided, the points cloud must contain the
  // normals.
  if (normals == nullptr)
  {
    std::cout << "normals is null" << std::endl;
    std::set<std::string> requiredFieldNames { "normal_x", "normal_y", "normal_z" };
    int index = vtkPCLConversions::GetPointTypeIndex(points, requiredFieldNames);
#define _statement(PointType) return this->InternalApplyPCLFilter2<PointType>(points, features);
    PCLP_INVOKE_WITH_XYZ_NORMAL_POINT_TYPE(index, _statement)
#undef _statement
  }

  // Otherwise both the point and normal type have to be determined.
  else
  {
    int index = vtkPCLConversions::GetPointTypeIndex(points);
#define _statement(PointType) return this->InternalApplyPCLFilter2<PointType, pcl::Normal>(points, normals, features);
    PCLP_INVOKE_WITH_PCL_XYZ_POINT_TYPE(index, _statement)
#undef _statement
  }

  return 0;
}

//------------------------------------------------------------------------------
template <typename PointType>
int vtkPCLRSDEstimationFilter2::InternalApplyPCLFilter2(
  vtkPolyData * points,
  vtkPolyData * features
)
{

  typedef pcl::PointCloud<PointType> PointCloudT;
  typename PointCloudT::Ptr pointsCloud(new PointCloudT);
  vtkPCLConversions::PointCloudFromPolyData(points, pointsCloud);



  pcl::RSDEstimation<PointType, PointType, FeatureType> estimator;
  estimator.setInputCloud(pointsCloud);
  estimator.setInputNormals(pointsCloud);
  return this->ComputeFeatures(estimator, features);
}

//------------------------------------------------------------------------------
template <typename PointType, typename NormalType>
int vtkPCLRSDEstimationFilter2::InternalApplyPCLFilter2(
  vtkPolyData * points,
  vtkPolyData * normals,
  vtkPolyData * features
)
{
  typedef pcl::PointCloud<PointType> PointCloudT;
  typedef pcl::PointCloud<NormalType> NormalCloudT;

  typename PointCloudT::Ptr pointsCloud(new PointCloudT);
  typename NormalCloudT::Ptr normalsCloud(new NormalCloudT);

  vtkPCLConversions::PointCloudFromPolyData(points, pointsCloud);
  vtkPCLConversions::PointCloudFromPolyData(normals, normalsCloud);


  pcl::RSDEstimation<PointType, NormalType, FeatureType> estimator;
  estimator.setInputCloud(pointsCloud);
  estimator.setInputNormals(normalsCloud);
  return this->ComputeFeatures(estimator, features);
}

//------------------------------------------------------------------------------
template <typename T>
int vtkPCLRSDEstimationFilter2::ComputeFeatures(
  T estimator,
  vtkPolyData * features
)
{
  estimator.setRadiusSearch(this->RadiusSearch);
  estimator.setPlaneRadius(this->PlaneRadius);
  estimator.setSaveHistograms(this->SaveHistograms);
  typename FeatureCloudT::Ptr featuresCloud(new FeatureCloudT);
  estimator.compute((* featuresCloud));
  vtkPCLConversions::PolyDataFromPointCloud(featuresCloud, features);
  
  if (this->SaveHistograms)
  {
    int n = estimator.getNrSubdivisions();
    n *= n;

    int index = vtkPCLConversions::GetHistogramPointTypeIndex(n);
    // This will just add the histograms array to the field data.
#define _statement(PointType)                                                       \
  {                                                                                 \
    pcl::PointCloud<PointType>::Ptr histogramCloud(new pcl::PointCloud<PointType>); \
    pcl::getFeaturePointCloud((* estimator.getHistograms()), (* histogramCloud));   \
    vtkPCLConversions::PolyDataFromPointCloud(histogramCloud, features, true);      \
  }
    
    PCLP_INVOKE_WITH_PCLP_HISTOGRAM_POINT_TYPE(index, _statement)

#undef _statement
  }
  return 1;
}

