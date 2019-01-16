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

#include "vtkPCLFPFHEstimationFilter2.h"
#include "vtkPCLConversions.h"
#include "vtkObjectFactory.h"

// #define PCL_NO_PRECOMPILE
#include <pcl/features/fpfh_omp.h>

vtkStandardNewMacro(vtkPCLFPFHEstimationFilter2);

//------------------------------------------------------------------------------
vtkPCLFPFHEstimationFilter2::vtkPCLFPFHEstimationFilter2()
{
  this->SecondPortOptional = true;
  this->Radius = 0.05;
}

//------------------------------------------------------------------------------
vtkPCLFPFHEstimationFilter2::~vtkPCLFPFHEstimationFilter2()
{
}

//------------------------------------------------------------------------------
void vtkPCLFPFHEstimationFilter2::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
int vtkPCLFPFHEstimationFilter2::ApplyPCLFilter2(
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
    PCLP_INVOKE_WITH_XYZ_POINT_TYPE(index, _statement)
#undef _statement
  }

  return 0;
}

//------------------------------------------------------------------------------
template <typename PointType>
int vtkPCLFPFHEstimationFilter2::InternalApplyPCLFilter2(
  vtkPolyData * points,
  vtkPolyData * features
)
{
  typedef pcl::FPFHSignature33 FeatureType;

  typedef pcl::PointCloud<PointType> PointCloudT;
  typedef pcl::PointCloud<FeatureType> FeatureCloudT;

  typename PointCloudT::Ptr pointsCloud(new PointCloudT);
  typename FeatureCloudT::Ptr featuresCloud(new FeatureCloudT);

  vtkPCLConversions::PointCloudFromPolyData(points, pointsCloud);


  pcl::FPFHEstimationOMP<PointType, PointType, FeatureType> fe;
  fe.setInputCloud(pointsCloud);
  fe.setInputNormals(pointsCloud);
  fe.setRadiusSearch(this->Radius);
  fe.compute((* featuresCloud));

  vtkPCLConversions::PolyDataFromPointCloud(featuresCloud, features);
  return 1;
}

//------------------------------------------------------------------------------
template <typename PointType, typename NormalType>
int vtkPCLFPFHEstimationFilter2::InternalApplyPCLFilter2(
  vtkPolyData * points,
  vtkPolyData * normals,
  vtkPolyData * features
)
{
  typedef pcl::FPFHSignature33 FeatureType;

  typedef pcl::PointCloud<PointType> PointCloudT;
  typedef pcl::PointCloud<NormalType> NormalCloudT;
  typedef pcl::PointCloud<FeatureType> FeatureCloudT;

  typename PointCloudT::Ptr pointsCloud(new PointCloudT);
  typename NormalCloudT::Ptr normalsCloud(new NormalCloudT);
  typename FeatureCloudT::Ptr featuresCloud(new FeatureCloudT);

  vtkPCLConversions::PointCloudFromPolyData(points, pointsCloud);
  vtkPCLConversions::PointCloudFromPolyData(normals, normalsCloud);


  pcl::FPFHEstimationOMP<PointType, NormalType, FeatureType> fe;
  fe.setInputCloud(pointsCloud);
  fe.setInputNormals(normalsCloud);
  fe.setRadiusSearch(this->Radius);
  fe.compute((* featuresCloud));

  vtkPCLConversions::PolyDataFromPointCloud(featuresCloud, features);
  return 1;
}

