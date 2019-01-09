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

#include "vtkPCLNormalEstimationFilter.h"
#include "vtkPCLConversions.h"
#include "vtkObjectFactory.h"

// #define PCL_NO_PRECOMPILE
#include <pcl/features/normal_3d_omp.h>

vtkStandardNewMacro(vtkPCLNormalEstimationFilter);

//------------------------------------------------------------------------------
vtkPCLNormalEstimationFilter::vtkPCLNormalEstimationFilter()
{
  this->Radius = 0.05;
}

//------------------------------------------------------------------------------
vtkPCLNormalEstimationFilter::~vtkPCLNormalEstimationFilter()
{
}

//------------------------------------------------------------------------------
void vtkPCLNormalEstimationFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
int vtkPCLNormalEstimationFilter::ApplyPCLFilter(
  vtkPolyData * input,
  vtkPolyData * output
)
{
  int index = vtkPCLConversions::GetPointTypeIndex(input);
#define _statement(PointType) return this->InternalApplyPCLFilter<PointType>(input, output);
  PCLP_INVOKE_WITH_XYZ_POINT_TYPE(index, _statement)
#undef _statement
  return 0;
}

//------------------------------------------------------------------------------
template <typename PointType>
int vtkPCLNormalEstimationFilter::InternalApplyPCLFilter(
  vtkPolyData * input,
  vtkPolyData * output
)
{
  typedef pcl::PointCloud<PointType> CloudT;
  typename CloudT::Ptr inputCloud(new CloudT);
  vtkPCLConversions::PointCloudFromPolyData(input, inputCloud);

  // Find a point type that includes normal components and as many of the
  // existing components as possible.
  std::set<std::string> requiredFieldNames { "normal_x", "normal_y", "normal_z" };
  PointType const point;
  int index = vtkPCLConversions::GetPointTypeIndex<PointType const &>(point, requiredFieldNames);
#define _statement(NormalPointType) return this->EstimateNormals<PointType, NormalPointType>(inputCloud, input, output);
  PCLP_INVOKE_WITH_NORMAL_POINT_TYPE(index, _statement)
#undef _statement
  return 0;
}

//------------------------------------------------------------------------------
template <typename PointType, typename NormalPointType>
int vtkPCLNormalEstimationFilter::EstimateNormals(
  typename pcl::PointCloud<PointType>::Ptr & inputCloud,
  vtkPolyData * input,
  vtkPolyData * output
)
{
  typedef pcl::PointCloud<NormalPointType> NormalCloudT;
  typename NormalCloudT::Ptr outputCloud(new NormalCloudT);

  // Copy the xyz data. The normal data will be inserted by the estimator below.
  vtkPCLConversions::PointCloudFromPolyData(input, outputCloud);

  pcl::NormalEstimationOMP<PointType, NormalPointType> ne;
  ne.setRadiusSearch(this->Radius);
  ne.setInputCloud(inputCloud);
  ne.compute((* outputCloud));

  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
  return 1;
}

