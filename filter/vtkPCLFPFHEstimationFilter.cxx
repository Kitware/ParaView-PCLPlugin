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

#include "vtkPCLFPFHEstimationFilter.h"
#include "vtkPCLConversions.h"
#include "vtkObjectFactory.h"

// #define PCL_NO_PRECOMPILE
#include <pcl/features/fpfh_omp.h>

vtkStandardNewMacro(vtkPCLFPFHEstimationFilter);

//------------------------------------------------------------------------------
vtkPCLFPFHEstimationFilter::vtkPCLFPFHEstimationFilter()
{
  this->Radius = 0.05;
}

//------------------------------------------------------------------------------
vtkPCLFPFHEstimationFilter::~vtkPCLFPFHEstimationFilter()
{
}

//------------------------------------------------------------------------------
void vtkPCLFPFHEstimationFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
int vtkPCLFPFHEstimationFilter::ApplyPCLFilter(
  vtkPolyData * input,
  vtkPolyData * output
)
{
  // This single-input filter expects the input cloud to include both the point
  // data and the normals. A separate 2-input filter can be created to separate
  // these if necessary.
  std::set<std::string> requiredFieldNames { "normal_x", "normal_y", "normal_z" };
  int index = vtkPCLConversions::GetPointTypeIndex(input, requiredFieldNames);
#define _statement(PointType) return this->InternalApplyPCLFilter<PointType>(input, output);
  PCLP_INVOKE_WITH_XYZ_NORMAL_POINT_TYPE(index, _statement)
#undef _statement
  return 0;
}

//------------------------------------------------------------------------------
template <typename PointType>
int vtkPCLFPFHEstimationFilter::InternalApplyPCLFilter(
  vtkPolyData * input,
  vtkPolyData * output
)
{
  typedef pcl::FPFHSignature33 FeatureType;
  typedef pcl::PointCloud<PointType> InputCloudT;
  typedef pcl::PointCloud<FeatureType> OutputCloudT;
  typename pcl::PointCloud<PointType>::Ptr inputCloud(new InputCloudT);
  typename pcl::PointCloud<FeatureType>::Ptr outputCloud(new OutputCloudT);

  vtkPCLConversions::PointCloudFromPolyData(input, inputCloud);

  pcl::FPFHEstimationOMP<PointType, PointType, FeatureType> fe;
  fe.setInputCloud(inputCloud);
  fe.setInputNormals(inputCloud);
  fe.setRadiusSearch(this->Radius);
  fe.compute((* outputCloud));

  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
  return 1;
}

