//==============================================================================
//
// Copyright 2012-2019 Kitware, Inc.
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

#include "vtkPCLVoxelGridFilter.h"
#include "vtkPCLConversions.h"
#include "vtkObjectFactory.h"

#include <pcl/filters/voxel_grid.h>

vtkStandardNewMacro(vtkPCLVoxelGridFilter);

//------------------------------------------------------------------------------
vtkPCLVoxelGridFilter::vtkPCLVoxelGridFilter()
{
}

//------------------------------------------------------------------------------
vtkPCLVoxelGridFilter::~vtkPCLVoxelGridFilter()
{
}

//------------------------------------------------------------------------------
void vtkPCLVoxelGridFilter::PrintSelf(ostream & os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "LeafSize: " << this->LeafSize[0] << ", " << this->LeafSize[1] << ", " << this->LeafSize[2] << '\n';
}

//------------------------------------------------------------------------------
int vtkPCLVoxelGridFilter::ApplyPCLFilter(
  vtkPolyData * input,
  vtkPolyData * output
)
{
  int index = vtkPCLConversions::GetPointTypeIndex(input);
#define _statement(PointType) return this->InternalApplyPCLFilter<PointType>(input, output);
  PCLP_INVOKE_WITH_PCL_XYZ_POINT_TYPE(index, _statement)
#undef _statement
  vtkErrorMacro(<< "no XYZ point data in input")
  return 0;
}

//------------------------------------------------------------------------------
template <typename PointType>
int vtkPCLVoxelGridFilter::InternalApplyPCLFilter(
  vtkPolyData * input,
  vtkPolyData * output
)
{
  typedef pcl::PointCloud<PointType> CloudT;
  typename CloudT::Ptr inputCloud(new CloudT);
  typename CloudT::Ptr outputCloud(new CloudT);

  vtkPCLConversions::PointCloudFromPolyData(input, inputCloud);

  pcl::VoxelGrid<PointType> filter;
  filter.setInputCloud(inputCloud);
  filter.setLeafSize(this->LeafSize[0], this->LeafSize[1], this->LeafSize[2]);
  filter.filter(* outputCloud);

  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
  return 1;
}

