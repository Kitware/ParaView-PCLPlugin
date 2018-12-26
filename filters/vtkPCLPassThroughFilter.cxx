//=============================================================================
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
//=============================================================================

#include "vtkPCLPassThroughFilter.h"
#include "vtkPCLConversions.h"
#include "_PCLInvokeWithPointType.h"

#include <pcl/filters/passthrough.h>

vtkStandardNewMacro(vtkPCLPassThroughFilter);

//----------------------------------------------------------------------------
vtkPCLPassThroughFilter::vtkPCLPassThroughFilter()
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
  this->SetFieldName("x");
  this->Limits[0] = 0.0;
  this->Limits[1] = 1.0;
  this->Invert = false;
}

//----------------------------------------------------------------------------
vtkPCLPassThroughFilter::~vtkPCLPassThroughFilter()
{
}

//----------------------------------------------------------------------------
void vtkPCLPassThroughFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
int vtkPCLPassThroughFilter::ApplyPCLFilter(
  vtkPolyData * input,
  vtkPolyData * output
)
{
  int index = vtkPCLConversions::GetPointTypeIndex(input);
  INVOKE_WITH_POINT_TYPE(index, this->InternalApplyPCLFilter, input, output)
  return 1;
}

//----------------------------------------------------------------------------
template <typename PointType>
void vtkPCLPassThroughFilter::InternalApplyPCLFilter(
  vtkPolyData * input,
  vtkPolyData * output
)
{
  typedef pcl::PointCloud<PointType> CloudT;
  typename CloudT::Ptr inputCloud(new CloudT);
  typename CloudT::Ptr outputCloud(new CloudT);

  vtkPCLConversions::PointCloudFromPolyData(input, inputCloud);

  pcl::PassThrough<PointType> filter;
  filter.setInputCloud(inputCloud);
  filter.setFilterFieldName(this->FieldName);
  filter.setFilterLimits(this->Limits[0], this->Limits[1]);
  filter.setFilterLimitsNegative(this->Invert);
  filter.filter(* outputCloud);
  
  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
}

