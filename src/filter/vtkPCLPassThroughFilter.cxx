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

#include "vtkPCLPassThroughFilter.h"
#include "vtkPCLConversions.h"
#include "vtkObjectFactory.h"

#include <pcl/filters/passthrough.h>

vtkStandardNewMacro(vtkPCLPassThroughFilter);

//------------------------------------------------------------------------------
vtkPCLPassThroughFilter::vtkPCLPassThroughFilter()
{
}

//------------------------------------------------------------------------------
vtkPCLPassThroughFilter::~vtkPCLPassThroughFilter()
{
}

//------------------------------------------------------------------------------
void vtkPCLPassThroughFilter::PrintSelf(ostream & os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "FilterFieldName: " << (this->FilterFieldName == nullptr ? "" : this->FilterFieldName) << '\n';
  os << indent << "FilterLimits: " << this->FilterLimits[0] << " - " << this->FilterLimits[1] << '\n';
  os << indent << "FilterLimitsNegative: " << (this->FilterLimitsNegative ? "yes" : "no") << '\n';
}

//------------------------------------------------------------------------------
int vtkPCLPassThroughFilter::ApplyPCLFilter(
  vtkPolyData * input,
  vtkPolyData * output
)
{
  // Determine the best matching point type for the input data. This inspects
  // the data embedded in the PolyData instance and returns an index that is
  // unique to a point type.
  int index = vtkPCLConversions::GetPointTypeIndex(input);

  // Define a macro that accepts a point type. This will be used to generate
  // switch statements for all point indices. The statement that corresponds to
  // the point type index found above will be executed. Note the return type.
  // This is important for propagating filter errors back to RequestData.
#define _statement(PointType) return this->InternalApplyPCLFilter<PointType>(input, output);
  
  // Generate the switch statement.
  // TODO: Check which non-XYZ point types are supported by this filter, if any.
  PCLP_INVOKE_WITH_PCL_XYZ_POINT_TYPE(index, _statement)

  // Undefine the statement. This is not necessary here but it is good practice
  // to avoid conflicting definitions in other cases.
#undef _statement

  // If control reaches this point then no switch statement was executed and no
  // filter was applied. Return 0 to indicate this.
  vtkErrorMacro(<< "no XYZ point data in input")
  return 0;
}

//------------------------------------------------------------------------------
// Apply the filter to any point type.
template <typename PointType>
int vtkPCLPassThroughFilter::InternalApplyPCLFilter(
  vtkPolyData * input,
  vtkPolyData * output
)
{
  // For convenience, typedef the cloud type based on the template point type.
  typedef pcl::PointCloud<PointType> CloudT;

  // Create the input and output cloud.
  typename CloudT::Ptr inputCloud(new CloudT);
  typename CloudT::Ptr outputCloud(new CloudT);

  // Convert the input PolyData instance to a point cloud.
  vtkPCLConversions::PointCloudFromPolyData(input, inputCloud);

  // Apply the filter using pure PCL code.
  pcl::PassThrough<PointType> filter;
  filter.setInputCloud(inputCloud);
  filter.setFilterFieldName(this->FilterFieldName);
  filter.setFilterLimits(this->FilterLimits[0], this->FilterLimits[1]);
  filter.setFilterLimitsNegative(this->FilterLimitsNegative);
  filter.filter(* outputCloud);

  // Convert the result to the output PolyData instance.
  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
  
  // Return 1 to indicate success. If a filter fails, it should return 0.
  return 1;
}

