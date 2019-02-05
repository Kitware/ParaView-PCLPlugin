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

#include "vtkPCLStatisticalOutlierRemovalFilter.h"
#include "vtkPCLConversions.h"
#include "vtkObjectFactory.h"

#include <pcl/filters/statistical_outlier_removal.h>

vtkStandardNewMacro(vtkPCLStatisticalOutlierRemovalFilter);

//------------------------------------------------------------------------------
vtkPCLStatisticalOutlierRemovalFilter::vtkPCLStatisticalOutlierRemovalFilter()
{
}

//------------------------------------------------------------------------------
vtkPCLStatisticalOutlierRemovalFilter::~vtkPCLStatisticalOutlierRemovalFilter()
{
}

//------------------------------------------------------------------------------
void vtkPCLStatisticalOutlierRemovalFilter::PrintSelf(ostream & os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "MeanK: " << this->MeanK << '\n';
  os << indent << "StddevMulThresh: " << this->StddevMulThresh << '\n';
}

//------------------------------------------------------------------------------
int vtkPCLStatisticalOutlierRemovalFilter::ApplyPCLFilter(
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
int vtkPCLStatisticalOutlierRemovalFilter::InternalApplyPCLFilter(
  vtkPolyData * input,
  vtkPolyData * output
)
{
  typedef pcl::PointCloud<PointType> CloudT;
  typename CloudT::Ptr inputCloud(new CloudT);
  typename CloudT::Ptr outputCloud(new CloudT);

  vtkPCLConversions::PointCloudFromPolyData(input, inputCloud);

  pcl::StatisticalOutlierRemoval<PointType> filter;
  filter.setInputCloud(inputCloud);
  filter.setMeanK(this->MeanK);
  filter.setStddevMulThresh(this->StddevMulThresh);
  filter.filter(* outputCloud);

  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
  return 1;
}

