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

#include "vtkPCLIterativeClosestPointFilter2.h"
#include "vtkPCLConversions.h"

#include "vtkObjectFactory.h"

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>



//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLIterativeClosestPointFilter2);

//------------------------------------------------------------------------------
vtkPCLIterativeClosestPointFilter2::vtkPCLIterativeClosestPointFilter2()
{
}

//------------------------------------------------------------------------------
vtkPCLIterativeClosestPointFilter2::~vtkPCLIterativeClosestPointFilter2()
{
}

//------------------------------------------------------------------------------
void vtkPCLIterativeClosestPointFilter2::PrintSelf(ostream & os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "UseReciprocalCorrespondences: " << (this->UseReciprocalCorrespondences ? "yes" : "no") << '\n';
}

//------------------------------------------------------------------------------
int vtkPCLIterativeClosestPointFilter2::ApplyPCLFilter2(
  vtkPolyData * input,
  vtkPolyData * target,
  vtkPolyData * output
)
{
  int index = vtkPCLConversions::GetPointTypeIndex(input);
#define _statement(PointType) return this->InternalApplyPCLFilter2<PointType>(input, target, output);
  PCLP_INVOKE_WITH_PCL_XYZ_POINT_TYPE(index, _statement)
#undef _statement
  vtkErrorMacro(<< "no XYZ point data in input")
  return 0;
}

//------------------------------------------------------------------------------
template <typename PointType>
int vtkPCLIterativeClosestPointFilter2::InternalApplyPCLFilter2(
  vtkPolyData * input,
  vtkPolyData * target,
  vtkPolyData * output
)
{
  typedef pcl::PointCloud<PointType> CloudT;
  typename CloudT::Ptr inputCloud(new CloudT);
  typename CloudT::Ptr outputCloud(new CloudT);

  vtkPCLConversions::PointCloudFromPolyData(input, inputCloud);

  // Perform the alignment if a cached transformation was not applied.
  if (! this->MaybeApplyCachedTransformation((* inputCloud), (* outputCloud)))
  {
    // Set the input and target clouds for the Iterative Closest Point (ICP)
    // filter.
    typename CloudT::Ptr targetCloud(new CloudT);
    vtkPCLConversions::PointCloudFromPolyData(target, targetCloud);

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputCloud(inputCloud);
    icp.setInputTarget(targetCloud);
    icp.setUseReciprocalCorrespondences(this->UseReciprocalCorrespondences);

    // Set common registration options and align.
    this->ConfigureAndAlign<PointType,PointType>(icp, (* outputCloud));
  }

  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
  return 1;
}

