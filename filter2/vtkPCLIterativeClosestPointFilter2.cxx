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

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

vtkStandardNewMacro(vtkPCLIterativeClosestPointFilter2);

//------------------------------------------------------------------------------
vtkPCLIterativeClosestPointFilter2::vtkPCLIterativeClosestPointFilter2()
{
  this->HasTransformation = false;
  this->ReuseTransformation = false;
}

//------------------------------------------------------------------------------
vtkPCLIterativeClosestPointFilter2::~vtkPCLIterativeClosestPointFilter2()
{
}

//------------------------------------------------------------------------------
void vtkPCLIterativeClosestPointFilter2::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
int vtkPCLIterativeClosestPointFilter2::ApplyPCLFilter2(
  vtkPolyData * input,
  vtkPolyData * reference,
  vtkPolyData * output
)
{
  int index = vtkPCLConversions::GetPointTypeIndex(input);
  PCLP_INVOKE_WITH_POINT_TYPE(index, this->InternalApplyPCLFilter2, input, reference, output)
  return 1;
}

//------------------------------------------------------------------------------
template <typename PointType>
void vtkPCLIterativeClosestPointFilter2::InternalApplyPCLFilter2(
  vtkPolyData * input,
  vtkPolyData * reference,
  vtkPolyData * output
)
{
  typedef pcl::PointCloud<PointType> CloudT;
  typename CloudT::Ptr inputCloud(new CloudT);
  typename CloudT::Ptr outputCloud(new CloudT);

  vtkPCLConversions::PointCloudFromPolyData(input, inputCloud);

  if (this->HasTransformation && this->ReuseTransformation)
  {
    pcl::transformPointCloud<PointType>((* inputCloud), (* outputCloud), this->Transformation);
  }
  else
  {
    
    typename CloudT::Ptr referenceCloud(new CloudT);
    vtkPCLConversions::PointCloudFromPolyData(reference, referenceCloud);

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputCloud(inputCloud);
    icp.setInputTarget(referenceCloud);
    // Configure convergence criteria inherited from pcl::Registration.
    this->ConfigureRegistration(& icp);
    icp.align((* outputCloud));
    this->Transformation = icp.getFinalTransformation();
    std::cout << this->Transformation << std::endl;
    this->HasTransformation = true;
    outputCloud = referenceCloud;
  }

  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
}

