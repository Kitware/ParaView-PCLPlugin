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

#include "vtkPCLPointCloudXYZRGBtoXYZIFilter.h"
#include "vtkPCLConversions.h"
#include "vtkObjectFactory.h"

#include <pcl/point_types_conversion.h>

vtkStandardNewMacro(vtkPCLPointCloudXYZRGBtoXYZIFilter);

//------------------------------------------------------------------------------
vtkPCLPointCloudXYZRGBtoXYZIFilter::vtkPCLPointCloudXYZRGBtoXYZIFilter()
{
}

//------------------------------------------------------------------------------
vtkPCLPointCloudXYZRGBtoXYZIFilter::~vtkPCLPointCloudXYZRGBtoXYZIFilter()
{
}

//------------------------------------------------------------------------------
void vtkPCLPointCloudXYZRGBtoXYZIFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
int vtkPCLPointCloudXYZRGBtoXYZIFilter::ApplyPCLFilter(
  vtkPolyData * input,
  vtkPolyData * output
)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZI>);

  vtkPCLConversions::PointCloudFromPolyData(input, inputCloud);
  pcl::PointCloudXYZRGBtoXYZI((* inputCloud), (* outputCloud));
  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
  
  return 1;
}

