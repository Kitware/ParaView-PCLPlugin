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

#include "vtkPCLIntensityGradientEstimationFilter2.h"
#include "vtkPCLConversions.h"
#include "FeatureExtractor.h"

#include "vtkObjectFactory.h"

#include <pcl/features/intensity_gradient.h>



//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLIntensityGradientEstimationFilter2);

//------------------------------------------------------------------------------
vtkPCLIntensityGradientEstimationFilter2::vtkPCLIntensityGradientEstimationFilter2()
{
}

//------------------------------------------------------------------------------
vtkPCLIntensityGradientEstimationFilter2::~vtkPCLIntensityGradientEstimationFilter2()
{
}

//------------------------------------------------------------------------------
void vtkPCLIntensityGradientEstimationFilter2::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
int vtkPCLIntensityGradientEstimationFilter2::ApplyPCLFilter2(
  vtkPolyData * intensities,
  vtkPolyData * normals,
  vtkPolyData * output
)
{
  // TODO
  // Decide how to support other types (e.g. by defining a preprocessor sequence
  // of intensity types?).
  return this->InternalApplyPCLFilter2<pcl::PointXYZI>(intensities, normals, output);
//   std::set<std::string> requiredFieldNames { "intensity" };
//   int index = vtkPCLConversions::GetPointTypeIndex(intensities, requiredFieldNames);
// #define _statement(PointType) return this->InternalApplyPCLFilter2<PointType>(intensities, normals, output);
//   PCLP_INVOKE_WITH_PCL_XYZ_POINT_TYPE(index, _statement)
// #undef _statement
//   vtkErrorMacro(<< "intensity data does not contain intensity values")
//   return 0;
}

//------------------------------------------------------------------------------
template <typename IPointType>
int vtkPCLIntensityGradientEstimationFilter2::InternalApplyPCLFilter2(
  vtkPolyData * intensities,
  vtkPolyData * normals,
  vtkPolyData * output
)
{
  // No need for required fields here due to PCLP_INVOKE_WITH_PCL_NORMAL_POINT_TYPE.
  int index = vtkPCLConversions::GetPointTypeIndex(normals);
#define _statement(NPointType) return this->InternalInternalApplyPCLFilter2<IPointType, NPointType>(intensities, normals, output);
  PCLP_INVOKE_WITH_PCL_NORMAL_POINT_TYPE(index, _statement)
#undef _statement
  vtkErrorMacro(<< "normal data input does not contain normal components")
  return 0;
}

//------------------------------------------------------------------------------
template <typename IPointType, typename NPointType>
int vtkPCLIntensityGradientEstimationFilter2::InternalInternalApplyPCLFilter2(
  vtkPolyData * intensities,
  vtkPolyData * normals,
  vtkPolyData * output
)
{
  typedef pcl::PointCloud<IPointType> ICloudT;
  typedef pcl::PointCloud<NPointType> NCloudT;
  typedef pcl::PointCloud<pcl::IntensityGradient> IGCloudT;

  typename ICloudT::Ptr intensityCloud(new ICloudT);
  typename NCloudT::Ptr normalCloud(new NCloudT);
  typename IGCloudT::Ptr outputCloud(new IGCloudT);
  
  vtkPCLConversions::PointCloudFromPolyData(intensities, intensityCloud);
  vtkPCLConversions::PointCloudFromPolyData(normals, normalCloud);

  pcl::IntensityGradientEstimation<
    IPointType, 
    NPointType, 
    pcl::IntensityGradient,
    pcl::common::IntensityFieldAccessor<IPointType>
  > ige;
  ige.setInputCloud(intensityCloud);
  ige.setInputNormals(normalCloud);
  ige.setRadiusSearch(this->RadiusSearch);
  ige.compute((* outputCloud));

  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
  return 1;
}

