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

#ifndef vtkPCLRegistrationFilter2_txx
#define vtkPCLRegistrationFilter2_txx

#include "vtkPCLRegistrationFilter2.h"

//------------------------------------------------------------------------------
inline
vtkPCLRegistrationFilter2::vtkPCLRegistrationFilter2()
  : MaxCorrespondenceDistance { 0.05 }
  , MaximumIterations { 50 }
  , TransformationEpsilon { 1e-8 }
  , TransformationRotationEpsilon { 1e-8 }
  , EuclideanFitnessEpsilon { 1 }

  , TransformationMatrix { decltype(TransformationMatrix)::Identity() }
  , HasTransformation { false }
  , ReuseTransformation { false }
{
}

//------------------------------------------------------------------------------
inline
vtkPCLRegistrationFilter2::~vtkPCLRegistrationFilter2()
{
}

//------------------------------------------------------------------------------
inline
void vtkPCLRegistrationFilter2::PrintSelf(ostream & os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
template <typename PointT>
bool vtkPCLRegistrationFilter2::MaybeApplyCachedTransformation(
  pcl::PointCloud<PointT> & inputCloud,
  pcl::PointCloud<PointT> & outputCloud
)
{
  if (this->HasTransformation && this->ReuseTransformation)
  {
    pcl::transformPointCloud<PointT>(inputCloud, outputCloud, this->TransformationMatrix);
    return true;
  }
  else
  {
    return false;
  }
}

//------------------------------------------------------------------------------
template <typename PointSource, typename PointTarget, typename Scalar>
void vtkPCLRegistrationFilter2::ConfigureAndAlign(
  pcl::Registration<PointSource, PointTarget, Scalar> & reg,
  pcl::PointCloud<PointSource> & outputCloud
)
{
  reg.setMaxCorrespondenceDistance(this->MaxCorrespondenceDistance);
  reg.setMaximumIterations(this->MaximumIterations);
  reg.setTransformationEpsilon(this->TransformationEpsilon);
  reg.setTransformationRotationEpsilon(this->TransformationRotationEpsilon);
  reg.setEuclideanFitnessEpsilon(this->EuclideanFitnessEpsilon);
  reg.align(outputCloud);
  this->TransformationMatrix = reg.getFinalTransformation();
  std::cout << this->TransformationMatrix << std::endl;
  this->HasTransformation = true;
}

#endif // vtkPCLRegistrationFilter2_txx

