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

#include "vtkPCLRegistrationProxy.h"

template <typename BaseClass>
void vtkPCLRegistrationProxy<BaseClass>::PrintSelf(ostream & os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

template <typename BaseClass>
vtkPCLRegistrationProxy<BaseClass>::vtkPCLRegistrationProxy()
{
  this->MaxCorrespondenceDistance = 0.05;
  this->MaximumIterations = 50;
  this->TransformationEpsilon = 1e-8;
  this->TransformationRotationEpsilon = 1e-8;
  this->EuclideanFitnessEpsilon = 1;
}

template <typename BaseClass>
vtkPCLRegistrationProxy<BaseClass>::~vtkPCLRegistrationProxy()
{
}

