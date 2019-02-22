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

#include "vtkPCLFilter.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"

#include <pcl/point_types.h>

//------------------------------------------------------------------------------
vtkPCLFilter::vtkPCLFilter()
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

//------------------------------------------------------------------------------
vtkPCLFilter::~vtkPCLFilter()
{
}

//------------------------------------------------------------------------------
void vtkPCLFilter::PrintSelf(ostream & os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
int vtkPCLFilter::RequestData(
  vtkInformation * request,
  vtkInformationVector * * inputVector,
  vtkInformationVector * outputVector
)
{
  // Get the polydata and pass it on to the derived classes method.
  vtkPolyData * input = vtkPolyData::GetData(inputVector[0]->GetInformationObject(0));
  vtkPolyData * output = vtkPolyData::GetData(outputVector->GetInformationObject(0));
  return this->ApplyPCLFilter(input, output);
}

