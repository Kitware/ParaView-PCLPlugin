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

#include "vtkPCLReader.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"

#include <pcl/point_types.h>

// vtkStandardNewMacro(vtkPCLReader);

//------------------------------------------------------------------------------
vtkPCLReader::vtkPCLReader()
{
  // TODO
  // Remove this once it's included in vtkAbstractPolyDataReader in the official
  // release.
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
  this->FileName = nullptr;
}

//------------------------------------------------------------------------------
vtkPCLReader::~vtkPCLReader()
{
}

//------------------------------------------------------------------------------
void vtkPCLReader::PrintSelf(ostream & os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
int vtkPCLReader::RequestData(
  vtkInformation * vtkNotUsed(request),
  vtkInformationVector * * inputVector,
  vtkInformationVector * outputVector
)
{
  vtkPolyData * output = vtkPolyData::GetData(outputVector->GetInformationObject(0));
  return this->LoadPCLReader(output);
}

