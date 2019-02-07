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

#include "vtkPCLDummyFilter.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"

vtkStandardNewMacro(vtkPCLDummyFilter);

//------------------------------------------------------------------------------
vtkPCLDummyFilter::vtkPCLDummyFilter()
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

//------------------------------------------------------------------------------
vtkPCLDummyFilter::~vtkPCLDummyFilter()
{
}

//------------------------------------------------------------------------------
void vtkPCLDummyFilter::PrintSelf(ostream & os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Message: " << this->Message << '\n';
}

//------------------------------------------------------------------------------
int vtkPCLDummyFilter::RequestData(
  vtkInformation * request,
  vtkInformationVector * * inputVector,
  vtkInformationVector * outputVector
)
{
  vtkInformation * inInfo = inputVector[0]->GetInformationObject(0);
  vtkPolyData * input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkInformation * outInfo = outputVector->GetInformationObject(0);
  vtkPolyData * output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));
  std::cout << this->Message << '\n';
  output->ShallowCopy(input);
  return 1;
}

