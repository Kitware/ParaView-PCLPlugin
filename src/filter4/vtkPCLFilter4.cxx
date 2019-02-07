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

#include "vtkPCLFilter4.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"

#include <pcl/point_types.h>

//------------------------------------------------------------------------------
vtkPCLFilter4::vtkPCLFilter4()
{
  this->SetNumberOfInputPorts(4);
  this->SetNumberOfOutputPorts(1);
}

//------------------------------------------------------------------------------
vtkPCLFilter4::~vtkPCLFilter4()
{
}

//------------------------------------------------------------------------------
void vtkPCLFilter4::PrintSelf(ostream & os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
int vtkPCLFilter4::RequestData(
  vtkInformation * request,
  vtkInformationVector * * inputVector,
  vtkInformationVector * outputVector
)
{
  // Extract the polydata and pass it on to the derived classes method.
  vtkInformation * inInfoA = inputVector[0]->GetInformationObject(0);
  vtkPolyData * inputA(vtkPolyData::SafeDownCast(inInfoA->Get(vtkDataObject::DATA_OBJECT())));

  vtkInformation * inInfoB = inputVector[1]->GetInformationObject(0);
  vtkPolyData * inputB(vtkPolyData::SafeDownCast(inInfoB->Get(vtkDataObject::DATA_OBJECT())));

  vtkInformation * inInfoC = inputVector[2]->GetInformationObject(0);
  vtkPolyData * inputC(vtkPolyData::SafeDownCast(inInfoC->Get(vtkDataObject::DATA_OBJECT())));

  vtkInformation * inInfoD = inputVector[3]->GetInformationObject(0);
  vtkPolyData * inputD(vtkPolyData::SafeDownCast(inInfoD->Get(vtkDataObject::DATA_OBJECT())));

  vtkInformation * outInfo = outputVector->GetInformationObject(0);
  vtkPolyData * output(vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT())));

  return this->ApplyPCLFilter4(inputA, inputB, inputC, inputD, output);
}

