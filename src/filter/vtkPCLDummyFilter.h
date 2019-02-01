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

#ifndef vtkPCLDummyFilter_h
#define vtkPCLDummyFilter_h

#include "vtkPolyDataAlgorithm.h"

//------------------------------------------------------------------------------
//! @brief Dummy filter for basic testing.
class VTK_EXPORT vtkPCLDummyFilter : public vtkPolyDataAlgorithm
{
public:
  static vtkPCLDummyFilter * New();
  vtkTypeMacro(vtkPCLDummyFilter, vtkPolyDataAlgorithm);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:
  vtkPCLDummyFilter();
  ~vtkPCLDummyFilter();

  int RequestData(
    vtkInformation * request,
    vtkInformationVector * * inputVector,
    vtkInformationVector * outputVector
  ) override;

private:
  vtkPCLDummyFilter(const vtkPCLDummyFilter &) = delete;
  void operator=(const vtkPCLDummyFilter &) = delete;

private:
  std::string Message;

public:
  vtkSetMacro(Message, std::string);
  vtkGetMacro(Message, std::string);
};

#endif // vtkPCLDummyFilter_h

