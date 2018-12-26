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

#ifndef vtkPCLPCDFileSource_h
#define vtkPCLPCDFileSource_h

#include "vtkPCLSource.h"

class VTK_EXPORT vtkPCLPCDFileSource : public vtkPCLSource
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLPCDFileSource * New();
  vtkTypeMacro(vtkPCLPCDFileSource, vtkPCLSource);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLPCDFileSource();
  ~vtkPCLPCDFileSource();

private:
  vtkPCLPCDFileSource(const vtkPCLPCDFileSource&) = delete;
  void operator=(const vtkPCLPCDFileSource&) = delete;

//------------------------------------------------------------------------------
// Source parameters.
private:
  std::string FileName;

public:
  vtkSetMacro(FileName, std::string);
  vtkGetMacro(FileName, std::string);

//------------------------------------------------------------------------------
protected:
  int LoadPCLSource(
    vtkSmartPointer<vtkPolyData> & output
  ) override;

  template <typename PointType>
  int InternalLoadPCLSource(
    vtkSmartPointer<vtkPolyData> & output
  );
};

#endif // vtkPCLPCDFileSource_h

