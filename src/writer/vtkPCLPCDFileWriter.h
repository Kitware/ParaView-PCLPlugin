//=============================================================================
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
//=============================================================================

#ifndef vtkPCLPCDFileWriter_h
#define vtkPCLPCDFileWriter_h

#include "vtkPCLWriter.h"

class VTK_EXPORT vtkPCLPCDFileWriter : public vtkPCLWriter
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLPCDFileWriter * New();
  vtkTypeMacro(vtkPCLPCDFileWriter, vtkPCLWriter);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLPCDFileWriter();
  ~vtkPCLPCDFileWriter();

private:
  vtkPCLPCDFileWriter(const vtkPCLPCDFileWriter&) = delete;
  void operator=(const vtkPCLPCDFileWriter&) = delete;

//------------------------------------------------------------------------------
protected:
  int WritePCL(
    vtkPolyData * output
  ) override;

  template <typename PointType>
  int InternalWritePCL(
    vtkPolyData * output
  );
};

#endif // vtkPCLPCDFileWriter_h

