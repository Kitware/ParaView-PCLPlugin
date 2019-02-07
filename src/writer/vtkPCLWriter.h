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

#ifndef vtkPCLWriter_h
#define vtkPCLWriter_h

#include "vtkWriter.h"
#include "vtkPolyData.h"

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

//------------------------------------------------------------------------------
//! @brief Common superclass for PCL sources.
class VTK_EXPORT vtkPCLWriter : public vtkWriter
{
public:
  vtkAbstractTypeMacro(vtkPCLWriter, vtkWriter);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:
  vtkPCLWriter();
  ~vtkPCLWriter();

  void WriteData() override;
  int FillInputPortInformation(int port, vtkInformation * info) override;

private:
  vtkPCLWriter(const vtkPCLWriter &) = delete;
  void operator=(const vtkPCLWriter &) = delete;

// TODO: remove this once it's included in vtkWriter
protected:
  char * FileName;
public:
  vtkSetStringMacro(FileName);
  vtkGetStringMacro(FileName);

private:
  /*!
   * @brief      Load the PCL source.
   * @param[out] outputCloud The output cloud produced by the source.
   */
  virtual
  int WritePCL(
    vtkPolyData * input
  ) = 0;
};

#endif // vtkPCLWriter_h

