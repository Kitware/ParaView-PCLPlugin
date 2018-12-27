//==============================================================================
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
//==============================================================================

#ifndef vtkPCLReader_h
#define vtkPCLReader_h

#include "vtkAbstractPolyDataReader.h"
#include "vtkObjectFactory.h"

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

//------------------------------------------------------------------------------
//! @brief Common superclass for PCL sources.
class VTK_EXPORT vtkPCLReader : public vtkAbstractPolyDataReader
{
public:
  static vtkPCLReader * New();
  vtkTypeMacro(vtkPCLReader, vtkAbstractPolyDataReader);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:
  vtkPCLReader();
  ~vtkPCLReader();

  int RequestData(
    vtkInformation * request,
    vtkInformationVector * * inputVector,
    vtkInformationVector * outputVector
  ) override;

private:
  vtkPCLReader(const vtkPCLReader &) = delete;
  void operator=(const vtkPCLReader &) = delete;

private:
  /*!
   * @brief      Load the PCL source.
   * @param[out] outputCloud The output cloud produced by the source.
   */
  virtual
  int LoadPCLReader(
    vtkPolyData * output
  ) = 0;
};

#endif // vtkPCLReader_h

