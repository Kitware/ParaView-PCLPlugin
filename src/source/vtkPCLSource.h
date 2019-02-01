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

#ifndef vtkPCLSource_h
#define vtkPCLSource_h

#include "vtkPolyDataAlgorithm.h"

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

//------------------------------------------------------------------------------
//! @brief Common superclass for PCL sources.
class VTK_EXPORT vtkPCLSource : public vtkPolyDataAlgorithm
{
public:
  vtkAbstractTypeMacro(vtkPCLSource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:
  vtkPCLSource();
  ~vtkPCLSource();

  int RequestData(
    vtkInformation * request,
    vtkInformationVector * * inputVector,
    vtkInformationVector * outputVector
  ) override;

private:
  vtkPCLSource(const vtkPCLSource &) = delete;
  void operator=(const vtkPCLSource &) = delete;

private:
  /*!
   * @brief      Load the PCL source.
   * @param[out] outputCloud The output cloud produced by the source.
   */
  virtual
  int LoadPCLSource(
    vtkPolyData * output
  ) = 0;
};

#endif // vtkPCLSource_h

