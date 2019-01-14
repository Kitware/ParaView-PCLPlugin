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

#ifndef vtkPCLFilter4_h
#define vtkPCLFilter4_h

#include "vtkPolyDataAlgorithm.h"

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

//------------------------------------------------------------------------------
//! @brief Common superclass for PCL filters.
class VTK_EXPORT vtkPCLFilter4 : public vtkPolyDataAlgorithm
{
public:
  vtkAbstractTypeMacro(vtkPCLFilter4, vtkPolyDataAlgorithm);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:
  vtkPCLFilter4();
  ~vtkPCLFilter4();

  int RequestData(
    vtkInformation * request,
    vtkInformationVector * * inputVector,
    vtkInformationVector * outputVector
  ) override;

private:
  vtkPCLFilter4(const vtkPCLFilter4 &) = delete;
  void operator=(const vtkPCLFilter4 &) = delete;

  /*!
   * @brief      Apply the PCL filter.
   * @param[in]  input  The input data.
   * @param[out] output The output data.
   */
  virtual
  int ApplyPCLFilter4(
    vtkPolyData * inputA,
    vtkPolyData * inputB,
    vtkPolyData * inputC,
    vtkPolyData * inputD,
    vtkPolyData * output
  ) = 0;
};

#endif // vtkPCLFilter4_h

