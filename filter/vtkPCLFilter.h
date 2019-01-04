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

#ifndef vtkPCLFilter_h
#define vtkPCLFilter_h

#include "vtkPolyDataAlgorithm.h"

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

//------------------------------------------------------------------------------
//! @brief Common superclass for PCL filters.
class VTK_EXPORT vtkPCLFilter : public vtkPolyDataAlgorithm
{
public:
  vtkAbstractTypeMacro(vtkPCLFilter, vtkPolyDataAlgorithm);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:
  vtkPCLFilter();
  ~vtkPCLFilter();

  int RequestData(
    vtkInformation * request,
    vtkInformationVector * * inputVector,
    vtkInformationVector * outputVector
  ) override;

private:
  vtkPCLFilter(const vtkPCLFilter &) = delete;
  void operator=(const vtkPCLFilter &) = delete;

  /*!
   * @brief      Apply the PCL filter.
   * @param[in]  input  The input data.
   * @param[out] output The output data.
   */
  virtual
  int ApplyPCLFilter(
    vtkPolyData * input,
    vtkPolyData * output
  ) = 0;
};

#endif // vtkPCLFilter_h
