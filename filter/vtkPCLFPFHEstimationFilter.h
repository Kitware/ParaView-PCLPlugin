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

#ifndef vtkPCLFPFHEstimationFilter_h
#define vtkPCLFPFHEstimationFilter_h

#include "vtkPCLFilter.h"

class VTK_EXPORT vtkPCLFPFHEstimationFilter : public vtkPCLFilter
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLFPFHEstimationFilter * New();
  vtkTypeMacro(vtkPCLFPFHEstimationFilter, vtkPCLFilter);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLFPFHEstimationFilter();
  ~vtkPCLFPFHEstimationFilter();

private:
  vtkPCLFPFHEstimationFilter(const vtkPCLFPFHEstimationFilter&) = delete;
  void operator=(const vtkPCLFPFHEstimationFilter&) = delete;

//------------------------------------------------------------------------------
// Filter parameters.
private:
  double Radius;

public:
  vtkSetMacro(Radius, double);
  vtkGetMacro(Radius, double);

//------------------------------------------------------------------------------
private:
  int ApplyPCLFilter(
    vtkPolyData * input,
    vtkPolyData * output
  ) override;

  template <typename PointType>
  int InternalApplyPCLFilter(
    vtkPolyData * input,
    vtkPolyData * output
  );

  // The first argument is just for template parameter deduction.
  template <typename PointType, typename FPFHPointType>
  int EstimateFPFHs(
    typename pcl::PointCloud<PointType>::Ptr & inputCloud,
    vtkPolyData * input,
    vtkPolyData * output
  );

};
#endif // vtkPCLFPFHEstimationFilter_h

