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

#ifndef vtkPCLStatisticalOutlierRemovalFilter_h
#define vtkPCLStatisticalOutlierRemovalFilter_h

#include "vtkPCLFilter.h"

class VTK_EXPORT vtkPCLStatisticalOutlierRemovalFilter : public vtkPCLFilter
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLStatisticalOutlierRemovalFilter * New();
  vtkTypeMacro(vtkPCLStatisticalOutlierRemovalFilter, vtkPCLFilter);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLStatisticalOutlierRemovalFilter();
  ~vtkPCLStatisticalOutlierRemovalFilter();

private:
  vtkPCLStatisticalOutlierRemovalFilter(const vtkPCLStatisticalOutlierRemovalFilter&) = delete;
  void operator=(const vtkPCLStatisticalOutlierRemovalFilter&) = delete;

//------------------------------------------------------------------------------
// Filter parameters.
private:
  double MeanK;
  double StddevMulThresh;

public:
  vtkSetMacro(MeanK, double);
  vtkGetMacro(MeanK, double);

  vtkSetMacro(StddevMulThresh, double);
  vtkGetMacro(StddevMulThresh, double);

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
};

#endif // vtkPCLStatisticalOutlierRemovalFilter_h

