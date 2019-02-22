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

#ifndef vtkPCLNormalEstimationFilter_h
#define vtkPCLNormalEstimationFilter_h

#include "vtkPCLFilter.h"

class VTK_EXPORT vtkPCLNormalEstimationFilter : public vtkPCLFilter
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLNormalEstimationFilter * New();
  vtkTypeMacro(vtkPCLNormalEstimationFilter, vtkPCLFilter);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLNormalEstimationFilter();
  ~vtkPCLNormalEstimationFilter();

private:
  vtkPCLNormalEstimationFilter(const vtkPCLNormalEstimationFilter&) = delete;
  void operator=(const vtkPCLNormalEstimationFilter&) = delete;

//------------------------------------------------------------------------------
// Filter parameters.
private:
  double Radius {0.05};
  
  // KdTree attributes
  bool UseKdTree {false};
  float Epsilon {0}; // default value of pcl::search::Kdtree

public:
  vtkSetMacro(Radius, double);
  vtkGetMacro(Radius, double);

  vtkSetMacro(UseKdTree, bool);
  vtkGetMacro(UseKdTree, bool);

  vtkSetMacro(Epsilon, float);
  vtkGetMacro(Epsilon, float);

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
  template <typename PointType, typename NormalPointType>
  int EstimateNormals(
    typename pcl::PointCloud<PointType>::Ptr & inputCloud,
    vtkPolyData * input,
    vtkPolyData * output
  );

};
#endif // vtkPCLNormalEstimationFilter_h

