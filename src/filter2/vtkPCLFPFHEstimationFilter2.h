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

#ifndef vtkPCLFPFHEstimationFilter2_h
#define vtkPCLFPFHEstimationFilter2_h

#include "vtkPCLFilter2.h"

// The second input is optional.
class VTK_EXPORT vtkPCLFPFHEstimationFilter2 : public vtkPCLFilter2
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLFPFHEstimationFilter2 * New();
  vtkTypeMacro(vtkPCLFPFHEstimationFilter2, vtkPCLFilter2);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLFPFHEstimationFilter2();
  ~vtkPCLFPFHEstimationFilter2();

private:
  vtkPCLFPFHEstimationFilter2(const vtkPCLFPFHEstimationFilter2&) = delete;
  void operator=(const vtkPCLFPFHEstimationFilter2&) = delete;

//------------------------------------------------------------------------------
// Filter2 parameters.
private:
  typedef pcl::FPFHSignature33 FeatureType;
  typedef pcl::PointCloud<FeatureType> FeatureCloudT;
  double Radius {0.07};

  // KdTree attributes
  bool UseKdTree {false};
  float Epsilon {0};

public:
  vtkSetMacro(Radius, double);
  vtkGetMacro(Radius, double);

  vtkSetMacro(UseKdTree, bool);
  vtkGetMacro(UseKdTree, bool);

  vtkSetMacro(Epsilon, float);
  vtkGetMacro(Epsilon, float);

//------------------------------------------------------------------------------
private:
  int ApplyPCLFilter2(
    vtkPolyData * points,
    vtkPolyData * normals,
    vtkPolyData * features
  ) override;

  template <typename PointType>
  int InternalApplyPCLFilter2(
    vtkPolyData * points,
    vtkPolyData * features
  );

  template <typename PointType, typename NormalType>
  int InternalApplyPCLFilter2(
    vtkPolyData * points,
    vtkPolyData * normals,
    vtkPolyData * features
  );

  template <typename T>
  int ComputeFeatures(
    T estimator,
    vtkPolyData * features
  );
};
#endif // vtkPCLFPFHEstimationFilter2_h

