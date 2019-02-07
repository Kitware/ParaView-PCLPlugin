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

#ifndef vtkPCLRSDEstimationFilter2_h
#define vtkPCLRSDEstimationFilter2_h

#include "vtkPCLFilter2.h"

// The second input is optional.
class VTK_EXPORT vtkPCLRSDEstimationFilter2 : public vtkPCLFilter2
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLRSDEstimationFilter2 * New();
  vtkTypeMacro(vtkPCLRSDEstimationFilter2, vtkPCLFilter2);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLRSDEstimationFilter2();
  ~vtkPCLRSDEstimationFilter2();

private:
  vtkPCLRSDEstimationFilter2(const vtkPCLRSDEstimationFilter2&) = delete;
  void operator=(const vtkPCLRSDEstimationFilter2&) = delete;

//------------------------------------------------------------------------------
// Filter2 parameters.
private:
  typedef pcl::PrincipalRadiiRSD FeatureType;
  typedef pcl::PointCloud<FeatureType> FeatureCloudT;

  // TODO
  // Maybe add NrSubdivisions. Supporting this will require supporting arbitrary
  // Histogram<N> types up to some limit, where N = max(NrSubdivisions)^2.
  double PlaneRadius {0.5};
  double RadiusSearch {0.05};
  bool SaveHistograms {false};

public:
  vtkSetMacro(PlaneRadius, double);
  vtkGetMacro(PlaneRadius, double);

  vtkSetMacro(RadiusSearch, double);
  vtkGetMacro(RadiusSearch, double);
  
  vtkSetMacro(SaveHistograms, bool);
  vtkGetMacro(SaveHistograms, bool);

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
#endif // vtkPCLRSDEstimationFilter2_h

