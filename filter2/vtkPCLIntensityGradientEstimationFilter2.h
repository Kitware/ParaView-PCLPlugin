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

#ifndef vtkPCLIntensityGradientEstimationFilter2_h
#define vtkPCLIntensityGradientEstimationFilter2_h

#include "vtkPCLFilter2.h"
#include "vtkPCLRegistrationFilter2.h"

class VTK_EXPORT vtkPCLIntensityGradientEstimationFilter2 : public vtkPCLRegistrationFilter2
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLIntensityGradientEstimationFilter2 * New();
  vtkTypeMacro(vtkPCLIntensityGradientEstimationFilter2, vtkPCLRegistrationFilter2);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLIntensityGradientEstimationFilter2();
  ~vtkPCLIntensityGradientEstimationFilter2();

private:
  vtkPCLIntensityGradientEstimationFilter2(const vtkPCLIntensityGradientEstimationFilter2&) = delete;
  void operator=(const vtkPCLIntensityGradientEstimationFilter2&) = delete;

//------------------------------------------------------------------------------
// Parameters specific to this filter.
private:
  float RadiusSearch {0.10};

public:
  vtkGetMacro(RadiusSearch, float);
  vtkSetMacro(RadiusSearch, float);

//------------------------------------------------------------------------------
private:
  int ApplyPCLFilter2(
    vtkPolyData * intensities,
    vtkPolyData * normals,
    vtkPolyData * output
  ) override;

  template <typename IPointType>
  int InternalApplyPCLFilter2(
    vtkPolyData * intensities,
    vtkPolyData * normals,
    vtkPolyData * output
  );

  template <typename IPointType, typename NPointType>
  int InternalInternalApplyPCLFilter2(
    vtkPolyData * intensities,
    vtkPolyData * normals,
    vtkPolyData * output
  );

};
#endif // vtkPCLIntensityGradientEstimationFilter2_h

