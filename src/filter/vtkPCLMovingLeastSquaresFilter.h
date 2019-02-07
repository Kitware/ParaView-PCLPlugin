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

#ifndef vtkPCLMovingLeastSquaresFilter_h
#define vtkPCLMovingLeastSquaresFilter_h

#include "vtkPCLFilter.h"

// TODO
// Make this a Filter2 class with optional input for the distinct cloud.
class VTK_EXPORT vtkPCLMovingLeastSquaresFilter : public vtkPCLFilter
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLMovingLeastSquaresFilter * New();
  vtkTypeMacro(vtkPCLMovingLeastSquaresFilter, vtkPCLFilter);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLMovingLeastSquaresFilter();
  ~vtkPCLMovingLeastSquaresFilter();

private:
  vtkPCLMovingLeastSquaresFilter(const vtkPCLMovingLeastSquaresFilter&) = delete;
  void operator=(const vtkPCLMovingLeastSquaresFilter&) = delete;

//------------------------------------------------------------------------------
private:
  // Keep these values in sync with the defaults in pcl/surface/mls.h.
  bool         ComputeNormals         = false;
  int          PolynomialOrder        = 2;
  // bool         PolynomialFit          = false;
  double       SearchRadius           = 0.0;
  double       SqrGaussParam          = 0.0;
  double       UpsamplingRadius       = 0.0;
  double       UpsamplingStepSize     = 0.0;
  int          PointDensity           = 0;
  float        DilationVoxelSize      = 1.0;
  int          DilationIterations     = 0;
  bool         CacheMLSResults        = true;
  unsigned int NumberOfThreads        = 1;

  unsigned int UpsamplingMethod = 0;
	unsigned int ProjectionMethod = 1;

public:
  vtkSetMacro(ComputeNormals, bool);
  vtkGetMacro(ComputeNormals, bool);

  vtkSetMacro(PolynomialOrder, int);
  vtkGetMacro(PolynomialOrder, int);

  // vtkSetMacro(PolynomialFit, bool);
  // vtkGetMacro(PolynomialFit, bool);

  vtkSetMacro(SearchRadius, double);
  vtkGetMacro(SearchRadius, double);

  vtkSetMacro(SqrGaussParam, double);
  vtkGetMacro(SqrGaussParam, double);

  vtkSetMacro(UpsamplingRadius, double);
  vtkGetMacro(UpsamplingRadius, double);

  vtkSetMacro(UpsamplingStepSize, double);
  vtkGetMacro(UpsamplingStepSize, double);

  vtkSetMacro(PointDensity, int);
  vtkGetMacro(PointDensity, int);

  vtkSetMacro(DilationVoxelSize, float);
  vtkGetMacro(DilationVoxelSize, float);

  vtkSetMacro(DilationIterations, int);
  vtkGetMacro(DilationIterations, int);

  vtkSetMacro(CacheMLSResults, bool);
  vtkGetMacro(CacheMLSResults, bool);

  vtkSetMacro(NumberOfThreads, unsigned int);
  vtkGetMacro(NumberOfThreads, unsigned int);

  vtkSetMacro(UpsamplingMethod, unsigned int);
  vtkGetMacro(UpsamplingMethod, unsigned int);

  vtkSetMacro(ProjectionMethod, unsigned int);
  vtkGetMacro(ProjectionMethod, unsigned int);

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

  template <typename InPointType, typename OutPointType>
  int InternalInternalApplyPCLFilter(
    vtkPolyData * input,
    vtkPolyData * output
  );

};
#endif // vtkPCLMovingLeastSquaresFilter_h

