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

#ifndef vtkPCLSampleConsensusModelRegistrationFilter4_h
#define vtkPCLSampleConsensusModelRegistrationFilter4_h

#include "vtkPCLFilter4.h"

class VTK_EXPORT vtkPCLSampleConsensusModelRegistrationFilter4 : public vtkPCLFilter4
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLSampleConsensusModelRegistrationFilter4 * New();
  vtkTypeMacro(vtkPCLSampleConsensusModelRegistrationFilter4, vtkPCLFilter4);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLSampleConsensusModelRegistrationFilter4();
  ~vtkPCLSampleConsensusModelRegistrationFilter4();

private:
  vtkPCLSampleConsensusModelRegistrationFilter4(const vtkPCLSampleConsensusModelRegistrationFilter4&) = delete;
  void operator=(const vtkPCLSampleConsensusModelRegistrationFilter4&) = delete;

//------------------------------------------------------------------------------
// Parameters specific to this filter.
private:
  double DistanceThreshold;
  int MaxIterations;
  double Probability;

  Eigen::Matrix4f TransformationMatrix;
  bool HasTransformation;
  bool ReuseTransformation;

public:
  vtkGetMacro(DistanceThreshold, double);
  vtkSetMacro(DistanceThreshold, double);

  vtkGetMacro(MaxIterations, int);
  vtkSetMacro(MaxIterations, int);

  vtkGetMacro(Probability, double);
  vtkSetMacro(Probability, double);

  vtkGetMacro(TransformationMatrix, Eigen::Matrix4f);

  vtkGetMacro(HasTransformation, bool);

  vtkGetMacro(ReuseTransformation, bool);
  vtkSetMacro(ReuseTransformation, bool);


  void Reset()
  {
    this->HasTransformation = false;
    this->Modified();
  }

//------------------------------------------------------------------------------
private:
  int ApplyPCLFilter4(
    vtkPolyData * pointsA,
    vtkPolyData * featuresA,
    vtkPolyData * pointsB,
    vtkPolyData * featuresB,
    vtkPolyData * output
  ) override;

  template <typename PointType>
  int InternalApplyPCLFilter4(
    vtkPolyData * pointsA,
    vtkPolyData * featuresA,
    vtkPolyData * pointsB,
    vtkPolyData * featuresB,
    vtkPolyData * output
  );

  template <typename PointType, typename FeatureType>
  int InternalInternalApplyPCLFilter4(
    vtkPolyData * pointsA,
    vtkPolyData * featuresA,
    vtkPolyData * pointsB,
    vtkPolyData * featuresB,
    vtkPolyData * output
  );

};
#endif // vtkPCLSampleConsensusModelRegistrationFilter4_h

