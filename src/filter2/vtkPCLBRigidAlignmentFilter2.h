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

#ifndef vtkPCLBRigidAlignmentFilter2_h
#define vtkPCLBRigidAlignmentFilter2_h

#include "vtkPCLFilter2.h"
#include "vtkPCLRegistrationFilter2.h"

class VTK_EXPORT vtkPCLBRigidAlignmentFilter2 : public vtkPCLFilter2
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLBRigidAlignmentFilter2 * New();
  vtkTypeMacro(vtkPCLBRigidAlignmentFilter2, vtkPCLFilter2);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLBRigidAlignmentFilter2();
  ~vtkPCLBRigidAlignmentFilter2();

private:
  vtkPCLBRigidAlignmentFilter2(const vtkPCLBRigidAlignmentFilter2&) = delete;
  void operator=(const vtkPCLBRigidAlignmentFilter2&) = delete;

//------------------------------------------------------------------------------
// Parameters specific to this filter.
private:
  double NormalRadius {0.02};
  double FeatureRadius {0.02};

  double DistanceThreshold {0.02};
  int MaxIterations {2500};
  double Probability {1.0};

  Eigen::Matrix4f TransformationMatrix {Eigen::Matrix4f::Identity()};
  bool HasTransformation {false};
  bool ReuseTransformation {false};

public:
  vtkGetMacro(NormalRadius, double);
  vtkSetMacro(NormalRadius, double);

  vtkGetMacro(FeatureRadius, double);
  vtkSetMacro(FeatureRadius, double);

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
  int ApplyPCLFilter2(
    vtkPolyData * input,
    vtkPolyData * reference,
    vtkPolyData * output
  ) override;

  template <typename PointT, typename NormalT, typename FeatureT>
  int InternalApplyPCLFilter2(
    vtkPolyData * input,
    vtkPolyData * reference,
    vtkPolyData * output
  );

};
#endif // vtkPCLBRigidAlignmentFilter2_h

