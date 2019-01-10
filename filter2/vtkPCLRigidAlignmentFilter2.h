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

#ifndef vtkPCLRigidAlignmentFilter2_h
#define vtkPCLRigidAlignmentFilter2_h

#include "vtkPCLFilter2.h"
#include "vtkPCLRegistrationFilter2.h"

class VTK_EXPORT vtkPCLRigidAlignmentFilter2 : public vtkPCLFilter2
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLRigidAlignmentFilter2 * New();
  vtkTypeMacro(vtkPCLRigidAlignmentFilter2, vtkPCLFilter2);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLRigidAlignmentFilter2();
  ~vtkPCLRigidAlignmentFilter2();

private:
  vtkPCLRigidAlignmentFilter2(const vtkPCLRigidAlignmentFilter2&) = delete;
  void operator=(const vtkPCLRigidAlignmentFilter2&) = delete;

//------------------------------------------------------------------------------
// Parameters specific to this filter.
private:
  double NormalRadius;
  double FeatureRadius;

  double DistanceThreshold;
  int MaxIterations;
  double Probability;

  Eigen::Matrix4f TransformationMatrix;
  bool HasTransformation;
  bool ReuseTransformation;

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

  template <
    typename PointType=pcl::PointXYZ, 
    typename NormalType=pcl::Normal, 
    typename FeatureType=pcl::FPFHSignature33
  >
  int InternalApplyPCLFilter2(
    vtkPolyData * input,
    vtkPolyData * reference,
    vtkPolyData * output
  );

};
#endif // vtkPCLRigidAlignmentFilter2_h

