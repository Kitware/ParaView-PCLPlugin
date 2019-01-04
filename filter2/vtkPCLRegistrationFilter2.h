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

#ifndef vtkPCLRegistrationFilter2_h
#define vtkPCLRegistrationFilter2_h

#include "vtkPCLFilter2.h"

#include <pcl/registration/registration.h>

class VTK_EXPORT vtkPCLRegistrationFilter2 : public vtkPCLFilter2
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  vtkAbstractTypeMacro(vtkPCLRegistrationFilter2, vtkPCLFilter2);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLRegistrationFilter2();
  ~vtkPCLRegistrationFilter2();

private:
  vtkPCLRegistrationFilter2(const vtkPCLRegistrationFilter2&) = delete;
  void operator=(const vtkPCLRegistrationFilter2&) = delete;

//------------------------------------------------------------------------------
// Filter parameters.
private:
  double MaxCorrespondenceDistance;
  unsigned int MaximumIterations;
  double TransformationEpsilon;
  double TransformationRotationEpsilon;
  double EuclideanFitnessEpsilon;

public:
  vtkGetMacro(MaxCorrespondenceDistance, double);
  vtkSetMacro(MaxCorrespondenceDistance, double);

  vtkGetMacro(MaximumIterations, unsigned int);
  vtkSetMacro(MaximumIterations, unsigned int);

  vtkGetMacro(TransformationEpsilon, double);
  vtkSetMacro(TransformationEpsilon, double);

  vtkGetMacro(TransformationRotationEpsilon, double);
  vtkSetMacro(TransformationRotationEpsilon, double);

  vtkGetMacro(EuclideanFitnessEpsilon, double);
  vtkSetMacro(EuclideanFitnessEpsilon, double);

  template <typename PointSource, typename PointTarget, typename Scalar>
  void ConfigureRegistration(pcl::Registration<PointSource, PointTarget, Scalar> * reg)
  {
    reg->setMaxCorrespondenceDistance(this->MaxCorrespondenceDistance);
    reg->setMaximumIterations(this->MaximumIterations);
    reg->setTransformationEpsilon(this->TransformationEpsilon);
    reg->setTransformationRotationEpsilon(this->TransformationRotationEpsilon);
    reg->setEuclideanFitnessEpsilon(this->EuclideanFitnessEpsilon);
  }
};

#endif // vtkPCLRegistrationFilter2_h

