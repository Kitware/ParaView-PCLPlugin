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

#ifndef vtkPCLRegistrationProxy_h
#define vtkPCLRegistrationProxy_h

#include <pcl/registration/registration.h>

/*!
 * @brief  Templated base class for all filters based on subclasses of
 *         pcl::Register.
 * @tparam BaseClass The vtk base class (e.g. vtkPCLFilter).
 *
 * This subclass exposes all options recognized by pcl::Register. Derived
 * classes should declare "PCLRegistrationProxy" as a base proxy in their server
 * manager source proxy to expose the options via the ParaView user interface.
 */
template <typename BaseClass>
class VTK_EXPORT vtkPCLRegistrationProxy : public BaseClass
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLRegistrationProxy * New();
  vtkAbstractTypeMacro(vtkPCLRegistrationProxy, BaseClass);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLRegistrationProxy();
  ~vtkPCLRegistrationProxy();

private:
  vtkPCLRegistrationProxy(const vtkPCLRegistrationProxy&) = delete;
  void operator=(const vtkPCLRegistrationProxy&) = delete;

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



#include "vtkPCLRegistrationProxy.txx"

#endif // vtkPCLRegistrationProxy_h

