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

#ifndef vtkPCLIterativeClosestPointFilter2_h
#define vtkPCLIterativeClosestPointFilter2_h

#include "vtkPCLRegistrationFilter2.h"

class VTK_EXPORT vtkPCLIterativeClosestPointFilter2 : public vtkPCLRegistrationFilter2
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLIterativeClosestPointFilter2 * New();
  vtkTypeMacro(vtkPCLIterativeClosestPointFilter2, vtkPCLRegistrationFilter2);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLIterativeClosestPointFilter2();
  ~vtkPCLIterativeClosestPointFilter2();

private:
  vtkPCLIterativeClosestPointFilter2(const vtkPCLIterativeClosestPointFilter2&) = delete;
  void operator=(const vtkPCLIterativeClosestPointFilter2&) = delete;

//------------------------------------------------------------------------------
private:
  bool UseReciprocalCorrespondences = false;

//------------------------------------------------------------------------------
public:
  vtkSetMacro(UseReciprocalCorrespondences, bool);
  vtkGetMacro(UseReciprocalCorrespondences, bool);

//------------------------------------------------------------------------------
private:
  int ApplyPCLFilter2(
    vtkPolyData * input,
    vtkPolyData * target,
    vtkPolyData * output
  ) override;

  template <typename PointType>
  int InternalApplyPCLFilter2(
    vtkPolyData * input,
    vtkPolyData * target,
    vtkPolyData * output
  );

};
#endif // vtkPCLIterativeClosestPointFilter2_h

