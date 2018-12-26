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

#ifndef vtkPCLPassThroughFilter_h
#define vtkPCLPassThroughFilter_h

#include "vtkPCLFilter.h"

class VTK_EXPORT vtkPCLPassThroughFilter : public vtkPCLFilter
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLPassThroughFilter * New();
  vtkTypeMacro(vtkPCLPassThroughFilter, vtkPCLFilter);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLPassThroughFilter();
  ~vtkPCLPassThroughFilter();

private:
  vtkPCLPassThroughFilter(const vtkPCLPassThroughFilter&) = delete;
  void operator=(const vtkPCLPassThroughFilter&) = delete;

//------------------------------------------------------------------------------
// Filter parameters.
private:
  std::string FieldName;
  double Limits[2];
  bool Invert;

public:
  vtkSetMacro(FieldName, std::string);
  vtkGetMacro(FieldName, std::string);

  vtkSetVector2Macro(Limits, double);
  vtkGetVector2Macro(Limits, double);

  vtkSetMacro(Invert, bool);
  vtkGetMacro(Invert, bool);

//------------------------------------------------------------------------------
private:
  int ApplyPCLFilter(
    vtkSmartPointer<vtkPolyData> & input,
    vtkSmartPointer<vtkPolyData> & output
  ) override;

  template <typename PointType>
  void InternalApplyPCLFilter(
    vtkSmartPointer<vtkPolyData> & input,
    vtkSmartPointer<vtkPolyData> & output
  );

};
#endif // vtkPCLPassThroughFilter_h

