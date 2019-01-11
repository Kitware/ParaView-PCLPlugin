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
// Filter parameters and their getters/setters. These are exposed through the
// ServerManager proxy.
private:
  char * FieldName;
  double Limits[2];
  bool Invert;

public:
  vtkSetStringMacro(FieldName);
  vtkGetStringMacro(FieldName);

  vtkSetVector2Macro(Limits, double);
  vtkGetVector2Macro(Limits, double);

  vtkSetMacro(Invert, bool);
  vtkGetMacro(Invert, bool);

//------------------------------------------------------------------------------
private:
  /*!
   * @brief      Apply the filter to the input data to generate the output data.
   * @param[in]  input  The input data.
   * @param[out] output The output data.
   *
   * This method has to be virtual so that it can be called by the base class.
   * As virtual methods cannot be templated, a second method
   * (InternalApplyPCLFilter) is therefore required to handle all of the
   * different point types.
   */
  int ApplyPCLFilter(
    vtkPolyData * input,
    vtkPolyData * output
  ) override;

  /*!
   * @brief Apply the filter for any point type.
   * @param[in]  input  The input data.
   * @param[out] output The output data.
   *
   * The passthrough filter can work on any point type so the application is
   * templated. A generic template might not make sense for other filters that
   * only handle specific types. In that case, specializations or overrides
   * would likely make more sense.
   */
  template <typename PointType>
  int InternalApplyPCLFilter(
    vtkPolyData * input,
    vtkPolyData * output
  );

};
#endif // vtkPCLPassThroughFilter_h

