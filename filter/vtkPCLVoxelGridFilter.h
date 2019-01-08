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

#ifndef vtkPCLVoxelGridFilter_h
#define vtkPCLVoxelGridFilter_h

#include "vtkPCLFilter.h"

class VTK_EXPORT vtkPCLVoxelGridFilter : public vtkPCLFilter
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLVoxelGridFilter * New();
  vtkTypeMacro(vtkPCLVoxelGridFilter, vtkPCLFilter);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLVoxelGridFilter();
  ~vtkPCLVoxelGridFilter();

private:
  vtkPCLVoxelGridFilter(const vtkPCLVoxelGridFilter&) = delete;
  void operator=(const vtkPCLVoxelGridFilter&) = delete;

//------------------------------------------------------------------------------
// Filter parameters.
private:
  float LeafSize[3];

public:
  vtkSetVector3Macro(LeafSize, float);
  vtkGetVector3Macro(LeafSize, float);

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
};

#endif // vtkPCLVoxelGridFilter_h

