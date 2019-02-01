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

#ifndef vtkPCLSampleConsensusInitialAlignmentFilter2_h
#define vtkPCLSampleConsensusInitialAlignmentFilter2_h

#include "vtkPCLFilter2.h"
#include "vtkPCLRegistrationFilter2.h"

class VTK_EXPORT vtkPCLSampleConsensusInitialAlignmentFilter2 : public vtkPCLRegistrationFilter2
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLSampleConsensusInitialAlignmentFilter2 * New();
  vtkTypeMacro(vtkPCLSampleConsensusInitialAlignmentFilter2, vtkPCLRegistrationFilter2);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLSampleConsensusInitialAlignmentFilter2();
  ~vtkPCLSampleConsensusInitialAlignmentFilter2();

private:
  vtkPCLSampleConsensusInitialAlignmentFilter2(const vtkPCLSampleConsensusInitialAlignmentFilter2&) = delete;
  void operator=(const vtkPCLSampleConsensusInitialAlignmentFilter2&) = delete;

//------------------------------------------------------------------------------
// Parameters specific to this filter.
private:
  float MinSampleDistance {0.05};
  float NormalRadius {0.05};
  float FeatureRadius {0.07};

public:
  vtkGetMacro(MinSampleDistance, float);
  vtkSetMacro(MinSampleDistance, float);

  vtkGetMacro(NormalRadius, float);
  vtkSetMacro(NormalRadius, float);

  vtkGetMacro(FeatureRadius, float);
  vtkSetMacro(FeatureRadius, float);

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
#endif // vtkPCLSampleConsensusInitialAlignmentFilter2_h

