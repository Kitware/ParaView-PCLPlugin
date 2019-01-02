//==============================================================================
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
//==============================================================================

#ifndef vtkPCLOpenNISource_h
#define vtkPCLOpenNISource_h

#include "vtkPCLSource.h"

class VTK_EXPORT vtkPCLOpenNISource : public vtkPCLSource
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLOpenNISource * New();
  vtkTypeMacro(vtkPCLOpenNISource, vtkPCLSource);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLOpenNISource();
  ~vtkPCLOpenNISource();

private:
  vtkPCLOpenNISource(const vtkPCLOpenNISource&) = delete;
  void operator=(const vtkPCLOpenNISource&) = delete;

//------------------------------------------------------------------------------
private:
  bool WithColor;

  class GrabberWrapperBase;
  template <typename PointType>
  class GrabberWrapper;
  GrabberWrapperBase * MyGrabberWrapper;


public:
  bool HasNewData();
  void Poll();
  void StartGrabber();
  void StopGrabber();

  vtkGetMacro(WithColor, bool);
  //! @brief Set the WithColor attribute and switch grabbers if necessary.
  void SetWithColor(bool withColor);

//------------------------------------------------------------------------------
protected:
  int LoadPCLSource(
    vtkPolyData * polyData
  ) override;
};

#endif // vtkPCLOpenNISource_h
