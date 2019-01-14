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

#include "vtkPCLPCDFileWriter.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <set>

vtkStandardNewMacro(vtkPCLPCDFileWriter);

//------------------------------------------------------------------------------
vtkPCLPCDFileWriter::vtkPCLPCDFileWriter()
{
}

//------------------------------------------------------------------------------
vtkPCLPCDFileWriter::~vtkPCLPCDFileWriter()
{
}

//------------------------------------------------------------------------------
void vtkPCLPCDFileWriter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
int vtkPCLPCDFileWriter::WritePCL(
  vtkPolyData * input
)
{
  if (this->FileName == nullptr)
  {
    return 0;
  }

  int index = vtkPCLConversions::GetPointTypeIndex(input);
#define _statement(PointType) return this->InternalWritePCL<PointType>(input);
  PCLP_INVOKE_WITH_ANY_POINT_TYPE(index, _statement)
#undef _statement
  return 0;
}

//------------------------------------------------------------------------------
template <typename PointType>
int vtkPCLPCDFileWriter::InternalWritePCL(
  vtkPolyData * input
)
{
  typedef pcl::PointCloud<PointType> CloudT;
  typename CloudT::Ptr inputCloud(new CloudT);
  vtkPCLConversions::PointCloudFromPolyData(input, inputCloud);
  return (pcl::io::savePCDFile(this->FileName, (* inputCloud)) == -1) ? 0 : 1;
}

