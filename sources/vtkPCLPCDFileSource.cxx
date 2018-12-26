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

#include "vtkPCLPCDFileSource.h"
#include "vtkPCLConversions.h"
#include "_PCLInvokeWithPointType.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <set>

vtkStandardNewMacro(vtkPCLPCDFileSource);

//----------------------------------------------------------------------------
vtkPCLPCDFileSource::vtkPCLPCDFileSource()
{
  this->FileName = "";
}

//----------------------------------------------------------------------------
vtkPCLPCDFileSource::~vtkPCLPCDFileSource()
{
}

//----------------------------------------------------------------------------
void vtkPCLPCDFileSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

#include <pcl/filters/passthrough.h>
//----------------------------------------------------------------------------
int vtkPCLPCDFileSource::LoadPCLSource(
  vtkSmartPointer<vtkPolyData> & output
)
{
  if (this->FileName == "")
  {
    return 0;
  }

  // Determine which fields are in the file.
	pcl::PCLPointCloud2 header;
  pcl::PCDReader reader {};
  reader.readHeader(this->FileName, header);
  std::set<std::string> fields;
	for (auto field : header.fields)
	{
    // The rgb and rgba fields are special. Replace them with the attribute
    // names here to avoid extra complexity in the ConvPoint classes.
    if (field.name == "rgb" || field.name == "rgba")
    {
      fields.insert("r");
      fields.insert("g");
      fields.insert("b");
      // The RGB type contains an "a" member apparently due to common definition
      // macros, so we cannot distinguish between RGB and RGBA.
      fields.insert("a");
    }
    else
    {
      fields.insert(field.name);
    }
	}

  int index = vtkPCLConversions::GetPointTypeIndex(fields);
  INVOKE_WITH_POINT_TYPE(index, return this->InternalLoadPCLSource, output)
  return 0;
}

//----------------------------------------------------------------------------
template <typename PointType>
int vtkPCLPCDFileSource::InternalLoadPCLSource(
  vtkSmartPointer<vtkPolyData> & output
)
{
  typedef pcl::PointCloud<PointType> CloudT;
  typename CloudT::Ptr outputCloud(new CloudT);
  if (pcl::io::loadPCDFile(this->FileName, (* outputCloud)) == -1)
  {
    return 0;
  }
  else
  {
    vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
    return 1;
  }
}
