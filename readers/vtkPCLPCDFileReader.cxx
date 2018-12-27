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

#include "vtkPCLPCDFileReader.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <set>

vtkStandardNewMacro(vtkPCLPCDFileReader);

//----------------------------------------------------------------------------
vtkPCLPCDFileReader::vtkPCLPCDFileReader()
{
}

//----------------------------------------------------------------------------
vtkPCLPCDFileReader::~vtkPCLPCDFileReader()
{
}

//----------------------------------------------------------------------------
void vtkPCLPCDFileReader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
int vtkPCLPCDFileReader::LoadPCLReader(
  vtkPolyData * output
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
    // The rgb and rgba fields are special cases. Replace them with the
    // attribute names here to avoid extra complexity in the ConvPoint classes.
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
  PCLP_INVOKE_WITH_POINT_TYPE(index, return this->InternalLoadPCLReader, output)
  return 0;
}

//----------------------------------------------------------------------------
template <typename PointType>
int vtkPCLPCDFileReader::InternalLoadPCLReader(
  vtkPolyData * output
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
