#include "vtkPCLPCDFileSource.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

vtkStandardNewMacro(vtkPCLPCDFileSource);

//----------------------------------------------------------------------------
vtkPCLPCDFileSource::vtkPCLPCDFileSource()
{
  this->FilePath = "";
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

//----------------------------------------------------------------------------
int vtkPCLPCDFileSource::LoadPCLSource(
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud
)
{
  if (this->FilePath == "")
  {
    return 0;
  }
  return (pcl::io::loadPCDFile(this->FilePath, (* outputCloud)) == -1) ? 0 : 1;
}

