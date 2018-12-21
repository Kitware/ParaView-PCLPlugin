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
  typedef pcl::PointXYZ PointType;
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

