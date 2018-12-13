#include "vtkPCLSource.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"

#include <pcl/point_types.h>

// vtkStandardNewMacro(vtkPCLSource);

//----------------------------------------------------------------------------
vtkPCLSource::vtkPCLSource()
{
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkPCLSource::~vtkPCLSource()
{
}

//----------------------------------------------------------------------------
void vtkPCLSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
int vtkPCLSource::RequestData(
  vtkInformation * vtkNotUsed(request),
  vtkInformationVector * * inputVector,
  vtkInformationVector * outputVector
)
{
  vtkInformation * outInfo = outputVector->GetInformationObject(0);
  vtkPolyData * output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  pcl::PointCloud<pcl::PointXYZ>::Ptr outputPCL(new pcl::PointCloud<pcl::PointXYZ>);
  
  int ret = this->LoadPCLSource(outputPCL);

  if (ret == 1)
  {
    output->ShallowCopy(vtkPCLConversions::PolyDataFromPointCloud(outputPCL));
  }

  return ret;
}

