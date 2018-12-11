#include "vtkPCLPassThroughFilter.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"

vtkStandardNewMacro(vtkPCLPassThroughFilter);

//----------------------------------------------------------------------------
vtkPCLPassThroughFilter::vtkPCLPassThroughFilter()
{
  this->Limits[0] = 0.0;
  this->Limits[1] = 1.0;
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkPCLPassThroughFilter::~vtkPCLPassThroughFilter()
{
}

//----------------------------------------------------------------------------
void vtkPCLPassThroughFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
int vtkPCLPassThroughFilter::RequestData(
  vtkInformation * request,
  vtkInformationVector ** inputVector,
  vtkInformationVector * outputVector
)
{
  vtkInformation * inInfo = inputVector[0]->GetInformationObject(0);
  vtkPolyData * input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkInformation * outInfo = outputVector->GetInformationObject(0);
  vtkPolyData * output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmpPCL = vtkPCLConversions::PointCloudFromPolyData(input);
  vtkSmartPointer<vtkPolyData> tmpPD = vtkPCLConversions::PolyDataFromPointCloud(tmpPCL);
  
  // output->ShallowCopy(input);

  output->ShallowCopy(tmpPD);

  return 1;
}

