#include "vtkPCLFilter.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"

#include <pcl/point_types.h>

// vtkStandardNewMacro(vtkPCLFilter);

//----------------------------------------------------------------------------
vtkPCLFilter::vtkPCLFilter()
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkPCLFilter::~vtkPCLFilter()
{
}

//----------------------------------------------------------------------------
void vtkPCLFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
int vtkPCLFilter::RequestData(
  vtkInformation * request,
  vtkInformationVector * * inputVector,
  vtkInformationVector * outputVector
)
{
  vtkInformation * inInfo = inputVector[0]->GetInformationObject(0);
  vtkPolyData * input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkInformation * outInfo = outputVector->GetInformationObject(0);
  vtkPolyData * output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  pcl::PointCloud<pcl::PointXYZ>::Ptr inputPCL = vtkPCLConversions::PointCloudFromPolyData(input);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputPCL(new pcl::PointCloud<pcl::PointXYZ>);
  
  this->ApplyPCLFilter(inputPCL, outputPCL);

  // TODO: remove temporary object
  vtkSmartPointer<vtkPolyData> tmpPD = vtkPCLConversions::PolyDataFromPointCloud(outputPCL);

  output->ShallowCopy(tmpPD);

  return 1;
}

