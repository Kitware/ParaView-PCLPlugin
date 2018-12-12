#include "vtkPCLPassThroughFilter.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

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

  pcl::PointCloud<pcl::PointXYZ>::Ptr inputPCL = vtkPCLConversions::PointCloudFromPolyData(input);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputPCL(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PassThrough<pcl::PointXYZ> passthrough_filter;
  passthrough_filter.setInputCloud(inputPCL);
  switch (this->Axis)
  {
    case 0:
      passthrough_filter.setFilterFieldName("x");
      break;
    case 1:
      passthrough_filter.setFilterFieldName("y");
      break;
    default:
      passthrough_filter.setFilterFieldName("z");
  }
  passthrough_filter.setFilterLimits(this->Limits[0], this->Limits[1]);
  passthrough_filter.filter(* outputPCL);

  vtkSmartPointer<vtkPolyData> tmpPD = vtkPCLConversions::PolyDataFromPointCloud(outputPCL);

  output->ShallowCopy(tmpPD);

  return 1;
}

