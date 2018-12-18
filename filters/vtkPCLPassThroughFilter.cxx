#include "vtkPCLPassThroughFilter.h"
#include "vtkPCLConversions.h"
#include "_PCLInvokeWithPointType.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

vtkStandardNewMacro(vtkPCLPassThroughFilter);

//----------------------------------------------------------------------------
template <typename PointType>
void vtkPCLPassThroughFilter::InternalApplyPCLFilter(
  vtkSmartPointer<vtkPolyData> & input,
  vtkSmartPointer<vtkPolyData> & output
)
{
  typedef pcl::PointCloud<PointType> CloudT;
  typename CloudT::Ptr inputCloud(new CloudT);
  typename CloudT::Ptr outputCloud(new CloudT);
  
  vtkPCLConversions::PointCloudFromPolyData(input, inputCloud);

  pcl::PassThrough<PointType> filter;
  filter.setInputCloud(inputCloud);
  filter.setFilterFieldName(this->FieldName);
  filter.setFilterLimits(this->Limits[0], this->Limits[1]);
  filter.setFilterLimitsNegative(this->Invert);
  filter.filter(* outputCloud);

  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
}


//----------------------------------------------------------------------------
vtkPCLPassThroughFilter::vtkPCLPassThroughFilter()
{
  this->FieldName = "x";
  this->Limits[0] = 0.0;
  this->Limits[1] = 1.0;
  this->Invert = false;
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
int vtkPCLPassThroughFilter::ApplyPCLFilter(
  vtkSmartPointer<vtkPolyData> & input,
  vtkSmartPointer<vtkPolyData> & output
)
{
  INVOKE_WITH_POINT_TYPE(this->InternalApplyPCLFilter, input, output);
  return 1;
}

