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
void vtkPCLPassThroughFilter::ApplyPCLFilter(
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud
)
{
  pcl::PassThrough<pcl::PointXYZ> filter;
  filter.setInputCloud(inputCloud);
  switch (this->Axis)
  {
    case 0:
      filter.setFilterFieldName("x");
      break;
    case 1:
      filter.setFilterFieldName("y");
      break;
    default:
      filter.setFilterFieldName("z");
  }
  filter.setFilterLimits(this->Limits[0], this->Limits[1]);
  filter.setFilterLimitsNegative(this->Invert);
  filter.filter(* outputCloud);
}

