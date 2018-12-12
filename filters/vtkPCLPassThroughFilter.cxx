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
  pcl::PassThrough<pcl::PointXYZ> passthrough_filter;
  passthrough_filter.setInputCloud(inputCloud);
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
  passthrough_filter.filter(* outputCloud);
}

