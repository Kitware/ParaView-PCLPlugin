#include "vtkPCLVoxelGridFilter.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

vtkStandardNewMacro(vtkPCLVoxelGridFilter);

//----------------------------------------------------------------------------
vtkPCLVoxelGridFilter::vtkPCLVoxelGridFilter()
{
  this->LeafSize[0] = 0.01;
  this->LeafSize[1] = 0.01;
  this->LeafSize[2] = 0.01;
}

//----------------------------------------------------------------------------
vtkPCLVoxelGridFilter::~vtkPCLVoxelGridFilter()
{
}

//----------------------------------------------------------------------------
void vtkPCLVoxelGridFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
void vtkPCLVoxelGridFilter::ApplyPCLFilter(
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud
)
{
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(inputCloud);
  filter.setLeafSize(this->LeafSize[0], this->LeafSize[1], this->LeafSize[2]);
  filter.filter(* outputCloud);
}

