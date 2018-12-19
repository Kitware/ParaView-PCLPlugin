#include "vtkPCLVoxelGridFilter.h"
#include "vtkPCLConversions.h"
#include "_PCLInvokeWithPointType.h"

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
int vtkPCLVoxelGridFilter::ApplyPCLFilter(
  vtkSmartPointer<vtkPolyData> & input,
  vtkSmartPointer<vtkPolyData> & output
)
{
  INVOKE_WITH_POINT_TYPE(this->InternalApplyPCLFilter, input, output);
  return 1;
}

//----------------------------------------------------------------------------
template <typename PointType>
void vtkPCLVoxelGridFilter::InternalApplyPCLFilter(
  vtkSmartPointer<vtkPolyData> & input,
  vtkSmartPointer<vtkPolyData> & output
)
{
  typedef pcl::PointCloud<PointType> CloudT;
  typename CloudT::Ptr inputCloud(new CloudT);
  typename CloudT::Ptr outputCloud(new CloudT);

  vtkPCLConversions::PointCloudFromPolyData(input, inputCloud);

  pcl::VoxelGrid<PointType> filter;
  filter.setInputCloud(inputCloud);
  filter.setLeafSize(this->LeafSize[0], this->LeafSize[1], this->LeafSize[2]);
  filter.filter(* outputCloud);

  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
}

