#ifndef vtkPCLVoxelGridFilter_h
#define vtkPCLVoxelGridFilter_h

#include "vtkPCLFilter.h"

class VTK_EXPORT vtkPCLVoxelGridFilter : public vtkPCLFilter
{
public:
  static vtkPCLVoxelGridFilter * New();
  vtkTypeMacro(vtkPCLVoxelGridFilter, vtkPCLFilter);
  void PrintSelf(ostream & os, vtkIndent indent) VTK_OVERRIDE;

  vtkSetVector3Macro(LeafSize, float);
  vtkGetVector3Macro(LeafSize, float);

protected:

  vtkPCLVoxelGridFilter();
  ~vtkPCLVoxelGridFilter();

private:
  vtkPCLVoxelGridFilter(const vtkPCLVoxelGridFilter&) = delete;
  void operator=(const vtkPCLVoxelGridFilter&) = delete;

  float LeafSize[3];

  void ApplyPCLFilter(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud
  ) override;
};

#endif // vtkPCLVoxelGridFilter_h

