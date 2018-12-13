#ifndef vtkPCLVoxelGridFilter_h
#define vtkPCLVoxelGridFilter_h

#include "vtkPCLFilter.h"

class VTK_EXPORT vtkPCLVoxelGridFilter : public vtkPCLFilter
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLVoxelGridFilter * New();
  vtkTypeMacro(vtkPCLVoxelGridFilter, vtkPCLFilter);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLVoxelGridFilter();
  ~vtkPCLVoxelGridFilter();

private:
  vtkPCLVoxelGridFilter(const vtkPCLVoxelGridFilter&) = delete;
  void operator=(const vtkPCLVoxelGridFilter&) = delete;

//------------------------------------------------------------------------------
// Filter parameters.
private:
  float LeafSize[3];

public:
  vtkSetVector3Macro(LeafSize, float);
  vtkGetVector3Macro(LeafSize, float);

//------------------------------------------------------------------------------
private:
  int ApplyPCLFilter(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud
  ) override;
};

#endif // vtkPCLVoxelGridFilter_h

