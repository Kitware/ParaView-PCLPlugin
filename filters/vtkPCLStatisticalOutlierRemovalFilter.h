#ifndef vtkPCLStatisticalOutlierRemovalFilter_h
#define vtkPCLStatisticalOutlierRemovalFilter_h

#include "vtkPCLFilter.h"

class VTK_EXPORT vtkPCLStatisticalOutlierRemovalFilter : public vtkPCLFilter
{
public:
  static vtkPCLStatisticalOutlierRemovalFilter * New();
  vtkTypeMacro(vtkPCLStatisticalOutlierRemovalFilter, vtkPCLFilter);
  void PrintSelf(ostream & os, vtkIndent indent) VTK_OVERRIDE;

  vtkSetMacro(MeanK, double);
  vtkGetMacro(MeanK, double);

  vtkSetMacro(StddevMulThresh, double);
  vtkGetMacro(StddevMulThresh, double);

protected:

  vtkPCLStatisticalOutlierRemovalFilter();
  ~vtkPCLStatisticalOutlierRemovalFilter();

private:
  vtkPCLStatisticalOutlierRemovalFilter(const vtkPCLStatisticalOutlierRemovalFilter&) = delete;
  void operator=(const vtkPCLStatisticalOutlierRemovalFilter&) = delete;

  double MeanK;
  double StddevMulThresh;

  void ApplyPCLFilter(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud
  ) override;
};

#endif // vtkPCLStatisticalOutlierRemovalFilter_h

