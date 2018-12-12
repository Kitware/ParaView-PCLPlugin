#ifndef vtkPCLPassThroughFilter_h
#define vtkPCLPassThroughFilter_h

#include "vtkPCLFilter.h"

class VTK_EXPORT vtkPCLPassThroughFilter : public vtkPCLFilter
{
public:
  static vtkPCLPassThroughFilter * New();
  vtkTypeMacro(vtkPCLPassThroughFilter, vtkPCLFilter);
  void PrintSelf(ostream & os, vtkIndent indent) VTK_OVERRIDE;

  vtkSetMacro(Axis, unsigned char);
  vtkGetMacro(Axis, unsigned char);

  vtkSetVector2Macro(Limits, double);
  vtkGetVector2Macro(Limits, double);

  vtkSetMacro(Invert, bool);
  vtkGetMacro(Invert, bool);

protected:

  vtkPCLPassThroughFilter();
  ~vtkPCLPassThroughFilter();

private:
  vtkPCLPassThroughFilter(const vtkPCLPassThroughFilter&) = delete;
  void operator=(const vtkPCLPassThroughFilter&) = delete;

  unsigned char Axis;
  double Limits[2];
  bool Invert;

  void ApplyPCLFilter(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud
  ) override;
};

#endif // vtkPCLPassThroughFilter_h

