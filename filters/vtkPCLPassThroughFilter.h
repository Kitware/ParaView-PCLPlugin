#ifndef vtkPCLPassThroughFilter_h
#define vtkPCLPassThroughFilter_h

#include "vtkPCLFilter.h"

class VTK_EXPORT vtkPCLPassThroughFilter : public vtkPCLFilter
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLPassThroughFilter * New();
  vtkTypeMacro(vtkPCLPassThroughFilter, vtkPCLFilter);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLPassThroughFilter();
  ~vtkPCLPassThroughFilter();

private:
  vtkPCLPassThroughFilter(const vtkPCLPassThroughFilter&) = delete;
  void operator=(const vtkPCLPassThroughFilter&) = delete;

//------------------------------------------------------------------------------
// Filter parameters.
private:
  unsigned char Axis;
  double Limits[2];
  bool Invert;

public:
  vtkSetMacro(Axis, unsigned char);
  vtkGetMacro(Axis, unsigned char);

  vtkSetVector2Macro(Limits, double);
  vtkGetVector2Macro(Limits, double);

  vtkSetMacro(Invert, bool);
  vtkGetMacro(Invert, bool);

//------------------------------------------------------------------------------
private:
  int ApplyPCLFilter(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud
  ) override;
};

#endif // vtkPCLPassThroughFilter_h

