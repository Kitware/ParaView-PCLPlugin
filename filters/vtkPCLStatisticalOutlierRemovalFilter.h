#ifndef vtkPCLStatisticalOutlierRemovalFilter_h
#define vtkPCLStatisticalOutlierRemovalFilter_h

#include "vtkPCLFilter.h"

class VTK_EXPORT vtkPCLStatisticalOutlierRemovalFilter : public vtkPCLFilter
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLStatisticalOutlierRemovalFilter * New();
  vtkTypeMacro(vtkPCLStatisticalOutlierRemovalFilter, vtkPCLFilter);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLStatisticalOutlierRemovalFilter();
  ~vtkPCLStatisticalOutlierRemovalFilter();

private:
  vtkPCLStatisticalOutlierRemovalFilter(const vtkPCLStatisticalOutlierRemovalFilter&) = delete;
  void operator=(const vtkPCLStatisticalOutlierRemovalFilter&) = delete;

//------------------------------------------------------------------------------
// Filter parameters.
private:
  double MeanK;
  double StddevMulThresh;

public:
  vtkSetMacro(MeanK, double);
  vtkGetMacro(MeanK, double);

  vtkSetMacro(StddevMulThresh, double);
  vtkGetMacro(StddevMulThresh, double);

//------------------------------------------------------------------------------
private:
  int ApplyPCLFilter(
    vtkSmartPointer<vtkPolyData> & input,
    vtkSmartPointer<vtkPolyData> & output
  ) override;

  template <typename PointType>
  void InternalApplyPCLFilter(
    vtkSmartPointer<vtkPolyData> & input,
    vtkSmartPointer<vtkPolyData> & output
  );
};

#endif // vtkPCLStatisticalOutlierRemovalFilter_h

