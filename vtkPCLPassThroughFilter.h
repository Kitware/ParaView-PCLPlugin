#ifndef vtkPCLPassThroughFilter_h
#define vtkPCLPassThroughFilter_h

#include "vtkPolyDataAlgorithm.h"

class VTK_EXPORT vtkPCLPassThroughFilter : public vtkPolyDataAlgorithm
{
public:
  static vtkPCLPassThroughFilter* New();
  vtkTypeMacro(vtkPCLPassThroughFilter, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) VTK_OVERRIDE;

  vtkSetMacro(Axis, unsigned char);
  vtkGetMacro(Axis, unsigned char);

  vtkSetVector2Macro(Limits, double);
  vtkGetVector2Macro(Limits, double);

protected:
  unsigned char Axis;
  double Limits[2];

  int RequestData(
    vtkInformation * request,
    vtkInformationVector ** inputVector,
    vtkInformationVector * outputVector
  );

  vtkPCLPassThroughFilter();
  ~vtkPCLPassThroughFilter();

private:
  vtkPCLPassThroughFilter(const vtkPCLPassThroughFilter&) = delete;
  void operator=(const vtkPCLPassThroughFilter&) = delete;
};

#endif
