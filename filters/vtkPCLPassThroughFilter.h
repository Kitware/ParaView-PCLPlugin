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
  std::string FieldName;
  double Limits[2];
  bool Invert;

public:
  vtkSetMacro(FieldName, std::string);
  vtkGetMacro(FieldName, std::string);

  vtkSetVector2Macro(Limits, double);
  vtkGetVector2Macro(Limits, double);

  vtkSetMacro(Invert, bool);
  vtkGetMacro(Invert, bool);

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
#endif // vtkPCLPassThroughFilter_h

