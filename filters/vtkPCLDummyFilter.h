#ifndef vtkPCLDummyFilter_h
#define vtkPCLDummyFilter_h

#include "vtkPolyDataAlgorithm.h"

//------------------------------------------------------------------------------
//! @brief Common superclass for PCL filters.
class VTK_EXPORT vtkPCLDummyFilter : public vtkPolyDataAlgorithm
{
public:
  static vtkPCLDummyFilter * New();
  vtkTypeMacro(vtkPCLDummyFilter, vtkPolyDataAlgorithm);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:
  vtkPCLDummyFilter();
  ~vtkPCLDummyFilter();

  int RequestData(
    vtkInformation * request,
    vtkInformationVector * * inputVector,
    vtkInformationVector * outputVector
  ) override;

private:
  vtkPCLDummyFilter(const vtkPCLDummyFilter &) = delete;
  void operator=(const vtkPCLDummyFilter &) = delete;

private:
  std::string Message;

public:
  vtkSetMacro(Message, std::string);
  vtkGetMacro(Message, std::string);
};

#endif // vtkPCLDummyFilter_h

