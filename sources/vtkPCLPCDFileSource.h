#ifndef vtkPCLPCDFileSource_h
#define vtkPCLPCDFileSource_h

#include "vtkPCLSource.h"

class VTK_EXPORT vtkPCLPCDFileSource : public vtkPCLSource
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  static vtkPCLPCDFileSource * New();
  vtkTypeMacro(vtkPCLPCDFileSource, vtkPCLSource);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLPCDFileSource();
  ~vtkPCLPCDFileSource();

private:
  vtkPCLPCDFileSource(const vtkPCLPCDFileSource&) = delete;
  void operator=(const vtkPCLPCDFileSource&) = delete;

//------------------------------------------------------------------------------
// Source parameters.
private:
  std::string FilePath;

public:
  vtkSetMacro(FilePath, std::string);
  vtkGetMacro(FilePath, std::string);

//------------------------------------------------------------------------------
protected:
  int LoadPCLSource(
    vtkSmartPointer<vtkPolyData> & output
  ) override;
};

#endif // vtkPCLPCDFileSource_h

