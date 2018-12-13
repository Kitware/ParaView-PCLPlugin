#ifndef vtkPCLSource_h
#define vtkPCLSource_h

#include "vtkPolyDataAlgorithm.h"

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

//------------------------------------------------------------------------------
//! @brief Common superclass for PCL sources.
class VTK_EXPORT vtkPCLSource : public vtkPolyDataAlgorithm
{
public:
  static vtkPCLSource * New();
  vtkTypeMacro(vtkPCLSource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:
  vtkPCLSource();
  ~vtkPCLSource();

  int RequestData(
    vtkInformation * request,
    vtkInformationVector * * inputVector,
    vtkInformationVector * outputVector
  ) override;

private:
  vtkPCLSource(const vtkPCLSource &) = delete;
  void operator=(const vtkPCLSource &) = delete;

  /*!
   * @brief      Load the PCL source.
   * @param[out] outputCloud The output cloud produced by the source.
   */
  virtual
  int LoadPCLSource(
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud
  ) = 0;
};

#endif // vtkPCLSource_h

