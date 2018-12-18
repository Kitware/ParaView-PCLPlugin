#ifndef vtkPCLFilter_h
#define vtkPCLFilter_h

#include "vtkPolyDataAlgorithm.h"

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

//------------------------------------------------------------------------------
//! @brief Common superclass for PCL filters.
class VTK_EXPORT vtkPCLFilter : public vtkPolyDataAlgorithm
{
public:
  static vtkPCLFilter * New();
  vtkTypeMacro(vtkPCLFilter, vtkPolyDataAlgorithm);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:
  vtkPCLFilter();
  ~vtkPCLFilter();

  int RequestData(
    vtkInformation * request,
    vtkInformationVector * * inputVector,
    vtkInformationVector * outputVector
  ) override;

private:
  vtkPCLFilter(const vtkPCLFilter &) = delete;
  void operator=(const vtkPCLFilter &) = delete;

  /*!
   * @brief      Apply the PCL filter.
   * @param[in]  inputCloud  The input cloud.
   * @param[out] outputCloud The output cloud that results from applying the
   *                         filter to the input cloud.
   */
  virtual
  int ApplyPCLFilter(
    vtkSmartPointer<vtkPolyData> & input,
    vtkSmartPointer<vtkPolyData> & output
  ) = 0;
};

#endif // vtkPCLFilter_h

