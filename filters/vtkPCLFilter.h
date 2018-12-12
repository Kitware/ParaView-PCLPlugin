#ifndef vtkPCLFilter_h
#define vtkPCLFilter_h

#include "vtkPolyDataAlgorithm.h"

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

//! @brief Common superclass for PCL filters.
class VTK_EXPORT vtkPCLFilter : public vtkPolyDataAlgorithm
{
public:
  static vtkPCLFilter * New();
  vtkTypeMacro(vtkPCLFilter, vtkPolyDataAlgorithm);
  void PrintSelf(ostream & os, vtkIndent indent) VTK_OVERRIDE;

protected:
  int RequestData(
    vtkInformation * request,
    vtkInformationVector * * inputVector,
    vtkInformationVector * outputVector
  ) override;

  vtkPCLFilter();
  ~vtkPCLFilter();

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
  void ApplyPCLFilter(
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud
  ) = 0;
};

#endif // vtkPCLFilter_h

