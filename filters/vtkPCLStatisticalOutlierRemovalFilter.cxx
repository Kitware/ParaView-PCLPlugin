#include "vtkPCLStatisticalOutlierRemovalFilter.h"
#include "vtkPCLConversions.h"
#include "_PCLInvokeWithPointType.h"

#include <pcl/filters/statistical_outlier_removal.h>

vtkStandardNewMacro(vtkPCLStatisticalOutlierRemovalFilter);

//----------------------------------------------------------------------------
vtkPCLStatisticalOutlierRemovalFilter::vtkPCLStatisticalOutlierRemovalFilter()
{
  this->MeanK = 50.0;
  this->StddevMulThresh = 1.0;
}

//----------------------------------------------------------------------------
vtkPCLStatisticalOutlierRemovalFilter::~vtkPCLStatisticalOutlierRemovalFilter()
{
}

//----------------------------------------------------------------------------
void vtkPCLStatisticalOutlierRemovalFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
int vtkPCLStatisticalOutlierRemovalFilter::ApplyPCLFilter(
  vtkSmartPointer<vtkPolyData> & input,
  vtkSmartPointer<vtkPolyData> & output
)
{
  INVOKE_WITH_POINT_TYPE(this->InternalApplyPCLFilter, input, output);
  return 1;
}

//----------------------------------------------------------------------------
template <typename PointType>
void vtkPCLStatisticalOutlierRemovalFilter::InternalApplyPCLFilter(
  vtkSmartPointer<vtkPolyData> & input,
  vtkSmartPointer<vtkPolyData> & output
)
{
  typedef pcl::PointCloud<PointType> CloudT;
  typename CloudT::Ptr inputCloud(new CloudT);
  typename CloudT::Ptr outputCloud(new CloudT);

  vtkPCLConversions::PointCloudFromPolyData(input, inputCloud);

  pcl::StatisticalOutlierRemoval<PointType> filter;
  filter.setInputCloud(inputCloud);
  filter.setMeanK(this->MeanK);
  filter.setStddevMulThresh(this->StddevMulThresh);
  filter.filter(* outputCloud);

  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
}

