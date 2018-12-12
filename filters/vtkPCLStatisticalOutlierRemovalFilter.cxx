#include "vtkPCLStatisticalOutlierRemovalFilter.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"

#include <pcl/point_types.h>
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
void vtkPCLStatisticalOutlierRemovalFilter::ApplyPCLFilter(
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud
)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
  filter.setInputCloud(inputCloud);
  filter.setMeanK(this->MeanK);
  filter.setStddevMulThresh(this->StddevMulThresh);
  filter.filter(* outputCloud);
}

