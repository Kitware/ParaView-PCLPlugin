//=========================================================================
//
// Copyright 2012-2019 Kitware, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//=========================================================================
//
// .NAME vtkPCLConversions - collection of pointcloud library routines
//
// .SECTION Description
//

#ifndef __vtkPCLConversions_h
#define __vtkPCLConversions_h
// #define DEBUG_MSG(msg) std::cout << "DEBUG: " << msg << " [" << __LINE__ << "]" << std::endl

#include <vtkObject.h>
#include <vtkSmartPointer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
// #include <vtkPCLFiltersModule.h>



class vtkPolyData;
class vtkCellArray;
class vtkIntArray;

class VTK_EXPORT vtkPCLConversions : public vtkObject
{
public:

  static vtkPCLConversions * New();

  vtkTypeMacro(vtkPCLConversions, vtkObject);

  void PrintSelf(ostream& os, vtkIndent indent);

  // static vtkSmartPointer<vtkPolyData> PolyDataFromPCDFile(const std::string& filename);


// The right way to do this doesn't work because of VTK wrapping. Nevertheless,
// it can be used to generate the code for manual insertion.
//
// TODO
// Use an include file or PIMPL.
//
// #include <boost/preprocessor/seq/for_each.hpp>
// #include <pcl/impl/point_types.hpp>
// #define DECLARE_CONVERTER(i, data, PointType) \
//   static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(pcl::PointCloud<PointType>::ConstPtr cloud);
//
//   BOOST_PP_SEQ_FOR_EACH(DECLARE_CONVERTER, _, PCL_XYZ_POINT_TYPES)

  // Handle all PCL point types with XYZ data.
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::PointXYZ          >::ConstPtr cloud);
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::PointXYZI         >::ConstPtr cloud);
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::PointXYZL         >::ConstPtr cloud);
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::PointXYZRGBA      >::ConstPtr cloud);
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::PointXYZRGB       >::ConstPtr cloud);
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::PointXYZRGBL      >::ConstPtr cloud);
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::PointXYZHSV       >::ConstPtr cloud);
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::InterestPoint     >::ConstPtr cloud);
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::PointNormal       >::ConstPtr cloud);
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::PointXYZRGBNormal >::ConstPtr cloud);
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::PointXYZINormal   >::ConstPtr cloud);
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::PointXYZLNormal   >::ConstPtr cloud);
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::PointWithRange    >::ConstPtr cloud);
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::PointWithViewpoint>::ConstPtr cloud);
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::PointWithScale    >::ConstPtr cloud);
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::PointSurfel       >::ConstPtr cloud);
  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud (pcl::PointCloud <pcl::PointDEM          >::ConstPtr cloud);

  // static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(
  //   pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
  //
  // static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(
  //   pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
  //
  // static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(
  //   pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud);

  static pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudFromPolyData(
    vtkPolyData * polyData);

  static vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts);
  //
  // static vtkSmartPointer<vtkIntArray> NewLabelsArray(pcl::IndicesConstPtr indices, vtkIdType length);
  // static vtkSmartPointer<vtkIntArray> NewLabelsArray(pcl::PointIndices::ConstPtr indices, vtkIdType length);
  // static vtkSmartPointer<vtkIntArray> NewLabelsArray(const std::vector<pcl::PointIndices>& indices, vtkIdType length);

  // static void PerformPointCloudConversionBenchmark(vtkPolyData* polyData);

protected:

  vtkPCLConversions();
  ~vtkPCLConversions();

private:

  vtkPCLConversions(const vtkPCLConversions&) = delete; // Not implemented
  void operator=(const vtkPCLConversions&) = delete; // Not implemented
};

#endif
