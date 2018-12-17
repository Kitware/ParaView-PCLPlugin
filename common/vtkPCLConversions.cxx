//=========================================================================
//
// Copyright 2012,2013,2014 Kitware, Inc.
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

#include "vtkPCLConversions.h"

#include <vtkObjectFactory.h>
#include <vtkPolyData.h>
#include <vtkTimerLog.h>
#include <vtkNew.h>
#include <vtkIdList.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkIntArray.h>
#include <vtkPointData.h>

#include <pcl/io/pcd_io.h>

#include <boost/preprocessor/seq/transform.hpp>
#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/variadic/to_seq.hpp>

#include <cassert>

//----------------------------------------------------------------------------
//! @brief Check if a point is spatiall finite.
#define PCL_IS_FINITE(point) \
  pcl_isfinite (point.x) &&  \
  pcl_isfinite (point.y) &&  \
  pcl_isfinite (point.z)

//----------------------------------------------------------------------------
/*!
 * @brief Bitfield for tracking point attributes.
 *
 * Different PCL point types share common attributes. For example, all of the
 * PointXYZ* types have x, y and z and all of the Point*RGB* have r, g and b.
 * The following enum values are used in templates below to convert these values
 * to and from PolyData values.
 */
enum PointAttrs
{
  PA_XYZ    = 1 << 0,
  PA_I      = 1 << 1,
  PA_L      = 1 << 2,
  PA_RGB    = 1 << 3,
  PA_A      = 1 << 4,
  PA_HSV    = 1 << 5,
  PA_Normal = 1 << 6
};

//----------------------------------------------------------------------------
// Boost-based macros for determining point attributes.
//----------------------------------------------------------------------------
//! @brief Internal macro passed to BOOST_PP_SEQ_TRANSFORM in _DECLARE_HAS_ATTR.
#define _CAST_VOID(i, name, attr) (void) name::attr

//----------------------------------------------------------------------------
/*!
 * @brief     Declare a templated struct that can be used to check if a given
 *            type as certain attributes.
 * @param[in] name The name of the struct to append to "Has", e.g. "XYZ" will
 *                 declare a struct named "HasXYZ".
 * @param[in] ...  A variadic list of attributes to check, e.g. x,y,z.
 *
 * The uses template specialization and SFINAE to check if the template type
 * contains all of the given attributes. For example, _DECLARE_HAS_ATTR(XYZ, x,
 * y, z) will declare the templated struct HasXYZ that checks its template type
 * for x, y and z attributes. HasXYZ<pcl::PointXYZ>::value will then be true.
 * This is used in templated conversion functions to determine which attributes
 * to copy at compile time. For example, if the point type is a template
 * parameter named PointType, then a conditional block guarded by
 * HasRGB<PointType>::value could be used to copy RGB values when present,
 * provided HasRGB has been declared with _DECLARE_HAS_ATTR(RGB, r, g, b).
 *
 * Details:
 * https://stackoverflow.com/questions/1005476/how-to-detect-whether-there-is-a-specific-member-variable-in-class
 */
#define _DECLARE_HAS_ATTR(name, ...)                                                                \
  template <typename T, typename = int>                                                             \
  struct Has ## name : std::false_type {};                                                          \
                                                                                                    \
  template <typename T>                                                                             \
  struct Has ## name<T, decltype(                                                                   \
    BOOST_PP_SEQ_ENUM(BOOST_PP_SEQ_TRANSFORM(_CAST_VOID, T, BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__))) \
    ,0)> : std::true_type {};

//----------------------------------------------------------------------------
//! @brief Check for x, y and z attributes.
_DECLARE_HAS_ATTR(XYZ, x, y, z)
//! @brief Check for r, g and b attributes.
_DECLARE_HAS_ATTR(RGB, r, g, b)
//! @brief Check for the alpha attribute.
_DECLARE_HAS_ATTR(ALPHA, a)


//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLConversions);

//----------------------------------------------------------------------------
vtkPCLConversions::vtkPCLConversions()
{
}

//----------------------------------------------------------------------------
vtkPCLConversions::~vtkPCLConversions()
{
}

//----------------------------------------------------------------------------
void vtkPCLConversions::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

// namespace {
//
//   template <typename T>
//     vtkSmartPointer<vtkPolyData> TemplatedPolyDataFromPCDFile(const std::string& filename)
//     {
//       typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>);
//       if (pcl::io::loadPCDFile(filename, *cloud) == -1)
//       {
//         std::cout << "Error reading pcd file: " << filename;
//         return 0;
//       }
//
//       return vtkPCLConversions::PolyDataFromPointCloud(cloud);
//     }
//
// }


//----------------------------------------------------------------------------
// vtkSmartPointer<vtkPolyData> vtkPCLConversions::PolyDataFromPCDFile(const std::string& filename)
// {
//   int version;
//   int type;
//   unsigned int idx;
//
//   pcl::PCLPointCloud2 cloud;
//   Eigen::Vector4f origin;
//   Eigen::Quaternionf orientation;
//   pcl::PCDReader reader;
//   reader.readHeader(filename, cloud, origin, orientation, version, type, idx);
//
//
//   if (pcl::getFieldIndex(cloud, "rgba") != -1) {
//     return TemplatedPolyDataFromPCDFile<pcl::PointXYZRGBA>(filename);
//   }
//   else if (pcl::getFieldIndex(cloud, "rgb") != -1) {
//     return TemplatedPolyDataFromPCDFile<pcl::PointXYZRGB>(filename);
//   }
//   else {
//     return TemplatedPolyDataFromPCDFile<pcl::PointXYZ>(filename);
//   }
// }

//----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkPCLConversions::PolyDataFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  vtkIdType nr_points = cloud->points.size();

  vtkNew<vtkPoints> points;
  points->SetDataTypeToDouble();
  points->SetNumberOfPoints(nr_points);

#define CHECK(pointtype) std::cout << "Type " #pointtype " has RGB: " << HasRGB<pcl:: pointtype>::value << std::endl;
  CHECK(PointXYZ)
  CHECK(PointXYZRGB)
  CHECK(PointSurfel)
  CHECK(Narf36)

  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      double point[3] {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
      points->SetPoint(i, point);
    }
  }
  else
  {
    vtkIdType j = 0;    // true point index
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      // Check if the point is invalid
      if (
        !pcl_isfinite (cloud->points[i].x) ||
        !pcl_isfinite (cloud->points[i].y) ||
        !pcl_isfinite (cloud->points[i].z)
      )
      {
        continue;
      }

      points->SetPoint(j++, cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    }
    nr_points = j;
    points->SetNumberOfPoints(nr_points);
  }
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points);
  polyData->SetVerts(NewVertexCells(nr_points));

  return polyData;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkPCLConversions::PolyDataFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  vtkIdType nr_points = cloud->points.size();

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(nr_points);

  vtkNew<vtkUnsignedCharArray> rgbArray;
  rgbArray->SetName("rgb_colors");
  rgbArray->SetNumberOfComponents(3);
  rgbArray->SetNumberOfTuples(nr_points);


  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i) {
      float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
      unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
      points->SetPoint(i, point);
      rgbArray->SetTuple3(i, color[0], color[1], color[2]);
    }
  }
  else
  {
    vtkIdType j = 0;    // true point index
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud->points[i].x) ||
          !pcl_isfinite (cloud->points[i].y) ||
          !pcl_isfinite (cloud->points[i].z))
        continue;

      float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
      unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
      points->SetPoint(j, point);
      rgbArray->SetTuple3(j, color[0], color[1], color[2]);
      j++;
    }
    nr_points = j;
    points->SetNumberOfPoints(nr_points);
    rgbArray->SetNumberOfTuples(nr_points);
  }

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points.GetPointer());
  polyData->GetPointData()->AddArray(rgbArray.GetPointer());
  polyData->SetVerts(NewVertexCells(nr_points));
  return polyData;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkPCLConversions::PolyDataFromPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
{
  vtkIdType nr_points = cloud->points.size();

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(nr_points);

  vtkNew<vtkUnsignedCharArray> rgbArray;
  rgbArray->SetName("rgb_colors");
  rgbArray->SetNumberOfComponents(3);
  rgbArray->SetNumberOfTuples(nr_points);


  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i) {
      float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
      unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
      points->SetPoint(i, point);
      rgbArray->SetTuple3(i, color[0], color[1], color[2]);
    }
  }
  else
  {
    vtkIdType j = 0;    // true point index
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud->points[i].x) ||
          !pcl_isfinite (cloud->points[i].y) ||
          !pcl_isfinite (cloud->points[i].z))
        continue;

      float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
      unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
      points->SetPoint(j, point);
      rgbArray->SetTuple3(j, color[0], color[1], color[2]);
      j++;
    }
    nr_points = j;
    points->SetNumberOfPoints(nr_points);
    rgbArray->SetNumberOfTuples(nr_points);
  }

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points.GetPointer());
  polyData->GetPointData()->AddArray(rgbArray.GetPointer());
  polyData->SetVerts(NewVertexCells(nr_points));
  return polyData;
}

//----------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr vtkPCLConversions::PointCloudFromPolyData(vtkPolyData* polyData)
{
  const vtkIdType numberOfPoints = polyData->GetNumberOfPoints();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = numberOfPoints;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize(numberOfPoints);

  if (! numberOfPoints)
  {
    return cloud;
  }

  vtkPoints const * points = polyData->GetPoints();
  // The original implementation used vtkFloatArray::SafeDownCast and
  // vtkDoubleArray::SafeDownCast to determine the element type of the array.
  // Pointer arithmetic was then performed to access the data directly via the
  // raw data pointers to avoid calls to GetPoint. The disadvantage of that
  // approach is that is makes assumptions about the layout of memory within the
  // array which are allegedly not upheld on all systems. The following approach
  // makes no such assumptions and is independent of the underlying type. If
  // speed becomes an issue, the previous approach can be revisited with
  // additional check to ensure that the assumptions about memory layout are
  // correct.
  for (vtkIdType i = 0; i < numberOfPoints; ++i)
  {
    double point[3];
    polyData->GetPoint(i, point);
    cloud->points[i].x = point[0];
    cloud->points[i].y = point[1];
    cloud->points[i].z = point[2];
  }

  // vtkFloatArray * floatPoints = vtkFloatArray::SafeDownCast(polyData->GetPoints()->GetData());
  // vtkDoubleArray * doublePoints = vtkDoubleArray::SafeDownCast(polyData->GetPoints()->GetData());
  // assert(floatPoints || doublePoints);
  //
  // if (floatPoints)
  // {
  //   float * data = floatPoints->GetPointer(0);
  //
  //   for (vtkIdType i = 0; i < numberOfPoints; ++i)
  //   {
  //     cloud->points[i].x = data[i * 3];
  //     cloud->points[i].y = data[i * 3 + 1];
  //     cloud->points[i].z = data[i * 3 + 2];
  //   }
  // }
  // else if (doublePoints)
  // {
  //   double * data = doublePoints->GetPointer(0);
  //
  //   for (vtkIdType i = 0; i < numberOfPoints; ++i)
  //   {
  //     cloud->points[i].x = data[i * 3];
  //     cloud->points[i].y = data[i * 3 + 1];
  //     cloud->points[i].z = data[i * 3 + 2];
  //   }
  // }

  return cloud;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkCellArray> vtkPCLConversions::NewVertexCells(vtkIdType numberOfVerts)
{
  vtkNew<vtkIdTypeArray> cells;
  cells->SetNumberOfValues(numberOfVerts * 2);
  vtkIdType * ids = cells->GetPointer(0);

  for (vtkIdType i = 0; i < numberOfVerts; ++i)
  {
    ids[i * 2] = 1;
    ids[i * 2 + 1] = i;
  }

  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
  cellArray->SetCells(numberOfVerts, cells.GetPointer());
  return cellArray;
}

//----------------------------------------------------------------------------
// void vtkPCLConversions::PerformPointCloudConversionBenchmark(vtkPolyData* polyData)
// {
//   if (!polyData)
//     {
//     return;
//     }
//
//   double start;
//   double elapsed;
//   unsigned long kilobytes;
//
//   const vtkIdType numberOfPoints = polyData->GetNumberOfPoints();
//   std::cout << "Number of input points: " << numberOfPoints << std::endl;
//
//   start = vtkTimerLog::GetUniversalTime();
//   pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud = PointCloudFromPolyData(polyData);
//   elapsed = vtkTimerLog::GetUniversalTime() - start;
//
//   std::cout << "Conversion to pcl::PointCloud took " << elapsed << " seconds. "
//             << numberOfPoints / elapsed << " points per second." << std::endl;
//
//
//   start = vtkTimerLog::GetUniversalTime();
//   vtkSmartPointer<vtkPolyData> tempPolyData = PolyDataFromPointCloud(tempCloud);
//   elapsed = vtkTimerLog::GetUniversalTime() - start;
//
//   std::cout << "Conversion to vtkPolyData took " << elapsed << " seconds. "
//             << numberOfPoints / elapsed << " points per second." << std::endl;
//
//
//   start = vtkTimerLog::GetUniversalTime();
//   vtkSmartPointer<vtkCellArray> tempCells = NewVertexCells(numberOfPoints);
//   elapsed = vtkTimerLog::GetUniversalTime() - start;
//
//   std::cout << "Constructing vertex cells took " << elapsed << " seconds. "
//             << numberOfPoints / elapsed << " points per second." << std::endl;
//
//
//   kilobytes = tempPolyData->GetActualMemorySize();
//   std::cout << "vtkPolyData uses " << kilobytes/1024.0 << " MB. "
//             << kilobytes*1024 / numberOfPoints << " bytes per point." << std::endl;
//
//   kilobytes = tempPolyData->GetPoints()->GetActualMemorySize();
//   std::cout << "vtkPolyData's points use " << kilobytes/1024.0 << " MB. "
//             << kilobytes*1024 / numberOfPoints << " bytes per point." << std::endl;
//
//   kilobytes = tempPolyData->GetVerts()->GetActualMemorySize();
//   std::cout << "vtkPolyData's cells use " << kilobytes/1024.0 << " MB. "
//             << kilobytes*1024 / numberOfPoints << " bytes per point." << std::endl;
// }

//----------------------------------------------------------------------------
// namespace {
//
//   vtkSmartPointer<vtkIntArray> NewLabelsArray(vtkIdType length)
//   {
//     vtkSmartPointer<vtkIntArray> labels = vtkSmartPointer<vtkIntArray>::New();
//     labels->SetNumberOfComponents(1);
//     labels->SetNumberOfTuples(length);
//     labels->FillComponent(0, 0);
//     return labels;
//   }
//
//   void LabelIndices(const std::vector<int>& indices, vtkIntArray* labels, const int labelValue)
//   {
//     const size_t numberOfIndices = indices.size();
//     for (size_t k = 0; k < numberOfIndices; ++k)
//     {
//       labels->SetValue(indices[k], labelValue);
//     }
//   }
//
// }

//----------------------------------------------------------------------------
// vtkSmartPointer<vtkIntArray> vtkPCLConversions::NewLabelsArray(pcl::IndicesConstPtr indices, vtkIdType length)
// {
//   vtkSmartPointer<vtkIntArray> labels = ::NewLabelsArray(length);
//   if (indices)
//     {
//     LabelIndices(*indices, labels, 1);
//     }
//   return labels;
// }

//----------------------------------------------------------------------------
// vtkSmartPointer<vtkIntArray> vtkPCLConversions::NewLabelsArray(pcl::PointIndices::ConstPtr indices, vtkIdType length)
// {
//   vtkSmartPointer<vtkIntArray> labels = ::NewLabelsArray(length);
//   if (indices)
//     {
//     LabelIndices(indices->indices, labels, 1);
//     }
//   return labels;
// }

//----------------------------------------------------------------------------
// vtkSmartPointer<vtkIntArray> vtkPCLConversions::NewLabelsArray(const std::vector<pcl::PointIndices>& indices, vtkIdType length)
// {
//   vtkSmartPointer<vtkIntArray> labels = ::NewLabelsArray(length);
//
//   for (size_t i = 0; i < indices.size(); ++i)
//     {
//     const int labelValue = i + 1;
//     LabelIndices(indices[i].indices, labels, labelValue);
//     }
//
//   return labels;
// }

