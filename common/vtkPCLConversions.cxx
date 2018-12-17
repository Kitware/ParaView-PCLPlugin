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
// Boost-based macros for determining point attributes.
//----------------------------------------------------------------------------
//! @brief Internal macro passed to BOOST_PP_SEQ_TRANSFORM in _DECLARE_HAS_ATTR.
#define _CAST_VOID(i, name, attr) (void) name::attr

//----------------------------------------------------------------------------
//! @brief Internal macro passed to BOOST_PP_SEQ_TRANSFORM in _DECLARE_HAS_ATTR.
#define _GET_ATTR(i, obj, attr) obj.attr

//----------------------------------------------------------------------------
//! @brief Internal macro passed to BOOST_PP_SEQ_TRANSFORM in _DECLARE_HAS_ATTR.
#define _SET(i, arr, var) var = arr[i];

//----------------------------------------------------------------------------
/*!
 * @brief Base class for SFINAE-based point conversions.
 *
 * This restricts all conversions to point types with XYZ data. The PolyData is
 * managed here in this base class along with the point data. Templated
 * subclasses declared using the _DECLARE_CONV macro will either declare to
 * shallow wrappers around this class if the point type lacks the passed
 * attributes, or a full wrapper that handles the passed attributes before
 * passing along the point to the parent. This allows a linear hierarchy of
 * converter classes to each handle their own set of attributes. The compiler
 * should flatten the nested calls into a single function.
 */
template <typename T>
struct ConvXYZ
{
  typedef T PointType;
  vtkSmartPointer<vtkPolyData> PolyData;
  vtkSmartPointer<vtkPoints> Points;

  ConvXYZ()
  {
    this->Points = vtkSmartPointer<vtkPoints>::New();
    this->Points->SetDataTypeToFloat();
    this->PolyData = vtkSmartPointer<vtkPolyData>::New();
    this->PolyData->SetPoints(this->Points);
  }

  ConvXYZ(vtkSmartPointer<vtkPolyData> polyData) : PolyData { polyData }
  {
    this->Points = this->PolyData->GetPoints;
  }

  virtual
  void SetNumberOfPoints(vtkIdType numberOfPoints)
  {
    this->Points->SetNumberOfPoints(numberOfPoints);
  }

  virtual
  void CopyFromPoint(vtkIdType i, PointType const & point)
  {
    this->Points->SetPoint(i, point.x, point.y, point.z);
  }

  virtual
  void CopyToPoint(vtkIdType i, PointType & point) const
  {
    auto pointData = this->Points->GetPoint(i);
    point.x = pointData[0];
    point.y = pointData[1];
    point.z = pointData[2];
  }
};

//----------------------------------------------------------------------------
/*!
 * @brief Declare a templated converter and a specialization for point types
 *        that possess the passed attributes.
 * @param[in] name The name of this class. The name will also be used for the
 * PolyData array.
 * @param[in] parent The parent class.
 * @param[in] ... The attributes that are handled by this class, e.g. "x,y,z" or
 * "r,g,b".
 */
#define _DECLARE_CONV(name, parent, ...)                                                                  \
  template <typename PointType, typename = int>                                                           \
  struct Conv ## name : public parent<PointType> {};                                                      \
                                                                                                          \
  template <typename PointType>                                                                           \
  struct Conv ## name<PointType, decltype(                                                                \
    BOOST_PP_SEQ_ENUM(                                                                                    \
      BOOST_PP_SEQ_TRANSFORM(_CAST_VOID, PointType, BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__))                \
    )                                                                                                     \
    ,0)> : public parent<PointType>                                                                       \
  {                                                                                                       \
    typedef decltype(PointType::BOOST_PP_SEQ_ELEM(0, BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__))) ElementType; \
    typedef vtkAOSDataArrayTemplate<ElementType> ArrayType;                                               \
    vtkSmartPointer<ArrayType> name ## Array;                                                             \
                                                                                                          \
    Conv ## name()                                                                                        \
    {                                                                                                     \
      this->name ## Array = vtkSmartPointer<ArrayType>::New();                                            \
      this->name ## Array->SetName(# name);                                                               \
      this->name ## Array->SetNumberOfComponents(BOOST_PP_VARIADIC_SIZE(__VA_ARGS__));                    \
      this->PolyData->GetPointData()->AddArray(this->name ## Array);                                      \
    }                                                                                                     \
    Conv ## name(vtkSmartPointer<vtkPolyData> polyData) : parent<PointType> { polyData }                  \
    {                                                                                                     \
      this->name ## Array = this->PolyData->GetAbstractArray(# name);                                     \
    }                                                                                                     \
                                                                                                          \
    virtual                                                                                               \
    void SetNumberOfPoints(vtkIdType numberOfPoints) override                                             \
    {                                                                                                     \
      this->name ## Array->SetNumberOfTuples(numberOfPoints);                                             \
      this->parent<PointType>::SetNumberOfPoints(numberOfPoints);                                         \
    }                                                                                                     \
                                                                                                          \
    virtual                                                                                               \
    void CopyFromPoint(vtkIdType i, PointType const & point) override                                     \
    {                                                                                                     \
      ElementType data[] {                                                                                \
        BOOST_PP_SEQ_ENUM(                                                                                \
          BOOST_PP_SEQ_TRANSFORM(                                                                         \
            _GET_ATTR,                                                                                    \
            point,                                                                                        \
            BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__)                                                         \
          )                                                                                               \
        )                                                                                                 \
      };                                                                                                  \
      this->name ## Array->SetTypedTuple(i, data);                                                        \
      this->parent<PointType>::CopyFromPoint(i, point);                                                   \
    }                                                                                                     \
                                                                                                          \
    virtual                                                                                               \
    void CopyToPoint(vtkIdType i, PointType & point) const override                                       \
    {                                                                                                     \
      ElementType values[BOOST_PP_VARIADIC_SIZE(__VA_ARGS__)];                                            \
      this->name ## Array->GetTypedTuple(i, values);                                                      \
      BOOST_PP_SEQ_FOR_EACH(                                                                              \
        _SET,                                                                                             \
        values,                                                                                           \
        BOOST_PP_SEQ_TRANSFORM(                                                                           \
          _GET_ATTR,                                                                                      \
          point,                                                                                          \
          BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__)                                                           \
        )                                                                                                 \
      )                                                                                                   \
      this->parent<PointType>::CopyToPoint(i, point);                                                     \
    }                                                                                                     \
  };

//----------------------------------------------------------------------------
// Declare attribute checks for all attributes that should be preserved across
// conversions between VTK PolyData and PCL point clouds.
//----------------------------------------------------------------------------
_DECLARE_CONV(RGB       , ConvXYZ       , r,g,b)
_DECLARE_CONV(Alpha     , ConvRGB       , a)
_DECLARE_CONV(HSV       , ConvAlpha     , h,s,v)
_DECLARE_CONV(Intensity , ConvHSV       , intensity)
_DECLARE_CONV(Label     , ConvIntensity , label)
_DECLARE_CONV(Strength  , ConvLabel     , strength)
_DECLARE_CONV(Normal    , ConvStrength  , normal_x,normal_y,normal_z)
_DECLARE_CONV(Curvature , ConvNormal    , curvature)
_DECLARE_CONV(Viewpoint , ConvCurvature , vp_x,vp_y, vp_z)

//! @brief Top-level wrapper for the conversion classes.
template <typename PointType>
struct ConvPoint : ConvViewpoint<PointType> {};

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
template <typename CloudT>
inline
vtkSmartPointer<vtkPolyData> InternalPolyDataFromPointCloud(boost::shared_ptr<CloudT const> cloud)
{
  vtkIdType numberOfPoints = cloud->points.size();
  ConvPoint<typename CloudT::PointType> conv;
  conv.SetNumberOfPoints(numberOfPoints);

  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < numberOfPoints; ++i)
    {
      conv.CopyFromPoint(i, cloud->points[i]);
    }
  }
  else
  {
    vtkIdType j = 0;    // true point index
    for (vtkIdType i = 0; i < numberOfPoints; ++i)
    {
      // Check if the point is invalid
      if (pcl::isFinite(cloud->points[i]))
      {
        conv.CopyFromPoint(j, cloud->points[i]);
        ++j;
      }
    }
    numberOfPoints = j;
    conv.SetNumberOfPoints(numberOfPoints);
  }
  conv.PolyData->SetVerts(vtkPCLConversions::NewVertexCells(numberOfPoints));
  return conv.PolyData;
}

#define DEFINE_CONVERTER(i, data, PointType)                                                                         \
  vtkSmartPointer<vtkPolyData> vtkPCLConversions::PolyDataFromPointCloud(pcl::PointCloud<PointType>::ConstPtr cloud) \
  {                                                                                                                  \
    return InternalPolyDataFromPointCloud(cloud);                                                                    \
  }
BOOST_PP_SEQ_FOR_EACH(DEFINE_CONVERTER, _, PCL_XYZ_POINT_TYPES)

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

