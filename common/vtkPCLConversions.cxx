//=========================================================================
//
// Copyright 2012-2018 Kitware, Inc.
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
#define _SET_FROM_ARRAY(i, arr, var) var = arr[i];

//----------------------------------------------------------------------------
/*!
 * @brief Base class for SFINAE-based point conversions.
 * @tparam T The PCL point type, e.g. PointXYZ.
 *
 * The PolyData is managed here in this base class along with the point data.
 * Templated subclasses declared using the _DECLARE_CONV macro will either
 * declare to shallow wrappers around this class if the point type lacks the
 * passed attributes, or a full wrapper that handles the passed attributes
 * before passing along the point to the parent. This allows a linear hierarchy
 * of converter classes to each handle their own set of attributes. The compiler
 * should flatten the nested calls into a single function.
 *
 * Note that using this as the common base class limits support to those PCL
 * points types that contain XYZ data. If at some point this plugin should move
 * beyond XYZ data to support all of the PCL point types then an empty base
 * class should be used instead and this one should be implemented using the
 * same approach as for the other attributes below.
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
    // PCL XYZ data is stored as floats.
    this->Points->SetDataTypeToFloat();
    this->PolyData = vtkSmartPointer<vtkPolyData>::New();
    this->PolyData->SetPoints(this->Points);
  }

  ConvXYZ(vtkSmartPointer<vtkPolyData> polyData) : PolyData { polyData }
  {
    this->Points = this->PolyData->GetPoints();
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
    float * pointData = static_cast<float *>(this->Points->GetData()->GetVoidPointer(i));
    point.x = pointData[0];
    point.y = pointData[1];
    point.z = pointData[2];
  }
};

//----------------------------------------------------------------------------
/*!
 * @brief Declare a templated converter and a specialization for point types
 *        that possess the passed attributes.
 * @param[in] name The name of this class. It will be prefixed with "Conv". The unprefixed name will also be used for the
 * PolyData array name.
 * @param[in] parent The parent class. It will also be prefixed with "Conv".
 * @param[in] ... The attributes that are handled by this class, e.g. "x,y,z" or
 * "r,g,b".
 */
#define _DECLARE_CONV(name, parent, ...)                                                    \
  /* Class to handle points without the given attributes. */                                \
  template <typename PointType, typename = int>                                             \
  struct Conv ## name : public Conv ## parent<PointType>                                    \
  {                                                                                         \
    /* Default constructor and constructor to pass though PolyData instance. */             \
    Conv ## name() {};                                                                      \
    Conv ## name(vtkSmartPointer<vtkPolyData> & polyData)                                   \
      : Conv ## parent<PointType>(polyData) {};                                             \
  };                                                                                        \
                                                                                            \
  /* Class to handle points with attributes. */                                             \
  template <typename PointType>                                                             \
  struct Conv ## name<PointType, decltype(                                                  \
    BOOST_PP_SEQ_ENUM(                                                                      \
      BOOST_PP_SEQ_TRANSFORM(                                                               \
        _CAST_VOID,                                                                         \
        PointType,                                                                          \
        BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__))                                              \
    )                                                                                       \
    ,0)> : public Conv ## parent<PointType>                                                 \
  {                                                                                         \
    /* The PolyData array type is determined from the first attribute of the point data. */ \
    typedef decltype(                                                                       \
      PointType::BOOST_PP_SEQ_ELEM(                                                         \
        0,                                                                                  \
        BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__)                                               \
      )                                                                                     \
    ) ElementType;                                                                          \
    typedef vtkAOSDataArrayTemplate<ElementType> ArrayType;                                 \
    /* The array */                                                                         \
    vtkSmartPointer<ArrayType> name ## Array;                                               \
                                                                                            \
    /* Default constructor. Create a new array to hold the attributes. */                   \
    Conv ## name()                                                                          \
    {                                                                                       \
      this->name ## Array = vtkSmartPointer<ArrayType>::New();                              \
      this->name ## Array->SetName(# name);                                                 \
      this->name ## Array->SetNumberOfComponents(BOOST_PP_VARIADIC_SIZE(__VA_ARGS__));      \
      this->PolyData->GetPointData()->AddArray(this->name ## Array);                        \
    }                                                                                       \
                                                                                            \
    /* Get the array from the PolyData instance. */                                         \
    Conv ## name(vtkSmartPointer<vtkPolyData> polyData)                                     \
      : Conv ## parent<PointType> { polyData }                                              \
    {                                                                                       \
      this->name ## Array =                                                                 \
        ArrayType::SafeDownCast(this->PolyData->GetPointData()->GetAbstractArray(# name));  \
    }                                                                                       \
                                                                                            \
    virtual                                                                                 \
    void SetNumberOfPoints(vtkIdType numberOfPoints) override                               \
    {                                                                                       \
      this->name ## Array->SetNumberOfTuples(numberOfPoints);                               \
      this->Conv ## parent<PointType>::SetNumberOfPoints(numberOfPoints);                   \
    }                                                                                       \
                                                                                            \
    virtual                                                                                 \
    void CopyFromPoint(vtkIdType i, PointType const & point) override                       \
    {                                                                                       \
      ElementType data[] {                                                                  \
        BOOST_PP_SEQ_ENUM(                                                                  \
          BOOST_PP_SEQ_TRANSFORM(                                                           \
            _GET_ATTR,                                                                      \
            point,                                                                          \
            BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__)                                           \
          )                                                                                 \
        )                                                                                   \
      };                                                                                    \
      this->name ## Array->SetTypedTuple(i, data);                                          \
      this->Conv ## parent<PointType>::CopyFromPoint(i, point);                             \
    }                                                                                       \
                                                                                            \
    virtual                                                                                 \
    void CopyToPoint(vtkIdType i, PointType & point) const override                         \
    {                                                                                       \
      ElementType values[BOOST_PP_VARIADIC_SIZE(__VA_ARGS__)] {0};                          \
      if (this->name ## Array != nullptr)                                                   \
      {                                                                                     \
        this->name ## Array->GetTypedTuple(i, values);                                      \
      }                                                                                     \
      BOOST_PP_SEQ_FOR_EACH(                                                                \
        _SET_FROM_ARRAY,                                                                    \
        values,                                                                             \
        BOOST_PP_SEQ_TRANSFORM(                                                             \
          _GET_ATTR,                                                                        \
          point,                                                                            \
          BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__)                                             \
        )                                                                                   \
      )                                                                                     \
      this->Conv ## parent<PointType>::CopyToPoint(i, point);                               \
    }                                                                                       \
  };

//----------------------------------------------------------------------------
// Declare classes for all attributes that should be preserved across
// conversions between VTK PolyData and PCL point clouds.
//----------------------------------------------------------------------------
_DECLARE_CONV(RGB      , XYZ      , r,g,b)
_DECLARE_CONV(Alpha    , RGB      , a)
_DECLARE_CONV(HSV      , Alpha    , h,s,v)
_DECLARE_CONV(Intensity, HSV      , intensity)
_DECLARE_CONV(Label    , Intensity, label)
_DECLARE_CONV(Strength , Label    , strength)
_DECLARE_CONV(Normal   , Strength , normal_x,normal_y,normal_z)
_DECLARE_CONV(Curvature, Normal   , curvature)
_DECLARE_CONV(Viewpoint, Curvature, vp_x,vp_y, vp_z)

// Create an alias for the final child that can be used transparently in
// functions.
template <typename PointType>
using ConvPointBaseClass = ConvViewpoint<PointType>;

template <typename PointType>
struct ConvPoint : public ConvPointBaseClass<PointType>
{
  ConvPoint() {};
  ConvPoint(vtkSmartPointer<vtkPolyData> polyData) : ConvPointBaseClass<PointType>(polyData) {};
};

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
/*!
 * @brief     Templated function to convert all PCL XYZ point types to a
 *            PolyData instance with attribute arrays that can be recovered when
 *            converting back to a PCL point cloud.
 * @tparam    CloudT The PCL cloud type.
 * @param[in] cloud  The shared pointer to the cloud.
 * @return    The PolyData instance.
 */
template <typename CloudT>
inline
// Ideally the input parameter should be "CloudT::ConstPtr" or
// "pcl::PointCloud<PointType>::ConstPtr" with "PointType" as the template
// parameter, but that raises compilation errors (e.g. templated variables). The
// workaround is to use the explicit type of PointCloud<PointType>::ConstPtr as
// defined in pcl/point_cloud.h.
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

//----------------------------------------------------------------------------
/*!
 * @brief     Templated function to convert a PolyData instance to a PCL point
 *            cloud.
 * @tparam    CloudT   The PCL cloud type.
 * @param[in] polyData The input polydata.
 * @return    The point cloud.
 *
 * The completeness of the return type depends on the presence of expected data
 * arrays in the PolyData instance. These will exist if the PolyData was
 * generated with InternalPolyDataFromPointCloud. If the PolyData was not
 * generated from a point cloud, the cloud type should be determined prior to
 * the invocation of this function by inspecting the PolyData instance.
 */
template <typename CloudT>
inline
// See notes for InternalPolyDataFromPointCloud about template parameters.
void InternalPointCloudFromPolyData(vtkPolyData * polyData, boost::shared_ptr<CloudT> & cloud)
{
  const vtkIdType numberOfPoints = polyData->GetNumberOfPoints();

  cloud->width = numberOfPoints;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize(numberOfPoints);
  if (! numberOfPoints)
  {
    return;
  }
  ConvPoint<typename CloudT::PointType> conv(polyData);
  // ConvXYZ<pcl::PointXYZ> conv(polyData);
  for (vtkIdType i = 0; i < numberOfPoints; ++i)
  {
    conv.CopyToPoint(i, cloud->points[i]);
  }
}

//----------------------------------------------------------------------------
// Define converters for all XYZ point types.
#define DEFINE_CONVERTER(i, data, PointType)                                            \
  vtkSmartPointer<vtkPolyData> vtkPCLConversions::PolyDataFromPointCloud(               \
    pcl::PointCloud<PointType>::ConstPtr cloud                                          \
  )                                                                                     \
  {                                                                                     \
    return InternalPolyDataFromPointCloud(cloud);                                       \
  }                                                                                     \
  void vtkPCLConversions::PointCloudFromPolyData(                                       \
    vtkSmartPointer<vtkPolyData> polyData,                                              \
    pcl::PointCloud<PointType>::Ptr & cloud                                             \
  )                                                                                     \
  {                                                                                     \
    return InternalPointCloudFromPolyData<pcl::PointCloud<PointType>>(polyData, cloud); \
  }
BOOST_PP_SEQ_FOR_EACH(DEFINE_CONVERTER, _, PCL_XYZ_POINT_TYPES)

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

