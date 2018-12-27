//==============================================================================
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
//==============================================================================

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

#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/seq/transform.hpp>
#include <boost/preprocessor/variadic/to_seq.hpp>

#include <cassert>

//------------------------------------------------------------------------------
// Boost-based macros for determining point attributes.
//------------------------------------------------------------------------------
//! @brief Internal macro passed to BOOST_PP_SEQ_TRANSFORM in _PCLP_DECLARE_CONV.
#define _PCLP_CAST_VOID(r, name, attr) (void) name::attr

//------------------------------------------------------------------------------
//! @brief Internal macro passed to BOOST_PP_SEQ_TRANSFORM in _PCLP_DECLARE_CONV.
#define _PCLP_GET_ATTR(r, obj, attr) obj.attr

//------------------------------------------------------------------------------
//! @brief Internal macro passed to BOOST_PP_SEQ_TRANSFORM in _PCLP_DECLARE_CONV.
#define _PCLP_SET_FROM_ARRAY(r, arr, i, var) var = arr[i];

//------------------------------------------------------------------------------
//! @brief Check if an element is in a set.
//! @todo  Remove this in favor of the set's "contains" method (C++20).
#define _PCLP_IS_IN_SET(set, element) (set.find(element) != set.end())

//------------------------------------------------------------------------------
//! @brief Return -1 if an element is not in the set.
#define _PCLP_RETURN_MINUS_ONE_IF_NOT_IN_SET(r, set, i, name) \
  if (! _PCLP_IS_IN_SET(set, BOOST_PP_STRINGIZE(name)))       \
  {                                                      \
    return -1;                                           \
  }

//------------------------------------------------------------------------------
/*!
 * @brief Base class for SFINAE-based point conversions.
 * @tparam T The PCL point type, e.g. PointXYZ.
 *
 * The PolyData is managed here in this base class along with the point data.
 * Templated subclasses declared using the _PCLP_DECLARE_CONV macro will either
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

  ConvXYZ(vtkPolyData * polyData) : PolyData { polyData }
  {
    this->Points = this->PolyData->GetPoints();
  }

  //! @brief Set the number of points.
  virtual
  void SetNumberOfPoints(vtkIdType numberOfPoints)
  {
    this->Points->SetNumberOfPoints(numberOfPoints);
  }

  //! @brief Copy data from a point into the internal array.
  virtual
  void CopyFromPoint(vtkIdType i, PointType const & point)
  {
    this->Points->SetPoint(i, point.x, point.y, point.z);
  }

  //! @brief Copy data from an internal array into a point.
  virtual
  void CopyToPoint(vtkIdType i, PointType & point) const
  {
    // float * pointData = static_cast<float *>(this->Points->GetData()->GetVoidPointer(i));
    auto * pointData = this->Points->GetPoint(i);
    point.x = pointData[0];
    point.y = pointData[1];
    point.z = pointData[2];
  }

  //! @brief Get a score to estimate how well a given PolyData instance matches
  //!        the expected attributes of this PCL point type.
  virtual
  int GetAttributeScore(vtkPolyData * polyData)
  {
    return (polyData->GetPoints() == nullptr) ? -1 : 3;
  }

  //! @brief Same as GetAttributeScore but for PCD file field names.
  virtual
  int GetFieldNameScore(std::set<std::string> & fields)
  {
    return (
      _PCLP_IS_IN_SET(fields, "x") &&
      _PCLP_IS_IN_SET(fields, "y") &&
      _PCLP_IS_IN_SET(fields, "z")
    ) ? 3 : -1;
  }
};

//------------------------------------------------------------------------------
/*!
 * @brief     Declare a templated converter and a specialization for point types
 *            that possess the passed attributes.
 * @param[in] name   The name of this class. The macro will add the "Conv"
 *                   prefix. The unprefixed name will be used for the PolyData
 *                   array name.
 * @param[in] parent The name of the parent class. The macro will add the "Conv"
 *                   prefix.
 * @param[in] attrs  The preprocessor sequence of attributes recognized by this
 *                   class, e.g. "(r)(g)(b)" or
 *                   "(normal_x)(normal_y)(normal_z)".
 */
#define _PCLP_DECLARE_CONV(parent, name, attrs)                                             \
  /* Class to handle points without the given attributes. */                                \
  template <typename PointType, typename = int>                                             \
  struct BOOST_PP_CAT(Conv, name) : public BOOST_PP_CAT(Conv, parent)<PointType>            \
  {                                                                                         \
    /* Default constructor and constructor to pass though PolyData instance. */             \
    BOOST_PP_CAT(Conv, name)() {};                                                          \
    BOOST_PP_CAT(Conv, name)(vtkPolyData * polyData)                                        \
      : BOOST_PP_CAT(Conv, parent)<PointType>(polyData) {};                                 \
  };                                                                                        \
                                                                                            \
  /* Class to handle points with attributes. */                                             \
  template <typename PointType>                                                             \
  struct BOOST_PP_CAT(Conv, name)<PointType, decltype(                                      \
    BOOST_PP_SEQ_ENUM(                                                                      \
      BOOST_PP_SEQ_TRANSFORM(                                                               \
        _PCLP_CAST_VOID,                                                                    \
        PointType,                                                                          \
        attrs                                                                               \
      )                                                                                     \
    )                                                                                       \
   ,0)> : public BOOST_PP_CAT(Conv, parent)<PointType>                                      \
  {                                                                                         \
    /* The PolyData array type is determined from the first attribute of the point data. */ \
    typedef decltype(                                                                       \
      PointType::BOOST_PP_SEQ_ELEM(                                                         \
        0,                                                                                  \
        attrs                                                                               \
      )                                                                                     \
    ) ElementType;                                                                          \
    typedef vtkAOSDataArrayTemplate<ElementType> ArrayType;                                 \
    /* The array */                                                                         \
    vtkSmartPointer<ArrayType> BOOST_PP_CAT(name, Array);                                   \
                                                                                            \
    /* Default constructor. Create a new array to hold the attributes. */                   \
    BOOST_PP_CAT(Conv, name)()                                                              \
    {                                                                                       \
      this->BOOST_PP_CAT(name, Array) = vtkSmartPointer<ArrayType>::New();                  \
      this->BOOST_PP_CAT(name, Array)->SetName(BOOST_PP_STRINGIZE(name));                   \
      this->BOOST_PP_CAT(name, Array)->SetNumberOfComponents(BOOST_PP_SEQ_SIZE(attrs));     \
      this->PolyData->GetPointData()->AddArray(this->BOOST_PP_CAT(name, Array));            \
    }                                                                                       \
                                                                                            \
    /* Get the array from the PolyData instance. */                                         \
    BOOST_PP_CAT(Conv, name)(vtkPolyData * polyData)                                        \
      : BOOST_PP_CAT(Conv, parent)<PointType> { polyData }                                  \
    {                                                                                       \
      this->BOOST_PP_CAT(name, Array) =                                                     \
        ArrayType::SafeDownCast(                                                            \
          this->PolyData->GetPointData()->GetAbstractArray(BOOST_PP_STRINGIZE(name))        \
        );                                                                                  \
    }                                                                                       \
                                                                                            \
    virtual                                                                                 \
    void SetNumberOfPoints(vtkIdType numberOfPoints) override                               \
    {                                                                                       \
      this->BOOST_PP_CAT(name, Array)->SetNumberOfTuples(numberOfPoints);                   \
      this->BOOST_PP_CAT(Conv, parent)<PointType>::SetNumberOfPoints(numberOfPoints);       \
    }                                                                                       \
                                                                                            \
    virtual                                                                                 \
    void CopyFromPoint(vtkIdType i, PointType const & point) override                       \
    {                                                                                       \
      ElementType data[] {                                                                  \
        BOOST_PP_SEQ_ENUM(                                                                  \
          BOOST_PP_SEQ_TRANSFORM(                                                           \
            _PCLP_GET_ATTR,                                                                 \
            point,                                                                          \
            attrs                                                                           \
          )                                                                                 \
        )                                                                                   \
      };                                                                                    \
      this->BOOST_PP_CAT(name, Array)->SetTypedTuple(i, data);                              \
      this->BOOST_PP_CAT(Conv, parent)<PointType>::CopyFromPoint(i, point);                 \
    }                                                                                       \
                                                                                            \
    virtual                                                                                 \
    void CopyToPoint(vtkIdType i, PointType & point) const override                         \
    {                                                                                       \
      ElementType values[BOOST_PP_SEQ_SIZE(attrs)] {0};                                     \
      if (this->BOOST_PP_CAT(name, Array) != nullptr)                                       \
      {                                                                                     \
        this->BOOST_PP_CAT(name, Array)->GetTypedTuple(i, values);                          \
      }                                                                                     \
      BOOST_PP_SEQ_FOR_EACH_I(                                                              \
        _PCLP_SET_FROM_ARRAY,                                                               \
        values,                                                                             \
        BOOST_PP_SEQ_TRANSFORM(                                                             \
          _PCLP_GET_ATTR,                                                                   \
          point,                                                                            \
          attrs                                                                             \
        )                                                                                   \
      )                                                                                     \
      this->BOOST_PP_CAT(Conv, parent)<PointType>::CopyToPoint(i, point);                   \
    }                                                                                       \
                                                                                            \
    virtual                                                                                 \
    int GetAttributeScore(vtkPolyData * polyData)                                           \
    {                                                                                       \
      int score = -1;                                                                       \
      if (polyData->GetPointData()->GetAbstractArray(BOOST_PP_STRINGIZE(name)) != nullptr)  \
      {                                                                                     \
        score = this->BOOST_PP_CAT(Conv, parent)<PointType>::GetAttributeScore(polyData);   \
        if (score >= 0)                                                                     \
        {                                                                                   \
          score += BOOST_PP_SEQ_SIZE(attrs);                                                \
        }                                                                                   \
      }                                                                                     \
      return score;                                                                         \
    }                                                                                       \
                                                                                            \
    virtual                                                                                 \
    int GetFieldNameScore(std::set<std::string> & fields)                                   \
    {                                                                                       \
      BOOST_PP_SEQ_FOR_EACH_I(                                                              \
        _PCLP_RETURN_MINUS_ONE_IF_NOT_IN_SET,                                               \
        fields,                                                                             \
        attrs                                                                               \
      )                                                                                     \
      int score = this->BOOST_PP_CAT(Conv, parent)<PointType>::GetFieldNameScore(fields);   \
      if (score >= 0)                                                                       \
      {                                                                                     \
        score += BOOST_PP_SEQ_SIZE(attrs);                                                  \
      }                                                                                     \
      return score;                                                                         \
    }                                                                                       \
  };

//------------------------------------------------------------------------------
// Declare a linear class hierarchy for all attributes that should be preserved
// across conversions between VTK PolyData and PCL point clouds.
//
// The first element in the sequence is just a placeholder to specify the base
// class ("XYZ" for "ConvXYZ"). Each element in the sequence after it is itself
// a sequence, the first element of which is the name to use for the class (it
// will be prefixed with "Conv") and the remaining elements of which are the
// names of the point attributes to handle. The name (without the "Conv" prefix)
// will also be used as the name of the array in the PolyData to transfer these
// attributes alongside the point data.
//
// The sequence below should include all the fields defined in
// pcl/impl/point_types.hpp. The very first sequence determines the base class
// (ConvXYZ). Subsequent sequences will use the first element of the preceding
// sequence as its own base class. For example, in this case ConvNormal inherits
// from ConvXYZ and ConvRGB inherits from ConvNormal. The order thus determines
// the class hierarchy but should have no effect on the conversion itself. To
// facilitate keeping this list up-to-date, it is recommended that the order
// follow the order of the PCL definitions so that the header can be easily
// scanned for changes.
//
// The result of these declarations is a templated class hierarchy that accepts
// the PCL point type as its sole template parameter. Each class in the
// hierarchy will handle specific attributes and then invoke the superclass's
// method to deal with other attributes. For each template instantiation, the
// compiler will inline these calls resulting in point-type-specific methods
// that deal with all of the point's attributes in a single call.
#define _PCLP_DC_ATTR_SEQ                  \
  ((XYZ))                                  \
  ((Normal)(normal_x)(normal_y)(normal_z)) \
  ((RGB)(r)(g)(b))                         \
  ((Alpha)(a))                             \
  ((HSV)(h)(s)(v))                         \
  ((Intensity)(intensity))                 \
  ((Label)(label))                         \
  ((Strength)(strength))                   \
  ((Curvature)(curvature))                 \
  ((Viewpoint)(vp_x)(vp_y)(vp_z))          \
  ((Scale)(scale))                         \
  ((Angle)(angle))                         \
  ((Response)(response))                   \
  ((Octave)(octave))

//------------------------------------------------------------------------------
// Macros for BOOST_PP_FOR to convert the sequence into a series of class
// declarations.

// The state, which is just a tuple of the current index and the max index.
#define _PCLP_DC_STATE (0, BOOST_PP_DEC(BOOST_PP_DEC(BOOST_PP_SEQ_SIZE(_PCLP_DC_ATTR_SEQ))))

// The for-loop condition. Stops when the index reaches the max index.
#define _PCLP_DC_PRED(r, state)                    \
  BOOST_PP_NOT_EQUAL(                              \
    BOOST_PP_TUPLE_ELEM(3, 0, state),              \
    BOOST_PP_INC(BOOST_PP_TUPLE_ELEM(3, 1, state)) \
  )

// The loop operation. Increments the index each loop.
#define _PCLP_DC_OP(r, state)                       \
  (                                                 \
    BOOST_PP_INC(BOOST_PP_TUPLE_ELEM(3, 0, state)), \
    BOOST_PP_TUPLE_ELEM(3, 1, state)                \
  )

// Internal macro to invoke _DECLARE_CONF with parent class, current class and
// current class attributes.
#define _PCLP_DC_MACRO_I(i,j)                                   \
  _PCLP_DECLARE_CONV(                                           \
    BOOST_PP_SEQ_HEAD(BOOST_PP_SEQ_ELEM(i, _PCLP_DC_ATTR_SEQ)), \
    BOOST_PP_SEQ_HEAD(BOOST_PP_SEQ_ELEM(j, _PCLP_DC_ATTR_SEQ)), \
    BOOST_PP_SEQ_TAIL(BOOST_PP_SEQ_ELEM(j, _PCLP_DC_ATTR_SEQ))  \
  )

// Loop macro. It's just a wrapper around _PCLP_DC_MACRO_I to invoke it with the
// current index and the current index + 1.
#define _PCLP_DC_MACRO(r, state)                   \
  _PCLP_DC_MACRO_I(                                \
    BOOST_PP_TUPLE_ELEM(3, 0, state),              \
    BOOST_PP_INC(BOOST_PP_TUPLE_ELEM(3, 0, state)) \
  )

// The for loop to declare the hierarchy.
BOOST_PP_FOR(_PCLP_DC_STATE, _PCLP_DC_PRED, _PCLP_DC_OP, _PCLP_DC_MACRO)

// The last class in the sequence, with the "Conv" prefix.
#define _PCLP_DC_LAST                                         \
    BOOST_PP_CAT(                                             \
      Conv,                                                   \
      BOOST_PP_SEQ_HEAD(                                      \
        BOOST_PP_SEQ_ELEM(                                    \
          BOOST_PP_DEC(BOOST_PP_SEQ_SIZE(_PCLP_DC_ATTR_SEQ)), \
          _PCLP_DC_ATTR_SEQ                                   \
        )                                                     \
      )                                                       \
    )

//------------------------------------------------------------------------------
// Create an alias for the final child that can be used transparently elsewhere.
// This avoids the need to update the rest of the code if the attribute sequence
// is modified.
//! @brief The only point converter class that should be used directly.
template <typename PointType>
using ConvPoint = _PCLP_DC_LAST<PointType>;



//------------------------------------------------------------------------------
// vtkPCLConversions
//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLConversions);

//------------------------------------------------------------------------------
vtkPCLConversions::vtkPCLConversions()
{
}

//------------------------------------------------------------------------------
vtkPCLConversions::~vtkPCLConversions()
{
}

//------------------------------------------------------------------------------
void vtkPCLConversions::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
/*!
 * @brief     Templated function to convert all PCL XYZ point types to a
 *            PolyData instance with attribute arrays that can be recovered when
 *            converting back to a PCL point cloud.
 * @tparam    CloudT The PCL cloud type.
 * @param[in] cloud  The shared pointer to the cloud.
 * @return    The PolyData instance.
 */
template <typename CloudT>
// Ideally the input parameter should be "CloudT::ConstPtr" or
// "pcl::PointCloud<PointType>::ConstPtr" with "PointType" as the template
// parameter, but that raises compilation errors (it's misrecognized as a
// templated variable). The workaround is to use the explicit type of
// PointCloud<PointType>::ConstPtr as defined in pcl/point_cloud.h.
void InternalPolyDataFromPointCloud(
  boost::shared_ptr<CloudT const> cloud,
  vtkPolyData * polyData
)
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
  polyData->ShallowCopy(conv.PolyData);
}

//------------------------------------------------------------------------------
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
// See notes for InternalPolyDataFromPointCloud about template parameters.
void InternalPointCloudFromPolyData(
  vtkPolyData * polyData,
  boost::shared_ptr<CloudT> & cloud
)
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

//------------------------------------------------------------------------------
// Define converters for all XYZ point types.
#define _DEFINE_CONVERTER(r, data, PointType)                                    \
  void vtkPCLConversions::PolyDataFromPointCloud(                                \
    pcl::PointCloud<PointType>::ConstPtr cloud,                                  \
    vtkPolyData * polyData                                                       \
  )                                                                              \
  {                                                                              \
    return InternalPolyDataFromPointCloud(cloud, polyData);                      \
  }                                                                              \
  void vtkPCLConversions::PointCloudFromPolyData(                                \
    vtkPolyData * polyData,                                                      \
    pcl::PointCloud<PointType>::Ptr & cloud                                      \
  )                                                                              \
  {                                                                              \
    InternalPointCloudFromPolyData<pcl::PointCloud<PointType>>(polyData, cloud); \
  }
BOOST_PP_SEQ_FOR_EACH(_DEFINE_CONVERTER, _, PCL_XYZ_POINT_TYPES)

//------------------------------------------------------------------------------
//! @brief Get the index of the best matching PCL point type in the
//!        PCL_XYZ_POINT_TYPES sequence.
int vtkPCLConversions::GetPointTypeIndex(vtkPolyData * polyData)
{
  int score = -1, highestScore = -1;
  int index = -1;

#define _POINT_TYPE_INDEX_UPDATER_PD(r, data, i, PointType)   \
  score = ConvPoint<PointType>().GetAttributeScore(polyData); \
  if (score > highestScore )                                  \
  {                                                           \
    highestScore = score;                                     \
    index = i;                                                \
  }

  BOOST_PP_SEQ_FOR_EACH_I(_POINT_TYPE_INDEX_UPDATER_PD, _, PCL_XYZ_POINT_TYPES)

  return index;
}

//------------------------------------------------------------------------------
int vtkPCLConversions::GetPointTypeIndex(std::set<std::string> & fieldNames)
{
  int score = -1, highestScore = -1;
  int index = -1;

#define _POINT_TYPE_INDEX_UPDATER_FN(r, data, i, PointType)     \
  score = ConvPoint<PointType>().GetFieldNameScore(fieldNames); \
  if (score > highestScore )                                    \
  {                                                             \
    highestScore = score;                                       \
    index = i;                                                  \
  }

  BOOST_PP_SEQ_FOR_EACH_I(_POINT_TYPE_INDEX_UPDATER_FN, _, PCL_XYZ_POINT_TYPES)

  return index;
}

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
// Legacy code to be removed?
//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
// vtkSmartPointer<vtkIntArray> vtkPCLConversions::NewLabelsArray(pcl::IndicesConstPtr indices, vtkIdType length)
// {
//   vtkSmartPointer<vtkIntArray> labels = ::NewLabelsArray(length);
//   if (indices)
//     {
//     LabelIndices(*indices, labels, 1);
//     }
//   return labels;
// }

//------------------------------------------------------------------------------
// vtkSmartPointer<vtkIntArray> vtkPCLConversions::NewLabelsArray(pcl::PointIndices::ConstPtr indices, vtkIdType length)
// {
//   vtkSmartPointer<vtkIntArray> labels = ::NewLabelsArray(length);
//   if (indices)
//     {
//     LabelIndices(indices->indices, labels, 1);
//     }
//   return labels;
// }

//------------------------------------------------------------------------------
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

