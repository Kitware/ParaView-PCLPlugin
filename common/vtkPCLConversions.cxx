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
#include "_PCLP_DECLARE_CONV.h"

#include <vtkCellArray.h>
// #include <vtkIdList.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
// #include <vtkTimerLog.h>

// #include <pcl/io/pcd_io.h>

#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/seq/transform.hpp>
#include <boost/preprocessor/variadic/to_seq.hpp>

#include <cassert>

//------------------------------------------------------------------------------
/*!
 * @brief Common converter base class.
 * @tparam PointT The PCL point type that this converter will handle.
 */
template <typename PointT>
struct ConvBase
{
protected:
  vtkSmartPointer<vtkPolyData> PolyData;
  /*!
   * @brief The field data used to store attributes, which may either be the
   *        PolyData instance's point data or field data.
   */
  vtkSmartPointer<vtkFieldData> FieldData;

  /*!
   * @brief Get an array pointer from a PolyData instance.
   * @param[in] polyData The PolyData instance.
   * @param[in] name The name of the array.
   * @return A pointer to the array, or nullptr if it does not exist.
   *
   * By default this retrieves the array from the PolyData instance's field
   * data. ConvXYZ overrides this method to retrieve data from the point data
   * instead.
   */
  static
  vtkAbstractArray * GetArray(vtkPolyData * polyData, char const * name)
  {
    vtkFieldData * fieldData = polyData->GetFieldData();
    if (fieldData != nullptr)
    {
      return fieldData->GetAbstractArray(name);
    }
    else
    {
      return nullptr;
    }
  }

public:
  typedef PointT PointType;


  ConvBase()
    : PolyData { vtkSmartPointer<vtkPolyData>::New() }
    , FieldData { PolyData->GetFieldData() }
  {
  }
  ConvBase(vtkPolyData * polyData)
    : PolyData { polyData }
    , FieldData { PolyData->GetFieldData() }
  {
  }

  //! @brief ShallowCopy the internal PolyData instance to the given pointer.
  void GetPolyData(vtkPolyData * polyData)
  {
    polyData->ShallowCopy(this->PolyData);
  }

  //! @brief Set the number of points.
  virtual
  void SetNumberOfPoints(vtkIdType numberOfPoints) {};

  //! @brief Copy data from a point into the internal array.
  virtual
  void CopyFromPoint(vtkIdType i, PointType const & point) {};

  //! @brief Copy data from an internal array into a point.
  virtual
  void CopyToPoint(vtkIdType i, PointType & point) const {};

  //! @brief Insert field names of this point type into a set.
  virtual
  void InsertFieldNames(std::set<std::string> & fieldNames) const {};

  //! @brief Check that the given point type contains the required fields.
  bool HasRequiredFields(std::set<std::string> const * requiredFieldNames)
  {
    // Return true if there are no required names.
    if (requiredFieldNames == nullptr || requiredFieldNames->size() == 0)
    {
      return true;
    }
    std::set<std::string> fields;
    this->InsertFieldNames(fields);
    for (auto & fieldName : (* requiredFieldNames))
    {
      if (fields.find(fieldName) == fields.end())
      {
        return false;
      }
    }
    return true;
  }

  /*!
   * @brief Placeholder GetScore method.
   *
   * Calls to GetScore are propagated up the class hierarchy so the base class
   * needs an invocation endpoint for the chain.
   *
   * This should return a score to measure how well the attributes handled by
   * the subclass match that argument. If the argument possesses all of the
   * recognized attributes, set the score to the number of attributes recognized
   * by the subclass. If any attributes are missing, immediately return -1 if
   * negativeIfMissing is true, otherwise set the score to 0 and invoke GetScore
   * with the same arguments in the subclass's base class. If that score is
   * negative, return it immediately, otherwise return the sum of the two
   * scores.
   */
  template <typename GetScoreArgT>
  static
  int GetScore(GetScoreArgT gsa, bool negativeIfMissing = true)
  {
    return 0;
  }
};

//------------------------------------------------------------------------------
/*!
 * @brief Common base-class for all XYZ PCL point types.
 *
 * All XYZ PCL point clouds (i.e. those with PCL point types in
 * PCL_XYZ_POINT_TYES) are interconverted to vtkPolyData instances. All others
 * are interconverted to vtkFieldData instances.
 *
 * This uses the same SFINAE trick as the classes declared with
 * _PCLP_DECLARE_CONV, i.e. point types with x, y and z attributes will
 * instantiate the full declaration below while those without will instantiate
 * the empty declaration.
 */
template <typename PointType, typename = int>
struct ConvXYZ : public ConvBase<PointType>
{
  ConvXYZ() {};
  ConvXYZ(vtkPolyData * polyData) : ConvBase<PointType> { polyData } {}
};

template <typename PointType>
struct ConvXYZ<
  PointType, 
  decltype(
    (void) PointType::x,
    (void) PointType::y,
    (void) PointType::z,
    0
  )
> : public ConvBase<PointType>
{
protected:
  vtkSmartPointer<vtkPoints> Points;

public:
  typedef ConvXYZ<PointType> ThisClassT;
  typedef ConvBase<PointType> BaseClassT;

  ConvXYZ()
    : Points { vtkSmartPointer<vtkPoints>::New() }
  {
    // PCL XYZ data is stored as floats.
    this->Points->SetDataTypeToFloat();
    this->PolyData->SetPoints(this->Points);
    this->FieldData = this->PolyData->GetPointData();
  }

  ConvXYZ(vtkPolyData * polyData)
    : BaseClassT { polyData }
    , Points { this->PolyData->GetPoints() }
  {
    this->FieldData = this->PolyData->GetPointData();
  }

  static
  vtkAbstractArray * GetArray(vtkPolyData * polyData, char const * name)
  {
    vtkFieldData * fieldData = polyData->GetPointData();
    if (fieldData != nullptr)
    {
      return fieldData->GetAbstractArray(name);
    }
    else
    {
      return nullptr;
    }
  }


  virtual
  void SetNumberOfPoints(vtkIdType numberOfPoints) override
  {
    this->Points->SetNumberOfPoints(numberOfPoints);

  }

  virtual
  void CopyFromPoint(vtkIdType i, PointType const & point) override
  {
    this->Points->SetPoint(i, point.x, point.y, point.z);
  }

  virtual
  void CopyToPoint(vtkIdType i, PointType & point) const override
  {
    // float * pointData = static_cast<float *>(this->Points->GetData()->GetVoidPointer(i));
    auto * pointData = this->Points->GetPoint(i);
    point.x = pointData[0];
    point.y = pointData[1];
    point.z = pointData[2];
  }

  static
  int GetScore(
    vtkPolyData * polyData, 
    bool negativeIfMissing = true
  )
  {
    int count = (polyData->GetPoints() == nullptr) ? 0 : 3;
    if (count == 0 && negativeIfMissing)
    {
      return -1;
    }
    int score = BaseClassT::GetScore(polyData, negativeIfMissing);
    return (score < 0) ? score : (score + count);
  }

  static
  int GetScore(
    std::set<std::string> const & fieldNames, 
    bool negativeIfMissing = true
  )
  {
    return (
      _PCLP_IS_IN_SET(fieldNames, "x") &&
      _PCLP_IS_IN_SET(fieldNames, "y") &&
      _PCLP_IS_IN_SET(fieldNames, "z")
    ) ? 3 : (negativeIfMissing ? -1 : 0);
  }

  virtual
  void InsertFieldNames(std::set<std::string> & fieldNames) const override
  {
    fieldNames.insert("x");
    fieldNames.insert("y");
    fieldNames.insert("z");
    BaseClassT::InsertFieldNames(fieldNames);
  }
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

// Internal macro to invoke _PCLP_DECLARE_CONV with parent class, current class
// and current class attributes.
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
// ScoreTracker
//------------------------------------------------------------------------------
/*!
 * @brief Convenience class to modularize score evaluations in
 *        vtkPCLConversions::GetPointTypeIndex().
 *
 * This just invokes ConvPoint::GetScore for all recognized PCL point types
 * while also checking that the type has all required fields.
 */
struct ScoreTracker
{
  int Score, HighestScore, Index;
  std::set<std::string> const * RequiredFieldNames;
  
  /*!
   * @param[in] requiredFieldNames An optional set of field name strings. Point
   *                               types that do not contain all of these string
   *                               will not be scored.
   */
  ScoreTracker(std::set<std::string> const * requiredFieldNames = nullptr)
    : Score { -1 }
    , HighestScore {-1 }
    , Index { -1 }
    , RequiredFieldNames { requiredFieldNames }
  {
  }

  /*!
   * @brief Update the score and index if the passed score is higher than that
   *        previous highest score.
   */
  void Update(int index, int score)
  {
    if (score > this->HighestScore)
    {
      this->HighestScore = score;
      this->Index = index;
    }
  }

  /*!
   * @brief     Calculate the score for the given point type and argument.
   * @tparam    PointType   The PCL point type.
   * @tparam    T           The type of the argument to pass to the
   *                        ConvPoint::GetScore().
   * @param[in] index       The index of this point type.
   * @param[in] getScoreArg The argument to pass to the ConvPoint::GetScore().
   */
  template <typename PointType, typename T>
  void Update(T getScoreArg)
  {
    ConvPoint<PointType> conv;
    if (conv.HasRequiredFields(this->RequiredFieldNames))
    {
      // Pass false to GetScore so that it counts the number of matched attributes
      // instead of returning -1 if any are missing.
      this->Update(getPointTypeIndex<PointType>(), conv.GetScore(getScoreArg, false));
    }
  }

  /*!
   * @brief     Score all PCL XYZ point types and return the index of the
   *            highest score.
   * @tparam    T           The type of the argument to pass to the
   *                        ConvPoint::GetScore().
   * @param[in] getScoreArg The argument to pass to the ConvPoint::GetScore().
   * @return    The index of the best matching point type.
   */
  template <typename T>
  int GetPointTypeIndex(T getScoreArg)
  {
#define _PCLP_SCORE_POINT_TYPES(r, data, i, PointType) this->Update<PointType>(getScoreArg);

    BOOST_PP_SEQ_FOR_EACH_I(_PCLP_SCORE_POINT_TYPES, _, PCL_XYZ_POINT_TYPES)
    return this->Index;
  }
};





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
 * @brief      Templated function to convert all PCL XYZ point types to a
 *             PolyData instance with attribute arrays that can be recovered
 *             when converting back to a PCL point cloud.
 * @tparam     CloudT   The PCL cloud type.
 * @param[in]  cloud    The shared pointer to the cloud.
 * @param[out] polyData The PolyData instance.
 * @return     True if the conversion was successful, false otherwise.
 */
template <typename CloudT>
void InternalPolyDataFromPointCloud(
  typename CloudT::ConstPtr & cloud,
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
  conv.GetPolyData(polyData);
}

//------------------------------------------------------------------------------
/*!
 * @brief      Templated function to convert a PolyData instance to a PCL point
 *             cloud.
 * @tparam     CloudT   The PCL cloud type.
 * @param[in]  polyData The input PolyData instance.
 * @param[out] cloud    The point cloud.
 * @return     True if the conversion was successful, false otherwise.
 *
 * The completeness of the return type depends on the presence of expected data
 * arrays in the PolyData instance. These will exist if the PolyData was
 * generated with InternalPolyDataFromPointCloud. If the PolyData was not
 * generated from a point cloud, the cloud type should be determined prior to
 * the invocation of this function by inspecting the PolyData instance.
 */
template <typename CloudT>
void InternalPointCloudFromPolyData(
  vtkPolyData * polyData,
  typename CloudT::Ptr & cloud
)
{
  vtkIdType numberOfPoints =  0;
  vtkPointData * pointData = polyData->GetPointData();
  vtkFieldData * fieldData = polyData->GetFieldData();
  if (pointData != nullptr)
  {
    numberOfPoints = polyData->GetNumberOfPoints();
  }
  else if (fieldData != nullptr)
  {
    numberOfPoints = fieldData->GetNumberOfTuples();
  }

  cloud->width = numberOfPoints;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize(numberOfPoints);
  if (! numberOfPoints)
  {
    return;
  }
  ConvPoint<typename CloudT::PointType> conv(polyData);
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
    InternalPolyDataFromPointCloud<pcl::PointCloud<PointType>>(cloud, polyData); \
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
template <typename T>
int vtkPCLConversions::GetPointTypeIndex(T getScoreArg)
{
  ScoreTracker st;
  return st.GetPointTypeIndex(getScoreArg);
}

//------------------------------------------------------------------------------
template <typename T>
int vtkPCLConversions::GetPointTypeIndex(
  T getScoreArg,
  std::set<std::string> const & requiredFieldNames
)
{
  ScoreTracker st(& requiredFieldNames);
  return st.GetPointTypeIndex(getScoreArg);
}

//------------------------------------------------------------------------------
template <typename PointType>
void vtkPCLConversions::GetFieldNames(std::set<std::string> & fieldNames)
{
  ConvPoint<PointType> conv;
  conv.InsertFieldNames(fieldNames);
}

//------------------------------------------------------------------------------
// Explicit instantiation of required templates for all supported
// ConvPoint::GetScore arguments.
#define _PCLP_INSTANTIATE_GetPointTypeIndex(type)                                               \
  template int vtkPCLConversions::GetPointTypeIndex<type>(type);                                \
  template int vtkPCLConversions::GetPointTypeIndex<type>(type, std::set<std::string> const &);

_PCLP_INSTANTIATE_GetPointTypeIndex(vtkPolyData *)
_PCLP_INSTANTIATE_GetPointTypeIndex(std::set<std::string> const &)

#undef _PCLP_INSTANTIATE_GetPointTypeIndex

//------------------------------------------------------------------------------
// Template specialization for all PCL XYZ point types.
#define _PCLP_INSTANTIATE_GetPointTypeIndex(r, data, i, PointType)                    \
  template void vtkPCLConversions::GetFieldNames<PointType>(std::set<std::string> &); \
                                                                                      \
  template <>                                                                         \
  int vtkPCLConversions::GetPointTypeIndex<PointType const &>(                        \
    PointType const & point                                                           \
  )                                                                                   \
  {                                                                                   \
    return getPointTypeIndex<PointType>();                                            \
  }                                                                                   \
                                                                                      \
  template <>                                                                         \
  int vtkPCLConversions::GetPointTypeIndex<PointType const &>(                        \
    PointType const & point,                                                          \
    std::set<std::string> const & requiredFieldNames                                  \
  )                                                                                   \
  {                                                                                   \
    std::set<std::string> currentFieldNames;                                          \
    vtkPCLConversions::GetFieldNames<PointType>(currentFieldNames);                   \
    ScoreTracker st(& requiredFieldNames);                                            \
    return st.GetPointTypeIndex(currentFieldNames);                                   \
  }


BOOST_PP_SEQ_FOR_EACH_I(_PCLP_INSTANTIATE_GetPointTypeIndex, _, PCL_XYZ_POINT_TYPES)

#undef _PCLP_INSTANTIATE_GetPointTypeIndex


//------------------------------------------------------------------------------
// Legacy code from previous version of plugin.
// TODO: update/remove?
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

