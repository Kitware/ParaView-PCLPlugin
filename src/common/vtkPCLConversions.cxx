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

/*!
 * @file
 * This unit provides methods to convert all PCL point clouds back and forth to
 * PolyData instances so that they may transit the ParaView data pipeline
 * without loss of data. All PCL point types defined in the PCL_POINT_TYPES
 * macro sequence in pcl/impl/point_types.hpp are supported. This relies on some
 * macro magic and a SFINAE trick to preserve attributes across conversions.
 * While the use of these functions in filters and sources typically only
 * requires a few boilerplate lines, the implementation here is non-trivial and
 * requires explanation.
 *
 * The different PCL point types are independent structs with different
 * attributes (member variables). In order to pass a point cloud of a given type
 * between filters/sources/etc. in ParaView, each point must be converted to a
 * VTK data type along with all of its attributes. A simple generic template is
 * not possible in this case given the variety of attribute sets and
 * specialization for each point type would require separate functions for the
 * (at the time or writing) 47 PCL point types.
 *
 * Although the point types remain independent, several share attributes of the
 * same name, often in sets. For example, there are a number of point types with
 * spatial "x", "y" and "z" attributes. Likewise for the color attributes "r",
 * "g" and "b". The approach used here is to define the attributes that should
 * be preserved across conversions and then use macros and templates to generate
 * converters for each point type that preserves its specific set of attributes.
 * Once all preservable attributes are identified, converters can be
 * automatically generated for all of the point types with a single preprocessor
 * sequence (and two special cases for XYZ and XY points).
 *
 * The SFINAE (substitution failure is not an error) trick is to declare a bare
 * class with passthrough constructors (if necessary) and then a templated class
 * specialization as follows:
 *
 *     template <typename PointType, typename = int>
 *     struct FooBarBaz : public BaseClass<PointType>
 *     {
 *       ...
 *     };
 *
 *     template <typename PointType>
 *     struct FooBarBaz<PointType, decltype(
 *       (void) PointType::foo,
 *       (void) PointType::bar,
 *       (void) PointType::baz,
 *       0
 *     )> : public BaseClass<PointType>
 *     {
 *       ...
 *     };
 *
 * decltype accepts an expression as an argument which in this case is a
 * comma-separated list of point attributes that should be handled by this
 * class. If all 3 attributes are present, then the expression is valid and
 * becomes the type of the last argument due to the behavior of the comma
 * operator. The second class definition will thus be instantiated as a
 * specialization of th efirst for the point type passed in the template. If any
 * of the given attributes are absent in the point type, the substitution will
 * fail and the first definition will be instantiated.
 *
 * This trick is used in macro definitions to declare classes to handle
 * different sets of attributes (such as r,g,b or h,s,v). By chaining together
 * multiple attribute handlers for all possible point attributes, the compiler
 * is able to instantiate classes for each point type that handles all of its
 * attributes. Given that the different point types are mostly different
 * combinations of the same attribute sets, this reduces the number of cases
 * that need to be handled. Furthermore, Boost preprocessor macros are used to
 * further generalize the definitions through attribute inspection so that a
 * single macro can declare a handle for any attribute set.
 *
 * All point clouds are interconverted to PolyData instances. XYZ attributes are
 * converted to points in the PolyData's point set while all other attributes
 * are converted to arrays in either the PolyData's PointData (if the point
 * conains XYZ data) or the PolyData's FieldData otherwise. Aside from
 * preserving data between filters, it also allows the user to do things such as
 * color points by their RGB and alpha values and display arrow glyphs for
 * surface normals.
 *
 * For ParaView users, the PCL data is just another PolyData instance with
 * multiple attributes which may or may not be interesting. For plugin
 * developers, PCL point clouds can be passed from filter to filter without
 * considering how the data is managed in transit. The flexibility of the
 * converters allow all attributes to be preserved if desired, with several
 * inspection methods provided to determine which point type best corresponds to
 * the input data so that it can be automatically converted if desired. If not,
 * the conversions can be explicitly chosen via template parameters to force
 * conversions which may either discard data or create points with uninitialized
 * attributes (for the initialization of which the developer is then responsible).
 *
 * See vtkPCLPassThroughFilter for example usage.
 */

#include "vtkPCLConversions.h"
#include "_PCLP_DECLARE_CONV.h"

#include <vtkCellArray.h>
// #include <vtkIdList.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkFloatArray.h>
#include <vtkDoubleArray.h>
// #include <vtkTimerLog.h>

// #include <cassert>

//------------------------------------------------------------------------------
/*!
  * @brief         Create a single vertex cell for each point if the given
  *                PolyData instance contains points.
  * @param[in,out] A PolyData instance to which to add cell data.
  */
void addVertexCells(vtkPolyData * polyData)
{
  vtkPointData * pointData = polyData->GetPointData();
  if (pointData == nullptr)
  {
    return;
  }
  vtkIdType numberOfVerts = polyData->GetNumberOfPoints();
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
  polyData->SetVerts(cellArray);
}

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
    return polyData->GetFieldData()->GetAbstractArray(name);
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

  //! @brief Check if the point is finite (true for all non-XYZ types).
  virtual
  bool IsFinite(PointType const & point) const
  {
    return true;
  }

  //! @brief ShallowCopy the internal PolyData instance to the given pointer.
  void GetPolyData(vtkPolyData * polyData)
  {
    addVertexCells(this->PolyData);
    polyData->ShallowCopy(this->PolyData);
  }


  //! @brief Return true if the given PolyData instance appears to be
  //!        convertible, i.e. all existing field data arrays have the expected
  //!        number of components..
  static
  bool AppearsConvertible(vtkPolyData * polyData)
  {
    return true;
  }

  //! @brief     Get the number of points expected after conversion.
  //! @param[in] polyData The PolyData instance to check. If nullptr, check the
  //!                     internal instance.
  virtual
  vtkIdType GetNumberOfPoints(vtkPolyData * polyData = nullptr)
  {
    if (polyData == nullptr)
    {
      polyData = this->PolyData;
    }
    return (polyData == nullptr) ? 0 : polyData->GetFieldData()->GetNumberOfTuples();
  }

  //! @brief Set the number of points.
  virtual
  void SetNumberOfPoints(vtkIdType numberOfPoints) {};

  //! @brief Copy data from a point into the internal array.
  virtual
  void CopyFromPoint(vtkIdType i, PointType const & point) {};

  //! @brief Copy data from a cloud into the internal array.
  virtual
  void CopyFromCloud(typename pcl::PointCloud<PointType>::ConstPtr & cloud) {};

  //! @brief Copy data from an internal array into a point.
  virtual
  void CopyToPoint(vtkIdType i, PointType & point) const {};

  //! @brief Copy data from an internal array into a cloud.
  virtual
  void CopyToCloud(typename pcl::PointCloud<PointType>::Ptr & cloud) const {};

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

  //! @brief Check if the given point type contains a particular field.
  bool HasField(std::string const & fieldName)
  {
    std::set<std::string> fields;
    this->InsertFieldNames(fields);
    return (fields.find(fieldName) != fields.end());
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
// See notes below at the redefinition of _PCLP_DC_ATTR_SEQ. This definition is
// here to handle the special case of XY points, which must be the base class of
// ConvXYZ in the case that the z attribute is missing. If this were declared
// below in the full chain then all XYZ points would contain a duplicate vector
// of XY points. With the definition here, point types with x, y and z
// attributes will instantiate the full definition of ConvXYZ with ConvBase as
// its parent, whereas those with only x and y will instantiate the bare
// definition with ConvXY as its parent.
#undef _PCLP_DC_ATTR_SEQ
#define _PCLP_DC_ATTR_SEQ \
  ((Base))                \
  ((XY)(x)(y))

BOOST_PP_FOR(_PCLP_DC_STATE, _PCLP_DC_PRED, _PCLP_DC_OP, _PCLP_DC_MACRO)

//------------------------------------------------------------------------------
/*!
 * @brief Common intermediate base-class for all XYZ PCL point types.
 *
 * All XYZ PCL point clouds (i.e. those with PCL point types in
 * PCL_XYZ_POINT_TYPES) will store their attributes in the PolyData's PointData
 * while non-XYZ point types will store theirs in the FieldData.
 *
 * This uses the same SFINAE trick as the classes declared with
 * _PCLP_DECLARE_CONV, i.e. point types with x, y and z attributes will
 * instantiate the full declaration below while those without will instantiate
 * the bare declaration.
 */
template <typename PointType, typename = int>
struct ConvXYZ : public ConvXY<PointType>
{
  ConvXYZ() {};
  ConvXYZ(vtkPolyData * polyData) : ConvXY<PointType> { polyData } {}
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

  static
  vtkAbstractArray * GetArray(vtkPolyData * polyData, char const * name)
  {
    return polyData->GetPointData()->GetAbstractArray(name);
  }

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

  //! @brief Check if the point is finite.
  virtual
  bool IsFinite(PointType const & point) const override
  {
    return pcl::isFinite(point);
  }

  static
  bool AppearsConvertible(vtkPolyData * polyData)
  {
    return true;
    // return (polyData->GetNumberOfPoints() > 0) ? BaseClassT::AppearsConvertible : false;
  }

  virtual
  vtkIdType GetNumberOfPoints(vtkPolyData * polyData = nullptr)
  {
    if (polyData == nullptr)
    {
      polyData = this->PolyData;
    }
    return (polyData == nullptr) ? 0 : polyData->GetNumberOfPoints();
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
  void CopyFromCloud(typename pcl::PointCloud<PointType>::ConstPtr & cloud) override
  {
    vtkIdType numberOfPoints = cloud->points.size();
    if (numberOfPoints == 0)
    {
      return;
    }
    vtkFloatArray * floatPoints = vtkFloatArray::SafeDownCast(this->Points->GetData());
    vtkDoubleArray * doublePoints = vtkDoubleArray::SafeDownCast(this->Points->GetData());
    // assert((floatPoints != nullptr) || (doublePoints != nullptr));

    if (floatPoints != nullptr)
    {
      float * data = floatPoints->GetPointer(0);
      for (vtkIdType i = 0; i < numberOfPoints; ++i)
      {
        data[i*3]   = cloud->points[i].x;
        data[i*3+1] = cloud->points[i].y;
        data[i*3+2] = cloud->points[i].z;
      }
    }
    else if (doublePoints != nullptr)
    {
      double * data = doublePoints->GetPointer(0);
      for (vtkIdType i = 0; i < numberOfPoints; ++i)
      {
        data[i*3]   = cloud->points[i].x;
        data[i*3+1] = cloud->points[i].y;
        data[i*3+2] = cloud->points[i].z;
      }
    }

    // auto * points = this->Points->GetPointer();
    // for (vtkIdType i = 0; i < n; ++i)
    // {
    //   auto * pd = points->GetPoint(i);
    //   auto * c = cloud->points[i];
    //   pd[0] = c.x;
    //   pd[1] = c.y;
    //   pd[2] = c.z;
    // }
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

  virtual
  void CopyToCloud(typename pcl::PointCloud<PointType>::Ptr & cloud) const override
  {
    vtkIdType numberOfPoints = this->PolyData->GetNumberOfPoints();
    if (numberOfPoints == 0)
    {
      return;
    }
    vtkFloatArray * floatPoints = vtkFloatArray::SafeDownCast(this->Points->GetData());
    vtkDoubleArray * doublePoints = vtkDoubleArray::SafeDownCast(this->Points->GetData());
    // assert((floatPoints != nullptr) || (doublePoints != nullptr));

    if (floatPoints != nullptr)
    {
      float * data = floatPoints->GetPointer(0);
      for (vtkIdType i = 0; i < numberOfPoints; ++i)
      {
        cloud->points[i].x = data[i*3];
        cloud->points[i].y = data[i*3+1];
        cloud->points[i].z = data[i*3+2];
      }
    }
    else if (doublePoints != nullptr)
    {
      double * data = doublePoints->GetPointer(0);
      for (vtkIdType i = 0; i < numberOfPoints; ++i)
      {
        cloud->points[i].x = data[i*3];
        cloud->points[i].y = data[i*3+1];
        cloud->points[i].z = data[i*3+2];
      }
    }

    // auto * points = this->Points->GetPointer();
    // for (vtkIdType i = 0; i < n; ++i)
    // {
    //   auto * pd = points->GetPoint(i);
    //   auto * c = cloud->points[i];
    //   c.x = pd[0];
    //   c.y = pd[1];
    //   c.z = pd[2];
    // }
  }

  static
  int GetScore(
    vtkPolyData * polyData,
    bool negativeIfMissing = true
  )
  {
    int count = (polyData->GetNumberOfPoints() > 0) ? 3 : 0;
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
// names of the point attributes to handle together (i.e. those that are
// inseparable such as r,g,b or r_min,r_max). The name (without the "Conv"
// prefix) will also be used as the name of the array in the PolyData to
// transfer these attributes alongside the point data.
//
// Aside from simple grouped attributes such as r,g,b and r_min,r_max, these
// declarations also handle c-array attributes such as "histogram" and "values".
// The size of the array will be used to set the size of the tuple of the
// internal PolyData array. Note that *c-array attributes must not be grouped*.
// For example, the different histogram and values arrays are all supported with
//
//     ((Histogram)(histogram))
//     ((Values)(values))
//
// but these *cannot* be grouped together as ((Foo)(histogram)(values)), unlike
// non-array types, e.g.:
//
//     ((HSV)(h)(s)(v))
//     ((RGBRatios)(r_ratio)(g_ratio)(b_ratio))
//
// Aside from the first element which determines the base class of this
// sub-hierarchy, the order is irrelevant and may be changed to facilitate
// keeping the list synchronized with the attributes in
// pcl/impl/point_types.hpp. The alias ConvPoint is set to the last class in the
// sequence so that this sequence may be changed without further modification of
// the code.
//
// The result of these declarations is a templated class hierarchy that accepts
// the PCL point type as its sole template parameter. Each class in the
// hierarchy will handle specific attributes and then invoke the superclass's
// method to deal with other attributes. For each template instantiation, the
// compiler will inline these calls resulting in point-type-specific methods
// that deal with all of the point's attributes in a single call.
//
// In order for this to handle all point types, the sequence must be kept this
// up to date with the attributes defined in pcl/impl/point_types.hpp. Check the
// PCL header directly to be sure and please notify the developers of this
// plugin of any unhandled attributes.
//
// A special note about the attributes f1-f10: At the time of writing, these
// attributes appear in different feature points where their values are
// unrelated across point types. f1-f4 appear in PPFSignature and PPFGBSignature
// while f1-f10 appear in CPPFSignature. Ideally, f1-f4 would be grouped
// together in a single internal array for the former two, while f1-f10 would be
// grouped together in a single array for the latter (it would be even better if
// there were simply a c-array in both cases, i.e. f[4] and f[10]). The
// limitation here is that only attribute names are considered so these cases
// cannot be distinguished directly: any point that contains f1-f10 also
// contains f1-f4 so declaring a group for both would lead to duplication of
// f1-f4 for all points with f1-f10. This could be handled using the method
// above for x,y,z and x,y at the cost of added complexity, but this is not
// necessary because the f* attributes are separable, unlike x,y,z for spatial
// data. They are thus grouped as f1-f4 and f5-f10. This will result in two
// internal arrays for points with f1-f10 but the relative simplicity should
// outweigh the cost of an extra array. If not, the xyz/xy method can be used.
#undef _PCLP_DC_ATTR_SEQ
#define _PCLP_DC_ATTR_SEQ                                                                               \
  ((XYZ))                                                                                               \
  ((Alpha)(a))                                                                                          \
  ((AlphaM)(alpha_m))                                                                                   \
  ((Angle)(angle))                                                                                      \
  ((BoundaryPoint)(boundary_point))                                                                     \
  ((Curvature)(curvature))                                                                              \
  ((Descriptor)(descriptor))                                                                            \
  ((F1F4)(f1)(f2)(f3)(f4))                                                                              \
  ((F5F10)(f5)(f6)(f7)(f8)(f9)(f10))                                                                    \
  ((Gradient)(gradient))                                                                                \
  ((HeightVariance)(height_variance))                                                                   \
  ((HSV)(h)(s)(v))                                                                                      \
  ((Histogram)(histogram))                                                                              \
  ((Intensity)(intensity))                                                                              \
  ((IntensityVariance)(intensity_variance))                                                             \
  ((Label)(label))                                                                                      \
  ((MomentInvariants)(j1)(j2)(j3))                                                                      \
  ((Normal)(normal_x)(normal_y)(normal_z))                                                              \
  ((Octave)(octave))                                                                                    \
  ((Orientation)(orientation))                                                                          \
  ((PrincipalRadiiRSD)(r_min)(r_max))                                                                   \
  ((PrincipleCurvature)(principal_curvature_x)(principal_curvature_y)(principal_curvature_z)(pc1)(pc2)) \
  ((Range)(range))                                                                                      \
  ((XAxis)(x_axis))                                                                                     \
  ((YAxis)(y_axis))                                                                                     \
  ((ZAxis)(z_axis))                                                                                     \
  ((Response)(response))                                                                                \
  ((RF)(rf))                                                                                            \
  ((RGB)(r)(g)(b))                                                                                      \
  ((RGBRatios)(r_ratio)(g_ratio)(b_ratio))                                                              \
  ((Scale)(scale))                                                                                      \
  ((Strength)(strength))                                                                                \
  ((UV)(u)(v))                                                                                          \
  ((Values)(values))                                                                                    \
  ((Viewpoint)(vp_x)(vp_y)(vp_z))

BOOST_PP_FOR(_PCLP_DC_STATE, _PCLP_DC_PRED, _PCLP_DC_OP, _PCLP_DC_MACRO)

//------------------------------------------------------------------------------
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
      this->Update(PointMeta<PointType>::GetIndex(), conv.GetScore(getScoreArg, false));
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

    BOOST_PP_SEQ_FOR_EACH_I(_PCLP_SCORE_POINT_TYPES, _, PCLP_POINT_TYPES)
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
//! @brief Internal function to copy missing fields from one FieldData to
//!        another.
template <typename FieldDataT>
void copyAbsentFields(FieldDataT * src, FieldDataT * dst)
{
  int srcSize = src->GetNumberOfArrays();
  int index;
  for (int i = 0; i < srcSize; ++i)
  {
    vtkAbstractArray * array = src->GetAbstractArray(i);
    char const * name = array->GetName();
    if (dst->GetAbstractArray(name, index) == nullptr)
    {
      dst->AddArray(array);
    }
  }
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
  vtkPolyData * polyData,
  bool preserveExistingFields = false
)
{
  vtkIdType numberOfPoints = cloud->points.size();

  ConvPoint<typename CloudT::PointType> conv;
  conv.SetNumberOfPoints(numberOfPoints);

  if (cloud->is_dense)
  {
    // conv.CopyFromCloud(cloud);
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
      if (conv.IsFinite(cloud->points[i]))
      {
        conv.CopyFromPoint(j, cloud->points[i]);
        ++j;
      }
    }
    numberOfPoints = j;
    conv.SetNumberOfPoints(numberOfPoints);
  }


  if (preserveExistingFields)
  {
    vtkNew<vtkPolyData> tmpPolyData;
    conv.GetPolyData(tmpPolyData);

    vtkFieldData * existingFieldData = polyData->GetFieldData();
    vtkPointData * existingPointData = polyData->GetPointData();
    vtkFieldData * extraFieldData = tmpPolyData->GetFieldData();
    vtkPointData * extraPointData = tmpPolyData->GetPointData();

    copyAbsentFields(extraFieldData, existingFieldData);
    copyAbsentFields(extraPointData, existingPointData);
  }
  else
  {
    conv.GetPolyData(polyData);
  }
}

//------------------------------------------------------------------------------
/*!
 * @brief      Templated function to convert a PolyData instance to a PCL point
 *             cloud.
 * @tparam     CloudT   The PCL cloud type.
 * @param[in]  polyData The input PolyData instance.
 * @param[out] cloud    The point cloud.
 * @return     True if the conversion was successful, false otherwise.
#define _DEFINE_CONVERTER(r, data, PointType)                                    \
  void vtkPCLConversions::PolyDataFromPointCloud(                                \
    pcl::PointCloud<PointType>::ConstPtr cloud,                                  \
    vtkPolyData * polyData,                                                      \
    bool preserveExistingFields                                                  \
  )                                                                              \
  {                                                                              \
    InternalPolyDataFromPointCloud<pcl::PointCloud<PointType>>( \
      cloud, \
      polyData, \
      preserveExistingFields \
    ); \
  }                                                                              \
  void vtkPCLConversions::PointCloudFromPolyData(                                \
    vtkPolyData * polyData,                                                      \
    pcl::PointCloud<PointType>::Ptr & cloud                                      \
  )                                                                              \
  {                                                                              \
    InternalPointCloudFromPolyData<pcl::PointCloud<PointType>>( \
      polyData, \
      cloud \
    ); \
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
  if (ConvPoint<typename CloudT::PointType>::AppearsConvertible(polyData))
  {
    ConvPoint<typename CloudT::PointType> conv(polyData);
    vtkIdType numberOfPoints = conv.GetNumberOfPoints();

    cloud->width = numberOfPoints;
    cloud->height = 1;
    cloud->is_dense = true;
    cloud->points.resize(numberOfPoints);
    if (! numberOfPoints)
    {
      return;
    }
    // conv.CopyToCloud(cloud);
    for (vtkIdType i = 0; i < numberOfPoints; ++i)
    {
      conv.CopyToPoint(i, cloud->points[i]);
    }
  }
  else
  {
    cloud->width = 0;
    cloud->height = 1;
    cloud->is_dense = true;
    cloud->points.resize(0);
  }
}

//------------------------------------------------------------------------------
// Define converters for all XYZ point types.
#define _DEFINE_CONVERTER(r, data, PointType)                   \
  void vtkPCLConversions::PolyDataFromPointCloud(               \
    pcl::PointCloud<PointType>::ConstPtr cloud,                 \
    vtkPolyData * polyData,                                     \
    bool preserveExistingFields                                 \
  )                                                             \
  {                                                             \
    InternalPolyDataFromPointCloud<pcl::PointCloud<PointType>>( \
      cloud,                                                    \
      polyData,                                                 \
      preserveExistingFields                                    \
    );                                                          \
  }                                                             \
  void vtkPCLConversions::PointCloudFromPolyData(               \
    vtkPolyData * polyData,                                     \
    pcl::PointCloud<PointType>::Ptr & cloud                     \
  )                                                             \
  {                                                             \
    InternalPointCloudFromPolyData<pcl::PointCloud<PointType>>( \
      polyData,                                                 \
      cloud                                                     \
    );                                                          \
  }
BOOST_PP_SEQ_FOR_EACH(_DEFINE_CONVERTER, _, PCLP_POINT_TYPES)

#undef _DEFINE_CONVERTER



//------------------------------------------------------------------------------
template <typename PointType>
std::string vtkPCLConversions::GetPointTypeName(PointType const & point)
{
  return PointMeta<PointType>::GetName();
}

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
int vtkPCLConversions::GetHistogramPointTypeIndex(int size)
{
#define _HPTI_CASE_STATEMENT(r, data, i, PointType) \
  case sizeof(PointType::histogram) / sizeof(std::remove_extent<decltype(PointType::histogram)>::type): \
    return PointMeta<PointType>::GetIndex();

  switch (size)
  {
    BOOST_PP_SEQ_FOR_EACH_I(_HPTI_CASE_STATEMENT, _, PCLP_HISTOGRAM_POINT_TYPES)
  }
  return -1;

#undef _HPTI_CASE_STATEMENT
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
#define _PCLP_INSTANTIATE_POINT_TYPE_METHODS(r, data, i, PointType)                   \
  template std::string vtkPCLConversions::GetPointTypeName<PointType>(                \
    PointType const & point                                                           \
  );                                                                                  \
                                                                                      \
  template void vtkPCLConversions::GetFieldNames<PointType>(std::set<std::string> &); \
                                                                                      \
  template <>                                                                         \
  int vtkPCLConversions::GetPointTypeIndex<PointType const &>(                        \
    PointType const & point                                                           \
  )                                                                                   \
  {                                                                                   \
    return PointMeta<PointType>::GetIndex();                                          \
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


BOOST_PP_SEQ_FOR_EACH_I(_PCLP_INSTANTIATE_POINT_TYPE_METHODS, _, PCLP_POINT_TYPES)

#undef _PCLP_INSTANTIATE_POINT_TYPE_METHODS


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

