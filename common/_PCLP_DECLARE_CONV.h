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
//! @brief Insert a string element in a set.
#define _PCLP_INSERT_IN_SET(r, set, i, element) \
  set.insert(BOOST_PP_STRINGIZE(element));

//------------------------------------------------------------------------------
//! @brief If "missing" is false, sets "missing" to true if the name is not in
//!        the set.
#define _PCLP_MISSING_IF_NOT_IN_SET(r, set, i, name)              \
  if (! missing)                                                  \
  {                                                               \
    missing = (! _PCLP_IS_IN_SET(set, BOOST_PP_STRINGIZE(name))); \
  }

//------------------------------------------------------------------------------
/*!
 * @brief     Declare a templated converter that handles specific attributes of
 *            PCL point types.
 * @param[in] parent The name of the parent class. The macro will add the "Conv"
 *                   prefix.
 * @param[in] name   The name of this class. The macro will add the "Conv"
 *                   prefix. The unprefixed name will be used for the
 *                   vtkFieldData array name.
 * @param[in] attrs  The preprocessor sequence of attributes recognized by this
 *                   class, e.g. "(r)(g)(b)" or
 *                   "(normal_x)(normal_y)(normal_z)".
 *
 * The converters that handle different attributes are chained together in a
 * linear class hierarchy. The compiler flattens the hierarchy for each
 * instantiation to avoid repeated function invocations. The main complexity
 * here is the use of a SFINAE template trick. The macro defines two templated
 * structs. One depends on the presence of the given attributes in the PCL point
 * type (e.g. pcl::PointXYZRGB::r). If any of the attribute are missing, the
 * empty definition will be used (and thus omitted).
 *
 * The resulting per-PointType instantiation will thus handle only those
 * attribute sets that are present in the given point type.
 */
#define _PCLP_DECLARE_CONV(parent, name, attrs)                                             \
  /* Class to handle points without the given attributes. */                                \
  template <typename PointType, typename = int>                                             \
  struct BOOST_PP_CAT(Conv, name) : public BOOST_PP_CAT(Conv, parent)<PointType>            \
  {                                                                                         \
    /* Default constructor and constructor to pass though vtkPolyData instance. */          \
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
    typedef BOOST_PP_CAT(Conv, name)<PointType> ThisClassT;                                 \
    typedef BOOST_PP_CAT(Conv, parent)<PointType> BaseClassT;                               \
    /* The internal array type is determined from the first attribute of the point data. */ \
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
      this->FieldData->AddArray(this->BOOST_PP_CAT(name, Array));                           \
    }                                                                                       \
                                                                                            \
    /* Pass through the PolyData instance and get the FieldData from the base class. */     \
    BOOST_PP_CAT(Conv, name)(vtkPolyData * polyData)                                        \
      : BaseClassT { polyData }                                                             \
    {                                                                                       \
      this->BOOST_PP_CAT(name, Array) =                                                     \
        ArrayType::SafeDownCast(                                                            \
          this->FieldData->GetAbstractArray(BOOST_PP_STRINGIZE(name))                       \
        );                                                                                  \
    }                                                                                       \
                                                                                            \
    virtual                                                                                 \
    void SetNumberOfPoints(vtkIdType numberOfPoints) override                               \
    {                                                                                       \
      this->BOOST_PP_CAT(name, Array)->SetNumberOfTuples(numberOfPoints);                   \
      this->BaseClassT::SetNumberOfPoints(numberOfPoints);                                  \
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
      this->BaseClassT::CopyFromPoint(i, point);                                            \
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
      this->BaseClassT::CopyToPoint(i, point);                                              \
    }                                                                                       \
                                                                                            \
    static                                                                                  \
    int GetScore(vtkPolyData * polyData, bool negativeIfMissing = true)                     \
    {                                                                                       \
      int count =                                                                           \
        (ThisClassT::GetArray(polyData, BOOST_PP_STRINGIZE(name)) != nullptr)               \
        ? BOOST_PP_SEQ_SIZE(attrs) : 0;                                                     \
      if (count == 0 && negativeIfMissing)                                                  \
      {                                                                                     \
        return -1;                                                                          \
      }                                                                                     \
      int score = BaseClassT::GetScore(polyData, negativeIfMissing);                        \
      return (score < 0) ? score : score + count;                                           \
    }                                                                                       \
                                                                                            \
    static                                                                                  \
    int GetScore(std::set<std::string> const & fieldNames, bool negativeIfMissing = true)   \
    {                                                                                       \
      /* Count matching attributes. */                                                      \
      bool missing = false;                                                                 \
      BOOST_PP_SEQ_FOR_EACH_I(                                                              \
        _PCLP_MISSING_IF_NOT_IN_SET,                                                        \
        fieldNames,                                                                         \
        attrs                                                                               \
      )                                                                                     \
      int count = missing ? 0 : BOOST_PP_SEQ_SIZE(attrs);                                   \
      if (count == 0 && negativeIfMissing)                                                  \
      {                                                                                     \
        return -1;                                                                          \
      }                                                                                     \
      int score = BaseClassT::GetScore(fieldNames, negativeIfMissing);                      \
      return (score < 0) ? score : score + count;                                           \
    }                                                                                       \
                                                                                            \
    virtual                                                                                 \
    void InsertFieldNames(std::set<std::string> & fieldNames) const override                \
    {                                                                                       \
      BOOST_PP_SEQ_FOR_EACH_I(                                                              \
        _PCLP_INSERT_IN_SET,                                                                \
        fieldNames,                                                                         \
        attrs                                                                               \
      )                                                                                     \
      BaseClassT::InsertFieldNames(fieldNames);                                             \
    }                                                                                       \
  };

