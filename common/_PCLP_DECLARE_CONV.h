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

#ifndef _PCLP_DECLARE_CONV_h
#define _PCLP_DECLARE_CONV_h

#include <type_traits>
#include <sstream>

#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/seq/transform.hpp>
#include <boost/preprocessor/variadic/to_seq.hpp>

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
#define _PCLP_VAR_EQUALS_ARRAY_VALUE(r, arr, i, var) var = arr[i];

//------------------------------------------------------------------------------
//! @brief Internal macro passed to BOOST_PP_SEQ_TRANSFORM in _PCLP_DECLARE_CONV.
#define _PCLP_ARRAY_VALUE_EQUALS_VAR(r, arr, i, var) arr[i] = var;

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
 *                   "(normal_x)(normal_y)(normal_z)". Array attributes should
 *                   be passed alone, e.g. (descriptor). They will be detected
 *                   with std::is_array and their size will set the size of the
 *                   internal tuple.
 *
 * The converters that handle different attributes are chained together in a
 * linear class hierarchy. The compiler flattens the hierarchy for each
 * instantiation to avoid repeated function invocations. The main complexity
 * here is the use of a SFINAE template trick. The macro defines two templated
 * structs. One depends on the presence of the given attributes in the PCL point
 * type (e.g. pcl::PointXYZRGB::r, pcl::PointXYZRGB::g and pcl::PointXYZRGB::b).
 * If any of the attribute are missing, the empty definition will be used and
 * optimized out by the compiler.
 *
 * For each point type the compiler will thus instantiate a class that handles
 * all of the attributes of that point type. For details of which attributes are
 * supported, refer to the documentation of the preprocessor sequence
 * _PCLP_DC_ATTR_SEQ in vtkPCLConversions.cxx.
 * */
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
  /* Class to handle points with the given attributes. */                                   \
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
                                                                                            \
    /* The internal array type is determined from the first attribute of the point data. */ \
    /* This will remove all extends if it is an array type. */                              \
    typedef typename std::remove_extent<                                                    \
      decltype(PointType::BOOST_PP_SEQ_ELEM(0, attrs))                                      \
    >::type ElementType;                                                                    \
                                                                                            \
    /* For template instantiation, the actual attribute type is required to */              \
    /* avoid compilation errors due to the absence of compile-time constant */              \
    /* if expressions.                                                      */              \
    typedef decltype(PointType::BOOST_PP_SEQ_ELEM(0, attrs)) AttributeType;                 \
                                                                                            \
    /* The VTK array type. */                                                               \
    typedef vtkAOSDataArrayTemplate<ElementType> ArrayType;                                 \
                                                                                            \
    /* The array */                                                                         \
    vtkSmartPointer<ArrayType> Array;                                                       \
                                                                                            \
    /* ArraySize is either the number of attributes, or number of elements */               \
    /* in the first (and only) attribute if it is an array.                */               \
    static constexpr size_t ArraySize =                                                     \
      std::is_array<decltype(PointType::BOOST_PP_SEQ_ELEM(0, attrs))>::value ?              \
      sizeof(PointType::BOOST_PP_SEQ_ELEM(0, attrs)) / sizeof(ElementType) :                \
      BOOST_PP_SEQ_SIZE(attrs);                                                             \
                                                                                            \
    /* Array attributes append their length to the array name for scoring. */               \
    static std::string GetArrayName()                                                       \
    {                                                                                       \
      std::ostringstream name;                                                              \
      name << BOOST_PP_STRINGIZE(name);                                                     \
      if (std::is_array<decltype(PointType::BOOST_PP_SEQ_ELEM(0, attrs))>::value)           \
      {                                                                                     \
        name << ArraySize;                                                                  \
      }                                                                                     \
      return name.str();                                                                    \
    }                                                                                       \
                                                                                            \
    /* Default constructor. Create a new array to hold the attributes. */                   \
    BOOST_PP_CAT(Conv, name)()                                                              \
    {                                                                                       \
      this->Array = vtkSmartPointer<ArrayType>::New();                                      \
      this->Array->SetName(ThisClassT::GetArrayName().c_str());                             \
      this->Array->SetNumberOfComponents(ThisClassT::ArraySize);                            \
      this->FieldData->AddArray(this->Array);                                               \
    }                                                                                       \
                                                                                            \
    /* Pass through the PolyData instance and get the FieldData from the base class. */     \
    BOOST_PP_CAT(Conv, name)(vtkPolyData * polyData)                                        \
      : BaseClassT { polyData }                                                             \
    {                                                                                       \
      this->Array =                                                                         \
        ArrayType::SafeDownCast(                                                            \
          this->FieldData->GetAbstractArray(ThisClassT::GetArrayName().c_str())             \
        );                                                                                  \
    }                                                                                       \
                                                                                            \
    virtual                                                                                 \
    void SetNumberOfPoints(vtkIdType numberOfPoints) override                               \
    {                                                                                       \
      this->Array->SetNumberOfTuples(numberOfPoints);                                       \
      this->BaseClassT::SetNumberOfPoints(numberOfPoints);                                  \
    }                                                                                       \
                                                                                            \
    static                                                                                  \
    int GetScore(vtkPolyData * polyData, bool negativeIfMissing = true)                     \
    {                                                                                       \
      int count =                                                                           \
        (ThisClassT::GetArray(polyData, ThisClassT::GetArrayName().c_str()) != nullptr)     \
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
                                                                                            \
                                                                                            \
                                                                                            \
    template <typename AttrT>                                                               \
    void InternalCopyFromPoint(AttrT & attr, vtkIdType i, PointType const & point)          \
    {                                                                                       \
      ElementType * values = this->Array->GetPointer(i * ArraySize);                        \
      BOOST_PP_SEQ_FOR_EACH_I(                                                              \
        _PCLP_ARRAY_VALUE_EQUALS_VAR,                                                       \
        values,                                                                             \
        BOOST_PP_SEQ_TRANSFORM(                                                             \
          _PCLP_GET_ATTR,                                                                   \
          point,                                                                            \
          attrs                                                                             \
        )                                                                                   \
      )                                                                                     \
    }                                                                                       \
                                                                                            \
    template <typename AttrT, unsigned int N>                                               \
    void InternalCopyFromPoint(AttrT (& attr)[N], vtkIdType i, PointType const & point)     \
    {                                                                                       \
      this->Array->SetTypedTuple(i, attr);                                                  \
    }                                                                                       \
                                                                                            \
    void CopyFromPoint(vtkIdType i, PointType const & point) override                       \
    {                                                                                       \
      if (this->Array != nullptr)                                                           \
      {                                                                                     \
        this->InternalCopyFromPoint(point.BOOST_PP_SEQ_ELEM(0, attrs), i, point);           \
      }                                                                                     \
      this->BaseClassT::CopyFromPoint(i, point);                                            \
    }                                                                                       \
                                                                                            \
                                                                                            \
                                                                                            \
    template <typename AttrT>                                                               \
    void InternalCopyFromCloud(                                                             \
      AttrT & attr,                                                                         \
      typename pcl::PointCloud<PointType>::ConstPtr & cloud                                 \
    )                                                                                       \
    {                                                                                       \
      vtkIdType numberOfPoints = cloud->points.size();                                      \
      ElementType * data = this->Array->GetPointer(0);                                      \
      for (vtkIdType i = 0; i < numberOfPoints; ++i, data += ArraySize)                     \
      {                                                                                     \
        BOOST_PP_SEQ_FOR_EACH_I(                                                            \
          _PCLP_ARRAY_VALUE_EQUALS_VAR,                                                     \
          data,                                                                             \
          BOOST_PP_SEQ_TRANSFORM(                                                           \
            _PCLP_GET_ATTR,                                                                 \
            cloud->points[i],                                                               \
            attrs                                                                           \
          )                                                                                 \
        )                                                                                   \
      }                                                                                     \
    }                                                                                       \
                                                                                            \
    template <typename AttrT, unsigned int N>                                               \
    void InternalCopyFromCloud(                                                             \
      AttrT (& attr)[N],                                                                    \
      typename pcl::PointCloud<PointType>::ConstPtr & cloud                                 \
    )                                                                                       \
    {                                                                                       \
      vtkIdType numberOfPoints = cloud->points.size();                                      \
      for (vtkIdType i = 0; i < numberOfPoints; ++i)                                        \
      {                                                                                     \
        this->Array->SetTypedTuple(i, cloud->points[i].BOOST_PP_SEQ_ELEM(0, attrs));        \
      }                                                                                     \
    }                                                                                       \
                                                                                            \
    virtual                                                                                 \
    void CopyFromCloud(typename pcl::PointCloud<PointType>::ConstPtr & cloud) override      \
    {                                                                                       \
      /* assert(this->Array != nullptr); */                                                 \
      if (this->Array != nullptr)                                                           \
      {                                                                                     \
        static PointType point;                                                             \
        this->InternalCopyFromCloud(point.BOOST_PP_SEQ_ELEM(0, attrs), cloud);              \
      }                                                                                     \
      this->BaseClassT::CopyFromCloud(cloud);                                               \
    }                                                                                       \
                                                                                            \
                                                                                            \
                                                                                            \
    template <typename AttrT>                                                               \
    void InternalCopyToPoint(AttrT & attr, vtkIdType i, PointType & point) const            \
    {                                                                                       \
      ElementType * values = this->Array->GetPointer(i * ArraySize);                        \
      BOOST_PP_SEQ_FOR_EACH_I(                                                              \
        _PCLP_VAR_EQUALS_ARRAY_VALUE,                                                       \
        values,                                                                             \
        BOOST_PP_SEQ_TRANSFORM(                                                             \
          _PCLP_GET_ATTR,                                                                   \
          point,                                                                            \
          attrs                                                                             \
        )                                                                                   \
      )                                                                                     \
    }                                                                                       \
                                                                                            \
    template <typename AttrT, unsigned int N>                                               \
    void InternalCopyToPoint(AttrT (& attr)[N], vtkIdType i, PointType & point) const       \
    {                                                                                       \
      this->Array->GetTypedTuple(i, attr);                                                  \
    }                                                                                       \
                                                                                            \
    void CopyToPoint(vtkIdType i, PointType & point) const override                         \
    {                                                                                       \
      if (this->Array != nullptr)                                                           \
      {                                                                                     \
        this->InternalCopyToPoint(point.BOOST_PP_SEQ_ELEM(0, attrs), i, point);             \
      }                                                                                     \
      this->BaseClassT::CopyToPoint(i, point);                                              \
    }                                                                                       \
                                                                                            \
                                                                                            \
    template <typename AttrT>                                                               \
    void InternalCopyToCloud(                                                               \
      AttrT & attr,                                                                         \
      typename pcl::PointCloud<PointType>::Ptr & cloud                                      \
    ) const                                                                                 \
    {                                                                                       \
      vtkIdType numberOfPoints = this->Array->GetNumberOfTuples();                          \
      ElementType * data = this->Array->GetPointer(0);                                      \
      for (vtkIdType i = 0; i < numberOfPoints; ++i)                                        \
      {                                                                                     \
        ElementType * values = data + (i * ArraySize);                                      \
        BOOST_PP_SEQ_FOR_EACH_I(                                                            \
          _PCLP_VAR_EQUALS_ARRAY_VALUE,                                                     \
          values,                                                                           \
          BOOST_PP_SEQ_TRANSFORM(                                                           \
            _PCLP_GET_ATTR,                                                                 \
            cloud->points[i],                                                               \
            attrs                                                                           \
          )                                                                                 \
        )                                                                                   \
      }                                                                                     \
    }                                                                                       \
                                                                                            \
    template <typename AttrT, unsigned int N>                                               \
    void InternalCopyToCloud(                                                               \
      AttrT (& attr)[N],                                                                    \
      typename pcl::PointCloud<PointType>::Ptr & cloud                                      \
    ) const                                                                                 \
    {                                                                                       \
      vtkIdType numberOfPoints = this->Array->GetNumberOfTuples();                          \
      for (vtkIdType i = 0; i < numberOfPoints; ++i)                                        \
      {                                                                                     \
        this->Array->GetTypedTuple(i, cloud->points[i].BOOST_PP_SEQ_ELEM(0, attrs));        \
      }                                                                                     \
    }                                                                                       \
                                                                                            \
    virtual                                                                                 \
    void CopyToCloud(typename pcl::PointCloud<PointType>::Ptr & cloud) const override       \
    {                                                                                       \
      /* assert(this->Array != nullptr); */                                                 \
      if (this->Array != nullptr)                                                           \
      {                                                                                     \
        static PointType point;                                                             \
        this->InternalCopyToCloud(point.BOOST_PP_SEQ_ELEM(0, attrs), cloud);                \
      }                                                                                     \
      this->BaseClassT::CopyToCloud(cloud);                                                 \
    }                                                                                       \
  };

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

//------------------------------------------------------------------------------
// The for loop to declare the hierarchy, which should be inserted in the source
// file at the appropriate location. A sequence named _PCLP_DC_ATTR_SEQ
// must have been previous defined before this is invoked.
// BOOST_PP_FOR(_PCLP_DC_STATE, _PCLP_DC_PRED, _PCLP_DC_OP, _PCLP_DC_MACRO)

#endif // _PCLP_DECLARE_CONV_h

