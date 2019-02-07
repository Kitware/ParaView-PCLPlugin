//=============================================================================
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
//=============================================================================

/*
 * @file
 *
 * PCL point types are determined at runtime based on input data such as the
 * attribute arrays in a PolyData instance or the attribute fields in a PCD file
 * header. The PCL library provides several macro sequences such as
 * PCL_POINT_TYPES and PCL_XYZ_POINT_TYPES that contain points in particular
 * categories.The macros defined in this file are used to dispatch calls to
 * templated functions that accept the PCL point type as its first parameter.
 * They are to be used in conjunction with vtkPCLConversions::GetIndex to invoke
 * the point-type-dependent function at runtime.
 *
 * In addition to the sequences provided by PCL, a sequence named
 * PCLP_HISTOGRAM_POINT_TYPES is also defined in a separate file and included
 * here. This sequence is used to define PCLP_POINT_TYPES as an augmentation of
 * PCL_POINT_TYPES with the histogram functions, and PCLP_FEATURE_POINT_TYPES as
 * a similar augmentation of PCL_FEATURE_POINT_TYPES.
 *
 * All macros here that rely on the extra point type sequences defined by this
 * plugin begin with "PCLP_INVOKE_WITH_PCLP_". Those that use PCL-defined
 * sequences without modification begin with "PCLP_INVOKE_WITH_PCL_" and the
 * part of the name after "WIDTH_" matches the sequence defined by PCL.
 */

#ifndef __PCLInvokeWithPointType_h
#define __PCLInvokeWithPointType_h

#include <boost/preprocessor.hpp>
#include <pcl/impl/point_types.hpp>

#include "PCLPHistogramPointTypes.h"

//------------------------------------------------------------------------------
//! @brief All point types, including supported templated histogram types.
#define PCLP_POINT_TYPES PCL_POINT_TYPES PCLP_HISTOGRAM_POINT_TYPES

//------------------------------------------------------------------------------
//! @brief All feature point types, including supported templated histogram
//!        types.
#define PCLP_FEATURE_POINT_TYPES PCL_FEATURE_POINT_TYPES PCLP_HISTOGRAM_POINT_TYPES

//------------------------------------------------------------------------------
/*!
 * @brief Internal struct for indexing point types and getting their names at
 *        run-time.
 */
template <typename T>
struct PointMeta
{
  static constexpr int GetIndex() { return -1; }
  static std::string GetName() { return ""; }
};

// Template specializations for all PCL types.
#define _PCLP_DECLARE_POINT_TYPE_INDEX_STRUCT(r, data, i, PointType)        \
  template <>                                                               \
  struct PointMeta<PointType>                                               \
  {                                                                         \
    static constexpr int GetIndex() { return i; }                           \
    static std::string GetName() { return BOOST_PP_STRINGIZE(PointType); } \
  };

BOOST_PP_SEQ_FOR_EACH_I(
  _PCLP_DECLARE_POINT_TYPE_INDEX_STRUCT,
  _,
  PCLP_POINT_TYPES
)

#undef _PCLP_DECLARE_POINT_TYPE_INDEX_STRUCT

//------------------------------------------------------------------------------
/*!
 * @brief Internal switch case macro for INVOKE_WITH_POINT_TYPE to dispatch
 *        calls based on point type index.
 */
#define _PCLP_POINT_TYPE_SWITCH_CASE(r, statement, i, PointType) \
  case PointMeta<PointType>::GetIndex():                         \
    statement(PointType)                                         \
    break;

//------------------------------------------------------------------------------
/*!
 * @brief     Invoke a templated function that accepts a PCL point type as its
 *            first template parameter (without Histogram point types).
 * @param[in] index       An int containing the point type index determined by
 *                        vtkPCLConversions::GetPointTypeIndex. Negative values
 *                        will fall through the switch statement.
 * @param[in] statement   A macro that accepts the point type to produce a valid
 *                        block of code in the case statement.
 * @param[in] point_types The preprocessor sequence of point types.
 *
 * This approach is used to circumvent the limitations of runtime type
 * detection. Nevertheless, because compile-time constant conditional blocks are
 * still initially compiled by the compiler, all conditional blocks must contain
 * compilable code, i.e. the statement must be compilable for all point types
 * handled by this macro. If not, use a different sequence of point types, or
 * defer calls to a templated function within the statement.
 */
#define PCLP_INVOKE_WITH_GIVEN_POINT_TYPE(index, statement, point_types) \
  switch (index)                                                         \
  {                                                                      \
    BOOST_PP_SEQ_FOR_EACH_I(                                             \
      _PCLP_POINT_TYPE_SWITCH_CASE,                                      \
      statement,                                                         \
      point_types                                                        \
    )                                                                    \
  }

//------------------------------------------------------------------------------
/*!
 * @copydoc PCLP_INVOKE_WITH_GIVEN_POINT_TYPE
 * @brief   Invoke PCLP_INVOKE_WITH_GIVEN_POINT_TYPE with all point types in
 *          PCL_POINT_TYPES.
 */
#define PCLP_INVOKE_WITH_PCL_POINT_TYPE(index, statement) \
  PCLP_INVOKE_WITH_GIVEN_POINT_TYPE(index, statement, PCL_POINT_TYPES)

//------------------------------------------------------------------------------
/*!
 * @copydoc PCLP_INVOKE_WITH_GIVEN_POINT_TYPE
 * @brief   Invoke PCLP_INVOKE_WITH_GIVEN_POINT_TYPE with all point types in
 *          PCL_XYZ_POINT_TYPES.
 */
#define PCLP_INVOKE_WITH_PCL_XYZ_POINT_TYPE(index, statement) \
  PCLP_INVOKE_WITH_GIVEN_POINT_TYPE(index, statement, PCL_XYZ_POINT_TYPES)

//------------------------------------------------------------------------------
/*!
 * @copydoc PCLP_INVOKE_WITH_GIVEN_POINT_TYPE
 * @brief   Invoke PCLP_INVOKE_WITH_GIVEN_POINT_TYPE with all point types in
 *          PCL_RGB_POINT_TYPES.
 */
#define PCLP_INVOKE_WITH_PCL_RGB_POINT_TYPE(index, statement) \
  PCLP_INVOKE_WITH_GIVEN_POINT_TYPE(index, statement, PCL_RGB_POINT_TYPES)

//------------------------------------------------------------------------------
/*!
 * @copydoc PCLP_INVOKE_WITH_GIVEN_POINT_TYPE
 * @brief   Invoke PCLP_INVOKE_WITH_GIVEN_POINT_TYPE with all point types in
 *          PCL_FEATURE_POINT_TYPES.
 */
#define PCLP_INVOKE_WITH_PCL_FEATURE_POINT_TYPE(index, statement) \
  PCLP_INVOKE_WITH_GIVEN_POINT_TYPE(index, statement, PCL_FEATURE_POINT_TYPES)

//------------------------------------------------------------------------------
/*!
 * @copydoc PCLP_INVOKE_WITH_GIVEN_POINT_TYPE
 * @brief   Invoke PCLP_INVOKE_WITH_GIVEN_POINT_TYPE with all point types in
 *          PCL_NORMAL_POINT_TYPES.
 */
#define PCLP_INVOKE_WITH_PCL_NORMAL_POINT_TYPE(index, statement) \
  PCLP_INVOKE_WITH_GIVEN_POINT_TYPE(index, statement, PCL_NORMAL_POINT_TYPES)

//------------------------------------------------------------------------------
/*!
 * @copydoc PCLP_INVOKE_WITH_GIVEN_POINT_TYPE
 * @brief Invoke PCLP_INVOKE_WITH_GIVEN_POINT_TYPE with all point types that
 *        contains both XYZ and normal data.
 */
#define PCLP_INVOKE_WITH_XYZ_NORMAL_POINT_TYPE(index, statement) \
  PCLP_INVOKE_WITH_GIVEN_POINT_TYPE(index, statement, BOOST_PP_SEQ_TAIL(PCL_NORMAL_POINT_TYPES))

//------------------------------------------------------------------------------
/*!
 * @copydoc PCLP_INVOKE_WITH_GIVEN_POINT_TYPE
 * @brief Invoke PCLP_INVOKE_WITH_GIVEN_POINT_TYPE with all supported
 *        pcl::Histogram types.
 */
#define PCLP_INVOKE_WITH_PCLP_HISTOGRAM_POINT_TYPE(index, statement) \
  PCLP_INVOKE_WITH_GIVEN_POINT_TYPE(index, statement, PCLP_HISTOGRAM_POINT_TYPES)

//------------------------------------------------------------------------------
/*!
 * @copydoc PCLP_INVOKE_WITH_GIVEN_POINT_TYPE
 * @brief Invoke PCLP_INVOKE_WITH_GIVEN_POINT_TYPE with all point types in
 *        PCLP_POINT_TYPES.
 */
#define PCLP_INVOKE_WITH_PCLP_POINT_TYPE(index, statement) \
  PCLP_INVOKE_WITH_GIVEN_POINT_TYPE(index, statement, PCLP_POINT_TYPES)

//------------------------------------------------------------------------------
/*!
 * @copydoc PCLP_INVOKE_WITH_GIVEN_POINT_TYPE
 * @brief Invoke PCLP_INVOKE_WITH_GIVEN_POINT_TYPE with all point types in
 *        PCLP_FEATURE_POINT_TYPES.
 */
#define PCLP_INVOKE_WITH_PCLP_FEATURE_POINT_TYPE(index, statement) \
  PCLP_INVOKE_WITH_GIVEN_POINT_TYPE(index, statement, PCLP_POINT_TYPES)



#endif // __PCLInvokeWithPointType_h

