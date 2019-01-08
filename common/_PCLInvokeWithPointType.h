//=============================================================================
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
//=============================================================================

/*
 * @file
 *
 * PCL point types are determined at runtime based on input data such as the
 * attribute arrays in a PolyData instance or the attribute fields in a PCD file
 * header. The PCL library provides a macro sequence named PCL_XYZ_POINT_TYPES
 * that enumerates all point types with XYZ data, which are the point types
 * supported by this plugin. The macros defined in this file are used to
 * dispatch calls to templated functions that accept the PCL point type as its
 * first parameter. They are to be used in conjunction with
 * vtkPCLConversions::GetPointTypeIndex to invoke the correct function at
 * runtime..
 */

#ifndef __PCLInvokeWithPointType_h
#define __PCLInvokeWithPointType_h

#include <boost/preprocessor.hpp>
#include <pcl/impl/point_types.hpp>

//------------------------------------------------------------------------------
// Declare a constexpr function with overrides to get the index of the different
// PCL point types.
template <typename T>
constexpr int getPointTypeIndex() { return -1; };

// Template specializations for all PCL types.
#define _PCLP_DECLARE_POINT_TYPE_INDEX_STRUCT(r, data, i, PointType) \
  template <>                                                        \
  constexpr int getPointTypeIndex<PointType>() { return i; }

BOOST_PP_SEQ_FOR_EACH_I(
  _PCLP_DECLARE_POINT_TYPE_INDEX_STRUCT,
  _,
  PCL_POINT_TYPES
)

#undef _PCLP_DECLARE_POINT_TYPE_INDEX_STRUCT

//------------------------------------------------------------------------------
/*!
 * @brief Internal switch case macro for INVOKE_WITH_POINT_TYPE to dispatch
 *        calls based on point type index.
 */
#define _PCLP_POINT_TYPE_SWITCH_CASE(r, statement, i, PointType) \
  case getPointTypeIndex<PointType>():                               \
    statement(PointType)                                         \
    break;

//------------------------------------------------------------------------------
/*!
 * @brief     Invoke a templated function that accepts a PCL point type as its
 *            first template parameter based on the point type's index in the
 *            PCL_XYZ_POINT_TYPES sequence.
 * @param[in] index     An int containing the point type index determined by
 *                      vtkPCLConversions::GetPointTypeIndex. Negative values
 *                      will use the plain PointXYZ type.
 * @param[in] statement A macro that accepts the point type to produce a valid
 *                      block of code in the case statement.
 *
 * This approach is used to circumvent the limitations of runtime type
 * detection.
 */
#define PCLP_INVOKE_WITH_XYZ_POINT_TYPE(index, statement) \
  switch (index)                                          \
  {                                                       \
    BOOST_PP_SEQ_FOR_EACH_I(                              \
      _PCLP_POINT_TYPE_SWITCH_CASE,                       \
      statement,                                          \
      PCL_XYZ_POINT_TYPES                                 \
    )                                                     \
  }

  //   [> All supported point types are expected to contain XYZ data. <] \
  //   default:                                                          \
  //     function<pcl::PointXYZ>(__VA_ARGS__);                           \
  // }


//------------------------------------------------------------------------------
/*!
 * @copydoc PCLP_INVOKE_WITH_XYZ_POINT_TYPE
 * @brief   Same as PCLP_INVOKE_WITH_XYZ_POINT_TYPE but for all point types in
 *          PCL_NORMAL_POINT_TYPES except PointNormal.
 */
#define PCLP_INVOKE_WITH_NORMAL_POINT_TYPE(index, statement) \
  switch (index)                                             \
  {                                                          \
    BOOST_PP_SEQ_FOR_EACH_I(                                 \
      _PCLP_POINT_TYPE_SWITCH_CASE,                          \
      statement,                                             \
      BOOST_PP_SEQ_TAIL(PCL_NORMAL_POINT_TYPES)              \
    )                                                        \
  }

#endif // __PCLInvokeWithPointType_h

