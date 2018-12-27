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
/*!
 * @brief Internal switch case macro for INVOKE_WITH_POINT_TYPE to dispatch
 *        calls based on point type index.
 */
#define _PCLP_POINT_TYPE_SWITCH_CASE(r, args, i, PointType)     \
  case i:                                                  \
    BOOST_PP_SEQ_ELEM(0, args)<PointType>                  \
      BOOST_PP_SEQ_TO_TUPLE(BOOST_PP_SEQ_POP_FRONT(args)); \
    break;

//------------------------------------------------------------------------------
/*!
 * @brief     Invoke a templated function that accepts a PCL point type as its
 *            first template parameter based on the point type's index in the
 *            PCL_XYZ_POINT_TYPES sequence.
 * @param[in] index    An int containing the point type index determined by
 *                     vtkPCLConversions::GetPointTypeIndex. Negative values
 *                     will use the plain PointXYZ type.
 * @param[in] function The templated function. It will be called with the PCL
 *                     point type as its first template parameter and the
 *                     remaining macro arguments as its function parameters.
 * @param[in] ...      The arguments to pass through to the function.
 *
 * This approach is used to circumvent the limitations of runtime type
 * detection.
 */
#define PCLP_INVOKE_WITH_POINT_TYPE(index, function, ...)             \
  switch (index)                                                      \
  {                                                                   \
    BOOST_PP_SEQ_FOR_EACH_I(                                          \
      _PCLP_POINT_TYPE_SWITCH_CASE,                                   \
      (function)BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__),                \
      PCL_XYZ_POINT_TYPES                                             \
    )                                                                 \
    /* All supported point types are expected to contain XYZ data. */ \
    default:                                                          \
      function<pcl::PointXYZ>(__VA_ARGS__);                           \
  }

#endif // __PCLInvokeWithPointType_h

