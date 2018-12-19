#ifndef __PCLInvokeWithPointType_h
#define __PCLInvokeWithPointType_h

#include <boost/preprocessor.hpp>
#include <pcl/impl/point_types.hpp>

//------------------------------------------------------------------------------
//! @brief Get the first argument.
#define _FIRST_ARG(arg, ...) arg

//------------------------------------------------------------------------------
//! @brief Internal switch case macro for INVOKE_WITH_POINT_TYPE
#define _POINT_TYPE_SWITCH_CASE(r, args, i, PointType)        \
  case i:                                                  \
    BOOST_PP_SEQ_ELEM(0, args)<PointType>                  \
      BOOST_PP_SEQ_TO_TUPLE(BOOST_PP_SEQ_POP_FRONT(args)); \
    break;

//------------------------------------------------------------------------------
/*!
 * @brief     A macro that can invoke a templated function that accepts a PCL
 *            point type as its first template parameter.
 * @param[in] function The templated function. It will be called with the PCL
 *                     point type as its first template parameter and the
 *                     remaining macro arguments as its function parameters.
 * @param[in] ...      The arguments to pass to the function. The first one must
 *                     be a PolyData instance that will be used to determine the
 *                     PCL point type. The remaining arguments are optional and
 *                     will be passed through.
 */
#define INVOKE_WITH_POINT_TYPE(function, ...)                                      \
  int _tmp_index = vtkPCLConversions::_GetPointTypeIndex(_FIRST_ARG(__VA_ARGS__)); \
  switch (_tmp_index)                                                              \
  {                                                                                \
    BOOST_PP_SEQ_FOR_EACH_I(                                                       \
      _POINT_TYPE_SWITCH_CASE,                                                     \
      (function)BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__),                             \
      PCL_XYZ_POINT_TYPES                                                          \
    )                                                                              \
    /* All supported point types are expected to contain XYZ data. */              \
    default:                                                                       \
      function<pcl::PointXYZ>(__VA_ARGS__);                                        \
  }

#endif // __PCLInvokeWithPointType_h
