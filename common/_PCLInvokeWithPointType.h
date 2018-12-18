// #include "vtkPCLConversions.h"

#include <boost/preprocessor/seq/elem.hpp>
#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/seq/pop_front.hpp>
#include <boost/preprocessor/seq/push_front.hpp>
#include <boost/preprocessor/seq/to_tuple.hpp>
#include <boost/preprocessor/seq/transform.hpp>
#include <boost/preprocessor/variadic/to_seq.hpp>
#include <pcl/impl/point_types.hpp>

//------------------------------------------------------------------------------
//! @brief Get the first argument.
#define _FIRST_ARG(arg, ...) arg

//------------------------------------------------------------------------------
//! @brief Internal switch case macro for INVOKE_WITH_POINT_TYPE
#define _POINT_TYPE_SWITCH_CASE(i, args, PointType)        \
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
    BOOST_PP_SEQ_ENUM(                                                             \
      BOOST_PP_SEQ_TRANSFORM(                                                      \
        _POINT_TYPE_SWITCH_CASE,                                                   \
        BOOST_PP_SEQ_PUSH_FRONT(                                                   \
          BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__),                                   \
          function                                                                 \
        ),                                                                         \
        PCL_XYZ_POINT_TYPES                                                        \
    )                                                                              \
    )                                                                              \
    default:                                                                       \
      function<pcl::PointXYZ>(__VA_ARGS_);                                         \
  }

