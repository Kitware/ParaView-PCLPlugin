//=========================================================================
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
//=========================================================================
//
// .NAME vtkPCLConversions - collection of pointcloud library routines
//
// .SECTION Description
//

#ifndef __vtkPCLConversions_h
#define __vtkPCLConversions_h
#define DEBUG_MSG(msg) std::cout << "DEBUG: " << msg << " [" << __LINE__ << "]" << std::endl;

#include <vtkObject.h>
#include <vtkSmartPointer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
// #include <vtkPCLFiltersModule.h>

#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/seq/transform.hpp>
#include <boost/preprocessor/variadic/to_tuple.hpp>
#include <pcl/impl/point_types.hpp>

//------------------------------------------------------------------------------
class vtkPolyData;
class vtkCellArray;
class vtkIntArray;

class VTK_EXPORT vtkPCLConversions : public vtkObject
{
public:

  static vtkPCLConversions * New();

  vtkTypeMacro(vtkPCLConversions, vtkObject);

  void PrintSelf(ostream& os, vtkIndent indent);

  // static vtkSmartPointer<vtkPolyData> PolyDataFromPCDFile(const std::string& filename);



#include "_PCLConversionsInternal.h"

public:
  //! @brief Get the index of the best matching PCL point type in the
  //         PCL_XYZ_POINT_TYPES sequence.
  static int _GetPointTypeIndex(vtkSmartPointer<vtkPolyData> & polyData);

  static vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts);
  
  // static vtkSmartPointer<vtkIntArray> NewLabelsArray(pcl::IndicesConstPtr indices, vtkIdType length);
  // static vtkSmartPointer<vtkIntArray> NewLabelsArray(pcl::PointIndices::ConstPtr indices, vtkIdType length);
  // static vtkSmartPointer<vtkIntArray> NewLabelsArray(const std::vector<pcl::PointIndices>& indices, vtkIdType length);

  // static void PerformPointCloudConversionBenchmark(vtkPolyData* polyData);

protected:

  vtkPCLConversions();
  ~vtkPCLConversions();

private:

  vtkPCLConversions(const vtkPCLConversions&) = delete; // Not implemented
  void operator=(const vtkPCLConversions&) = delete; // Not implemented
};


//------------------------------------------------------------------------------
//! @brief Get the first argument.
#define _FIRST_ARG(arg, ...) arg

//------------------------------------------------------------------------------
//! @brief Internal switch case macro for INVOKE_WITH_POINT_TYPE
#define _POINT_TYPE_SWITCH_CASE(i, args, PointType) \
  case i:                                           \
    function<PointType> args;                       \
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
#define INVOKE_WITH_POINT_TYPE(function, ...)                                        \
  namespace                                                                          \
  {                                                                                  \
    int _tmp_index = vtkPCLConversions::_GetPointTypeIndex(_FIRST_ARG(__VA_ARGS__)); \
    switch (_tmp_index)                                                              \
    {                                                                                \
      BOOST_PP_SEQ_ENUM(                                                             \
        BOOST_PP_SEQ_TRANSFORM(                                                      \
          _POINT_TYPE_SWITCH_CASE,                                                   \
          BOOST_PP_VARIADIC_TO_TUPLE(__VA_ARGS__),                                   \
          PCL_XYZ_POINT_TYPES                                                        \
      )                                                                              \
      )                                                                              \
      default:                                                                       \
        function<pcl::PointXYZ>(__VA_ARGS_);                                         \
    }                                                                                \
  }


#endif // __vtkPCLConversions_h
