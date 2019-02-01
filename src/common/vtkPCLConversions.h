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


#ifndef __vtkPCLConversions_h
#define __vtkPCLConversions_h

#include <vtkObject.h>
#include <vtkSmartPointer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/pcl_base.h>
// #include <pcl/PointIndices.h>
// #include <pcl/ModelCoefficients.h>

#include <set>

#include "_PCLInvokeWithPointType.h"

// Required for the inclusion of _PCLConversionsInternal.h
#include <pcl/impl/point_types.hpp>
#include <boost/preprocessor/seq/for_each.hpp>


//------------------------------------------------------------------------------
class vtkPolyData;
class vtkCellArray;
class vtkIntArray;

class VTK_EXPORT vtkPCLConversions : public vtkObject
{
//------------------------------------------------------------------------------
// VTK boilerplate.
public:

  static vtkPCLConversions * New();

  vtkTypeMacro(vtkPCLConversions, vtkObject);

  void PrintSelf(ostream& os, vtkIndent indent);

protected:

  vtkPCLConversions();
  ~vtkPCLConversions();

private:

  vtkPCLConversions(const vtkPCLConversions&) = delete; // Not implemented
  void operator=(const vtkPCLConversions&) = delete; // Not implemented

//------------------------------------------------------------------------------
// The file must be included to avoid VTK wrapping errors when attempting to
// parse the macros it uses.
#include "_PCLConversionsInternal.h"

//------------------------------------------------------------------------------
// Additional methods.
public:
  /*!
   * @brief  Return the point type as a string.
   * @tparam PointType The PCL point type.
   * @return The name of the point, with the "pcl::" prefix.
   */
  template <typename PointType>
  static
  std::string GetPointTypeName();

  /*!
   * @brief     Get the index of the best matching PCL point type in the
   *            PCL_POINT_TYPES sequence defined in pcl/impl/point_types.hpp.
   * @tparam    T           The type of the argument to pass to the
   *                        ConvPoint::GetScore().
   * @param[in] getScoreArg The argument to pass to the ConvPoint::GetScore().
   * @return    The index of the PCL point type in the PCL_POINT_TYPES sequence.
   *
   * The returned index is only intended to be used as an argument to the
   * INVOKE_WITH_POINT_TYPE macro.
   */
  template <typename T>
  static 
  int GetPointTypeIndex(T getScoreArg);

  /*!
  * @copydoc   GetPointTypeIndex(T getScoreArg)
  * @param[in] requiredFieldNames Required field names. Points without these
  *                               names will be excluded from matching.
  */
  template <typename T>
  static
  int GetPointTypeIndex(
    T getScoreArg,
    std::set<std::string> const & requiredFieldNames
  );

  /*!
   * @brief Fill a set of strings with the field names for the given point type.
   * @tparam PointType The PCL point type.
   * @param[out] fieldNames The set to fill with the field names.
   */
  template <typename PointType>
  static
  void GetFieldNames(std::set<std::string> & fieldNames);


  /*!
   * @brief     Get the index of the pcl::Histogram point type of size n.
   * @param[in] n The size of the histogram point.
   * @return    The index of the corresponding point for use with the
   *            PCLP_INVOKE_WITH_* macros.
   */
  static
  int GetHistogramPointTypeIndex(int size);

  // TODO
  // Remove or update this legacy code from the previous version of the plugin..
  // static vtkSmartPointer<vtkIntArray> NewLabelsArray(pcl::IndicesConstPtr indices, vtkIdType length);
  // static vtkSmartPointer<vtkIntArray> NewLabelsArray(pcl::PointIndices::ConstPtr indices, vtkIdType length);
  // static vtkSmartPointer<vtkIntArray> NewLabelsArray(const std::vector<pcl::PointIndices>& indices, vtkIdType length);
  // static void PerformPointCloudConversionBenchmark(vtkPolyData* polyData);
};

#endif // __vtkPCLConversions_h

