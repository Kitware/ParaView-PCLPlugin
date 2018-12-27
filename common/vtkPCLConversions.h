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
   * @brief     Get the index of the best matching PCL point type in the
   *            PCL_XYZ_POINT_TYPES sequence defined in
   *            pcl/impl/point_types.hpp.
   * @param[in] polyData A PolyData instance that will be inspected to determine
   *                     which PCL point type best matches the attribute arrays
   *                     that it contains.
   * @return    The index of the PCL point type in the PCL_XYZ_POINT_TYPES
   *            sequence.
   *
   * The returned index is only intended to be used as an argument to the
   * INVOKE_WITH_POINT_TYPE macro.
   */
  static int GetPointTypeIndex(vtkPolyData * polyData);
  /*!
   * @copydoc   GetPointTypeIndex(vtkPolyData *)
   * @param[in] fieldNames A set of names of attribute fields such as those
   *                       parsed from a PCD file that can be used to determine
   *                       the best corresponding PCL point type.
   */
  static int GetPointTypeIndex(std::set<std::string> & fieldNames);

  /*!
   * @brief     Create vertex cells.
   * @param[in] numberOfVerts The number of vertex cells to create.
   * @return    A smart pointer to the new vertex cell array.
   * @todo      Revise this function. The current implementation was taken
   *            directly from the previous version of this function without
   *            revision.
   */
  static vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts);
  
  // static vtkSmartPointer<vtkIntArray> NewLabelsArray(pcl::IndicesConstPtr indices, vtkIdType length);
  // static vtkSmartPointer<vtkIntArray> NewLabelsArray(pcl::PointIndices::ConstPtr indices, vtkIdType length);
  // static vtkSmartPointer<vtkIntArray> NewLabelsArray(const std::vector<pcl::PointIndices>& indices, vtkIdType length);
  // static void PerformPointCloudConversionBenchmark(vtkPolyData* polyData);
};


#endif // __vtkPCLConversions_h

