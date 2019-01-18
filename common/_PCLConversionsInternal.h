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


// These declarations needs to generated in a separate file due to the
// limitations of VTK's wrapping parsers.

/*!
 * @brief Temporary internal macro for declaring point cloud conversion
 *        functions.
 */
#define _PCLP_DECLARE_CONVERTER(i, data, PointType) \
  static void PolyDataFromPointCloud(               \
    pcl::PointCloud<PointType>::ConstPtr cloud,     \
    vtkPolyData * polyData,                         \
    bool preserveExistingFields = false             \
  );                                                \
  static void PointCloudFromPolyData(               \
    vtkPolyData * polyData,                         \
    pcl::PointCloud<PointType>::Ptr & cloud         \
  );

// Declare public conversion functions for all PCL XYZ point types.
public:
BOOST_PP_SEQ_FOR_EACH(_PCLP_DECLARE_CONVERTER, _, PCLP_POINT_TYPES)

// Undefine the temporary macro.
#undef _PCLP_DECLARE_CONVERTER

