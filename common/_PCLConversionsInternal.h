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

#ifndef __PCLConversionsInternal_h
#define __PCLConversionsInternal_h
/*
 * These declarations needs to generated in a separate file due to the
 * limitations of VTK's wrapping parsers.
 */
#include <boost/preprocessor/seq/for_each.hpp>
#include <pcl/impl/point_types.hpp>
#define _DECLARE_CONVERTER(i, data, PointType)  \
  static void PolyDataFromPointCloud(           \
    pcl::PointCloud<PointType>::ConstPtr cloud, \
    vtkSmartPointer<vtkPolyData> & polyData     \
  );                                            \
  static void PointCloudFromPolyData(           \
    vtkSmartPointer<vtkPolyData> & polyData,    \
    pcl::PointCloud<PointType>::Ptr & cloud     \
  );

public:
BOOST_PP_SEQ_FOR_EACH(_DECLARE_CONVERTER, _, PCL_XYZ_POINT_TYPES)

#endif // __PCLConversionsInternal_h

