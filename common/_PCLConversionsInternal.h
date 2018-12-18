#ifndef __PCLConversionsInternal_h
#define __PCLConversionsInternal_h
/*
 * These declarations needs to generated in a separate file due to the
 * limitations of VTK's wrapping parsers.
 */
#include <boost/preprocessor/seq/for_each.hpp>
#include <pcl/impl/point_types.hpp>
#define DECLARE_CONVERTER(i, data, PointType)     \
  static void PolyDataFromPointCloud(             \
    pcl::PointCloud<PointType>::ConstPtr cloud, \
    vtkSmartPointer<vtkPolyData> & polyData       \
  );                                              \
  static void PointCloudFromPolyData(             \
    vtkSmartPointer<vtkPolyData> & polyData,      \
    pcl::PointCloud<PointType>::Ptr & cloud       \
  );

public:
BOOST_PP_SEQ_FOR_EACH(DECLARE_CONVERTER, _, PCL_XYZ_POINT_TYPES)

#endif // __PCLConversionsInternal_h

