//=============================================================================
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
//=============================================================================

#ifndef vtkPCLRegistrationFilter2_h
#define vtkPCLRegistrationFilter2_h

#include "vtkPCLFilter2.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <Eigen/Dense>

class VTK_EXPORT vtkPCLRegistrationFilter2 : public vtkPCLFilter2
{
//------------------------------------------------------------------------------
// Boilerplate VTK code.
public:
  vtkAbstractTypeMacro(vtkPCLRegistrationFilter2, vtkPCLFilter2);
  void PrintSelf(ostream & os, vtkIndent indent) override;

protected:

  vtkPCLRegistrationFilter2();
  ~vtkPCLRegistrationFilter2();

private:
  vtkPCLRegistrationFilter2(const vtkPCLRegistrationFilter2&) = delete;
  void operator=(const vtkPCLRegistrationFilter2&) = delete;

//------------------------------------------------------------------------------
protected:
  // Registration convergence criteria.
  double MaxCorrespondenceDistance {0.05};
  unsigned int MaximumIterations {50};
  double TransformationEpsilon {1e-8};
  double TransformationRotationEpsilon {1e-8};
  double EuclideanFitnessEpsilon {1.0};

  // Transformation caching.
  Eigen::Matrix4f TransformationMatrix {Eigen::Matrix4f::Identity()};
  bool HasTransformation {false};
  bool ReuseTransformation {false};

public:
  vtkGetMacro(MaxCorrespondenceDistance, double);
  vtkSetMacro(MaxCorrespondenceDistance, double);

  vtkGetMacro(MaximumIterations, unsigned int);
  vtkSetMacro(MaximumIterations, unsigned int);

  vtkGetMacro(TransformationEpsilon, double);
  vtkSetMacro(TransformationEpsilon, double);

  vtkGetMacro(TransformationRotationEpsilon, double);
  vtkSetMacro(TransformationRotationEpsilon, double);

  vtkGetMacro(EuclideanFitnessEpsilon, double);
  vtkSetMacro(EuclideanFitnessEpsilon, double);
  
  vtkGetMacro(ReuseTransformation, bool);
  vtkSetMacro(ReuseTransformation, bool);

  void Reset()
  {
    this->HasTransformation = false;
    this->Modified();
  }

  /*!
   * @brief      Apply a cached transformation if it exists and if the option to
   *             re- use it is set to true.
   * @tparam     PointT      The point type of the input and output clouds.
   * @param[in]  inputCloud  The cloud to transform.
   * @param[out] outputCloud The result of transforming the input cloud.
   * @return     True if the transform was applied, false if not.
   */
  template <typename PointT>
  bool MaybeApplyCachedTransformation(
    pcl::PointCloud<PointT> & inputCloud,
    pcl::PointCloud<PointT> & outputCloud
  );

  /*!
   * @brief      Apply settings common to all registration filters and align the
   *             input cloud.
   * @tparam     PointSource The point type of the input and output clouds.
   * @tparam     PointTarget The point type of the alignment's target cloud.
   * @tparam     Scalar      The PCL Registration class's Scalar parameter.
   * @param[in]  reg         An instance of a PCL Registration subclass. The
   *                         input and target clouds must be set before invoking
   *                         this method, along with any other configuration
   *                         such as their respective surface normals and/or
   *                         features.
   * @param[out] outputCloud The result of aligning the input cloud.
   *
   * After applying common configuration settings such as the maximum number of
   * iterations, the alignment will be run and the results will be inserted into
   * the output cloud. The transformation matrix will also be cached so that it
   * can be re-applied with MaybeApplyCachedTransformation when
   * ReuseTransformation is true.
   */
  template <typename PointSource, typename PointTarget, typename Scalar>
  void ConfigureAndAlign(
    pcl::Registration<PointSource, PointTarget, Scalar> & reg,
    pcl::PointCloud<PointSource> & outputCloud
  );
};

#include "vtkPCLRegistrationFilter2.txx"

#endif // vtkPCLRegistrationFilter2_h

