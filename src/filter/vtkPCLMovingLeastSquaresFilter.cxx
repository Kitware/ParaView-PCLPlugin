//==============================================================================
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
//==============================================================================

#include "vtkPCLMovingLeastSquaresFilter.h"
#include "vtkPCLConversions.h"
#include "vtkObjectFactory.h"

#define PCL_NO_PRECOMPILE 1
#include <pcl/surface/mls.h>

//------------------------------------------------------------------------------
// The indices must correspond to the ones in the proxy.
template <typename PointType, typename NormalType>
typename pcl::MovingLeastSquares<PointType,NormalType>::UpsamplingMethod
getUpsamplingMethod(unsigned int index)
{
	switch (index)
	{
		case 1:
			return pcl::MovingLeastSquares<PointType,NormalType>::DISTINCT_CLOUD;
		case 2:
			return pcl::MovingLeastSquares<PointType,NormalType>::SAMPLE_LOCAL_PLANE;
		case 3:
			return pcl::MovingLeastSquares<PointType,NormalType>::RANDOM_UNIFORM_DENSITY;
		case 4:
			return pcl::MovingLeastSquares<PointType,NormalType>::VOXEL_GRID_DILATION;
		case 0:
		default:
			return pcl::MovingLeastSquares<PointType,NormalType>::NONE;
	}
}

//------------------------------------------------------------------------------
char const * getUpsamplingMethodString(unsigned int index)
{
	switch (index)
	{
		case 1:
			return "DISTINCT_CLOUD";
		case 2:
			return "SAMPLE_LOCAL_PLANE";
		case 3:
			return "RANDOM_UNIFORM_DENSITY";
		case 4:
			return "VOXEL_GRID_DILATION";
		case 0:
		default:
			return "NONE";
	}
}

//------------------------------------------------------------------------------
// The indices must correspond to the ones in the proxy.
pcl::MLSResult::ProjectionMethod
getProjectionMethod(unsigned int index)
{
	switch (index)
	{
		case 1:
			return pcl::MLSResult::SIMPLE;
		case 2:
			return pcl::MLSResult::ORTHOGONAL;
		case 0:
		default:
			return pcl::MLSResult::NONE;
	}
}

//------------------------------------------------------------------------------
char const * getProjectionMethodString(unsigned int index)
{
	switch (index)
	{
		case 1:
			return "SIMPLE";
		case 2:
			return "ORTHOGONAL";
		case 0:
		default:
			return "NONE";
	}
}

//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLMovingLeastSquaresFilter);

//------------------------------------------------------------------------------
vtkPCLMovingLeastSquaresFilter::vtkPCLMovingLeastSquaresFilter()
{
}

//------------------------------------------------------------------------------
vtkPCLMovingLeastSquaresFilter::~vtkPCLMovingLeastSquaresFilter()
{
}

//------------------------------------------------------------------------------
void vtkPCLMovingLeastSquaresFilter::PrintSelf(ostream & os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "ComputeNormals: " << (this->ComputeNormals ? "yes" : "no") << '\n';
  os << indent << "PolynomialOrder: " << this->PolynomialOrder << '\n';
  os << indent << "SearchRadius: " << this->SearchRadius << '\n';
  os << indent << "SqrGaussParam: " << this->SqrGaussParam << '\n';
  os << indent << "UpsamplingRadius: " << this->UpsamplingRadius << '\n';
  os << indent << "UpsamplingStepSize: " << this->UpsamplingStepSize << '\n';
  os << indent << "PointDensity: " << this->PointDensity << '\n';
  os << indent << "DilationVoxelSize: " << this->DilationVoxelSize << '\n';
  os << indent << "CacheMLSResults: " << this->CacheMLSResults << '\n';
  os << indent << "NumberOfThreads: " << this->NumberOfThreads << '\n';
  os << indent << "UpsamplingMethod: " << getUpsamplingMethodString(this->UpsamplingMethod) << '\n';
  os << indent << "ProjectionMethod: " << getProjectionMethodString(this->ProjectionMethod) << '\n';
}

//------------------------------------------------------------------------------
int vtkPCLMovingLeastSquaresFilter::ApplyPCLFilter(
  vtkPolyData * input,
  vtkPolyData * output
)
{
  int index = vtkPCLConversions::GetPointTypeIndex(input);
#define _statement(PointType) return this->InternalApplyPCLFilter<PointType>(input, output);
  PCLP_INVOKE_WITH_PCL_XYZ_POINT_TYPE(index, _statement)
#undef _statement
  vtkErrorMacro(<< "no XYZ point data in input")
  return 0;
}

//------------------------------------------------------------------------------
// Apply the filter to any point type.
template <typename PointType>
int vtkPCLMovingLeastSquaresFilter::InternalApplyPCLFilter(
  vtkPolyData * input,
  vtkPolyData * output
)
{
  if (this->ComputeNormals)
  {
    std::set<std::string> requiredFieldNames { "normal_x", "normal_y", "normal_z" };
    PointType const point;
    int index = vtkPCLConversions::GetPointTypeIndex<PointType const &>(point, requiredFieldNames);
#define _statement(NormalPointType) return this->InternalInternalApplyPCLFilter<PointType, NormalPointType>(input, output);
    PCLP_INVOKE_WITH_XYZ_NORMAL_POINT_TYPE(index, _statement)
#undef _statement
    vtkErrorMacro(<< "failed to determine a corresponding point type with normal attributes")
    return 0;
  }
  else
  {
    return this->InternalInternalApplyPCLFilter<PointType, PointType>(input, output);
  }
}

template <typename InPointType, typename OutPointType>
int vtkPCLMovingLeastSquaresFilter::InternalInternalApplyPCLFilter(
  vtkPolyData * input,
  vtkPolyData * output
)
{
  // For convenience, typedef the cloud type based on the template point type.
	// typedef pcl::Normal NormalPointType;

  typedef pcl::PointCloud<InPointType> InCloudT;
  typedef pcl::PointCloud<OutPointType> OutCloudT;
  // typedef pcl::PointCloud<NormalPointType> NormalCloudT;

  typename InCloudT::Ptr inputCloud(new InCloudT);
  typename OutCloudT::Ptr outputCloud(new OutCloudT);
  // typename NormalCloudT::Ptr normalCloud(new NormalCloudT);

  vtkPCLConversions::PointCloudFromPolyData(input, inputCloud);

	// pcl::MovingLeastSquares<InPointType, NormalPointType> mls;
	pcl::MovingLeastSquares<InPointType, OutPointType> mls;
	mls.setUpsamplingMethod(
    getUpsamplingMethod<InPointType, OutPointType>(
      this->UpsamplingMethod
    )
  );
	mls.setProjectionMethod(getProjectionMethod(this->ProjectionMethod));

	mls.setInputCloud(inputCloud);
	// mls.setOutputNormals(normalCloud);

	mls.setComputeNormals(this->ComputeNormals);
	mls.setPolynomialOrder(this->PolynomialOrder);
	// mls.setPolynomialFit(this->PolynomialFit);
	mls.setSearchRadius(this->SearchRadius);
  mls.setSqrGaussParam(this->SqrGaussParam);
  mls.setUpsamplingRadius(this->UpsamplingRadius);
  mls.setUpsamplingStepSize(this->UpsamplingStepSize);
  mls.setPointDensity(this->PointDensity);
  mls.setDilationVoxelSize(this->DilationVoxelSize);
  mls.setDilationIterations(this->DilationIterations);
  mls.setCacheMLSResults(this->CacheMLSResults);
  mls.setNumberOfThreads(this->NumberOfThreads);

  mls.process((* outputCloud));

  vtkPCLConversions::PolyDataFromPointCloud(outputCloud, output);
  return 1;
}

