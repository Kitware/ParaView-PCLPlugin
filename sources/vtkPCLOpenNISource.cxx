//==============================================================================
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
//==============================================================================

#include "vtkPCLOpenNISource.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/openni_grabber.h>
#include <boost/thread/thread.hpp>


//------------------------------------------------------------------------------
/*!
 * @brief Base class for all grabber wrappers.
 *
 * This class provides the functionality to start and stop the grabber without
 * considering the PCL point type that it will handle. Point-type-specific
 * functionality is relegated to derived classes via the virtual functions
 * RegisterCallback and GetLatestPolyData.
 */
class vtkPCLOpenNISource::GrabberWrapperBase
{
protected:
  pcl::Grabber * Grabber;
  boost::mutex Mutex;
  bool DataIsNew;

public:
  GrabberWrapperBase()
    : Grabber { new pcl::OpenNIGrabber() }
    , DataIsNew { false }
  {
  }

  ~GrabberWrapperBase()
  {
    delete this->Grabber;
  }

  virtual
  void RegisterCallback() = 0;

  virtual
  vtkSmartPointer<vtkPolyData> GetLatestPolyData() = 0;

  bool HasNewData()
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    return this->DataIsNew;
  }

  void Start()
  {
    this->Grabber->start();
    this->RegisterCallback();
    this->DataIsNew = false;
  }

  void Stop()
  {
    this->Grabber->stop();
  }
};

//------------------------------------------------------------------------------
/*!
 * @brief  Point-type-specific grabber wrappers.
 * @tparam PointType The PCL point type of the point cloud returned by the
 *                   grabber.
 */
template <typename PointType>
class vtkPCLOpenNISource::GrabberWrapper
  : public vtkPCLOpenNISource::GrabberWrapperBase
{
public:
  //! @brief The point cloud type.
  typedef pcl::PointCloud<PointType> CloudT;
  typedef typename CloudT::ConstPtr CloudPtrT;

private:
  /*!
   * @brief The most recent point cloud returned by the grabber.
   *
   * The point cloud is stored instead of a polydata so that conversions are
   * only performed when a new PolyData instance is actually requested.
   */
  CloudPtrT Cloud;

protected:
  void RegisterCallback() override
  {
    boost::function<void (CloudPtrT &)> callback =
      boost::bind(
        &vtkPCLOpenNISource::GrabberWrapper<PointType>::HandleIncomingCloud,
        this,
        _1
      );
    this->Grabber->registerCallback(callback);
  }

  void HandleIncomingCloud(CloudPtrT & cloud)
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    this->Cloud = cloud;
    this->DataIsNew = true;
  }

  vtkSmartPointer<vtkPolyData> GetLatestPolyData() override
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    vtkSmartPointer<vtkPolyData> polyData;
    vtkPCLConversions::PolyDataFromPointCloud(this->Cloud, polyData);
    this->DataIsNew = false;
    return polyData;
  }
};

//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLOpenNISource);

//------------------------------------------------------------------------------
vtkPCLOpenNISource::vtkPCLOpenNISource()
{
  this->MyGrabberWrapper = nullptr;
  // Invoke SetWithColor to set the grabber wrapper to ensure consistency.
  this->SetWithColor(false);
}

//------------------------------------------------------------------------------
vtkPCLOpenNISource::~vtkPCLOpenNISource()
{
  if (this->MyGrabberWrapper != nullptr)
  {
    delete this->MyGrabberWrapper;
  }
}

//------------------------------------------------------------------------------
void vtkPCLOpenNISource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
void vtkPCLOpenNISource::SetWithColor(bool withColor)
{
  if (withColor == this->WithColor)
  {
    return;
  }
  this->WithColor = withColor;
  // The destructor will stop the grabber if it is running.
  if (this->MyGrabberWrapper != nullptr)
  {
    delete this->MyGrabberWrapper;
  }
  // If the grabber supports other point types in the future, they should be
  // added here.
  if (this->WithColor)
  {
    this->MyGrabberWrapper = new vtkPCLOpenNISource::GrabberWrapper<pcl::PointXYZRGB>;
  }
  else
  {
    this->MyGrabberWrapper = new vtkPCLOpenNISource::GrabberWrapper<pcl::PointXYZ>;
  }
}

//------------------------------------------------------------------------------
int vtkPCLOpenNISource::LoadPCLSource(
  vtkPolyData * polyData
)
{
  polyData->ShallowCopy(this->MyGrabberWrapper->GetLatestPolyData());
  return 1;
}

//------------------------------------------------------------------------------
// Grabber management.
//------------------------------------------------------------------------------
void vtkPCLOpenNISource::StartGrabber()
{
  this->MyGrabberWrapper->Start();
}

//------------------------------------------------------------------------------
void vtkPCLOpenNISource::StopGrabber()
{
  this->MyGrabberWrapper->Stop();
}

//------------------------------------------------------------------------------
bool vtkPCLOpenNISource::HasNewData()
{
  return this->MyGrabberWrapper->HasNewData();
}

//------------------------------------------------------------------------------
void vtkPCLOpenNISource::Poll()
{
  if (this->MyGrabberWrapper->HasNewData())
  {
    this->Modified();
  }
}

