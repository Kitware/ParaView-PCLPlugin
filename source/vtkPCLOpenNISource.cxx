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
  std::string DeviceID;
  boost::mutex Mutex;
  bool DataIsNew;
  vtkPCLOpenNISource * Parent;

public:
  GrabberWrapperBase(
    vtkPCLOpenNISource * parent,
    std::string deviceID = ""
  )
    : Grabber { nullptr }
    , DeviceID { deviceID }
    , DataIsNew { false }
    , Parent { parent }
  {
  }

  ~GrabberWrapperBase()
  {
    if (this->Grabber != nullptr)
    {
      this->Grabber->stop();
      delete this->Grabber;
    }
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
    if (this->Grabber == nullptr)
    {
      try
      {
        this->Grabber = new pcl::OpenNIGrabber(this->DeviceID);
      }
      catch(pcl::IOException e)
      {
        return;
      }
    }
    this->RegisterCallback();
    this->Grabber->start();
    this->DataIsNew = false;
  }

  void Stop()
  {
    this->Grabber->stop();
  }

  bool IsRunning()
  {
    return (this->Grabber != nullptr) && this->Grabber->isRunning();
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

public:
  GrabberWrapper(
    vtkPCLOpenNISource * parent,
    std::string deviceID = ""
  )
    : vtkPCLOpenNISource::GrabberWrapperBase { parent, deviceID }
    , Cloud { nullptr }
  {
  }

protected:
  void HandleIncomingCloud(CloudPtrT const & cloud)
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    this->Cloud = cloud;
    this->DataIsNew = true;
    this->Parent->Modified();
  }

  void RegisterCallback() override
  {
    boost::function<void (CloudPtrT const &)> callback =
      boost::bind(
        &vtkPCLOpenNISource::GrabberWrapper<PointType>::HandleIncomingCloud,
        this,
        _1
      );
    this->Grabber->registerCallback(callback);
  }

  vtkSmartPointer<vtkPolyData> GetLatestPolyData() override
  {
    boost::lock_guard<boost::mutex> lock(this->Mutex);
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    if (this->Cloud != nullptr)
    {
      vtkPCLConversions::PolyDataFromPointCloud(this->Cloud, polyData);
    }
    this->DataIsNew = false;
    return polyData;
  }
};

//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLOpenNISource);

//------------------------------------------------------------------------------
vtkPCLOpenNISource::vtkPCLOpenNISource()
  : MyGrabberWrapper { nullptr }
  , WithColor { false }
  , DeviceID { "" }
{
  this->Reset();
}

//------------------------------------------------------------------------------
vtkPCLOpenNISource::~vtkPCLOpenNISource()
{
  if (this->MyGrabberWrapper != nullptr)
  {
    this->MyGrabberWrapper->Stop();
    delete this->MyGrabberWrapper;
  }
}

//------------------------------------------------------------------------------
void vtkPCLOpenNISource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
void vtkPCLOpenNISource::Reset()
{
  // The destructor will stop the grabber if it is running.
  bool isRunning = false;
  if (this->MyGrabberWrapper != nullptr)
  {
    isRunning = this->MyGrabberWrapper->IsRunning();
    delete this->MyGrabberWrapper;
  }
  // If the grabber supports other point types in the future, they should be
  // added here.
  try
  {
    if (this->WithColor)
    {
      this->MyGrabberWrapper = new vtkPCLOpenNISource::GrabberWrapper<pcl::PointXYZRGBA>(this, this->DeviceID);
    }
    else
    {
      this->MyGrabberWrapper = new vtkPCLOpenNISource::GrabberWrapper<pcl::PointXYZ>(this, this->DeviceID);
    }
  }
  catch (pcl::IOException & e)
  {
    vtkErrorMacro(<< e.detailedMessage());
  }
  if (isRunning)
  {
    this->StartGrabber();
  }
  this->Modified();
}

//------------------------------------------------------------------------------
int vtkPCLOpenNISource::LoadPCLSource(
  vtkPolyData * outputPolyData
)
{
  if (this->MyGrabberWrapper != nullptr)
  {
    auto cloudPolyData = this->MyGrabberWrapper->GetLatestPolyData();
    if (cloudPolyData != nullptr)
    {
      outputPolyData->ShallowCopy(cloudPolyData);
      return 1;
    }
  }
  return 0;
}

//------------------------------------------------------------------------------
// Grabber management.
//------------------------------------------------------------------------------
void vtkPCLOpenNISource::StartGrabber()
{
  if (! this->IsRunning())
  {
    if (this->MyGrabberWrapper == nullptr)
    {
      this->Reset();
    }
    if (this->MyGrabberWrapper != nullptr)
    {
      this->MyGrabberWrapper->Start();
      this->Modified();
    }
  }
}

//------------------------------------------------------------------------------
void vtkPCLOpenNISource::StopGrabber()
{
  if (this->IsRunning())
  {
    this->MyGrabberWrapper->Stop();
    this->Modified();
  }
}

//------------------------------------------------------------------------------
bool vtkPCLOpenNISource::IsRunning()
{
  return (this->MyGrabberWrapper != nullptr) && this->MyGrabberWrapper->IsRunning();
}

//------------------------------------------------------------------------------
bool vtkPCLOpenNISource::HasNewData()
{
  return (this->MyGrabberWrapper != nullptr) && this->MyGrabberWrapper->HasNewData();
}

//------------------------------------------------------------------------------
void vtkPCLOpenNISource::Poll()
{
  if (this->HasNewData())
  {
    this->Modified();
  }
}

//------------------------------------------------------------------------------
void vtkPCLOpenNISource::SetDeviceID(std::string deviceID)
{
  if (this->DeviceID != deviceID)
  {
    this->DeviceID = deviceID;
    this->Reset();
  }
}

//------------------------------------------------------------------------------
void vtkPCLOpenNISource::SetWithColor(bool withColor)
{
  if (this->WithColor != withColor)
  {
    this->WithColor = withColor;
    this->Reset();
  }
}

