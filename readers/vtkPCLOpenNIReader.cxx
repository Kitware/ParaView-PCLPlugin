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
#include <pcl/io/openni_grabber.h>
#include <boost/thread/thread.hpp>


//------------------------------------------------------------------------------
class vtkPCLOpenNISource::PCLOpenNISourceInternal
{
public:
  bool NewData;
  pcl::OpenNIGrabber * Grabber;
  boost::mutex Mutex;
  vtkSmartPointer<vtkPolyData> PolyData;
  boost::function<void (CloudConstPtr const &)> Callback;

  PCLOpenNISourceInternal()
  {
    this->Grabber = 0;
    this->NewData = false;
  }

  ~PCLOpenNISourceInternal()
  {
    delete this->Grabber;
  }

  void HandleIncomingCloud(CloudConstPtr const & newCloud)
  {
    vtkSmartPointer<vtkPolyData> newPolyData = vtkPCLConversions::PolyDataFromPointCloud(newCloud);
    boost::lock_guard<boost::mutex> lock(this->mutex);
    this->PolyData = newPolyData;
    this->NewData = true;
  }

  vtkSmartPointer<vtkPolyData> GetLatestPolyData()
  {
    boost::lock_guard<boost::mutex> lock(this->mutex);
    vtkSmartPointer<vtkPolyData> polyData = this->PolyData;
    this->PolyData = NULL;
    this->NewData = false;
    return polyData;
  }

  bool HasNewData()
  {
    boost::lock_guard<boost::mutex> lock(this->mutex);
    return this->NewData;
  }
};

//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLOpenNISource);

//------------------------------------------------------------------------------
vtkPCLOpenNISource::vtkPCLOpenNISource()
{
  this->Internal = new vtkPCLOpenNISource::PCLOPenNISourceInternal;
}

//------------------------------------------------------------------------------
vtkPCLOpenNISource::~vtkPCLOpenNISource()
{
  delete this->Internal;
}

//------------------------------------------------------------------------------
void vtkPCLOpenNISource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//------------------------------------------------------------------------------
int vtkPCLOpenNISource::LoadPCLSource(
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud
)
{
  if (this->HasNewData())
  {
    outputCloud =
  }
  return 1;
}

//------------------------------------------------------------------------------
// Grabber management.
//------------------------------------------------------------------------------
void vtkPCLOpenNISource::StartGrabber()
{
  if (! this->Internal->Grabber)
  {
    this->Internal->Grabber = new pcl::OpenNIGrabber();
    this->Internal->Callback = boost::bind(& vtkPCLOpenNISource::PCLOpenNISourceInternal::HandleIncomingCloud, this->Internal, _1);
    this->Internal->Grabber->registerCallback(this->Internal->Callback);
  }
  this->Internal->Grabber->start();
}

//------------------------------------------------------------------------------
void vtkPCLOpenNISource::StopGrabber()
{
  if (this->Internal->Grabber)
  {
    this->Internal->Grabber->stop();
  }
}

//------------------------------------------------------------------------------
void vtkPCLOpenNISource::HasNewData()
{
  if (this->Internal->Grabber)
  {
    this->Internal->HasNewData();
  }
}

//------------------------------------------------------------------------------
void vtkPCLOpenNISource::Poll()
{
  if (this->HasNewData())
  {
    this->Modified();
  }
}

