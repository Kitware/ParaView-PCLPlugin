#include "vtkPCLDummyFilter.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"

vtkStandardNewMacro(vtkPCLDummyFilter);

//----------------------------------------------------------------------------
vtkPCLDummyFilter::vtkPCLDummyFilter()
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkPCLDummyFilter::~vtkPCLDummyFilter()
{
}

//----------------------------------------------------------------------------
void vtkPCLDummyFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
int vtkPCLDummyFilter::RequestData(
  vtkInformation * request,
  vtkInformationVector * * inputVector,
  vtkInformationVector * outputVector
)
{
  vtkInformation * inInfo = inputVector[0]->GetInformationObject(0);
  vtkSmartPointer<vtkPolyData> input(vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT())));
  vtkInformation * outInfo = outputVector->GetInformationObject(0);
  vtkSmartPointer<vtkPolyData> output(vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT())));
  std::cout << this->Message << std::endl;
  output->ShallowCopy(input);
  return 1;
}

