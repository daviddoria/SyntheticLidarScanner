/*=========================================================================
 *
 *  Copyright David Doria 2011 daviddoria@gmail.com
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0.txt
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *=========================================================================*/

#include "ScannerInteractorStyle.h"

#include <vtkObjectFactory.h>
#include <vtkAxesActor.h>
#include <vtkCommand.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkBoxWidget2.h>
#include <vtkBoxRepresentation.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkPolyData.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkTransform.h>

#include "vtkLidarScanner.h"

vtkStandardNewMacro(ScannerInteractorStyle);

ScannerInteractorStyle::ScannerInteractorStyle()
{
  this->LidarScanner = vtkSmartPointer<vtkLidarScanner>::New();
  this->LidarScannerRepresentation = vtkSmartPointer<vtkPolyData>::New();
  this->LidarScannerMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  this->LidarScannerActor = vtkSmartPointer<vtkActor>::New();

  this->Scene = vtkSmartPointer<vtkPolyData>::New();
  this->SceneActor = vtkSmartPointer<vtkActor>::New();
  this->SceneMapper = vtkSmartPointer<vtkPolyDataMapper>::New();

  this->Scan = vtkSmartPointer<vtkPolyData>::New();
  this->ScanActor = vtkSmartPointer<vtkActor>::New();
  this->ScanMapper = vtkSmartPointer<vtkPolyDataMapper>::New();

  // Box widget
  this->BoxWidget = vtkSmartPointer<vtkBoxWidget2>::New();
  this->BoxWidget->AddObserver(vtkCommand::InteractionEvent, this, &ScannerInteractorStyle::HandleBoxWidgetEvent);
  vtkBoxRepresentation::SafeDownCast(this->BoxWidget->GetRepresentation())->HandlesOff();
  
  // Orientation widget
  this->OrientationAxes = vtkSmartPointer<vtkAxesActor>::New();
  this->OrientationWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
  this->OrientationWidget->SetOrientationMarker(this->OrientationAxes);
  this->OrientationWidget->SetViewport(0.0, 0.0, 0.2, 0.2);

  
}

void ScannerInteractorStyle::Initialize()
{
  // Widgets cannot be enabled in constructor
  this->BoxWidget->SetInteractor(this->Interactor);
  this->BoxWidget->On();

  this->OrientationWidget->SetInteractor(this->Interactor);
  this->OrientationWidget->On();
}

void ScannerInteractorStyle::CreateRepresentation()
{
  this->LidarScanner->CreateRepresentation(this->LidarScannerRepresentation);
  /*
  std::cout << "Representation has " << this->LidarScannerRepresentation->GetNumberOfPoints() << " points." << std::endl;
  for(vtkIdType i = 0; i < this->LidarScannerRepresentation->GetNumberOfPoints(); i++)
    {
    double p[3];
    this->LidarScannerRepresentation->GetPoint(i,p);
    std::cout << "P: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
    }
  */
}

void ScannerInteractorStyle::HandleBoxWidgetEvent(vtkObject* caller, long unsigned int eventId, void* callData)
{
  //std::cout << "BoxWidget event" << std::endl;
  vtkBoxWidget2* boxWidget = static_cast<vtkBoxWidget2*>(caller);
  vtkBoxRepresentation::SafeDownCast(boxWidget->GetRepresentation())->GetTransform(this->LidarScanner->GetTransform());

  //std::cout << "Current transform: " << *(this->LidarScannerTransform) << std::endl;

  CreateRepresentation();

  this->Refresh();
}

void ScannerInteractorStyle::Refresh()
{
  // Refresh
  this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->Render();

  this->Interactor->GetRenderWindow()->Render();
}