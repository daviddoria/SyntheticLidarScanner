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
  this->LidarScannerTransform = vtkSmartPointer<vtkTransform>::New();

  this->Scene = vtkSmartPointer<vtkPolyData>::New();
  this->SceneActor = vtkSmartPointer<vtkActor>::New();
  this->SceneMapper = vtkSmartPointer<vtkPolyDataMapper>::New();

  this->Scan = vtkSmartPointer<vtkPolyData>::New();
  this->ScanActor = vtkSmartPointer<vtkActor>::New();
  this->ScanMapper = vtkSmartPointer<vtkPolyDataMapper>::New();

  this->BoxWidget = vtkSmartPointer<vtkBoxWidget2>::New();
  //this->BoxWidget->AddObserver(vtkCommand::InteractionEvent, this, &Form::HandleBoxWidgetEvent);
  this->BoxWidget->AddObserver(vtkCommand::InteractionEvent, this, &ScannerInteractorStyle::HandleBoxWidgetEvent);

  // Orientation widget
  this->OrientationAxes = vtkSmartPointer<vtkAxesActor>::New();
  this->OrientationWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
  this->OrientationWidget->SetOrientationMarker(this->OrientationAxes);
  this->OrientationWidget->SetViewport( 0.0, 0.0, 0.4, 0.4 );

  vtkBoxRepresentation::SafeDownCast(this->BoxWidget->GetRepresentation())->HandlesOff();
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
  std::cout << "Representation has " << this->LidarScannerRepresentation->GetNumberOfPoints() << " points." << std::endl;
  for(vtkIdType i = 0; i < this->LidarScannerRepresentation->GetNumberOfPoints(); i++)
    {
    double p[3];
    this->LidarScannerRepresentation->GetPoint(i,p);
    std::cout << "P: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
    }
}

void ScannerInteractorStyle::HandleBoxWidgetEvent(vtkObject* caller, long unsigned int eventId, void* callData)
{
  vtkBoxWidget2* boxWidget = static_cast<vtkBoxWidget2*>(caller);
  vtkBoxRepresentation::SafeDownCast(boxWidget->GetRepresentation())->GetTransform(this->LidarScannerTransform);

  //CreateScannerRepresentation();

  this->LidarScannerMapper->SetInputConnection(this->LidarScannerRepresentation->GetProducerPort());
  this->LidarScannerActor->SetMapper(this->LidarScannerMapper);

  //this->Renderer->AddActor(this->LidarScannerActor);

  this->Refresh();
}

void ScannerInteractorStyle::Refresh()
{
  // Refresh
  this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->Render();

  this->Interactor->GetRenderWindow()->Render();
}