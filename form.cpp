#include <QtGui>
#include <QFileDialog>
#include <QString>

#include "form.h"

#include <vtkSmartPointer.h>
#include <vtkBoxRepresentation.h>
#include <vtkMath.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkSphereSource.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPointData.h>
#include <vtkRendererCollection.h>
#include <vtkProperty.h>
#include <vtkPlanes.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkBoxWidget2.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>

#include "vtkLidarScanner.h"

#include <sstream>

void Form::ConnectSlots()
{
  connect( this->ui.btnPreview, SIGNAL( clicked() ), this, SLOT(btnPreview_clicked()) );
  connect( this->ui.btnScan, SIGNAL( clicked() ), this, SLOT(btnScan_clicked()) );

  connect( this->ui.btnSaveScan, SIGNAL( clicked() ), this, SLOT(btnSaveScan_clicked()) );
  connect( this->ui.btnOpenFile, SIGNAL( clicked() ), this, SLOT(btnOpenFile_clicked()) );

}

Form::Form(int numArgs, char** args, QWidget *parent)
    : QWidget(parent)
{

  this->argc = numArgs;
  this->argv = args;

  this->LidarScanner = vtkSmartPointer<vtkLidarScanner>::New();
  this->LidarScannerRepresentation = vtkSmartPointer<vtkPolyData>::New();
  this->LidarScannerMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  this->LidarScannerActor = vtkSmartPointer<vtkActor>::New();

  this->Renderer = vtkSmartPointer<vtkRenderer>::New();

  this->Scene = vtkSmartPointer<vtkPolyData>::New();
  this->SceneActor = vtkSmartPointer<vtkActor>::New();
  this->SceneMapper = vtkSmartPointer<vtkPolyDataMapper>::New();

  this->Scan = vtkSmartPointer<vtkPolyData>::New();
  this->ScanActor = vtkSmartPointer<vtkActor>::New();
  this->ScanMapper = vtkSmartPointer<vtkPolyDataMapper>::New();

  this->boxWidget = vtkSmartPointer<vtkBoxWidget2>::New();

  // Orientation widget
  this->OrientationAxes = vtkSmartPointer<vtkAxesActor>::New();
  this->OrientationWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
  this->OrientationWidget->SetOrientationMarker(this->OrientationAxes);
  this->OrientationWidget->SetViewport( 0.0, 0.0, 0.4, 0.4 );

  ui.setupUi(this);

  // GUI initializations
  this->ui.txtMinThetaAngle->setText(QString("10"));
  this->ui.txtMaxThetaAngle->setText(QString("10"));

  this->ui.txtMinPhiAngle->setText(QString("10"));
  this->ui.txtMaxPhiAngle->setText(QString("10"));

  this->ui.txtNumberOfThetaPoints->setText(QString("10"));
  this->ui.txtNumberOfPhiPoints->setText(QString("10"));

  // Setup Qt things
  this->ConnectSlots();

  this->ui.qvtkWidget->GetRenderWindow()->AddRenderer(this->Renderer);

  this->boxWidget->SetInteractor(this->ui.qvtkWidget->GetRenderWindow()->GetInteractor());
  this->boxWidget->On();

  this->OrientationWidget->SetInteractor(this->ui.qvtkWidget->GetRenderWindow()->GetInteractor());
  this->OrientationWidget->On();

  //this->boxWidget->ScalingEnabledOff();
  vtkBoxRepresentation::SafeDownCast(this->boxWidget->GetRepresentation())->HandlesOff();


  //this->ui.qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle( this->Style);

  this->Renderer->SetBackground(.5,.5,1); // Background color white

  this->Refresh();
}


void Form::ResetAndRefresh()
{
  // Refresh
  this->ui.qvtkWidget->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->ResetCamera();

  this->ui.qvtkWidget->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->Render();

  this->ui.qvtkWidget->GetRenderWindow()->Render();

}

void Form::Refresh()
{
  // Refresh
  this->ui.qvtkWidget->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->Render();

  this->ui.qvtkWidget->GetRenderWindow()->Render();

}

void Form::btnScan_clicked()
{
  // Set the scan parameters
  //this->LidarScanner->SetPhiSpan(phiSpan);
  //this->LidarScanner->SetThetaSpan(thetaSpan);
  this->LidarScanner->SetMinPhiAngle(this->ui.txtMinPhiAngle->text().toDouble());

  this->LidarScanner->SetMaxPhiAngle(this->ui.txtMaxPhiAngle->text().toDouble());

  this->LidarScanner->SetMinThetaAngle(this->ui.txtMinThetaAngle->text().toDouble());

  this->LidarScanner->SetMaxThetaAngle(this->ui.txtMaxThetaAngle->text().toDouble());

  this->LidarScanner->SetNumberOfThetaPoints(this->ui.txtNumberOfThetaPoints->text().toUInt());

  this->LidarScanner->SetNumberOfPhiPoints(this->ui.txtNumberOfPhiPoints->text().toUInt());

  //this->LidarScanner->SetTransform(transform);

  this->LidarScanner->MakeSphericalGrid(); //indicate to use uniform spherical spacing

  //this->LidarScanner->SetCreateMesh(createMesh); // run delaunay on the resulting points?

  this->LidarScanner->SetInputConnection(this->Scene->GetProducerPort());
  this->LidarScanner->Update();

  this->LidarScanner->GetAllOutputPoints(this->Scan);

  // Setup the visualization
  this->ScanMapper->SetInputConnection(this->Scan->GetProducerPort());
  this->ScanActor->SetMapper(this->ScanMapper);
  this->Renderer->AddActor(this->ScanActor);

  this->Refresh();

}

void Form::btnSaveScan_clicked()
{
  //set a filename to save
  QString fileName = QFileDialog::getSaveFileName(this,
     tr("Save Scan"), "/home/doriad", tr("Image Files (*.vtp)"));

  std::cout << "Set filename: " << fileName.toStdString() << std::endl;
}

void Form::btnPreview_clicked()
{
  this->LidarScanner->SetMinThetaAngle(this->ui.txtMinThetaAngle->text().toDouble());
  this->LidarScanner->SetMinThetaAngle(this->ui.txtMaxThetaAngle->text().toDouble());

  this->LidarScanner->SetMinThetaAngle(this->ui.txtMinPhiAngle->text().toDouble());
  this->LidarScanner->SetMinThetaAngle(this->ui.txtMaxPhiAngle->text().toDouble());

  this->LidarScanner->SetMinThetaAngle(this->ui.txtNumberOfThetaPoints->text().toDouble());
  this->LidarScanner->SetMinThetaAngle(this->ui.txtNumberOfPhiPoints->text().toDouble());

  this->LidarScanner->SetRepresentationLength(10);

  this->LidarScanner->CreateRepresentation(this->LidarScannerRepresentation);
  this->LidarScannerMapper->SetInputConnection(this->LidarScannerRepresentation->GetProducerPort());
  this->LidarScannerActor->SetMapper(this->LidarScannerMapper);

  this->Renderer->AddActor(this->LidarScannerActor);

  this->Refresh();

}

void Form::btnOpenFile_clicked()
{
  // Get a filename to open
  QString fileName = QFileDialog::getOpenFileName(this,
     //tr("Open File"), "/home/doriad", tr("Image Files (*.vtp)"));
     tr("Open File"), "/media/portable/src/github/SyntheticLidarScanner", tr("Image Files (*.vtp)"));

  std::cout << "Got filename: " << fileName.toStdString() << std::endl;
  if(fileName.toStdString().empty())
    {
    return;
    }
  // Open the file
  vtkSmartPointer<vtkXMLPolyDataReader> reader =
    vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(fileName.toStdString().c_str());
  reader->Update();

  // Store the data as the scene
  this->Scene->ShallowCopy(reader->GetOutput());

  this->SceneMapper->SetInputConnection(this->Scene->GetProducerPort());
  this->SceneActor->SetMapper(this->SceneMapper);
  this->Renderer->AddActor(this->SceneActor);

  this->ResetAndRefresh();

}
