#include <QtGui>
#include <QFileDialog>
#include <QString>

#include "form.h"

#include <vtkTransformPolyDataFilter.h>
#include <vtkTransform.h>
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
#include "ScannerInteractorStyle.h"

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

  this->ScannerStyle = vtkSmartPointer<ScannerInteractorStyle>::New();
  this->ui.qvtkWidget->GetRenderWindow()->GetInteractor()->SetInteractorStyle(this->ScannerStyle);
  this->ScannerStyle->SetCurrentRenderer(this->ui.qvtkWidget->GetRenderWindow()
                    ->GetInteractor()->GetRenderWindow()->GetRenderers()->GetFirstRenderer());

  this->Renderer = vtkSmartPointer<vtkRenderer>::New();



  ui.setupUi(this);

  // GUI initializations
  this->ui.txtMinThetaAngle->setText(QString("-10"));
  this->ui.txtMaxThetaAngle->setText(QString("10"));

  this->ui.txtMinPhiAngle->setText(QString("-10"));
  this->ui.txtMaxPhiAngle->setText(QString("10"));

  this->ui.txtNumberOfThetaPoints->setText(QString("10"));
  this->ui.txtNumberOfPhiPoints->setText(QString("10"));

  // Setup Qt things
  this->ConnectSlots();

  this->ui.qvtkWidget->GetRenderWindow()->AddRenderer(this->Renderer);

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
  this->ScannerStyle->LidarScanner->SetMinPhiAngle(this->ui.txtMinPhiAngle->text().toDouble());

  this->ScannerStyle->LidarScanner->SetMaxPhiAngle(this->ui.txtMaxPhiAngle->text().toDouble());

  this->ScannerStyle->LidarScanner->SetMinThetaAngle(this->ui.txtMinThetaAngle->text().toDouble());

  this->ScannerStyle->LidarScanner->SetMaxThetaAngle(this->ui.txtMaxThetaAngle->text().toDouble());

  this->ScannerStyle->LidarScanner->SetNumberOfThetaPoints(this->ui.txtNumberOfThetaPoints->text().toUInt());

  this->ScannerStyle->LidarScanner->SetNumberOfPhiPoints(this->ui.txtNumberOfPhiPoints->text().toUInt());

  //this->ScannerStyle->LidarScanner->SetTransform(transform);

  this->ScannerStyle->LidarScanner->MakeSphericalGrid(); //indicate to use uniform spherical spacing

  //this->LidarScanner->SetCreateMesh(createMesh); // run delaunay on the resulting points?

  this->ScannerStyle->LidarScanner->SetInputConnection(this->ScannerStyle->Scene->GetProducerPort());
  this->ScannerStyle->LidarScanner->Update();

  this->ScannerStyle->LidarScanner->GetAllOutputPoints(this->ScannerStyle->Scan);

  // Setup the visualization
  this->ScannerStyle->ScanMapper->SetInputConnection(this->ScannerStyle->Scan->GetProducerPort());
  this->ScannerStyle->ScanActor->SetMapper(this->ScannerStyle->ScanMapper);

  this->Renderer->AddActor(this->ScannerStyle->ScanActor);

  this->Refresh();

}

void Form::btnSaveScan_clicked()
{
  //set a filename to save
  QString fileName = QFileDialog::getSaveFileName(this,
     tr("Save Scan"), "/home/doriad", tr("Image Files (*.vtp)"));

  std::cout << "Set filename: " << fileName.toStdString() << std::endl;
}

void Form::CreateScannerRepresentation()
{
  this->ScannerStyle->LidarScanner->SetMinThetaAngle(vtkMath::RadiansFromDegrees(this->ui.txtMinThetaAngle->text().toDouble()));
  this->ScannerStyle->LidarScanner->SetMaxThetaAngle(vtkMath::RadiansFromDegrees(this->ui.txtMaxThetaAngle->text().toDouble()));

  this->ScannerStyle->LidarScanner->SetMinPhiAngle(vtkMath::RadiansFromDegrees(this->ui.txtMinPhiAngle->text().toDouble()));
  this->ScannerStyle->LidarScanner->SetMaxPhiAngle(vtkMath::RadiansFromDegrees(this->ui.txtMaxPhiAngle->text().toDouble()));

  this->ScannerStyle->LidarScanner->SetNumberOfThetaPoints(vtkMath::RadiansFromDegrees(this->ui.txtNumberOfThetaPoints->text().toDouble()));
  this->ScannerStyle->LidarScanner->SetNumberOfPhiPoints(vtkMath::RadiansFromDegrees(this->ui.txtNumberOfPhiPoints->text().toDouble()));

  this->ScannerStyle->LidarScanner->SetRepresentationLength(10);

  vtkSmartPointer<vtkPolyData> temp =
    vtkSmartPointer<vtkPolyData>::New();
  this->ScannerStyle->LidarScanner->CreateRepresentation(temp);

  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
    vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilter->SetTransform(this->ScannerStyle->LidarScannerTransform);
  transformFilter->SetInputConnection(temp->GetProducerPort());
  transformFilter->Update();

  this->ScannerStyle->LidarScannerRepresentation->ShallowCopy(transformFilter->GetOutput());
}

void Form::btnPreview_clicked()
{
  CreateScannerRepresentation();

  this->ScannerStyle->LidarScannerMapper->SetInputConnection(this->ScannerStyle->LidarScannerRepresentation->GetProducerPort());
  this->ScannerStyle->LidarScannerActor->SetMapper(this->ScannerStyle->LidarScannerMapper);

  this->Renderer->AddActor(this->ScannerStyle->LidarScannerActor);

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
  this->ScannerStyle->Scene->ShallowCopy(reader->GetOutput());

  this->ScannerStyle->SceneMapper->SetInputConnection(this->ScannerStyle->Scene->GetProducerPort());
  this->ScannerStyle->SceneActor->SetMapper(this->ScannerStyle->SceneMapper);
  this->Renderer->AddActor(this->ScannerStyle->SceneActor);

  this->ResetAndRefresh();

}
