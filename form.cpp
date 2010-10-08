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
#include <vtkAxesActor.h>

#include "vtkLidarScanner.h"
#include "ScannerInteractorStyle.h"

#include <sstream>

void Form::ConnectSlots()
{
  // The text boxes for the angles should update the display
  connect( this->ui.txtMinThetaAngle, SIGNAL( returnPressed() ), this, SLOT(btnPreview_clicked()) );
  connect( this->ui.txtMinThetaAngle, SIGNAL( editingFinished()), this, SLOT(btnPreview_clicked()) );
  connect( this->ui.txtMaxThetaAngle, SIGNAL( returnPressed() ), this, SLOT(btnPreview_clicked()) );
  connect( this->ui.txtMaxThetaAngle, SIGNAL( editingFinished()), this, SLOT(btnPreview_clicked()) );
  connect( this->ui.txtMinPhiAngle, SIGNAL( returnPressed() ), this, SLOT(btnPreview_clicked()) );
  connect( this->ui.txtMinPhiAngle, SIGNAL( editingFinished()), this, SLOT(btnPreview_clicked()) );
  connect( this->ui.txtMaxPhiAngle, SIGNAL( returnPressed() ), this, SLOT(btnPreview_clicked()) );
  connect( this->ui.txtMaxPhiAngle, SIGNAL( editingFinished()), this, SLOT(btnPreview_clicked()) );

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

  // This must be called first.
  ui.setupUi(this);

  // This must come before the widget setup
  this->Renderer = vtkSmartPointer<vtkRenderer>::New();
  this->ui.qvtkWidget->GetRenderWindow()->AddRenderer(this->Renderer);
  this->Renderer->SetBackground(.5,.5,1);
  
  this->ScannerStyle = vtkSmartPointer<ScannerInteractorStyle>::New();
  this->ScannerStyle->SetCurrentRenderer(this->Renderer);
  this->ScannerStyle->SetInteractor(this->ui.qvtkWidget->GetInteractor());
  this->ScannerStyle->Initialize();

  this->ui.qvtkWidget->GetInteractor()->SetInteractorStyle(this->ScannerStyle);
  
  // GUI initializations
  this->ui.txtMinThetaAngle->setText(QString("-10"));
  this->ui.txtMaxThetaAngle->setText(QString("10"));

  this->ui.txtMinPhiAngle->setText(QString("-10"));
  this->ui.txtMaxPhiAngle->setText(QString("10"));

  this->ui.txtNumberOfThetaPoints->setText(QString("10"));
  this->ui.txtNumberOfPhiPoints->setText(QString("10"));

  this->ConnectSlots();

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

void Form::SetScannerParameters()
{
  this->ScannerStyle->LidarScanner->SetMinPhiAngleDegrees(this->ui.txtMinPhiAngle->text().toDouble());

  this->ScannerStyle->LidarScanner->SetMaxPhiAngleDegrees(this->ui.txtMaxPhiAngle->text().toDouble());

  this->ScannerStyle->LidarScanner->SetMinThetaAngleDegrees(this->ui.txtMinThetaAngle->text().toDouble());

  this->ScannerStyle->LidarScanner->SetMaxThetaAngleDegrees(this->ui.txtMaxThetaAngle->text().toDouble());

  this->ScannerStyle->LidarScanner->SetNumberOfThetaPoints(this->ui.txtNumberOfThetaPoints->text().toUInt());

  this->ScannerStyle->LidarScanner->SetNumberOfPhiPoints(this->ui.txtNumberOfPhiPoints->text().toUInt());
}

void Form::btnScan_clicked()
{

  SetScannerParameters();

  this->ScannerStyle->LidarScanner->SetInputConnection(this->ScannerStyle->Scene->GetProducerPort());
  this->ScannerStyle->LidarScanner->Update();

  this->ScannerStyle->LidarScanner->GetValidOutputPoints(this->ScannerStyle->Scan);

  // Setup the visualization
  this->ScannerStyle->ScanMapper->SetInputConnection(this->ScannerStyle->Scan->GetProducerPort());
  this->ScannerStyle->ScanActor->SetMapper(this->ScannerStyle->ScanMapper);
  this->ScannerStyle->ScanActor->GetProperty()->SetPointSize(4);
  this->ScannerStyle->ScanActor->GetProperty()->SetColor(1,0,0);

  this->Renderer->AddActor(this->ScannerStyle->ScanActor);

  this->Refresh();

}

void Form::btnSaveScan_clicked()
{
  // Set a filename to save
  QString fileName = QFileDialog::getSaveFileName(this,
     tr("Save Scan"), "/home/doriad", tr("Image Files (*.vtp)"));

  std::cout << "Saving to " << fileName.toStdString() << "..." << std::endl;

  vtkSmartPointer<vtkXMLPolyDataWriter> writer =
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInputConnection(this->ScannerStyle->Scan->GetProducerPort());
  writer->SetFileName(fileName.toStdString().c_str());
  writer->Write();
  
}


void Form::btnPreview_clicked()
{
  SetScannerParameters();

  this->ScannerStyle->CreateRepresentation();
  this->ScannerStyle->LidarScannerMapper->SetInputConnection(this->ScannerStyle->LidarScannerRepresentation->GetProducerPort());
  this->ScannerStyle->LidarScannerActor->SetMapper(this->ScannerStyle->LidarScannerMapper);
  this->ScannerStyle->LidarScannerActor->GetProperty()->SetOpacity(.5);
  this->ScannerStyle->LidarScannerActor->GetProperty()->SetColor(224./255., 176./255., 1);
  
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
