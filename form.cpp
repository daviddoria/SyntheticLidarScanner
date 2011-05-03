#include <QtGui>
#include <QFileDialog>
#include <QString>

#include "form.h"

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkBoxRepresentation.h>
#include <vtkBoxWidget2.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkImageData.h>
#include <vtkMath.h>
#include <vtkPlanes.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRendererCollection.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLImageDataWriter.h>

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

  connect( this->ui.btnSavePoints, SIGNAL( clicked() ), this, SLOT(btnSavePoints_clicked()) );
  connect( this->ui.btnSaveFullOutput, SIGNAL( clicked() ), this, SLOT(btnSaveFullOutput_clicked()) );
  connect( this->ui.btnOpenFile, SIGNAL( clicked() ), this, SLOT(btnOpenFile_clicked()) );
  connect( this->ui.btnWritePTX, SIGNAL( clicked()), this, SLOT(btnWritePTX_clicked()) );
}

Form::Form(int numArgs, char** args, QWidget *parent)
    : QWidget(parent)
{

  this->argc = numArgs;
  this->argv = args;

  // This must be called first.
  ui.setupUi(this);

  // Set the progress bar to maruee mode
  this->ui.progressBar->setMinimum(0);
  this->ui.progressBar->setMaximum(0);
  this->ui.progressBar->hide();
  this->ui.lblScanning->hide();

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
  // Start the progress bar
  this->ui.progressBar->show();
  this->ui.lblScanning->show();

  // Get the parameters from the UI and set them in the LidarScanner object
  SetScannerParameters();

  // Perform the scan
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

  // Stop the progress bar
  this->ui.progressBar->hide();
  this->ui.lblScanning->hide();
}

void Form::btnSavePoints_clicked()
{
  // Set a filename to save
  QString fileName = QFileDialog::getSaveFileName(this,
     tr("Save Scan"), "/home/doriad", tr("Image Files (*.vtp)"));

  std::cout << "Saving to " << fileName.toStdString() << "..." << std::endl;

  if(this->ScannerStyle->Scan)
    {
    vtkSmartPointer<vtkXMLPolyDataWriter> writer =
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    writer->SetInputConnection(this->ScannerStyle->Scan->GetProducerPort());
    writer->SetFileName(fileName.toStdString().c_str());
    writer->Write();
    }
  else
    {
    std::cerr << "You must scan before you can save!" << std::endl;
    }

}


void Form::btnWritePTX_clicked()
{
  // Set a filename to save
  QString fileName = QFileDialog::getSaveFileName(this,
     tr("Save Scan"), "/home/doriad", tr("Image Files (*.ptx)"));

  std::cout << "Saving to " << fileName.toStdString() << "..." << std::endl;

  if(this->ScannerStyle->Scan)
    {
    this->ScannerStyle->LidarScanner->WritePTX(fileName.toStdString());
    }
  else
    {
    std::cerr << "You must scan before you can save!" << std::endl;
    }

}

void Form::btnSaveFullOutput_clicked()
{
  // Set a filename to save
  QString fileName = QFileDialog::getSaveFileName(this,
     tr("Save Scan"), "/home/doriad", tr("Image Files (*.vti)"));

  std::cout << "Saving to " << fileName.toStdString() << "..." << std::endl;

  vtkImageData* fullOutput = this->ScannerStyle->LidarScanner->GetOutput();
  if(fullOutput)
    {
    vtkSmartPointer<vtkXMLImageDataWriter> writer =
      vtkSmartPointer<vtkXMLImageDataWriter>::New();
    writer->SetInputConnection(fullOutput->GetProducerPort());
    writer->SetFileName(fileName.toStdString().c_str());
    writer->Write();
    }
  else
    {
    std::cerr << "You must scan before you can save!" << std::endl;
    }
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
