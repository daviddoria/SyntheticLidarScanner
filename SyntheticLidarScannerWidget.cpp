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

#include <QtGui>
#include <QFileDialog>
#include <QString>

#include "SyntheticLidarScannerWidget.h"

#include "Helpers.h"

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

void SyntheticLidarScannerWidget::ConnectSlots()
{
  // The text boxes for the angles should update the display
  connect( this->txtMinThetaAngle, SIGNAL( returnPressed() ), this, SLOT(btnPreview_clicked()) );
  connect( this->txtMinThetaAngle, SIGNAL( editingFinished()), this, SLOT(btnPreview_clicked()) );
  connect( this->txtMaxThetaAngle, SIGNAL( returnPressed() ), this, SLOT(btnPreview_clicked()) );
  connect( this->txtMaxThetaAngle, SIGNAL( editingFinished()), this, SLOT(btnPreview_clicked()) );
  connect( this->txtMinPhiAngle, SIGNAL( returnPressed() ), this, SLOT(btnPreview_clicked()) );
  connect( this->txtMinPhiAngle, SIGNAL( editingFinished()), this, SLOT(btnPreview_clicked()) );
  connect( this->txtMaxPhiAngle, SIGNAL( returnPressed() ), this, SLOT(btnPreview_clicked()) );
  connect( this->txtMaxPhiAngle, SIGNAL( editingFinished()), this, SLOT(btnPreview_clicked()) );

  connect( this->btnPreview, SIGNAL( clicked() ), this, SLOT(btnPreview_clicked()) );
  connect( this->btnScan, SIGNAL( clicked() ), this, SLOT(btnScan_clicked()) );
}

void SyntheticLidarScannerWidget::SharedConstructor()
{
  // This must be called first.
  this->setupUi(this);

  // Set the progress bar to maruee mode
  this->progressBar->setMinimum(0);
  this->progressBar->setMaximum(0);
  this->progressBar->hide();
  this->lblScanning->hide();

  // This must come before the widget setup
  this->Renderer = vtkSmartPointer<vtkRenderer>::New();
  this->qvtkWidget->GetRenderWindow()->AddRenderer(this->Renderer);
  this->Renderer->SetBackground(.5,.5,1);

  this->ScannerStyle = vtkSmartPointer<ScannerInteractorStyle>::New();
  this->ScannerStyle->SetCurrentRenderer(this->Renderer);
  this->ScannerStyle->SetInteractor(this->qvtkWidget->GetInteractor());
  this->ScannerStyle->Initialize();

  this->qvtkWidget->GetInteractor()->SetInteractorStyle(this->ScannerStyle);

  // GUI initializations
  this->txtMinThetaAngle->setText(QString("-10"));
  this->txtMaxThetaAngle->setText(QString("10"));

  this->txtMinPhiAngle->setText(QString("-10"));
  this->txtMaxPhiAngle->setText(QString("10"));

  this->txtNumberOfThetaPoints->setText(QString("10"));
  this->txtNumberOfPhiPoints->setText(QString("10"));

  this->ConnectSlots();

  this->Refresh();
}

SyntheticLidarScannerWidget::SyntheticLidarScannerWidget() : QMainWindow()
{
  SharedConstructor();
}

SyntheticLidarScannerWidget::SyntheticLidarScannerWidget(const std::string& fileName)
{
  SharedConstructor();
  OpenFile(fileName);
}

void SyntheticLidarScannerWidget::ResetAndRefresh()
{
  // Refresh
  this->qvtkWidget->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->ResetCamera();

  this->qvtkWidget->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->Render();

  this->qvtkWidget->GetRenderWindow()->Render();

}

void SyntheticLidarScannerWidget::Refresh()
{
  // Refresh
  this->qvtkWidget->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->Render();

  this->qvtkWidget->GetRenderWindow()->Render();

}

void SyntheticLidarScannerWidget::SetScannerParameters()
{
  this->ScannerStyle->LidarScanner->SetMinPhiAngleDegrees(this->txtMinPhiAngle->text().toDouble());

  this->ScannerStyle->LidarScanner->SetMaxPhiAngleDegrees(this->txtMaxPhiAngle->text().toDouble());

  this->ScannerStyle->LidarScanner->SetMinThetaAngleDegrees(this->txtMinThetaAngle->text().toDouble());

  this->ScannerStyle->LidarScanner->SetMaxThetaAngleDegrees(this->txtMaxThetaAngle->text().toDouble());

  this->ScannerStyle->LidarScanner->SetNumberOfThetaPoints(this->txtNumberOfThetaPoints->text().toUInt());

  this->ScannerStyle->LidarScanner->SetNumberOfPhiPoints(this->txtNumberOfPhiPoints->text().toUInt());
}

void SyntheticLidarScannerWidget::btnScan_clicked()
{
  // Start the progress bar
  this->progressBar->show();
  this->lblScanning->show();

  // Get the parameters from the UI and set them in the LidarScanner object
  SetScannerParameters();

  // Perform the scan
  this->ScannerStyle->LidarScanner->SetInputConnection(this->ScannerStyle->Scene->GetProducerPort());
  this->ScannerStyle->LidarScanner->Update();

  this->ScannerStyle->LidarScanner->GetValidOutputPoints(this->ScannerStyle->Scan);
  Helpers::WritePolyData(this->ScannerStyle->Scan, "scan.vtp");
  
  // Setup the visualization
  this->ScannerStyle->ScanMapper->SetInputConnection(this->ScannerStyle->Scan->GetProducerPort());
  this->ScannerStyle->ScanActor->SetMapper(this->ScannerStyle->ScanMapper);
  this->ScannerStyle->ScanActor->GetProperty()->SetPointSize(4);
  this->ScannerStyle->ScanActor->GetProperty()->SetColor(1,0,0);

  this->Renderer->AddActor(this->ScannerStyle->ScanActor);

  this->Refresh();

  // Stop the progress bar
  this->progressBar->hide();
  this->lblScanning->hide();
}

void SyntheticLidarScannerWidget::on_actionSavePoints_activated()
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


void SyntheticLidarScannerWidget::on_actionSavePTX_activated()
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

void SyntheticLidarScannerWidget::on_actionSaveFullOutput_activated()
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


void SyntheticLidarScannerWidget::btnPreview_clicked()
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

void SyntheticLidarScannerWidget::OpenFile(const std::string& fileName)
{

  // Open the file
  vtkSmartPointer<vtkXMLPolyDataReader> reader =
    vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(fileName.c_str());
  reader->Update();

  // Store the data as the scene
  this->ScannerStyle->Scene->ShallowCopy(reader->GetOutput());

  this->ScannerStyle->SceneMapper->SetInputConnection(this->ScannerStyle->Scene->GetProducerPort());
  this->ScannerStyle->SceneActor->SetMapper(this->ScannerStyle->SceneMapper);
  this->Renderer->AddActor(this->ScannerStyle->SceneActor);

  this->ResetAndRefresh();
}

void SyntheticLidarScannerWidget::on_actionOpen_activated()
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

  OpenFile(fileName.toStdString());

}
