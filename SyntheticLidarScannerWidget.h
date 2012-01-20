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

#ifndef SyntheticLidarScannerWidget_H
#define SyntheticLidarScannerWidget_H

#include "ui_SyntheticLidarScannerWidget.h"

#include "vtkSmartPointer.h"

class vtkRenderer;

class ScannerInteractorStyle;

class SyntheticLidarScannerWidget : public QMainWindow
{
  Q_OBJECT

public:
  SyntheticLidarScannerWidget(int argc, char** argv, QWidget *parent = 0 );
  int argc;
  char** argv;

public slots:
  void btnScan_clicked();
  void btnPreview_clicked();

  void on_actionSavePoints_activated();
  void on_actionSaveFullOutput_activated();
  void on_actionOpenFile_activated();
  void on_actionWritePTX_activated();

  void SetScannerParameters();

private:
  Ui::SyntheticLidarScannerWidget ui;

  void ConnectSlots();

  void Refresh();
  void ResetAndRefresh();

  vtkSmartPointer<vtkRenderer> Renderer;

  vtkSmartPointer<ScannerInteractorStyle> ScannerStyle;

};

#endif
