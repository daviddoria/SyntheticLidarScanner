#ifndef FORM_H
#define FORM_H

#include "ui_form.h"

#include "vtkSmartPointer.h"

class vtkRenderer;
class vtkPolyData;
class vtkLidarScanner;
class vtkActor;
class vtkPolyDataMapper;
class vtkBoxWidget2;
class vtkAxesActor;
class vtkOrientationMarkerWidget;
class vtkTransform;

class Form : public QWidget
{
  Q_OBJECT

public:
  Form(int argc, char** argv, QWidget *parent = 0 );
  int argc;
  char** argv;

public slots:
  void btnScan_clicked();
  void btnPreview_clicked();

  void btnSaveScan_clicked();
  void btnOpenFile_clicked();


private:
  Ui::Form ui;

  void ConnectSlots();

  void Refresh();
  void ResetAndRefresh();

  void CreateScannerRepresentation();
  void HandleBoxWidgetEvent();

  vtkSmartPointer<vtkLidarScanner> LidarScanner;
  vtkSmartPointer<vtkPolyData> LidarScannerRepresentation;
  vtkSmartPointer<vtkPolyDataMapper> LidarScannerMapper;
  vtkSmartPointer<vtkActor> LidarScannerActor;
  vtkSmartPointer<vtkTransform> LidarScannerTransform;

  vtkSmartPointer<vtkRenderer> Renderer;

  vtkSmartPointer<vtkPolyData> Scene;
  vtkSmartPointer<vtkActor> SceneActor;
  vtkSmartPointer<vtkPolyDataMapper> SceneMapper;

  vtkSmartPointer<vtkPolyData> Scan;
  vtkSmartPointer<vtkActor> ScanActor;
  vtkSmartPointer<vtkPolyDataMapper> ScanMapper;

  vtkSmartPointer<vtkBoxWidget2> BoxWidget;

  vtkSmartPointer<vtkAxesActor> OrientationAxes;
  vtkSmartPointer<vtkOrientationMarkerWidget> OrientationWidget;

};

#endif
