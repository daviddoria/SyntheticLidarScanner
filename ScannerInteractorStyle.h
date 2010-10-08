#ifndef SCANNERINTERACTORSTYLE_H
#define SCANNERINTERACTORSTYLE_H

#include <vtkSmartPointer.h>
#include <vtkInteractorStyleTrackballCamera.h>

class vtkPolyData;
class vtkLidarScanner;
class vtkActor;
class vtkPolyDataMapper;
class vtkBoxWidget2;
class vtkAxesActor;
class vtkOrientationMarkerWidget;
class vtkTransform;

class ScannerInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
  static ScannerInteractorStyle* New();
  vtkTypeMacro(ScannerInteractorStyle, vtkInteractorStyleTrackballCamera);
  ScannerInteractorStyle();

  void Initialize();

  void CreateRepresentation();

  void Refresh();

  void HandleBoxWidgetEvent(vtkObject* caller, long unsigned int eventId, void* callData);

  vtkSmartPointer<vtkLidarScanner> LidarScanner;
  vtkSmartPointer<vtkPolyData> LidarScannerRepresentation;
  vtkSmartPointer<vtkPolyDataMapper> LidarScannerMapper;
  vtkSmartPointer<vtkActor> LidarScannerActor;

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