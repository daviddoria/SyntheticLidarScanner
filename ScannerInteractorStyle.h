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