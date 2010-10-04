#include <vtkConeSource.h>
#include <vtkCamera.h>
#include <vtkProperty.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>

int main(int, char *[])
{
  //Create a cone
  vtkSmartPointer<vtkConeSource> coneSource =
    vtkSmartPointer<vtkConeSource>::New();
  coneSource->SetResolution(4);
  coneSource->Update();

  vtkSmartPointer<vtkTransform> transform =
    vtkSmartPointer<vtkTransform>::New();
  transform->PostMultiply();
  transform->RotateX(45);
  transform->RotateZ(-90);

  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
    vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilter->SetTransform(transform);
  transformFilter->SetInputConnection(coneSource->GetOutputPort());
  transformFilter->Update();

  vtkSmartPointer<vtkPolyData> tempPolydata =
    vtkSmartPointer<vtkPolyData>::New();
  tempPolydata->SetPoints(transformFilter->GetOutput()->GetPoints());

  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
    vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputConnection(tempPolydata->GetProducerPort());
  glyphFilter->Update();

  vtkSmartPointer<vtkPolyData> polydata =
    vtkSmartPointer<vtkPolyData>::New();
  polydata->ShallowCopy(glyphFilter->GetOutput());
  std::cout << polydata->GetNumberOfPoints() << " points and " << polydata->GetNumberOfCells() << " cells." << std::endl;
  // Setup colors
  unsigned char black[3] = {0, 0, 0};
  unsigned char red[3] = {255, 0, 0};
  unsigned char green[3] = {0, 255, 0};
  unsigned char blue[3] = {0, 0, 255};
  unsigned char white[3] = {255, 255, 255};

  vtkSmartPointer<vtkUnsignedCharArray> colors =
    vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName ("Colors");
  colors->InsertNextTupleValue(black);
  colors->InsertNextTupleValue(red);
  colors->InsertNextTupleValue(green);
  colors->InsertNextTupleValue(blue);
  colors->InsertNextTupleValue(white);

  polydata->GetPointData()->SetScalars(colors);

  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(polydata->GetProducerPort());

  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetPointSize(6);

  //Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkSmartPointer<vtkAxesActor> axes =
    vtkSmartPointer<vtkAxesActor>::New();

  vtkSmartPointer<vtkOrientationMarkerWidget> widget =
    vtkSmartPointer<vtkOrientationMarkerWidget>::New();
  widget->SetOrientationMarker( axes );
  widget->SetInteractor( renderWindowInteractor );
  widget->SetViewport( 0.0, 0.0, 0.4, 0.4 );
  widget->SetEnabled( 1 );
  //widget->InteractiveOn();

  renderer->AddActor(actor);
  renderer->SetBackground(.3, .2, .1); // Background color dark red

  //renderer->ResetCamera();

  vtkSmartPointer<vtkCamera> camera =
    vtkSmartPointer<vtkCamera>::New();
  camera->SetPosition(0, 0, 0);
  camera->SetFocalPoint(0, 1, 0);
  renderer->SetActiveCamera(camera);
  renderer->ResetCamera();

  renderer->Render();
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}